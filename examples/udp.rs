#![no_main]
#![no_std]

#[macro_use]
extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt;
extern crate stm32f429 as board;
extern crate stm32_eth as eth;
extern crate smoltcp;
#[macro_use]
extern crate log;
extern crate panic_itm;

use board::{Peripherals, CorePeripherals, SYST};

use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex};
// use cortex_m::asm;
use cortex_m_rt::ExceptionFrame;

use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr};
use smoltcp::iface::{NeighborCache, EthernetInterfaceBuilder};
use smoltcp::socket::{SocketSet, UdpSocket, UdpSocketBuffer};
use smoltcp::storage::PacketMetadata;
use log::{Record, Level, Metadata, LevelFilter};

use eth::{Eth, RingEntry};

static mut LOGGER: ItmLogger = ItmLogger;

struct ItmLogger;

impl log::Log for ItmLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Trace
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let mut cp = unsafe { CorePeripherals::steal() };
            iprintln!(&mut cp.ITM.stim[0], "[{}] {}", record.level(), record.args());
        }
    }

    fn flush(&self) {}
}

static TIME: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));

entry!(main);

fn main() -> ! {
    unsafe { log::set_logger(&LOGGER).unwrap(); }
    log::set_max_level(LevelFilter::Debug);

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    setup_systick(&mut cp.SYST);

    eth::setup(&p);
    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 4] = Default::default();
    let mut eth = Eth::new(
        p.ETHERNET_MAC, p.ETHERNET_DMA,
        &mut rx_ring[..], &mut tx_ring[..]
    );
    // eth.enable_interrupt(&mut cp.NVIC);

    // enable RNG
    p.RCC.ahb2enr.modify(|_, w| w.rngen().set_bit());
    p.RNG.cr.modify(|_, w| w.rngen().set_bit());

    let ip_addr = IpCidr::new(IpAddress::v4(192, 168, 1, 120), 24);
    let mut ip_addrs = [ip_addr];
    let mut neighbor_storage = [None; 16];
    let neighbor_cache = NeighborCache::new(&mut neighbor_storage[..]);
    let serial: u32 = unsafe {
        *(0x1FFF_7A10 as *const u32) ^
        *(0x1FFF_7A14 as *const u32) ^
        *(0x1FFF_7A18 as *const u32)
    };
    let ethernet_addr = EthernetAddress([
        0x46, 0x52, 0x4d,  // F R M
        (serial >> 16) as u8, (serial >> 8) as u8, serial as u8
    ]);
    let mut iface = EthernetInterfaceBuilder::new(&mut eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(neighbor_cache)
        .finalize();

    let mut rx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut tx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut rx_data_buffer = [0; 1500*4];
    let mut tx_data_buffer = [0; 1500*4];
    let mut udp_socket = UdpSocket::new(
        UdpSocketBuffer::new(&mut rx_meta_buffer[..], &mut rx_data_buffer[..]),
        UdpSocketBuffer::new(&mut tx_meta_buffer[..], &mut tx_data_buffer[..])
    );
    udp_socket.bind((IpAddress::v4(192, 168, 1, 120), 50000)).unwrap();
    let mut sockets_storage = [None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let handle = sockets.add(udp_socket);
    let mut target = None;

    info!("------------------------------------------------------------------------");
    let mut seq_number = 0u32;
    loop {
        //let random = p.RNG.dr.read().bits();
        {
            let mut socket = sockets.get::<UdpSocket>(handle);
            // if socket.can_recv() {
                while let Ok((msg, ep)) = socket.recv() {
                    if msg == b"start" {
                        info!("got start message");
                        target = Some(ep);
                        seq_number = 0;
                    } else if msg == b"stop " {
                        info!("got stop message");
                        target = None;
                    }
                }
            // }
            if let Some(tgt) = target {
                while let Ok(buf) = socket.send(1400, tgt) {
                    buf[0] = (seq_number >> 24) as u8;
                    buf[1] = (seq_number >> 16) as u8;
                    buf[2] = (seq_number >> 8) as u8;
                    buf[3] = seq_number as u8;
                    seq_number += 1;
                }
            }
        }
        let time = interrupt::free(|cs| *TIME.borrow(cs).borrow());
        if let Err(e) = iface.poll(&mut sockets, Instant::from_millis(time as i64)) {
            warn!("{}", e);
        }
    }
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(SYST::get_ticks_per_10ms() / 10);
    syst.enable_counter();
    syst.enable_interrupt();
}

exception!(SysTick, systick_interrupt_handler);

fn systick_interrupt_handler() {
    interrupt::free(|cs| *TIME.borrow(cs).borrow_mut() += 1);
}

exception!(HardFault, hard_fault);

fn hard_fault(ef: &ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

exception!(*, default_handler);

fn default_handler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
