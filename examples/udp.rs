#![no_main]
#![no_std]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
// #[macro_use]
extern crate stm32f429 as board;
extern crate stm32_eth as eth;
extern crate smoltcp;
extern crate log;
extern crate panic_semihosting;

// use cortex_m::asm;
use board::{Peripherals, CorePeripherals, SYST};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::ExceptionFrame;

use core::fmt::Write;
use cortex_m_semihosting::hio;

use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr,
                    IpEndpoint, Ipv4Address};
use smoltcp::iface::{NeighborCache, EthernetInterfaceBuilder};
use smoltcp::socket::{SocketSet, UdpSocket, UdpSocketBuffer};
use smoltcp::storage::PacketMetadata;
use log::{Record, Level, Metadata, LevelFilter};

use eth::{Eth, RingEntry};

static mut LOGGER: HioLogger = HioLogger {};

struct HioLogger {}

impl log::Log for HioLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Trace
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let mut stdout = hio::hstdout().unwrap();
            writeln!(stdout, "{} - {}", record.level(), record.args())
                .unwrap();
        }
    }
    fn flush(&self) {}
}

static TIME: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));

entry!(main);

fn main() -> ! {
    unsafe { log::set_logger(&LOGGER).unwrap(); }
    log::set_max_level(LevelFilter::Info);

    let mut stdout = hio::hstdout().unwrap();

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    setup_systick(&mut cp.SYST);

    writeln!(stdout, "Enabling ethernet...").unwrap();
    eth::setup(&p);
    let mut rx_ring: [RingEntry<_>; 8] = Default::default();
    let mut tx_ring: [RingEntry<_>; 4] = Default::default();
    let mut eth = Eth::new(
        p.ETHERNET_MAC, p.ETHERNET_DMA,
        &mut rx_ring[..], &mut tx_ring[..]
    );
    // eth.enable_interrupt(&mut cp.NVIC);

    let local_addr = Ipv4Address::new(192, 168, 1, 120);
    let ip_addr = IpCidr::new(IpAddress::from(local_addr), 24);
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

    let mut rx_meta_buffer = [PacketMetadata::EMPTY; 1];
    let mut tx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut rx_data_buffer = [0; 1500];
    let mut tx_data_buffer = [0; 1500*4];
    let mut udp_socket = UdpSocket::new(
        UdpSocketBuffer::new(&mut rx_meta_buffer[..], &mut rx_data_buffer[..]),
        UdpSocketBuffer::new(&mut tx_meta_buffer[..], &mut tx_data_buffer[..])
    );
    udp_socket.bind((IpAddress::v4(192, 168, 1, 120), 50000)).unwrap();
    let mut sockets_storage = [None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let handle = sockets.add(udp_socket);
    let target = IpEndpoint { addr: IpAddress::v4(192, 168, 1, 1), port: 12345 };

    writeln!(stdout, "Ready").unwrap();
    let mut nn = 0u32;
    loop {
        while let Ok(buf) = sockets.get::<UdpSocket>(handle).send(1400, target) {
            buf[0] = (nn >> 24) as u8;
            buf[1] = (nn >> 16) as u8;
            buf[2] = (nn >> 8) as u8;
            buf[3] = nn as u8;
            nn = (nn + 1) % (10*1024);
        }
        let time = cortex_m::interrupt::free(|cs| *TIME.borrow(cs).borrow());
        let _ = iface.poll(&mut sockets, Instant::from_millis(time as i64));
    }
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(SYST::get_ticks_per_10ms() / 10);
    syst.enable_counter();
    syst.enable_interrupt();
}

exception!(SysTick, systick_interrupt_handler);

fn systick_interrupt_handler() {
    cortex_m::interrupt::free(|cs| *TIME.borrow(cs).borrow_mut() += 1);
}

exception!(HardFault, hard_fault);

fn hard_fault(ef: &ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

exception!(*, default_handler);

fn default_handler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
