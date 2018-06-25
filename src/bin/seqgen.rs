#![no_main]
#![no_std]
#![feature(cell_update)]

#[macro_use]
extern crate log;
#[macro_use]
extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt;
extern crate stm32f429 as board;
extern crate stm32_eth as eth;
extern crate panic_itm;
extern crate smoltcp;

use core::cell::Cell;
use board::{Peripherals, CorePeripherals, SYST};
use log::{Record, Level, Metadata, LevelFilter};
use cortex_m::interrupt::{self, Mutex};
use cortex_m::peripheral;
use cortex_m_rt::ExceptionFrame;
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr};
use smoltcp::iface::{NeighborCache, EthernetInterfaceBuilder, Routes};
use smoltcp::socket::{SocketSet, UdpSocket, UdpSocketBuffer, RawSocketBuffer};
use smoltcp::storage::PacketMetadata;
use smoltcp::dhcp::Dhcpv4Client;
use eth::{Eth, RingEntry};

struct ItmLogger;

impl log::Log for ItmLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Trace
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let stim = unsafe { &mut (*peripheral::ITM::ptr()).stim[0] };
            iprintln!(stim, "[{}] {}", record.level(), record.args());
        }
    }

    fn flush(&self) {}
}

static LOGGER: ItmLogger = ItmLogger;
static ETH_TIME: Mutex<Cell<i64>> = Mutex::new(Cell::new(0));

entry!(main);

fn main() -> ! {
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(LevelFilter::Info);

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    setup_clock(&p);
    setup_systick(&mut cp.SYST);
    eth::setup(&p);

    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 4] = Default::default();
    let mut eth = Eth::new(
        p.ETHERNET_MAC, p.ETHERNET_DMA,
        &mut rx_ring[..], &mut tx_ring[..]
    );

    let serial = read_serno();
    let ethernet_addr = EthernetAddress([
        0x46, 0x52, 0x4d,  // F R M
        (serial >> 16) as u8, (serial >> 8) as u8, serial as u8
    ]);
    let mut ip_addrs = [IpCidr::new(IpAddress::v4(0, 0, 0, 0), 0)];
    let mut neighbor_storage = [None; 16];
    let mut routes_storage = [None; 2];
    let mut iface = EthernetInterfaceBuilder::new(&mut eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(NeighborCache::new(&mut neighbor_storage[..]))
        .routes(Routes::new(&mut routes_storage[..]))
        .finalize();

    let mut udp_rx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut udp_tx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut udp_rx_data_buffer = [0; 1500*4];
    let mut udp_tx_data_buffer = [0; 1500*4];
    let mut dhcp_rx_meta_buffer = [PacketMetadata::EMPTY; 1];
    let mut dhcp_tx_meta_buffer = [PacketMetadata::EMPTY; 1];
    let mut dhcp_rx_data_buffer = [0; 1500];
    let mut dhcp_tx_data_buffer = [0; 1500*2];

    let udp_socket = UdpSocket::new(
        UdpSocketBuffer::new(&mut udp_rx_meta_buffer[..], &mut udp_rx_data_buffer[..]),
        UdpSocketBuffer::new(&mut udp_tx_meta_buffer[..], &mut udp_tx_data_buffer[..])
    );
    let mut sockets_storage = [None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let mut target = None;

    let dhcp_rx_buffer = RawSocketBuffer::new(&mut dhcp_rx_meta_buffer[..], &mut dhcp_rx_data_buffer[..]);
    let dhcp_tx_buffer = RawSocketBuffer::new(&mut dhcp_tx_meta_buffer[..], &mut dhcp_tx_data_buffer[..]);
    let mut dhcp = Dhcpv4Client::new(&mut sockets, dhcp_rx_buffer, dhcp_tx_buffer, Instant::from_millis(0));

    let udp_handle = sockets.add(udp_socket);
    let mut seq_number = 0u32;
    let mut setup_done = false;

    info!("------------------------------------------------------------------------");
    loop {
        {
            let mut socket = sockets.get::<UdpSocket>(udp_handle);
            while let Ok((msg, ep)) = socket.recv() {
                if msg == b"start" {
                    info!("got start message");
                    target = Some(ep);
                    seq_number = 0;
                } else if msg == b"stop" {
                    info!("got stop message");
                    target = None;
                }
            }
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
        let time = Instant::from_millis(interrupt::free(|cs| ETH_TIME.borrow(cs).get()));
        if let Err(e) = iface.poll(&mut sockets, time) {
            warn!("poll: {}", e);
        }
        if !setup_done {
            let ip_addr = iface.ipv4_addr().unwrap();
            if !ip_addr.is_unspecified() {
                let gw = iface.routes().lookup(&IpAddress::v4(8, 8, 8, 8), time);
                info!("dhcp done: {} via {:?}", ip_addr, gw);
                sockets.get::<UdpSocket>(udp_handle).bind((ip_addr, 50000)).unwrap();
                setup_done = true;
            } else if let Err(e) = dhcp.poll(&mut iface, &mut sockets, time) {
                warn!("dhcp: {}", e);
            }
        }
    }
}

fn read_serno() -> u32 {
    unsafe {
        *(0x1FFF_7A10 as *const u32) ^
        *(0x1FFF_7A14 as *const u32) ^
        *(0x1FFF_7A18 as *const u32)
    }
}

fn setup_clock(p: &Peripherals) {
    // setup for 168 MHz, 84 MHz, 42 MHz
    let pll_n_bits = 336;
    let hpre_bits = 0b111;
    let ppre2_bits = 0b100;
    let ppre1_bits = 0b101;

    // adjust flash wait states
    p.FLASH.acr.modify(|_, w| unsafe { w.latency().bits(0b110) });

    // use PLL as source
    p.RCC.pllcfgr.modify(|_, w| unsafe { w.plln().bits(pll_n_bits as u16) });

    // enable PLL
    p.RCC.cr.modify(|_, w| w.pllon().set_bit());
    // wait for PLL ready
    while p.RCC.cr.read().pllrdy().bit_is_clear() {}

    // enable PLL
    p.RCC.cfgr.write(|w| unsafe { w
                                  // APB high-speed prescaler (APB2)
                                  .ppre2().bits(ppre2_bits)
                                  // APB Low speed prescaler (APB1)
                                  .ppre1().bits(ppre1_bits)
                                  // AHB prescaler
                                  .hpre().bits(hpre_bits)
                                  // System clock switch
                                  // PLL selected as system clock
                                  .sw1().bit(true)
                                  .sw0().bit(false) });
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(SYST::get_ticks_per_10ms() / 10);
    syst.enable_counter();
    syst.enable_interrupt();
}

exception!(SysTick, systick_interrupt_handler);

fn systick_interrupt_handler() {
    interrupt::free(|cs| ETH_TIME.borrow(cs).update(|v| v.wrapping_add(1)));
}

exception!(HardFault, hard_fault);

fn hard_fault(ef: &ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

exception!(*, default_handler);

fn default_handler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
