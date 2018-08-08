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
use log::{Record, Metadata, LevelFilter};
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

fn itm() -> &'static mut peripheral::itm::Stim {
    unsafe { &mut (*peripheral::ITM::ptr()).stim[0] }
}

impl log::Log for ItmLogger {
    fn log(&self, record: &Record) {
        iprintln!(itm(), "[{}] {}", record.level(), record.args());
    }

    fn enabled(&self, _: &Metadata) -> bool { true }
    fn flush(&self) {}
}

static LOGGER: ItmLogger = ItmLogger;
static ETH_TIME: Mutex<Cell<i64>> = Mutex::new(Cell::new(0));

entry!(main);

fn main() -> ! {
    // enable logging if someone is listening on ITM
    if itm().is_fifo_ready() {
        log::set_logger(&LOGGER).unwrap();
        log::set_max_level(LevelFilter::Info);
    }

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

    let dhcp_rx_buffer = RawSocketBuffer::new(&mut dhcp_rx_meta_buffer[..], &mut dhcp_rx_data_buffer[..]);
    let dhcp_tx_buffer = RawSocketBuffer::new(&mut dhcp_tx_meta_buffer[..], &mut dhcp_tx_data_buffer[..]);
    let mut dhcp = Dhcpv4Client::new(&mut sockets, dhcp_rx_buffer, dhcp_tx_buffer, Instant::from_millis(0));

    let udp_handle = sockets.add(udp_socket);
    let mut setup_done = false;

    let mut target = None;
    let mut seq_number = 0u32;

    info!("------------------------------------------------------------------------");
    loop {
        // process packets
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
        // handle ethernet
        let time = Instant::from_millis(interrupt::free(|cs| ETH_TIME.borrow(cs).get()));
        if let Err(e) = iface.poll(&mut sockets, time) {
            warn!("poll: {}", e);
        }
        // ethernet setup
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

fn setup_clock(p: &Peripherals) {
    // setup for 180 MHz, 90 MHz, 45 MHz from 8MHz HSE (Nucleo MCO)
    let pll_m = 8;
    let pll_n = 360; // fVCO = 360 MHz
    let pll_p = 2;
    let pll_q = 8; // to get <= 48 MHz
    let flash_latency = 5;
    let ahb_div  = 0b111;
    let apb2_div = 0b100; // div2
    let apb1_div = 0b101; // div4

    // enable HSE
    p.RCC.cr.modify(|_, w| w.hseon().set_bit());
    while p.RCC.cr.read().hserdy().bit_is_clear() {}

    // select regulator voltage scale 1
    p.RCC.apb1enr.modify(|_, w| w.pwren().set_bit());
    p.PWR.cr.modify(|_, w| unsafe { w.vos().bits(0b11) });

    // configure PLL frequency
    p.RCC.pllcfgr.modify(|_, w| unsafe { w.pllm().bits(pll_m)
                                          .plln().bits(pll_n)
                                          .pllp().bits((pll_p >> 1) - 1)
                                          .pllq().bits(pll_q)
                                          .pllsrc().set_bit() });

    // enable PLL
    p.RCC.cr.modify(|_, w| w.pllon().set_bit());
    // wait for PLL ready
    while p.RCC.cr.read().pllrdy().bit_is_clear() {}

    // enable overdrive
    p.PWR.cr.modify(|_, w| w.oden().set_bit());
    while p.PWR.csr.read().odrdy().bit_is_clear() {}
    p.PWR.cr.modify(|_, w| w.odswen().set_bit());
    while p.PWR.csr.read().odswrdy().bit_is_clear() {}

    // adjust icache and flash wait states
    p.FLASH.acr.modify(|_, w| unsafe { w.icen().set_bit()
                                        .dcen().set_bit()
                                        .prften().set_bit()
                                        .latency().bits(flash_latency) });

    // enable PLL as clock source
    p.RCC.cfgr.write(|w| unsafe { w
                                  // APB high-speed prescaler (APB2)
                                  .ppre2().bits(apb2_div)
                                  // APB low-speed prescaler (APB1)
                                  .ppre1().bits(apb1_div)
                                  // AHB prescaler
                                  .hpre().bits(ahb_div)
                                  // PLL selected as system clock
                                  .sw1().bit(true)
                                  .sw0().bit(false) });
    while p.RCC.cfgr.read().sws1().bit_is_clear() {}
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(SYST::get_ticks_per_10ms() / 10);
    syst.enable_counter();
    syst.enable_interrupt();
}

fn read_serno() -> u32 {
    unsafe {
        *(0x1FFF_7A10 as *const u32) ^
        *(0x1FFF_7A14 as *const u32) ^
        *(0x1FFF_7A18 as *const u32)
    }
}

exception!(SysTick, systick);

fn systick() {
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
