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
extern crate arrayvec;
extern crate byteorder;

use core::cell::Cell;
use arrayvec::ArrayVec;
use byteorder::{ByteOrder, LE};
use board::{Peripherals, CorePeripherals, SYST};
use log::{Record, Metadata, LevelFilter};
use cortex_m::peripheral;
use cortex_m::interrupt::{self, Mutex};
use cortex_m_rt::ExceptionFrame;
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, IpEndpoint};
use smoltcp::iface::{NeighborCache, EthernetInterfaceBuilder, Routes};
use smoltcp::socket::{SocketSet, UdpSocket, UdpSocketBuffer, RawSocketBuffer};
use smoltcp::storage::PacketMetadata;
use smoltcp::dhcp::Dhcpv4Client;
use eth::{Eth, RingEntry};

const PORT: u16 = 54321;

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
    setup_rng(&p);
    setup_10mhz(&p);
    eth::setup(&p);
    setup_gpio(&p);

    // red LED: indicate "booting"
    set_leds(true, false, false);

    // DHCP if user button pressed
    let use_dhcp = p.GPIOC.idr.read().idr13().bit_is_set();

    // set up ring buffers for network handling tokens
    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 16] = Default::default();
    let mut eth = Eth::new(
        p.ETHERNET_MAC, p.ETHERNET_DMA,
        &mut rx_ring[..], &mut tx_ring[..]
    );

    // determine MAC address from board's serial number
    let serial = read_serno();
    let ethernet_addr = EthernetAddress([
        0x46, 0x52, 0x4d,  // F R M
        (serial >> 16) as u8, (serial >> 8) as u8, serial as u8
    ]);

    // select the default Mesytec IP if static configuration
    let mut ip_addrs = if !use_dhcp {
        [IpCidr::new(IpAddress::v4(192, 168, 168, 121), 24)]
    } else {
        [IpCidr::new(IpAddress::v4(0, 0, 0, 0), 0)]
    };
    let mut neighbor_storage = [None; 16];
    let mut routes_storage = [None; 2];
    let mut iface = EthernetInterfaceBuilder::new(&mut eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(NeighborCache::new(&mut neighbor_storage[..]))
        .routes(Routes::new(&mut routes_storage[..]))
        .finalize();

    // set up buffers for packet content and metadata
    let mut udp_rx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut udp_tx_meta_buffer = [PacketMetadata::EMPTY; 16];
    let mut udp_rx_data_buffer = [0; 1500*4];
    let mut udp_tx_data_buffer = [0; 1500*16];
    let mut dhcp_rx_meta_buffer = [PacketMetadata::EMPTY; 1];
    let mut dhcp_tx_meta_buffer = [PacketMetadata::EMPTY; 1];
    let mut dhcp_rx_data_buffer = [0; 1500];
    let mut dhcp_tx_data_buffer = [0; 1500*2];

    // create the UDP socket
    let udp_socket = UdpSocket::new(
        UdpSocketBuffer::new(&mut udp_rx_meta_buffer[..], &mut udp_rx_data_buffer[..]),
        UdpSocketBuffer::new(&mut udp_tx_meta_buffer[..], &mut udp_tx_data_buffer[..])
    );
    let mut sockets_storage = [None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);

    // set up buffers for DHCP handling
    let dhcp_rx_buffer = RawSocketBuffer::new(&mut dhcp_rx_meta_buffer[..], &mut dhcp_rx_data_buffer[..]);
    let dhcp_tx_buffer = RawSocketBuffer::new(&mut dhcp_tx_meta_buffer[..], &mut dhcp_tx_data_buffer[..]);
    let mut dhcp = Dhcpv4Client::new(&mut sockets, dhcp_rx_buffer, dhcp_tx_buffer, Instant::from_millis(0));

    let udp_handle = sockets.add(udp_socket);
    let mut setup_done = false;
    let mut cmd_buf = [0; 128];

    let mut gen = Generator::new(p.TIM2);

    info!("------------------------------------------------------------------------");
    loop {
        // process packets
        {
            let mut socket = sockets.get::<UdpSocket>(udp_handle);
            while let Ok((n, ep)) = socket.recv_slice(&mut cmd_buf) {
                gen.process_command(&mut socket, &cmd_buf[..n], ep);
            }
            gen.maybe_send_data(&mut socket);
        }
        // handle ethernet
        let time = Instant::from_millis(interrupt::free(|cs| ETH_TIME.borrow(cs).get()));
        if let Err(e) = iface.poll(&mut sockets, time) {
            warn!("poll: {}", e);
        }
        // ethernet setup
        if !setup_done {
            let ip_addr = iface.ipv4_addr().unwrap();
            // setup is done as soon as we have an IP (via DHCP or static)
            if !ip_addr.is_unspecified() {
                info!("IP setup done ({}), binding to {}:{}",
                      if use_dhcp { "dhcp" } else { "static" }, ip_addr, PORT);
                sockets.get::<UdpSocket>(udp_handle).bind((ip_addr, PORT)).unwrap();
                setup_done = true;
                // green LED: bootup finished
                set_leds(false, true, false);
            } else if let Err(e) = dhcp.poll(&mut iface, &mut sockets, time) {
                warn!("dhcp: {}", e);
            }
        }
    }
}

struct Generator {
    endpoint: IpEndpoint,
    timer: board::TIM2,

    mcpd_id: u8,
    run_id: u16,
    interval: u32,     // 10MHz times between events
    pkt_interval: u32, // 10MHz times between packets

    buf_no: u16,
    run: bool,
    time: u64,
    lastpkt: u64,
    npkt: u32,
    nevt: u32,
    npkt_per_size: [u32; 221],
}

impl Generator {
    fn new(timer: board::TIM2) -> Self {
        // default rate: 1000 events/sec
        Generator { timer, endpoint: (IpAddress::v4(0, 0, 0, 0), PORT).into(),
                    mcpd_id: 0, run_id: 0, interval: 10_000, pkt_interval: 100_000,
                    buf_no: 0, time: 0, lastpkt: 0, run: false,
                    npkt: 0, nevt: 0, npkt_per_size: [0; 221] }
    }

    fn process_command(&mut self, sock: &mut UdpSocket, msg: &[u8], ep: IpEndpoint) {
        // parse body: all command buffers end in 0xffff
        let req_body = msg[20..].chunks(2).map(|c| LE::read_u16(c))
                                          .take_while(|&v| v != 0xffff)
                                          .collect::<ArrayVec<[u16; 32]>>();
        let mut body = ArrayVec::<[u16; 24]>::new();
        let cmd = LE::read_u16(&msg[8..]);
        match cmd {
            22 => { // get MCPD-8 capabilities and mode
                info!("Get capabilities: {:?}", req_body);
                body.push(7); body.push(2);  // TOF + Pos/Amp selected
            }
            23 => { // set MCPD-8 mode
                info!("Set bus mode: {:?}", req_body);
                body.push(2);  // TOF + Pos/Amp selected
            }
            31 => { // write MCPD register
                info!("Write MCPD register: {:?}", req_body);
                if req_body[0] == 1 && req_body[1] == 103 {
                    body.push(1); body.push(103); body.push(2);
                }
            }
            32 => { // read MCPD register
                info!("Read MCPD register: {:?}", req_body);
                if req_body[0] == 1 && req_body[1] == 102 {
                    body.push(1); body.push(102); body.push(2);
                } else if req_body[0] == 1 && req_body[1] == 103 {
                    body.push(1); body.push(102); body.push(2);
                }
            }
            36 => { // get hw_types
                info!("Get hardware types");
                for _ in 0..8 { body.push(103); }
            }
            51 => { // get version information
                info!("Get version information");
                body.push(3); body.push(4); body.push(0x0102);
            }
            52 => { // read MPSD-8 register
                info!("Read MPSD register: {:?}", req_body);
                body.push(req_body[0]);
                body.push(req_body[1]);
                body.push(match req_body[1] {
                    0 => 7,
                    1 => 2,
                    2 => 0x0101,
                    _ => 0,
                });
            }
            0 | 2 => { // reset or stop DAQ
                info!("Stopped");
                self.stop();
            }
            1 | 3 => { // start or continue DAQ
                info!("Started");
                if self.endpoint.addr.is_unspecified() {
                    // to avoid needing reconfiguration of the IP all the time
                    self.endpoint = ep;
                }
                self.start();
            }
            4 => { // set MCPD id
                info!("MCPD id set to {}", req_body[0]);
            }
            5 => { // set protocol parameters
                info!("Set protocol parameters");
                if req_body[0] != 0 {
                    info!("  requested IP change to {}.{}.{}.{}",
                          req_body[0], req_body[1], req_body[2], req_body[3]);
                }
                if req_body[4] != 0 {
                    let port = if req_body[9] != 0 { req_body[9] } else { 54321 };
                    info!("  events should go to {}.{}.{}.{}:{}",
                          req_body[4], req_body[5], req_body[6], req_body[7], port);
                    self.endpoint = (IpAddress::v4(req_body[4] as u8,
                                                   req_body[5] as u8,
                                                   req_body[6] as u8,
                                                   req_body[7] as u8), port).into();
                }
                if req_body[4] == 0 && req_body[5] == 0 {
                    info!("  events should go to sender");
                    self.endpoint = ep;
                }
                if req_body[10] != 0 {
                    info!("  accept commands only from {}.{}.{}.{}:{}",
                          req_body[10], req_body[11], req_body[12], req_body[13],
                          if req_body[8] != 0 { req_body[8] } else { 54321 });
                }
                if req_body[10] == 0 && req_body[11] == 0 {
                    info!("  accept commands only from sender");
                }
            }
            6 => { // set timing setup
                info!("Set timing setup: {:?}", req_body);
            }
            7 => { // set master clock value
                info!("Master clock is {}",
                      req_body[0] as u64 | (req_body[1] as u64) << 16 | (req_body[2] as u64) << 32);
            }
            8 => { // set run ID
                self.run_id = req_body[0];
                info!("Run ID set to {}", self.run_id);
            }
            9 => { // set counter cells
                info!("Set counter cells: {:?}", req_body);
            }
            10 => { // set aux timer
                info!("Set aux timer: {:?}", req_body);
            }
            11 => { // set parameter source
                info!("Set param source: {:?}", req_body);
            }
            12 => { // get all parameters
                info!("CMD 12 not implemented, returning zeros");
                for _ in 0..21 {
                    body.push(0);
                }
            }
            0xF1F0 => { // generator parameters -- generator specific command
                let rate = (req_body[1] as u32) << 16 | req_body[0] as u32;
                self.interval = 10_000_000 / rate;
                let minpkt = (req_body[2] as u32).min(200);
                self.pkt_interval = self.interval * minpkt;
                info!("Configure: set rate to {}/s, min events/packet to {}",
                      10_000_000 / self.interval, minpkt);
            }
            _ => { // unknown
                info!("CMD {} not handled...", cmd);
                return;
            }
        }
        body.push(0xffff);

        // send back reply
        if let Ok(buf) = sock.send(20 + 2*body.len(), ep) {
            LE::write_u16(&mut buf[0..], (10 + body.len()) as u16);
            buf[2..12].copy_from_slice(&msg[2..12]);
            LE::write_u16(&mut buf[12..], self.time as u16);
            LE::write_u16(&mut buf[14..], (self.time >> 16) as u16);
            LE::write_u16(&mut buf[16..], (self.time >> 32) as u16);
            buf[18..20].copy_from_slice(&[0, 0]);
            for (i, val) in body.into_iter().enumerate() {
                LE::write_u16(&mut buf[20+i*2..22+i*2], val);
            }
            let cksum = buf.chunks(2).fold((0, 0), |acc, item| (acc.0 ^ item[0],
                                                                acc.1 ^ item[1]));
            buf[18] = cksum.0;
            buf[19] = cksum.1;
        }
    }

    fn maybe_send_data(&mut self, sock: &mut UdpSocket) {
        if !self.run {
            return;
        }

        // keep track of 64-bit time
        let low_time = self.timer.cnt.read().bits();
        let overflow = if low_time < self.time as u32 { 1 << 32 } else { 0 };
        self.time = ((self.time & 0xFFFFFFFF00000000) + overflow) | low_time as u64;

        // calculate time since last packet
        let elapsed = (self.time - self.lastpkt) as u32;
        if elapsed < self.pkt_interval {
            return;
        }
        let mut nevents = (elapsed / self.interval) as usize;
        if nevents > 220 {
            info!("too many events for single packet, limiting...");
            nevents = 220;
        }

        // now we can send a packet
        match sock.send(42 + 6*nevents, self.endpoint) {
            Ok(buf) => {
                self.buf_no += 1;
                LE::write_u16(&mut buf[0..], 21 + 3*nevents as u16);
                LE::write_u16(&mut buf[2..], 0);
                LE::write_u16(&mut buf[4..], 21);
                LE::write_u16(&mut buf[6..], self.buf_no);
                LE::write_u16(&mut buf[8..], self.run_id);
                buf[10] = 1;  // DAQ running
                buf[11] = self.mcpd_id;
                LE::write_u16(&mut buf[12..], self.time as u16);
                LE::write_u16(&mut buf[14..], (self.time >> 16) as u16);
                LE::write_u16(&mut buf[16..], (self.time >> 32) as u16);
                buf[18..42].copy_from_slice(&[0; 24]);
                for n in 0..nevents {
                    let (y, x) = loop {
                        let random = read_rand();
                        let y = (random >> 22) as u16;
                        if y < 960 {
                            break (y, random as u16 & 0b11100111);
                        }
                    };
                    LE::write_u16(&mut buf[42+6*n+2..], y << 3);
                    LE::write_u16(&mut buf[42+6*n+4..], x << 7);
                    self.nevt += 1;
                }
                self.lastpkt = self.time - (elapsed % self.interval) as u64;
                self.npkt += 1;
                self.npkt_per_size[nevents as usize] += 1;
            }
            Err(e) => warn!("send: {}", e),
        }
    }

    fn start(&mut self) {
        self.run = true;
        self.time = 0;
        self.lastpkt = 0;
        self.nevt = 0;
        self.npkt = 0;
        self.npkt_per_size.iter_mut().for_each(|p| *p = 0);
        // LED blue is "running"
        set_leds(false, true, true);
        // reset the timer
        self.timer.cnt.write(|w| unsafe { w.bits(0) });
        self.timer.cr1.write(|w| w.cen().set_bit());
    }

    fn stop(&mut self) {
        self.run = false;
        self.timer.cr1.write(|w| w.cen().clear_bit());
        set_leds(false, true, false);
        info!("generated {} pkts, {} evts in {} ticks", self.npkt, self.nevt, self.time);
        info!("event rate: {:.0}/s", self.nevt as f64 / (self.time as f64 / 10_000_000.));
        info!("per size:");
        for (sz, &n) in self.npkt_per_size.iter().enumerate() {
            if n != 0 {
                info!("{} - {}", sz, n);
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
    // systick is used for advancing the Ethernet clock for timeouts etc.
    syst.set_reload(SYST::get_ticks_per_10ms() / 10);
    syst.enable_counter();
    syst.enable_interrupt();
}

fn setup_gpio(p: &Peripherals) {
    // Set up button and LEDs, clocks already enabled by ethernet
    p.GPIOC.moder.modify(|_, w| unsafe { w.moder13().bits(0b00) });
    p.GPIOC.pupdr.modify(|_, w| unsafe { w.pupdr13().bits(0b10) });
    p.GPIOB.moder.modify(|_, w| unsafe { w.moder0().bits(0b01).moder7().bits(0b01).moder14().bits(0b01) });
}

fn set_leds(red: bool, green: bool, blue: bool) {
    let gpiob = unsafe { &(*board::GPIOB::ptr()) };
    gpiob.odr.modify(|_, w| w.odr0().bit(green).odr7().bit(blue).odr14().bit(red));
}

fn setup_rng(p: &Peripherals) {
    p.RCC.ahb2enr.modify(|_, w| w.rngen().set_bit());
    p.RNG.cr.modify(|_, w| w.rngen().set_bit());
}

fn setup_10mhz(p: &Peripherals) {
    p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
    p.TIM2.psc.write(|w| unsafe { w.psc().bits(8) }); // 90 MHz/9
    p.TIM2.egr.write(|w| w.ug().set_bit());
}

fn read_serno() -> u32 {
    unsafe {
        *(0x1FFF_7A10 as *const u32) ^
        *(0x1FFF_7A14 as *const u32) ^
        *(0x1FFF_7A18 as *const u32)
    }
}

fn read_rand() -> u32 {
    // read a random number from the hardware generator
    unsafe {
        (*board::RNG::ptr()).dr.read().bits()
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
