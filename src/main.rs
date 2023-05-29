//! SPI slave mode test
//!
//! spi1 master <-> spi2 slave
//! PA5 <-SCK-> PB13
//! PA6 <-MISO-> PB14
//! PA7 <-MOSI-> PB15

#![allow(clippy::empty_loop)]
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nb::Error;
use panic_halt as _;

use cortex_m::{asm, singleton};
use embedded_hal::spi::{Mode as SpiMode, Phase, Polarity};
pub const SPI_MODE: SpiMode = SpiMode {
    phase: Phase::CaptureOnFirstTransition,
    polarity: Polarity::IdleLow,
};

use stm32f1xx_hal::{
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Alternate, Floating, Input, PushPull,
    },
    pac::{self, interrupt, Peripherals, SPI2},
    prelude::*,
    spi::{Event, Slave, Spi, Spi2NoRemap, NoMiso},
    i2c::{BlockingI2c, DutyCycle, Mode},
};

type SlaveSpi = Spi<
    SPI2,
    Spi2NoRemap,
    (
        PB13<Input<Floating>>,
        PB14<Alternate<PushPull>>,
        PB15<Input<Floating>>,
    ),
    u8,
    Slave,
>;

static mut SPI2SLAVE: Option<SlaveSpi> = None;

#[entry]
fn main() -> ! {
    // Let's get the different variables needed to make the STM32 work
    let dp = Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain();

    // Setup the clock via external oscillator
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze(&mut flash.acr);
    let mut delay = dp.TIM2.delay_us(&clocks);

    // Get the differents pins
    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();
    
    // Disable JTAG, to access PB3 and PB4
    let pa15 = gpioa.pa15;
    let pb3 = gpiob.pb3;
    let pb4= gpiob.pb4;
    let (_, pb3, pb4) = afio.mapr.disable_jtag(pa15, pb3, pb4);


    // Use SPI2 as slave
    let nss = gpioa.pa4.into_pull_up_input(&mut gpioa.crl);
    let sck = gpioa.pa5;
    let miso = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);
    let mosi = gpioa.pa7;


    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let mut i2c: BlockingI2c<pac::I2C1, (stm32f1xx_hal::gpio::Pin<'B', 6, Alternate<stm32f1xx_hal::gpio::OpenDrain>>, stm32f1xx_hal::gpio::Pin<'B', 7, Alternate<stm32f1xx_hal::gpio::OpenDrain>>)> = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio16to9,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    //let spi2 = Spi::spi2_slave(dp.SPI2, (sck, miso, mosi), MODE);
    //let spi2 = Spi::spi1_slave(dp.SPI1, (sck, miso, mosi), MODE);

    // Set up the DMA device
    let dma = dp.DMA1.split();
    //let spi2_dma = spi2.with_rx_dma(dma.2);

    // test SPI with interrupts
    //let (mut spi2, _) = spi2_dma.release();


    ////spi2.listen(Event::Rxne);
    ////spi2.listen(Event::Txe);
    ////spi2.listen(Event::Error);

    ////cortex_m::interrupt::free(|_| unsafe {
    ////    SPI2SLAVE.replace(spi2);
    ////});

    ////unsafe {
    ////    cortex_m::peripheral::NVIC::unmask(pac::Interrupt::SPI2);
    ////}

    //let master_transfer = spi1_dma.read_write(buffer.0, buffer.1);
    //let (_buffer, _spi1_dma) = master_transfer.wait();

    //let mut led = gpiob.pb5.into_push_pull_output(&mut gpiob.crl);
    //led.set_low();
    
    // ================================
    // ====== TPS65185 PMIC
    let mut tps_wake_up = gpiob.pb11.into_push_pull_output(&mut gpiob.crh);
    let mut tps_power_up = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    let mut tps_vcom_ctrl = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    let tps_pwrgood = gpiob.pb14.into_floating_input(&mut gpiob.crh);

    // Activate the I2C communication to configure the TPS
    let tps_address = 0x68;
    tps_wake_up.set_high();
    delay.delay_ms(50_u32);  // Give time to the TPS to wakeup
    
    // Enable 3.3V
    let mut configured = match i2c.write(tps_address, &[0x01, 0x20]) { Ok(_) => true, Err(_) => false };

    // Enable VCOM (-1.27V => 0x7F on VCOM1, 0 on VCOM2)
    configured = match i2c.write(tps_address, &[0x03, 0x7F]) { Ok(_) => configured, Err(_) => false};
    configured = match i2c.write(tps_address, &[0x04, 0x00]) { Ok(_) => configured, Err(_) => false};
    tps_vcom_ctrl.set_high();

    // Start powering up the rails if the configuration was successful
    if configured {
        tps_power_up.set_high();
    }
    else { loop {} }
    
    // Check from the TPS that all the rails are OK
    if !tps_pwrgood.is_high() { loop {} }


    // ================================
    // ====== EPD SETUP
    let mut epd_oe = gpioc.pc13.into_alternate_push_pull(&mut gpioc.crh);
    let mut epd_gmode = gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh);
    let mut epd_spv = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);
    let mut epd_ckv = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let mut epd_clk = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let mut epd_sph = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let mut epd_le = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let mut epd_d0 = gpioc.pc14.into_alternate_push_pull(&mut gpioc.crh);
    let mut epd_d1 = gpioc.pc15.into_alternate_push_pull(&mut gpioc.crh);
    let mut epd_d2 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    let mut epd_d3 = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);
    let mut epd_d4 = gpiob.pb2.into_alternate_push_pull(&mut gpiob.crl);
    let mut epd_d5 = pb3.into_alternate_push_pull(&mut gpiob.crl);
    let mut epd_d6 = pb4.into_alternate_push_pull(&mut gpiob.crl);
    let mut epd_d7 = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);


    const buffer_len: usize = 8;
    let mut buffer_index: usize = 0;
    let mut read_buffer = [0; buffer_len];

    loop {
        //match nb::block!(spi2.read())
        //{
        //    Err(_) => {},
        //    Ok(data) => {
        //        // Only take the data into account if NSS is active (low)
        //        if nss.is_low() {
        //            read_buffer[buffer_index] = data;
        //            buffer_index += 1;


        //            //if buffer_index == 8 && (&read_buffer == b"turn_on!") { 
        //            //    led.set_high();
        //            //}
        //            //else if buffer_index == 8 && (&read_buffer == b"turn_off") { 
        //            //    led.set_low();
        //            //}

        //            //if buffer_index >= buffer_len {
        //            //    buffer_index = 0;
        //            //}
        //        }
        //    }
        //}
    }
}

const R_BUFFER_LEN: usize = 16;
static mut R_BUFFER: &mut [u8; R_BUFFER_LEN] = &mut [0; R_BUFFER_LEN];
static mut RIDX: usize = 0;

const W_BUFFER_LEN: usize = 3;
static W_BUFFER: &[u8; W_BUFFER_LEN] = &[1, 2, 3];
static mut WIDX: usize = 0;

#[interrupt]
unsafe fn SPI2() {
    cortex_m::interrupt::free(|_| {
        if let Some(spi2) = SPI2SLAVE.as_mut() {
            if spi2.is_overrun() {
                // mcu processing speed is not enough
                //asm::bkpt();
            }
            if spi2.is_rx_not_empty() {
                if let Ok(w) = nb::block!(spi2.read()) {
                    R_BUFFER[RIDX] = w;
                    RIDX += 1;
                    if RIDX >= R_BUFFER_LEN - 1 {
                        RIDX = 0;
                    }
                }
            }
            if spi2.is_tx_empty() {
                if let Ok(()) = nb::block!(spi2.send(W_BUFFER[WIDX])) {
                    WIDX += 1;
                    if WIDX >= W_BUFFER_LEN {
                        WIDX = 0;
                    }
                }
            }
        }
    })
}