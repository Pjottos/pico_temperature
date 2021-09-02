#![no_std]
#![no_main]
#![feature(asm)]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::{
    fixed_point::FixedPoint,
    rate::Extensions,
};


use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    gpio::FunctionI2C,
    i2c::I2C,
};

use core::panic::PanicInfo;

const SYS_HZ: u32 = 125_000_000;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 0x100] = rp2040_boot2::BOOT_LOADER;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();

    let sda_pin = pins.gpio0.into_mode::<FunctionI2C>();
    let scl_pin = pins.gpio1.into_mode::<FunctionI2C>();

    let mut i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        SYS_HZ.Hz(),
    );

    loop {
        delay.delay_ms(5000);
        am2320::measure(&mut i2c).ok().unwrap();

        led_pin.set_high().unwrap();
    }
}

#[panic_handler]
fn handle_panic(_info: &PanicInfo) -> ! {
    // halt, but don't put the cpu under full load
    loop {
        unsafe {
            asm!("wfe");
        }
    }
}
