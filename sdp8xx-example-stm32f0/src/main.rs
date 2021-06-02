#![no_std]
#![no_main]

use cortex_m::Peripherals;
use panic_halt as _;

use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};

use crate::hal::{delay::Delay, i2c::I2c, pac, prelude::*};
use sdp8xx::*;
use stm32f0xx_hal as hal;

#[entry]
fn main() -> ! {
    rtt_init_print!();

    if let (Some(p), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) {
        let mut flash = p.FLASH;
        let mut rcc = p.RCC.configure().freeze(&mut flash);

        let mut delay = Delay::new(cp.SYST, &rcc);

        let gpiob = p.GPIOB.split(&mut rcc);

        // Configure pins for I2C
        let (scl, sda) = cortex_m::interrupt::free(move |cs| {
            (
                gpiob.pb8.into_alternate_af1(cs),
                gpiob.pb9.into_alternate_af1(cs),
            )
        });

        // Configure I2C with 100kHz rate
        let i2c = I2c::i2c1(p.I2C1, (scl, sda), 100.khz(), &mut rcc);

        let mut sdp8xx = Sdp8xx::new(i2c, 0x25, delay.clone());

        if let Ok(product_id) = sdp8xx.read_product_id() {
            rprintln!("Product ID: {:x?}", product_id);
        } else {
            rprintln!("Failed to read product ID!");
        }
        delay.delay_ms(1000u32);

        rprintln!("Sending sensor to sleep.");
        let woken_up = if let Ok(sleeping) = sdp8xx.go_to_sleep() {
            delay.delay_ms(1000u16);
            sleeping.wake_up()
        } else {
            rprintln!("Failed to send sensor to sleep!");
            loop {}
        };
        let mut sdp8xx = match woken_up {
            Ok(s) => s,
            Err(_) => {
                rprintln!("Failed to wake up sensor!");
                loop {}
            }
        };
        rprintln!("Woke up sensor!");

        rprintln!("Taking 10 triggered samples");
        for _ in 0..=10 {
            if let Ok(m) = sdp8xx.trigger_differential_pressure_sample() {
                rprintln!("{:?}", m);
            } else {
                rprintln!("Error!");
            }
            delay.delay_ms(1000u32);
        }

        let mut sdp_sampling = match sdp8xx.start_sampling_differential_pressure(true) {
            Ok(s) => s,
            Err(_) => {
                rprintln!("Could not start sampling, going to infinite loop.");
                loop {}
            }
        };
        for _ in 0..=50 {
            delay.delay_ms(100u16);
            let result = sdp_sampling.read_continuous_sample();
            match result {
                Ok(r) => rprintln!("{:?}", r),
                Err(_) => rprintln!("Error while getting result."),
            }
        }
        let mut idle_sensor = {
            match sdp_sampling.stop_sampling() {
                Ok(s) => s,
                Err(_) => {
                    rprintln!("Could not stop sampling. Infinite loop ensues.");
                    loop {}
                }
            }
        };
        loop {
            if let Ok(m) = idle_sensor.trigger_differential_pressure_sample() {
                rprintln!("{:?}", m);
            } else {
                rprintln!("Error!");
            }
            delay.delay_ms(1000u32);
        }
    }
    loop {}
}
