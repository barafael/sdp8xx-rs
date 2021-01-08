use embedded_hal::blocking::delay::DelayMs;
use linux_embedded_hal::{Delay, I2cdev};
use sdp8xx::Sdp8xx;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let address = 0x58;
    let mut sdp = Sdp8xx::new(dev, address, Delay);

    println!("Starting Sdp8xx tests.");
}
