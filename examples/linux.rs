use embedded_hal::blocking::delay::DelayMs;
use linux_embedded_hal::{Delay, I2cdev};
use sdp800::Sdp800;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let address = 0x58;
    let mut sdp = Sdp800::new(dev, address, Delay);

    println!("Starting Sdp800 tests.");
}
