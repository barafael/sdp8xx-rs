//! A platform agnostic Rust driver for the Sensirion SDP800 differential pressure sensor, based
//! on the [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.
//! Heavily inspired by the [`sgp30 driver by Danilo Bergen`](https://github.com/dbrgn/sgp30-rs)
//!
//! ## The Device
//!
//! The Sensirion SDP800 is a differential pressure sensor. It has an I2C interface.
//!
//! - [Datasheet](https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/8_Differential_Pressure/Datasheets/Sensirion_Differential_Pressure_Sensors_SDP8xx_Digital_Datasheet.pdf)
//! - [Product Page](https://www.sensirion.com/en/flow-sensors/differential-pressure-sensors/sdp800-proven-and-improved/)
//!
//! ## Usage
//!
//! ### Instantiating
//!
//! Import this crate and an `embedded_hal` implementation, then instantiate
//! the device:
//!
//! ```no_run
//! use linux_embedded_hal as hal;
//!
//! use hal::{Delay, I2cdev};
//! use sdp800::Sdp800;
//!
//! # fn main() {
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let address = 0x25;
//! let mut sdp = Sdp800::new(dev, address, Delay);
//! # }
//! ```
//!
//! ### Fetching Device Information
//!
//! You can fetch the product id of your sensor:
//!
//! ```no_run
//! use linux_embedded_hal as hal;
//! use hal::{Delay, I2cdev};
//! use sdp800::ProductIdentifier;
//! use sdp800::Sdp800;
//!
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let mut sdp = Sdp800::new(dev, 0x25, Delay);
//! let product_id: ProductIdentifier = sdp.read_product_id().unwrap();
//!
//! ```
//!
//! The SDP800 uses a dynamic baseline compensation algorithm and on-chip
//! calibration parameters to provide two complementary air quality signals.
//! Calling this method starts the air quality measurement. **After
//! initializing the measurement, the `measure()` method must be called in
//! regular intervals of 1 second** to ensure proper operation of the dynamic
//! baseline compensation algorithm. It is the responsibility of the user of
//! this driver to ensure that these periodic measurements are being done!
//!

#![deny(unsafe_code)]
#![deny(missing_docs)]
#![cfg_attr(not(test), no_std)]

use core::convert::TryFrom;

use embedded_hal as hal;
use i2c::read_words_with_crc;

use crate::hal::blocking::delay::{DelayMs, DelayUs};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use sensirion_i2c::{crc8, i2c};

use crc8::validate;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),
    /// CRC checksum validation failed
    Crc,
}

impl<E, I2cWrite, I2cRead> From<i2c::Error<I2cWrite, I2cRead>> for Error<E>
where
    I2cWrite: Write<Error = E>,
    I2cRead: Read<Error = E>,
{
    fn from(err: i2c::Error<I2cWrite, I2cRead>) -> Self {
        match err {
            i2c::Error::Crc => Error::Crc,
            i2c::Error::I2cWrite(e) => Error::I2c(e),
            i2c::Error::I2cRead(e) => Error::I2c(e),
        }
    }
}

/// I2C commands sent to the sensor.
#[derive(Debug, Copy, Clone)]
enum Command {
    /// General Call Reset
    GeneralCallReset,
    /// Read product identifier 1
    ReadProductId0,
    /// Read product identifier 2
    ReadProductId1,
    /// Enter sleep mode
    EnterSleepMode,
    /// Trigger Mass Flow Reading with no clock stretching
    TriggerMassFlowRead,
    /// Trigger Mass Flow Reading with clock stretching
    TriggerMassFlowReadSync,
    /// Trigger Differential Pressure Reading with no clock stretching
    TriggerDifferentialPressureRead,
    /// Trigger Differential Pressure Reading with clock stretching
    TriggerDifferentialPressureReadSync,
    /// Stop continuous measurement
    StopContinuousMeasurement,
}

impl Command {
    fn as_bytes(self) -> [u8; 2] {
        match self {
            Command::ReadProductId0 => [0x36, 0x7C],
            Command::ReadProductId1 => [0xE1, 0x02],
            Command::TriggerMassFlowRead => [0x36, 0x24],
            Command::TriggerMassFlowReadSync => [0x37, 0x26],
            Command::TriggerDifferentialPressureRead => [0x36, 0x2F],
            Command::TriggerDifferentialPressureReadSync => [0x37, 0x2D],
            Command::GeneralCallReset => [0x00, 0x06],
            Command::EnterSleepMode => [0x36, 0x77],
            Command::StopContinuousMeasurement => [0x3F, 0xF9],
        }
    }
}

/// Driver for the SDP800
#[derive(Debug, Default)]
pub struct Sdp800<I2C, D> {
    /// The concrete I2C device implementation.
    i2c: I2C,
    /// The I2C device address.
    address: u8,
    /// The concrete Delay implementation.
    delay: D,
}

impl<I2C, D, E> Sdp800<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayUs<u16> + DelayMs<u16>,
{
    /// Create a new instance of the SDP800 driver.
    pub fn new(i2c: I2C, address: u8, delay: D) -> Self {
        Sdp800 {
            i2c,
            address,
            delay,
        }
    }

    /// Destroy driver instance, return I2C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Write an I2C command to the sensor.
    fn send_command(&mut self, command: Command) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &command.as_bytes())
            .map_err(Error::I2c)
    }

    /// Write an I2C command and data to the sensor.
    ///
    /// The data slice must have a length of 2 or 4.
    ///
    /// CRC checksums will automatically be added to the data.
    fn send_command_and_data(&mut self, command: Command, data: &[u8]) -> Result<(), Error<E>> {
        assert!(data.len() == 2 || data.len() == 4);
        let mut buf = [0; 2 /* command */ + 6 /* max length of data + crc */];
        buf[0..2].copy_from_slice(&command.as_bytes());
        buf[2..4].copy_from_slice(&data[0..2]);
        buf[4] = crc8::calculate(&data[0..2]);
        if data.len() > 2 {
            buf[5..7].copy_from_slice(&data[2..4]);
            buf[7] = crc8::calculate(&data[2..4]);
        }
        let payload = if data.len() > 2 {
            &buf[0..8]
        } else {
            &buf[0..5]
        };
        self.i2c.write(self.address, payload).map_err(Error::I2c)
    }

    /// Return the product id of the SDP800
    pub fn read_product_id(&mut self) -> Result<ProductIdentifier, Error<E>> {
        let mut buf = [0; 18];
        // Request product id
        self.send_command(Command::ReadProductId0)?;
        self.send_command(Command::ReadProductId1)?;

        self.i2c.read(self.address, &mut buf).map_err(Error::I2c)?;

        ProductIdentifier::try_from(buf).map_err(|_| Error::Crc)
    }

    /// Trigger a differential pressure read without clock stretching.
    /// This function blocks for at least 60 milliseconds to await a result.
    pub fn read_sample_triggered(&mut self) -> Result<Measurement, Error<E>> {
        let mut buffer = [0; 9];

        self.send_command(Command::TriggerDifferentialPressureRead)?;

        self.delay.delay_ms(60);

        read_words_with_crc(&mut self.i2c, self.address, &mut buffer)?;

        Measurement::try_from(buffer).map_err(|_| Error::Crc)
    }
}

/// Product Identifier as described in the datasheet
#[derive(Debug)]
pub struct ProductIdentifier {
    serial_number: u64,
    product_number: u32,
}

impl TryFrom<[u8; 18]> for ProductIdentifier {
    type Error = Error<()>;

    fn try_from(buf: [u8; 18]) -> Result<Self, Self::Error> {
        if validate(&buf).is_err() {
            return Err(Error::Crc);
        }

        let product_number: u32 = (buf[0] as u32) << 24
            | (buf[1] as u32) << 16
            | (buf[3] as u32) << 8
            | (buf[4] as u32) << 0;

        let serial_number: u64 = (buf[6] as u64) << 56
            | (buf[7] as u64) << 48
            | (buf[9] as u64) << 40
            | (buf[10] as u64) << 32
            | (buf[12] as u64) << 24
            | (buf[13] as u64) << 16
            | (buf[15] as u64) << 8
            | (buf[16] as u64) << 0;

        Ok(ProductIdentifier {
            serial_number,
            product_number,
        })
    }
}

/// A measurement result from the sensor.
#[derive(Debug, PartialEq, Clone)]
pub struct Measurement {
    /// Pressure in Pa
    pub differential_pressure: f32,
    /// Temperature reading
    pub temperature: f32,
}

impl TryFrom<[u8; 9]> for Measurement {
    type Error = Error<()>;

    fn try_from(buffer: [u8; 9]) -> Result<Self, Self::Error> {
        if validate(&buffer).is_err() {
            return Err(Error::Crc);
        }

        let dp_raw: i16 = (buffer[0] as i16) << 8 | buffer[1] as i16;
        let temp_raw: i16 = (buffer[3] as i16) << 8 | buffer[4] as i16;
        let dp_scale: i16 = (buffer[6] as i16) << 8 | buffer[7] as i16;

        let differential_pressure = dp_raw as f32 / dp_scale as f32;
        let temperature = temp_raw as f32 / 200.0f32;

        Ok(Measurement {
            differential_pressure,
            temperature,
        })
    }
}

/// Product variant as listed in the datasheet
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum ProductVariant {
    /// SDP800 500 Pascal range with manifold connection, I2C address 0x25
    Sdp800_500Pa,
    /// SDP810 500 Pascal range with tube connection, I2C address 0x25
    Sdp810_500Pa,
    /// SDP801 500 Pascal range with manifold connection, I2C address 0x26
    Sdp801_500Pa,
    /// SDP811 500 Pascal range with tube connection, I2C address 0x26
    Sdp811_500Pa,
    /// SDP800 125 Pascal range with manifold connection, I2C address 0x25
    Sdp800_125Pa,
    /// SDP810 125 Pascal range with tube connection, I2C address 0x25
    Sdp810_125Pa,
}

impl ProductVariant {
    /// Parse the product variant. The last byte is the revision number and might change.
    pub fn parse(val: [u8; 4]) -> Option<(Self, u8)> {
        match val {
            [0x03, 0x02, 0x01, n] => Some((ProductVariant::Sdp800_500Pa, n)),
            [0x03, 0x02, 0x0A, n] => Some((ProductVariant::Sdp810_500Pa, n)),
            [0x03, 0x02, 0x04, n] => Some((ProductVariant::Sdp801_500Pa, n)),
            [0x03, 0x02, 0x0D, n] => Some((ProductVariant::Sdp811_500Pa, n)),
            [0x03, 0x02, 0x02, n] => Some((ProductVariant::Sdp800_125Pa, n)),
            [0x03, 0x02, 0x0B, n] => Some((ProductVariant::Sdp810_125Pa, n)),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal_mock as hal;

    use self::hal::delay::MockNoop as DelayMock;
    use self::hal::i2c::{Mock as I2cMock, Transaction};
    use super::*;

    /// Test the `product_id` function
    #[test]
    fn test_product_id() {
        let expectations = [
            Transaction::write(0x25, Command::ReadProductId0.as_bytes()[..].into()),
            Transaction::write(0x25, Command::ReadProductId1.as_bytes()[..].into()),
            Transaction::read(0x25, vec![0x00, 0x11, 0xF3, 0x22, 0x33, 0x12, 0x44, 0x55, 0x00, 0x66, 0x77, 0xE1, 0x88, 0x99, 0x24, 0xaa, 0xbb, 0xC5]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sdp = Sdp800::new(mock, 0x25, DelayMock);
        let id = sdp.read_product_id().unwrap();
        assert_eq!(0x00112233, id.product_number);
        assert_eq!(0x445566778899aabb, id.serial_number);
        sdp.destroy().done();
    }
}
