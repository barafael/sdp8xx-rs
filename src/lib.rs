//! A platform agnostic Rust driver for the Sensirion SDP8xx differential pressure sensor, based
//! on the [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.
//! Heavily inspired by the [`sgp30 driver by Danilo Bergen`](https://github.com/dbrgn/sgp30-rs)
//!
//! ## The Device
//!
//! The Sensirion SDP8xx is a differential pressure sensor. It has an I2C interface.
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
//! use sdp8xx::Sdp8xx;
//!
//! # fn main() {
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let address = 0x25;
//! let mut sdp = Sdp8xx::new(dev, address, Delay);
//! # }
//! ```
//!
//! See [sdp8xx-rpi-test](https://github.com/barafael/sdp8xx-rpi-test) for an example on the raspberry pi.
//!
//! ### Fetching Device Information
//!
//! You can fetch the product id of your sensor:
//!
//! ```no_run
//! use linux_embedded_hal as hal;
//! use hal::{Delay, I2cdev};
//! use sdp8xx::ProductIdentifier;
//! use sdp8xx::Sdp8xx;
//!
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let mut sdp = Sdp8xx::new(dev, 0x25, Delay);
//! if let Ok(product_id) = sdp.read_product_id() {
//!     println!("{:?}", product_id);
//! } else {
//!     eprintln!("Error during reading product ID.");
//! }
//! ```

#![deny(unsafe_code)]
#![deny(missing_docs)]
#![cfg_attr(not(test), no_std)]

use core::convert::TryFrom;
use core::marker::PhantomData;

pub mod product_info;
pub use crate::product_info::*;

pub mod command;
use crate::command::Command;

pub mod sample;
pub use crate::sample::*;

use embedded_hal as hal;

use crate::hal::blocking::delay::{DelayMs, DelayUs};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use sensirion_i2c::i2c_buffer::I2cBuffer;

pub mod states;
use crate::states::*;

/// All possible errors in this crate
#[derive(Debug, PartialEq)]
pub enum SdpError<I2cWrite: Write, I2cRead: Read> {
    /// I2C write error
    I2cWrite(I2cWrite::Error),
    /// I2C read error
    I2cRead(I2cRead::Error),
    /// CRC checksum validation failed
    CrcError,
    /// Wrong Buffer Size
    InvalidBufferSize,
    /// Invalid sensor variant
    InvalidVariant,
    /// Wake up sent while sensor was not in sleep state
    WakeUpWhileNotSleeping,
    /// Cannot wake up sensor
    CannotWakeUp,
    /// Invalid scale factor
    InvalidScaleFactor,
    /// Sampling error
    SampleError,
    /// Buffer too small for requested operation
    BufferTooSmall,
}

impl<I2cWrite: embedded_hal::blocking::i2c::Write, I2cRead: embedded_hal::blocking::i2c::Read>
    From<sensirion_i2c::i2c::Error<I2cWrite, I2cRead>> for SdpError<I2cWrite, I2cRead>
{
    fn from(error: sensirion_i2c::i2c::Error<I2cWrite, I2cRead>) -> Self {
        match error {
            sensirion_i2c::i2c::Error::I2cWrite(w) => SdpError::I2cWrite(w),
            sensirion_i2c::i2c::Error::I2cRead(r) => SdpError::I2cRead(r),
            sensirion_i2c::i2c::Error::CrcError => SdpError::CrcError,
            sensirion_i2c::i2c::Error::InvalidBufferSize => SdpError::InvalidBufferSize,
            sensirion_i2c::i2c::Error::InvalidArgument => SdpError::InvalidBufferSize,
            sensirion_i2c::i2c::Error::BufferTooSmall => SdpError::BufferTooSmall,
        }
    }
}

/// State of the SDP8xx
#[derive(Debug, Default)]
pub struct Sdp8xx<I2C, D, State> {
    /// The concrete I2C device implementation.
    i2c: I2C,
    /// The I2C device address.
    address: u8,
    /// The concrete Delay implementation.
    delay: D,
    /// The state of the sensor
    state: PhantomData<State>,
}

impl<I2C, D, E> Sdp8xx<I2C, D, IdleState>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayUs<u16> + DelayMs<u16>,
{
    /// Create a new instance of the SDP8xx driver.
    pub fn new(i2c: I2C, address: u8, delay: D) -> Self {
        // TODO try to communicate and get parameters from chip
        Sdp8xx {
            i2c,
            address,
            delay,
            state: PhantomData::<IdleState>,
        }
    }

    /// Destroy driver instance, return I2C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Write an I2C command to the sensor.
    fn send_command(&mut self, command: Command) -> Result<(), SdpError<I2C, I2C>> {
        let command: [u8; 2] = command.into();
        self.i2c
            .write(self.address, &command)
            .map_err(SdpError::I2cWrite)
    }

    /// Return the product id of the SDP8xx
    pub fn read_product_id(&mut self) -> Result<ProductIdentifier, SdpError<I2C, I2C>> {
        let mut buf = [0; 18];
        // Request product id
        self.send_command(Command::ReadProductId0)?;
        self.send_command(Command::ReadProductId1)?;

        self.i2c
            .read(self.address, &mut buf)
            .map_err(SdpError::I2cRead)?;

        ProductIdentifier::try_from(buf).map_err(|_| SdpError::CrcError)
    }

    /// Trigger a differential pressure read without clock stretching.
    /// This function blocks for at least 60 milliseconds to await a result.
    pub fn trigger_differential_pressure_sample(
        &mut self,
    ) -> Result<Sample<DifferentialPressure>, SdpError<I2C, I2C>> {
        self.send_command(Command::TriggerDifferentialPressureRead)?;

        self.delay.delay_ms(60);

        let mut i2c_buffer = I2cBuffer::<9>::new();

        i2c_buffer.read_and_validate::<I2C, I2C>(self.address, &mut self.i2c)?;

        match Sample::<DifferentialPressure>::try_from(i2c_buffer) {
            Ok(s) => Ok(s),
            Err(_) => Err(SdpError::SampleError),
        }
    }

    /// Trigger a mass flow read without clock stretching.
    /// This function blocks for at least 60 milliseconds to await a result.
    pub fn trigger_mass_flow_sample(&mut self) -> Result<Sample<MassFlow>, SdpError<I2C, I2C>> {
        self.send_command(Command::TriggerMassFlowRead)?;

        self.delay.delay_ms(60);

        let mut i2c_buffer = I2cBuffer::<9>::new();
        i2c_buffer.read_and_validate::<I2C, I2C>(self.address, &mut self.i2c)?;

        match Sample::<MassFlow>::try_from(i2c_buffer) {
            Ok(s) => Ok(s),
            Err(_) => Err(SdpError::SampleError),
        }
    }

    /// Start sampling in continuous mode
    pub fn start_sampling_differential_pressure(
        mut self,
        averaging: bool,
    ) -> Result<Sdp8xx<I2C, D, ContinuousSamplingState<DifferentialPressure>>, SdpError<I2C, I2C>>
    {
        let command = if averaging {
            Command::SampleDifferentialPressureAveraging
        } else {
            Command::SampleDifferentialPressureRaw
        };
        self.send_command(command)?;
        Ok(Sdp8xx {
            i2c: self.i2c,
            address: self.address,
            delay: self.delay,
            state: PhantomData::<ContinuousSamplingState<DifferentialPressure>>,
        })
    }

    /// Start sampling mass flow mode
    pub fn start_sampling_mass_flow(
        mut self,
        averaging: bool,
    ) -> Result<Sdp8xx<I2C, D, ContinuousSamplingState<MassFlow>>, SdpError<I2C, I2C>> {
        let command = if averaging {
            Command::SampleMassFlowAveraging
        } else {
            Command::SampleMassFlowRaw
        };
        self.send_command(command)?;
        Ok(Sdp8xx {
            i2c: self.i2c,
            address: self.address,
            delay: self.delay,
            state: PhantomData::<ContinuousSamplingState<MassFlow>>,
        })
    }

    /// Enter the SDP8xx sleep state
    pub fn go_to_sleep(mut self) -> Result<Sdp8xx<I2C, D, SleepState>, SdpError<I2C, I2C>> {
        self.send_command(Command::EnterSleepMode)?;
        Ok(Sdp8xx {
            i2c: self.i2c,
            address: self.address,
            delay: self.delay,
            state: PhantomData::<SleepState>,
        })
    }
}

impl<I2C, D, E> Sdp8xx<I2C, D, SleepState>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayUs<u16> + DelayMs<u16>,
{
    /// Wake the sensor up from the sleep state
    /// This function blocks for at least 2 milliseconds
    pub fn wake_up(mut self) -> Result<Sdp8xx<I2C, D, IdleState>, SdpError<I2C, I2C>> {
        // TODO polling with timeout.
        // Send wake up signal (not acked)
        // TODO this does not work currently on the hardware, though the unit tests are fine.
        let _ = self.i2c.write(self.address, &[]);
        self.delay.delay_ms(3);
        if self.i2c.write(self.address, &[]).is_ok() {
            return Err(SdpError::WakeUpWhileNotSleeping);
        }
        self.delay.delay_ms(3);
        match self.i2c.write(self.address, &[]) {
            Ok(_) => {}
            Err(_) => return Err(SdpError::CannotWakeUp),
        }
        Ok(Sdp8xx {
            i2c: self.i2c,
            address: self.address,
            delay: self.delay,
            state: PhantomData::<IdleState>,
        })
    }

    /// Wake the sensor up from the sleep state by polling the I2C bus
    /// This function blocks for at least 2 milliseconds
    pub fn wake_up_poll(mut self) -> Result<Sdp8xx<I2C, D, IdleState>, SdpError<I2C, I2C>> {
        // TODO timeout
        // Send wake up signal (not acked)
        // TODO this does not work currently on the hardware, though the unit tests are fine.
        if self.i2c.write(self.address, &[]).is_ok() {
            return Err(SdpError::WakeUpWhileNotSleeping);
        }
        loop {
            // timeout here
            if self.i2c.write(self.address, &[]).is_ok() {
                break;
            }
        }
        Ok(Sdp8xx {
            i2c: self.i2c,
            address: self.address,
            delay: self.delay,
            state: PhantomData::<IdleState>,
        })
    }
}

impl<I2C, D, E> Sdp8xx<I2C, D, ContinuousSamplingState<DifferentialPressure>>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayUs<u16> + DelayMs<u16>,
{
    /// Read a sample in continuous mode
    pub fn read_continuous_sample(&mut self) -> Result<Sample<MassFlow>, SdpError<I2C, I2C>> {
        let mut buffer = [0u8; 9];
        // TODO rate limiting no faster than 0.5ms
        self.i2c
            .read(self.address, &mut buffer)
            .map_err(SdpError::I2cRead)?;
        // TODO improve error handling
        match Sample::try_from(buffer) {
            Ok(s) => Ok(s),
            Err(_) => Err(SdpError::InvalidScaleFactor),
        }
    }

    /// Stop sampling continuous mode
    pub fn stop_sampling(mut self) -> Result<Sdp8xx<I2C, D, IdleState>, SdpError<I2C, I2C>> {
        let bytes: [u8; 2] = Command::StopContinuousMeasurement.into();
        self.i2c
            .write(self.address, &bytes)
            .map_err(SdpError::I2cWrite)?;
        Ok(Sdp8xx {
            i2c: self.i2c,
            address: self.address,
            delay: self.delay,
            state: PhantomData::<IdleState>,
        })
    }
}

impl<I2C, D, E> Sdp8xx<I2C, D, ContinuousSamplingState<MassFlow>>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayUs<u16> + DelayMs<u16>,
{
    /// Read a sample in continuous mode
    pub fn read_continuous_sample(&mut self) -> Result<Sample<MassFlow>, SdpError<I2C, I2C>> {
        let mut buffer = [0u8; 9];
        // TODO rate limiting no faster than 0.5ms
        self.i2c
            .read(self.address, &mut buffer)
            .map_err(SdpError::I2cRead)?;
        // TODO improve error handling
        match Sample::try_from(buffer) {
            Ok(s) => Ok(s),
            Err(_) => Err(SdpError::InvalidScaleFactor),
        }
    }

    /// Stop sampling continuous mode
    pub fn stop_sampling(mut self) -> Result<Sdp8xx<I2C, D, IdleState>, SdpError<I2C, I2C>> {
        let bytes: [u8; 2] = Command::StopContinuousMeasurement.into();
        self.i2c
            .write(self.address, &bytes)
            .map_err(SdpError::I2cWrite)?;
        Ok(Sdp8xx {
            i2c: self.i2c,
            address: self.address,
            delay: self.delay,
            state: PhantomData::<IdleState>,
        })
    }
}

#[cfg(test)]
mod tests {
    use hal::MockError;
    use std::io::ErrorKind;

    use embedded_hal_mock as hal;

    use self::hal::delay::MockNoop as DelayMock;
    use self::hal::i2c::{Mock as I2cMock, Transaction};
    use super::*;

    /// Test the `product_id` function
    #[test]
    fn product_id() {
        let data = vec![
            0x03, 0x02, 206, 0x02, 0x01, 105, 0x44, 0x55, 0x00, 0x66, 0x77, 225, 0x88, 0x99, 0x24,
            0xaa, 0xbb, 0xC5,
        ];
        let bytes_0: [u8; 2] = Command::ReadProductId0.into();
        let bytes_1: [u8; 2] = Command::ReadProductId1.into();
        let expectations = [
            Transaction::write(0x25, bytes_0.into()),
            Transaction::write(0x25, bytes_1.into()),
            Transaction::read(0x25, data.clone()),
        ];
        //println!("{:x}", crc8::calculate(&data[15..17]));
        let mock = I2cMock::new(&expectations);
        let mut sdp = Sdp8xx::new(mock, 0x25, DelayMock);
        let id = sdp.read_product_id().unwrap();
        assert_eq!(
            ProductVariant::Sdp800_125Pa { revision: 0x01 },
            id.product_number
        );
        assert_eq!(0x445566778899aabb, id.serial_number);
        sdp.destroy().done();
    }

    /// Test triggering a differential pressure sample
    #[test]
    fn trigger_differential_pressure_read() {
        let bytes: [u8; 2] = Command::TriggerDifferentialPressureRead.into();
        let data = vec![0, 1, 0xb0, 3, 4, 0x68, 6, 7, 0x4c];
        let expectations = [
            Transaction::write(0x10, bytes.into()),
            Transaction::read(0x10, data.clone()),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sdp = Sdp8xx::new(mock, 0x10, DelayMock);
        let _data = sdp.trigger_differential_pressure_sample().unwrap();
        sdp.destroy().done();
    }

    /// Test triggering a mass flow sample
    #[test]
    fn trigger_mass_flow_read() {
        let bytes: [u8; 2] = Command::TriggerMassFlowRead.into();
        let data = vec![3, 4, 0x68, 6, 7, 0x4c, 0, 1, 0xb0];
        let expectations = [
            Transaction::write(0x10, bytes.into()),
            Transaction::read(0x10, data.clone()),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sdp = Sdp8xx::new(mock, 0x10, DelayMock);
        let _data = sdp.trigger_mass_flow_sample().unwrap();
        sdp.destroy().done();
    }

    /// Test sampling differential pressure
    #[test]
    fn sample_differential_pressure() {
        let bytes: [u8; 2] = Command::SampleDifferentialPressureAveraging.into();
        let stop: [u8; 2] = Command::StopContinuousMeasurement.into();

        let data = vec![0, 1, 0xb0, 3, 4, 0x68, 6, 7, 0x4c];
        let expectations = [
            Transaction::write(0x10, bytes.into()),
            Transaction::read(0x10, data.clone()),
            Transaction::read(0x10, data.clone()),
            Transaction::read(0x10, data.clone()),
            Transaction::write(0x10, stop.into()),
        ];
        let mock = I2cMock::new(&expectations);
        let sdp = Sdp8xx::new(mock, 0x10, DelayMock);
        let mut sampling = sdp.start_sampling_differential_pressure(true).unwrap();
        let _data1 = sampling.read_continuous_sample().unwrap();
        let _data2 = sampling.read_continuous_sample().unwrap();
        let _data3 = sampling.read_continuous_sample().unwrap();

        let sdp = sampling.stop_sampling().unwrap();
        sdp.destroy().done();
    }

    /// Test sampling mass flow
    #[test]
    fn sample_mass_flow() {
        let bytes: [u8; 2] = Command::SampleMassFlowAveraging.into();
        let stop: [u8; 2] = Command::StopContinuousMeasurement.into();

        let data = vec![0, 1, 0xb0, 3, 4, 0x68, 6, 7, 0x4c];
        let expectations = [
            Transaction::write(0x10, bytes.into()),
            Transaction::read(0x10, data.clone()),
            Transaction::read(0x10, data.clone()),
            Transaction::read(0x10, data.clone()),
            Transaction::write(0x10, stop.into()),
        ];
        let mock = I2cMock::new(&expectations);
        let sdp = Sdp8xx::new(mock, 0x10, DelayMock);
        let mut sampling = sdp.start_sampling_mass_flow(true).unwrap();
        let _data1 = sampling.read_continuous_sample().unwrap();
        let _data2 = sampling.read_continuous_sample().unwrap();
        let _data3 = sampling.read_continuous_sample().unwrap();

        let sdp = sampling.stop_sampling().unwrap();
        sdp.destroy().done();
    }

    /// Test the sleep function
    #[test]
    fn go_to_sleep() {
        let bytes: [u8; 2] = Command::EnterSleepMode.into();
        let expectations = [
            Transaction::write(0x25, bytes.into()),
            /* dummy */ Transaction::write(0x25, vec![]),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]),
        ];
        let mock = I2cMock::new(&expectations);
        let sdp = Sdp8xx::new(mock, 0x25, DelayMock);
        let sleeping = sdp.go_to_sleep().unwrap();
        let sdp = sleeping.wake_up().unwrap();
        sdp.destroy().done();
    }

    /// Test waking up from sleep by polling
    #[test]
    fn wakeup_by_polling() {
        let bytes: [u8; 2] = Command::EnterSleepMode.into();
        let expectations = [
            Transaction::write(0x25, bytes.into()),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]).with_error(MockError::Io(ErrorKind::Other)),
            Transaction::write(0x25, vec![]),
        ];
        let mock = I2cMock::new(&expectations);
        let sdp = Sdp8xx::new(mock, 0x25, DelayMock);
        let sleeping = sdp.go_to_sleep().unwrap();
        let sdp = sleeping.wake_up_poll().unwrap();
        sdp.destroy().done();
    }
}
