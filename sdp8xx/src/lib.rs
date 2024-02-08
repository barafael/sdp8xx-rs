//! A platform agnostic Rust driver for the Sensirion `SDP8xx` differential pressure sensor, based
//! on the [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.
//! Heavily inspired by the [`sgp30 driver by Danilo Bergen`](https://github.com/dbrgn/sgp30-rs)
//!
//! ## The Device
//!
//! The Sensirion `SDP8xx` is a differential pressure sensor. It has an I2C interface.
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
//!
//! ### Fetching Some Data
//!
//! You can fetch the differential pressure:
//!
//! ```no_run
//! use linux_embedded_hal as hal;
//! use hal::{Delay, I2cdev};
//! use sdp8xx::Sdp8xx;
//!
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let mut sdp = Sdp8xx::new(dev, 0x25, Delay);
//!
//! let data = match sdp.trigger_differential_pressure_sample() {
//!     Ok(d) => dbg!(d),
//!     Err(_) => panic!(),
//! };
//! ```

#![warn(unsafe_code)]
#![warn(missing_docs)]
#![cfg_attr(not(test), no_std)]

#[cfg(test)]
mod test;

use crate::command::Command;
pub use crate::sample::*;
use core::convert::TryFrom;
use core::marker::PhantomData;
use embedded_hal::{delay::DelayNs, i2c::I2c};
pub use product_info::ProductIdentifier;
use states::{
    ContinuousSamplingState, IdleState, SleepState, SleepToIdle, ToDifferentialPressureSampling,
    ToIdle, ToMassflowSampling, ToSleep,
};

pub mod command;
pub mod product_info;
pub mod sample;
pub mod states;

/// All possible errors in this crate
#[derive(Debug)]
pub enum SdpError<I>
where
    I: I2c,
{
    /// Error from the underlying sensirion I2C device
    Device(sensirion_i2c::i2c::Error<I>),
    /// Wrong Buffer Size
    InvalidBufferSize,
    /// Invalid sensor variant
    InvalidVariant,
    /// Wake up sent while sensor was not in sleep state
    WakeUpWhileNotSleeping,
    /// Cannot wake up sensor
    CannotWakeUp,
    /// Sampling error
    SampleError,
    /// Buffer too small for requested operation
    BufferTooSmall,
}

impl<I> From<sensirion_i2c::i2c::Error<I>> for SdpError<I>
where
    I: I2c,
{
    fn from(error: sensirion_i2c::i2c::Error<I>) -> Self {
        SdpError::Device(error)
    }
}

/// State of the `SDP8xx`
#[derive(Debug)]
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

impl<I, D> Sdp8xx<I, D, IdleState>
where
    I: I2c,
    D: DelayNs,
{
    /// Create a new instance of the `SDP8xx` driver.
    pub fn new(i2c: I, address: u8, delay: D) -> Self {
        Self {
            i2c,
            address,
            delay,
            state: PhantomData::<IdleState>,
        }
    }

    /// Destroy driver instance, return I2C bus instance.
    pub fn release(self) -> I {
        self.i2c
    }

    /// Write an I2C command to the sensor.
    fn send_command(&mut self, command: Command) -> Result<(), SdpError<I>> {
        sensirion_i2c::i2c::write_command_u16(&mut self.i2c, self.address, command.into())
            .map_err(|e| SdpError::Device(sensirion_i2c::i2c::Error::I2cWrite(e)))
    }

    /// Return the product id of the `SDP8xx`
    pub fn read_product_id(&mut self) -> Result<ProductIdentifier, SdpError<I>> {
        let mut buf = [0; 18];
        // Request product id
        self.send_command(Command::ReadProductId0)?;
        self.send_command(Command::ReadProductId1)?;

        self.i2c
            .read(self.address, &mut buf)
            .map_err(|e| SdpError::Device(sensirion_i2c::i2c::Error::I2cRead(e)))?;

        Ok(ProductIdentifier::from(buf))
    }

    /// Trigger a differential pressure read without clock stretching.
    /// This function blocks for at least 60 milliseconds to await a result.
    pub fn trigger_differential_pressure_sample(
        &mut self,
    ) -> Result<Sample<DifferentialPressure>, SdpError<I>> {
        self.send_command(Command::TriggerDifferentialPressureRead)?;
        self.delay.delay_ms(60);
        let mut buffer = [0u8; 9];
        sensirion_i2c::i2c::read_words_with_crc(&mut self.i2c, self.address, &mut buffer)?;
        Sample::<DifferentialPressure>::try_from(buffer).map_err(|_| SdpError::SampleError)
    }

    /// Trigger a mass flow read without clock stretching.
    /// This function blocks for at least 60 milliseconds to await a result.
    pub fn trigger_mass_flow_sample(&mut self) -> Result<Sample<MassFlow>, SdpError<I>> {
        self.send_command(Command::TriggerMassFlowRead)?;
        self.delay.delay_ms(60);
        let mut buffer = [0u8; 9];
        sensirion_i2c::i2c::read_words_with_crc(&mut self.i2c, self.address, &mut buffer)?;
        Sample::<MassFlow>::try_from(buffer).map_err(|_| SdpError::SampleError)
    }

    /// Trigger a differential pressure read with clock stretching.
    /// This function blocks until the data becomes available (if clock stretching is supported).
    pub fn trigger_differential_pressure_sample_sync(
        &mut self,
    ) -> Result<Sample<DifferentialPressure>, SdpError<I>> {
        self.send_command(Command::TriggerDifferentialPressureReadSync)?;
        let mut buffer = [0u8; 9];
        sensirion_i2c::i2c::read_words_with_crc(&mut self.i2c, self.address, &mut buffer)?;
        Sample::<DifferentialPressure>::try_from(buffer).map_err(|_| SdpError::SampleError)
    }

    /// Trigger a mass flow read with clock stretching.
    /// This function blocks until the data becomes available (if clock stretching is supported).
    pub fn trigger_mass_flow_sample_sync(&mut self) -> Result<Sample<MassFlow>, SdpError<I>> {
        self.send_command(Command::TriggerMassFlowReadSync)?;
        let mut buffer = [0u8; 9];
        sensirion_i2c::i2c::read_words_with_crc(&mut self.i2c, self.address, &mut buffer)?;
        Sample::<MassFlow>::try_from(buffer).map_err(|_| SdpError::SampleError)
    }

    /// Start sampling in continuous mode
    pub fn start_sampling_differential_pressure(
        mut self,
        averaging: bool,
    ) -> ToDifferentialPressureSampling<I, D> {
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
    pub fn start_sampling_mass_flow(mut self, averaging: bool) -> ToMassflowSampling<I, D> {
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

    /// Enter the `SDP8xx` sleep state
    pub fn go_to_sleep(mut self) -> ToSleep<I, D> {
        self.send_command(Command::EnterSleepMode)?;
        Ok(Sdp8xx {
            i2c: self.i2c,
            address: self.address,
            delay: self.delay,
            state: PhantomData::<SleepState>,
        })
    }
}

impl<I, D> Sdp8xx<I, D, SleepState>
where
    I: I2c,
    D: DelayNs,
{
    /// Wake the sensor up from the sleep state
    /// This function blocks for at least 2 milliseconds
    pub fn wake_up(mut self) -> SleepToIdle<I, D> {
        // TODO polling with timeout.
        // Send wake up signal (not acked)
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
    pub fn wake_up_poll(mut self) -> SleepToIdle<I, D> {
        // Send wake up signal (not acked)
        if self.i2c.write(self.address, &[]).is_ok() {
            return Err(SdpError::WakeUpWhileNotSleeping);
        }
        loop {
            // TODO timeout
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

impl<I, D, T> Sdp8xx<I, D, ContinuousSamplingState<T>>
where
    I: I2c,
    D: DelayNs,
{
    /// Read a sample in continuous mode
    pub fn read_continuous_sample(&mut self) -> Result<Sample<T>, SdpError<I>> {
        let mut buffer = [0u8; 9];
        // TODO rate limiting no faster than 0.5ms
        sensirion_i2c::i2c::read_words_with_crc(&mut self.i2c, self.address, &mut buffer)?;
        Sample::try_from(buffer).map_err(|_| SdpError::SampleError)
    }

    /// Stop sampling continuous mode
    pub fn stop_sampling(mut self) -> ToIdle<I, D> {
        let bytes: [u8; 2] = Command::StopContinuousMeasurement.into();
        self.i2c
            .write(self.address, &bytes)
            .map_err(|e| SdpError::Device(sensirion_i2c::i2c::Error::I2cWrite(e)))?;
        Ok(Sdp8xx {
            i2c: self.i2c,
            address: self.address,
            delay: self.delay,
            state: PhantomData::<IdleState>,
        })
    }
}
