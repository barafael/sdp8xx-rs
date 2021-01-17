//! Sample type for SDP8xx differential pressure sensor

use core::convert::TryFrom;
use core::marker::PhantomData;

use sensirion_i2c::{
    crc8::{self, *},
    i2c::I2CBuffer,
};

const TEMPERATURE_SCALE_FACTOR: f32 = 200.0f32;

/// Product Identification Error
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Error {
    /// Wrong CRC
    CrcError,
    /// Invalid scale factor
    InvalidScaleFactor,
    /// Wrong Buffer Size
    WrongBufferSize,
}

impl From<crc8::Error> for Error {
    fn from(val: crc8::Error) -> Self {
        match val {
            crc8::Error::CrcError => Error::CrcError,
        }
    }
}

/// Marker type for differential pressure
pub struct DifferentialPressure;

/// Marker type for mass flow
pub struct MassFlow;

/// A measurement result from the sensor.
#[derive(Debug, PartialEq, Clone)]
pub struct Sample<T> {
    /// Pressure in Pa
    pub value: f32,
    /// Temperature reading
    pub temperature: f32,
    /// Sample data type
    state: PhantomData<T>,
}

impl<T> TryFrom<[u8; 9]> for Sample<T> {
    type Error = Error;

    fn try_from(mut buffer: [u8; 9]) -> Result<Self, Self::Error> {
        let i2c_buffer = I2CBuffer::try_from(&mut buffer[..]).unwrap();
        validate(&i2c_buffer)?;

        let dp_raw: i16 = (buffer[0] as i16) << 8 | buffer[1] as i16;
        let temp_raw: i16 = (buffer[3] as i16) << 8 | buffer[4] as i16;
        let dp_scale: i16 = (buffer[6] as i16) << 8 | buffer[7] as i16;

        if dp_scale == 0 {
            return Err(Error::InvalidScaleFactor);
        }

        let value = dp_raw as f32 / dp_scale as f32;
        let temperature = temp_raw as f32 / TEMPERATURE_SCALE_FACTOR;

        Ok(Sample::<T> {
            value,
            temperature,
            state: PhantomData::<T>,
        })
    }
}

impl<T> Sample<T> {
    /// Get the temperature
    pub fn get_temperature(&self) -> f32 {
        self.temperature
    }
}

impl Sample<MassFlow> {
    /// Get mass flow reading
    pub fn get_mass_flow(&self) -> f32 {
        self.value
    }
}

impl Sample<DifferentialPressure> {
    /// Get differential pressure reading
    pub fn get_differential_pressure(&self) -> f32 {
        self.value
    }
}
