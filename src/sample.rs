//! Sample type for SDP8xx differential pressure sensor

use core::convert::TryFrom;

use sensirion_i2c::crc8::{self, *};

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

/// A measurement result from the sensor.
#[derive(Debug, PartialEq, Clone)]
pub struct Sample {
    /// Pressure in Pa
    pub differential_pressure: f32,
    /// Temperature reading
    pub temperature: f32,
}

impl TryFrom<[u8; 9]> for Sample {
    type Error = Error;

    fn try_from(buffer: [u8; 9]) -> Result<Self, Self::Error> {
        validate(&buffer)?;

        let dp_raw: i16 = (buffer[0] as i16) << 8 | buffer[1] as i16;
        let temp_raw: i16 = (buffer[3] as i16) << 8 | buffer[4] as i16;
        let dp_scale: i16 = (buffer[6] as i16) << 8 | buffer[7] as i16;

        if dp_scale == 0 {
            return Err(Error::InvalidScaleFactor);
        }

        let differential_pressure = dp_raw as f32 / dp_scale as f32;
        let temperature = temp_raw as f32 / TEMPERATURE_SCALE_FACTOR;

        Ok(Sample {
            differential_pressure,
            temperature,
        })
    }
}
