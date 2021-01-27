//! Sample type for SDP8xx differential pressure sensor

use core::convert::TryFrom;
use core::marker::PhantomData;

use sensirion_i2c::i2c_buffer::I2cBuffer;

const TEMPERATURE_SCALE_FACTOR: f32 = 200.0f32;

/// Product Identification Error
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum SampleError {
    /// Invalid scale factor
    InvalidScaleFactor,
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

impl<T> TryFrom<I2cBuffer<9>> for Sample<T> {
    type Error = SampleError;

    fn try_from(buffer: I2cBuffer<9>) -> Result<Self, Self::Error> {
        let dp_raw: i16 = (*buffer.get(0).unwrap() as i16) << 8 | *buffer.get(1).unwrap() as i16;
        let temp_raw: i16 = (*buffer.get(3).unwrap() as i16) << 8 | *buffer.get(4).unwrap() as i16;
        let dp_scale: i16 = (*buffer.get(6).unwrap() as i16) << 8 | *buffer.get(7).unwrap() as i16;

        if dp_scale == 0 {
            return Err(SampleError::InvalidScaleFactor);
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

impl<T> TryFrom<[u8; 9]> for Sample<T> {
    type Error = SampleError;

    fn try_from(buffer: [u8; 9]) -> Result<Self, Self::Error> {
        let dp_raw: i16 = (buffer[0] as i16) << 8 | buffer[1] as i16;
        let temp_raw: i16 = (buffer[3] as i16) << 8 | buffer[4] as i16;
        let dp_scale: i16 = (buffer[6] as i16) << 8 | buffer[7] as i16;

        if dp_scale == 0 {
            return Err(SampleError::InvalidScaleFactor);
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

#[cfg(test)]
mod tests {
    use std::marker::PhantomData;

    use crate::{DifferentialPressure, MassFlow, Sample};

    #[test]
    fn get_mass_flow_temperature() {
        let sample = Sample::<MassFlow> {
            temperature: 31.0,
            value: 1.0,
            state: PhantomData::<MassFlow>,
        };
        assert_eq!(sample.get_temperature(), sample.temperature);
        assert_eq!(sample.get_mass_flow(), sample.value);
    }

    #[test]
    fn get_differential_pressure_temperature() {
        let sample = Sample {
            temperature: -14.0,
            value: 10.0,
            state: PhantomData::<DifferentialPressure>,
        };
        assert_eq!(sample.get_temperature(), sample.temperature);
        assert_eq!(sample.get_differential_pressure(), sample.value);
    }
}
