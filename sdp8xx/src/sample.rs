//! Sample type for `SDP8xx` differential pressure sensor

use core::convert::TryFrom;
use core::marker::PhantomData;

const TEMPERATURE_SCALE_FACTOR: f32 = 200.0f32;

/// Product Identification Error
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Error {
    /// Invalid scale factor
    InvalidScaleFactor,
}

/// Marker type for differential pressure
#[derive(Debug, Default, Clone, Copy)]
pub struct DifferentialPressure;

/// Marker type for mass flow
#[derive(Debug, Default, Clone, Copy)]
pub struct MassFlow;

/// A measurement result from the sensor.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct Sample<T> {
    /// Value (unit depends on state)
    pub value: f32,
    /// Temperature reading
    pub temperature: f32,
    /// Sample data type
    measurement_type: PhantomData<T>,
}

impl<T> TryFrom<[u8; 9]> for Sample<T> {
    type Error = Error;

    fn try_from(buffer: [u8; 9]) -> Result<Self, Self::Error> {
        let dp_raw = i16::from(buffer[0]) << 8 | i16::from(buffer[1]);
        let temp_raw = i16::from(buffer[3]) << 8 | i16::from(buffer[4]);
        let dp_scale = i16::from(buffer[6]) << 8 | i16::from(buffer[7]);

        if dp_scale == 0 {
            return Err(Error::InvalidScaleFactor);
        }

        let value = f32::from(dp_raw) / f32::from(dp_scale);
        let temperature = f32::from(temp_raw) / TEMPERATURE_SCALE_FACTOR;

        Ok(Self {
            value,
            temperature,
            measurement_type: PhantomData::<T>,
        })
    }
}

impl<T> Sample<T> {
    /// Get the temperature
    #[must_use]
    pub const fn get_temperature(&self) -> f32 {
        self.temperature
    }
}

impl Sample<MassFlow> {
    /// Get mass flow reading
    #[must_use]
    pub const fn get_mass_flow(&self) -> f32 {
        self.value
    }
}

impl Sample<DifferentialPressure> {
    /// Get differential pressure reading
    #[must_use]
    pub const fn get_differential_pressure(&self) -> f32 {
        self.value
    }
}

#[cfg(test)]
mod tests {
    use crate::{DifferentialPressure, MassFlow, Sample};
    use std::{convert::TryFrom, marker::PhantomData};

    #[test]
    fn get_mass_flow_temperature() {
        let sample = Sample::<MassFlow> {
            temperature: 31.0,
            value: 1.0,
            measurement_type: PhantomData::<MassFlow>,
        };
        assert_eq!(sample.get_temperature(), sample.temperature);
        assert_eq!(sample.get_mass_flow(), sample.value);
    }

    #[test]
    fn get_differential_pressure_temperature() {
        let sample = Sample {
            temperature: -14.0,
            value: 10.0,
            measurement_type: PhantomData::<DifferentialPressure>,
        };
        assert_eq!(sample.get_temperature(), sample.temperature);
        assert_eq!(sample.get_differential_pressure(), sample.value);
    }

    #[test]
    fn try_from_buffer_invalid_scale_factor() {
        let data: [u8; 9] = [0x0, 0xc, 0x0, 0x01, 0x31, 0x0, 0x0, 0x0, 0x0];
        let error = Sample::<DifferentialPressure>::try_from(data);
        assert!(matches!(error, Err(super::Error::InvalidScaleFactor)));
    }
}
