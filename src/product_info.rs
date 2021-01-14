//! Product Identification Types

use core::convert::TryFrom;
use core::convert::TryInto;

use sensirion_i2c::crc8::{self, *};
use sensirion_i2c::i2c::I2CBuffer;

/// Product Identification Error
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Error {
    /// Wrong buffer size
    WrongBufferSize,
    /// Wrong CRC
    CrcError,
    /// Invalid variant
    InvalidVariant,
}

impl From<crc8::Error> for Error {
    fn from(val: crc8::Error) -> Self {
        match val {
            crc8::Error::CrcError => Error::CrcError,
        }
    }
}

/// Product Identification as described in the datasheet (6.3.6 Read Product Identifier)
#[derive(Debug)]
pub struct ProductIdentifier {
    /// The serial number (64 bit)
    pub serial_number: u64,
    /// The product number (32 bit)
    pub product_number: ProductVariant,
}

impl TryFrom<[u8; 18]> for ProductIdentifier {
    type Error = Error;

    fn try_from(mut buf: [u8; 18]) -> Result<Self, Self::Error> {
        let i2c_buffer = I2CBuffer::try_from(&mut buf[..]).unwrap();
        validate(&i2c_buffer)?;

        let product_number = ((buf[0] as u32) << 24
            | (buf[1] as u32) << 16
            | (buf[3] as u32) << 8
            | (buf[4] as u32) << 0)
            .try_into()?;

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

/// Product variant as listed in the datasheet
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum ProductVariant {
    /// SDP800 500 Pascal range with manifold connection, I2C address 0x25
    Sdp800_500Pa {
        /// The chip revision number
        revision: u8,
    },
    /// SDP810 500 Pascal range with tube connection, I2C address 0x25
    Sdp810_500Pa {
        /// The chip revision number
        revision: u8,
    },
    /// SDP801 500 Pascal range with manifold connection, I2C address 0x26
    Sdp801_500Pa {
        /// The chip revision number
        revision: u8,
    },
    /// SDP811 500 Pascal range with tube connection, I2C address 0x26
    Sdp811_500Pa {
        /// The chip revision number
        revision: u8,
    },
    /// SDP800 125 Pascal range with manifold connection, I2C address 0x25
    Sdp800_125Pa {
        /// The chip revision number
        revision: u8,
    },
    /// SDP810 125 Pascal range with tube connection, I2C address 0x25
    Sdp810_125Pa {
        /// The chip revision number
        revision: u8,
    },
}

impl TryFrom<u32> for ProductVariant {
    type Error = Error;

    /// Parse the product variant. The last byte is the revision number and might change.
    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value.to_be_bytes() {
            [0x03, 0x02, 0x01, n] => Ok(ProductVariant::Sdp800_500Pa { revision: n }),
            [0x03, 0x02, 0x0A, n] => Ok(ProductVariant::Sdp810_500Pa { revision: n }),
            [0x03, 0x02, 0x04, n] => Ok(ProductVariant::Sdp801_500Pa { revision: n }),
            [0x03, 0x02, 0x0D, n] => Ok(ProductVariant::Sdp811_500Pa { revision: n }),
            [0x03, 0x02, 0x02, n] => Ok(ProductVariant::Sdp800_125Pa { revision: n }),
            [0x03, 0x02, 0x0B, n] => Ok(ProductVariant::Sdp810_125Pa { revision: n }),
            _ => Err(Error::InvalidVariant),
        }
    }
}

impl ProductVariant {
    /// Get the conversion factor for differential pressure in 1/Pa
    fn get_default_conversion_factor(&self) -> i16 {
        match self {
            ProductVariant::Sdp800_500Pa { .. } => 60,
            ProductVariant::Sdp810_500Pa { .. } => 60,
            ProductVariant::Sdp801_500Pa { .. } => 60,
            ProductVariant::Sdp811_500Pa { .. } => 60,
            ProductVariant::Sdp800_125Pa { .. } => 240,
            ProductVariant::Sdp810_125Pa { .. } => 240,
        }
    }

    /// Get the sensor default I2C address
    fn get_default_i2c_address(&self) -> u8 {
        match self {
            ProductVariant::Sdp801_500Pa { .. } | ProductVariant::Sdp811_500Pa { .. } => 0x26,
            _ => 0x25,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test getting the default i2c address
    #[test]
    fn test_default_i2c_address() {
        let data = [
            0x03, 0x02, 206, 0x02, 0x01, 105, 0x44, 0x55, 0x00, 0x66, 0x77, 225, 0x88, 0x99, 0x24,
            0xaa, 0xbb, 0xC5,
        ];
        let product_id = ProductIdentifier::try_from(data).unwrap();
        assert_eq!(
            ProductVariant::Sdp800_125Pa { revision: 0x01 },
            product_id.product_number
        );
        assert_eq!(0x25, product_id.product_number.get_default_i2c_address());
    }

    /// Test getting the default conversion factor
    #[test]
    fn test_default_conversion_factor() {
        let data = [
            0x03, 0x02, 206, 0x02, 0x01, 105, 0x44, 0x55, 0x00, 0x66, 0x77, 225, 0x88, 0x99, 0x24,
            0xaa, 0xbb, 0xC5,
        ];
        let product_id = ProductIdentifier::try_from(data).unwrap();
        assert_eq!(
            ProductVariant::Sdp800_125Pa { revision: 0x01 },
            product_id.product_number
        );
        assert_eq!(
            240,
            product_id.product_number.get_default_conversion_factor()
        );
    }
}
