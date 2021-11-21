//! Product Identification Types

/// Product Identification Error
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Error {
    /// Unknown variant
    UnknownVariant,
}

/// Product Identification as described in the datasheet (6.3.6 Read Product Identifier)
#[derive(Debug)]
pub struct ProductIdentifier {
    /// The serial number (64 bit)
    pub serial_number: u64,
    /// The product number (32 bit)
    pub product_number: ProductVariant,
}

impl From<[u8; 18]> for ProductIdentifier {
    fn from(buf: [u8; 18]) -> Self {
        // TODO check CRC? Or is it checked already at this point.
        // Probably should be `TryFrom` then.
        let product_number = (u32::from(buf[0]) << 24
            | u32::from(buf[1]) << 16
            // 2: CRC
            | u32::from(buf[3]) << 8
            | u32::from(buf[4]))
        // 5: CRC
        .into();

        let serial_number: u64 = u64::from(buf[6]) << 56
            | u64::from(buf[7]) << 48
            // 8: CRC
            | u64::from(buf[9]) << 40
            | u64::from(buf[10]) << 32
            // 11: CRC
            | u64::from(buf[12]) << 24
            | u64::from(buf[13]) << 16
            // 14: CRC
            | u64::from(buf[15]) << 8
            | u64::from(buf[16]);
        // 17: CRC

        Self {
            serial_number,
            product_number,
        }
    }
}

/// Product variant as listed in the datasheet
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum ProductVariant {
    /// Unknown product variant
    Unknown,
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

impl From<u32> for ProductVariant {
    /// Parse the product variant. The last byte is the revision number and might change.
    fn from(value: u32) -> Self {
        match value.to_be_bytes() {
            [0x03, 0x02, 0x01, n] => Self::Sdp800_500Pa { revision: n },
            [0x03, 0x02, 0x0A, n] => Self::Sdp810_500Pa { revision: n },
            [0x03, 0x02, 0x04, n] => Self::Sdp801_500Pa { revision: n },
            [0x03, 0x02, 0x0D, n] => Self::Sdp811_500Pa { revision: n },
            [0x03, 0x02, 0x02, n] => Self::Sdp800_125Pa { revision: n },
            [0x03, 0x02, 0x0B, n] => Self::Sdp810_125Pa { revision: n },
            _ => Self::Unknown,
        }
    }
}

impl ProductVariant {
    /// Get the conversion factor for differential pressure in 1/Pa
    #[must_use]
    pub const fn get_default_conversion_factor(&self) -> Option<i16> {
        match self {
            ProductVariant::Sdp800_500Pa { .. } => Some(60),
            ProductVariant::Sdp810_500Pa { .. } => Some(60),
            ProductVariant::Sdp801_500Pa { .. } => Some(60),
            ProductVariant::Sdp811_500Pa { .. } => Some(60),
            ProductVariant::Sdp800_125Pa { .. } => Some(240),
            ProductVariant::Sdp810_125Pa { .. } => Some(240),
            ProductVariant::Unknown => None,
        }
    }

    /// Get the sensor default I2C address
    #[must_use]
    pub const fn get_default_i2c_address(&self) -> Option<u8> {
        match self {
            ProductVariant::Sdp801_500Pa { .. } | ProductVariant::Sdp811_500Pa { .. } => Some(0x26),
            ProductVariant::Unknown => None,
            _ => Some(0x25),
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
            0x03, 0x02, 0x00, 0x02, 0x01, 0x00, 0x44, 0x55, 0x00, 0x66, 0x77, 0x00, 0x88, 0x99,
            0x00, 0xaa, 0xbb, 0x00,
        ];
        let product_id = ProductIdentifier::from(data);
        assert_eq!(
            ProductVariant::Sdp800_125Pa { revision: 0x01 },
            product_id.product_number
        );
        assert_eq!(
            0x25,
            product_id.product_number.get_default_i2c_address().unwrap()
        );
    }

    /// Test getting the default conversion factor
    #[test]
    fn test_default_conversion_factor() {
        let data = [
            0x03, 0x02, 0x00, 0x02, 0x01, 0x00, 0x44, 0x55, 0x00, 0x66, 0x77, 0x00, 0x88, 0x99,
            0x00, 0xaa, 0xbb, 0x00,
        ];
        let product_id = ProductIdentifier::from(data);
        assert_eq!(
            ProductVariant::Sdp800_125Pa { revision: 0x01 },
            product_id.product_number
        );
        assert_eq!(
            240,
            product_id
                .product_number
                .get_default_conversion_factor()
                .unwrap()
        );
    }
}
