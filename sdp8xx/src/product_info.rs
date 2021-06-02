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
        let product_number = ((buf[0] as u32) << 24
            | (buf[1] as u32) << 16
            | (buf[3] as u32) << 8
            | (buf[4] as u32))
            .into();

        let serial_number: u64 = (buf[6] as u64) << 56
            | (buf[7] as u64) << 48
            | (buf[9] as u64) << 40
            | (buf[10] as u64) << 32
            | (buf[12] as u64) << 24
            | (buf[13] as u64) << 16
            | (buf[15] as u64) << 8
            | (buf[16] as u64);

        ProductIdentifier {
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
            [0x03, 0x02, 0x01, n] => ProductVariant::Sdp800_500Pa { revision: n },
            [0x03, 0x02, 0x0A, n] => ProductVariant::Sdp810_500Pa { revision: n },
            [0x03, 0x02, 0x04, n] => ProductVariant::Sdp801_500Pa { revision: n },
            [0x03, 0x02, 0x0D, n] => ProductVariant::Sdp811_500Pa { revision: n },
            [0x03, 0x02, 0x02, n] => ProductVariant::Sdp800_125Pa { revision: n },
            [0x03, 0x02, 0x0B, n] => ProductVariant::Sdp810_125Pa { revision: n },
            _ => ProductVariant::Unknown,
        }
    }
}

impl ProductVariant {
    /// Get the conversion factor for differential pressure in 1/Pa
    pub fn get_default_conversion_factor(&self) -> Option<i16> {
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
    pub fn get_default_i2c_address(&self) -> Option<u8> {
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
