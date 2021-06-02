use super::*;

use embedded_hal_mock;
use embedded_hal_mock::MockError;
use sensirion_i2c::i2c_buffer::Appendable;
use std::io::ErrorKind;

use embedded_hal_mock::delay::MockNoop as DelayMock;
use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction};

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
    let mock = I2cMock::new(&expectations);
    let mut sdp = Sdp8xx::new(mock, 0x25, DelayMock);
    let id = sdp.read_product_id().unwrap();
    assert_eq!(
        ProductVariant::Sdp800_125Pa { revision: 0x01 },
        id.product_number
    );
    assert_eq!(0x445566778899aabb, id.serial_number);
    sdp.release().done();
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
    sdp.release().done();
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
    sdp.release().done();
}

/// Test triggering a differential pressure sample with clock stretching
#[test]
fn trigger_differential_pressure_read_sync() {
    let bytes: [u8; 2] = Command::TriggerDifferentialPressureReadSync.into();
    let data = vec![0, 1, 0xb0, 3, 4, 0x68, 6, 7, 0x4c];
    let expectations = [
        Transaction::write(0x10, bytes.into()),
        Transaction::read(0x10, data.clone()),
    ];
    let mock = I2cMock::new(&expectations);
    let mut sdp = Sdp8xx::new(mock, 0x10, DelayMock);
    let _data = sdp.trigger_differential_pressure_sample_sync().unwrap();
    sdp.release().done();
}

/// Test triggering a mass flow sample with clock stretching
#[test]
fn trigger_mass_flow_read_sync() {
    let bytes: [u8; 2] = Command::TriggerMassFlowReadSync.into();
    let data = vec![3, 4, 0x68, 6, 7, 0x4c, 0, 1, 0xb0];
    let expectations = [
        Transaction::write(0x10, bytes.into()),
        Transaction::read(0x10, data.clone()),
    ];
    let mock = I2cMock::new(&expectations);
    let mut sdp = Sdp8xx::new(mock, 0x10, DelayMock);
    let _data = sdp.trigger_mass_flow_sample_sync().unwrap();
    sdp.release().done();
}

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

    // TODO improve the meaning of this test by checking data

    let sdp = sampling.stop_sampling().unwrap();
    sdp.release().done();
}

#[test]
fn sample_differential_pressure_zero() {
    let mut buffer: I2cBuffer<9> = I2cBuffer::new();
    buffer.append(0u16).unwrap();
    buffer.append(0u16).unwrap();
    buffer.append(0u16).unwrap();
    buffer.validate().unwrap();

    let bytes: [u8; 2] = Command::SampleDifferentialPressureAveraging.into();
    let stop: [u8; 2] = Command::StopContinuousMeasurement.into();

    let expectations = [
        Transaction::write(0x10, bytes.into()),
        Transaction::read(0x10, buffer.to_vec()),
        Transaction::write(0x10, stop.into()),
    ];
    let mock = I2cMock::new(&expectations);
    let sdp = Sdp8xx::new(mock, 0x10, DelayMock);
    let mut sampling = sdp.start_sampling_differential_pressure(true).unwrap();
    let result = sampling.read_continuous_sample();
    match result {
        Ok(_) => panic!("Succeeded with invalid data."),
        Err(SdpError::SampleError) => {}
        _ => panic!("Wrong error variant."),
    }

    let sdp = sampling.stop_sampling().unwrap();
    sdp.release().done();
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
    sdp.release().done();
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
    sdp.release().done();
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
    sdp.release().done();
}

use proptest::prelude::*;

proptest! {
    #[test]
    fn fuzz_differential_pressure_sampling(mut bytes in prop::collection::vec(0..255u8, 9)) {
        bytes[2] = sensirion_i2c::crc8::calculate(&bytes[0..2]);
        bytes[5] = sensirion_i2c::crc8::calculate(&bytes[3..5]);
        bytes[8] = sensirion_i2c::crc8::calculate(&bytes[6..8]);
        let command: [u8; 2] = Command::SampleDifferentialPressureAveraging.into();
        let stop: [u8; 2] = Command::StopContinuousMeasurement.into();

        let expectations = [
            Transaction::write(0x10, command.into()),
            Transaction::read(0x10, bytes.clone()),
            Transaction::write(0x10, stop.into()),
        ];
        let mock = I2cMock::new(&expectations);
        let sdp = Sdp8xx::new(mock, 0x10, DelayMock);
        let mut sampling = sdp.start_sampling_differential_pressure(true).unwrap();
        let _result = sampling.read_continuous_sample();

        let sdp = sampling.stop_sampling().unwrap();
        sdp.release().done();
    }
}
