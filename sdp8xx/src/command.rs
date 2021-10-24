//! I2C commands for SDP8xx differential pressure sensor

use core::convert::From;

/// I2C commands sent to the sensor.
/// Missing commands: General call reset, exit sleep mode
#[derive(Debug, Copy, Clone)]
pub(crate) enum Command {
    /// Trigger Mass Flow Reading with no clock stretching
    TriggerMassFlowRead,
    /// Trigger Mass Flow Reading with clock stretching
    TriggerMassFlowReadSync,
    /// Trigger Differential Pressure Reading with no clock stretching
    TriggerDifferentialPressureRead,
    /// Trigger Differential Pressure Reading with clock stretching
    TriggerDifferentialPressureReadSync,
    /// Continuous Mass Flow Sampling with Average till read
    SampleMassFlowAveraging,
    /// Continuous Mass Flow Sampling with no averaging
    SampleMassFlowRaw,
    /// Continuous Differential Pressure Sampling with Average till read
    SampleDifferentialPressureAveraging,
    /// Continuous Differential Pressure Sampling with no averaging
    SampleDifferentialPressureRaw,
    /// Stop continuous measurement
    StopContinuousMeasurement,
    /// Enter sleep mode
    EnterSleepMode,
    /// Read product identifier 0
    ReadProductId0,
    /// Read product identifier 1
    ReadProductId1,
}

impl From<Command> for [u8; 2] {
    fn from(val: Command) -> Self {
        match val {
            Command::TriggerMassFlowRead => [0x36, 0x24],
            Command::TriggerMassFlowReadSync => [0x37, 0x26],
            Command::TriggerDifferentialPressureRead => [0x36, 0x2F],
            Command::TriggerDifferentialPressureReadSync => [0x37, 0x2D],
            Command::SampleMassFlowAveraging => [0x36, 0x03],
            Command::SampleMassFlowRaw => [0x36, 0x08],
            Command::SampleDifferentialPressureAveraging => [0x36, 0x15],
            Command::SampleDifferentialPressureRaw => [0x36, 0x1E],
            Command::StopContinuousMeasurement => [0x3F, 0xF9],
            Command::EnterSleepMode => [0x36, 0x77],
            Command::ReadProductId0 => [0x36, 0x7C],
            Command::ReadProductId1 => [0xE1, 0x02],
        }
    }
}

impl From<Command> for u16 {
    fn from(val: Command) -> Self {
        match val {
            Command::TriggerMassFlowRead => 0x3624,
            Command::TriggerMassFlowReadSync => 0x3726,
            Command::TriggerDifferentialPressureRead => 0x362F,
            Command::TriggerDifferentialPressureReadSync => 0x372D,
            Command::SampleMassFlowAveraging => 0x3603,
            Command::SampleMassFlowRaw => 0x3608,
            Command::SampleDifferentialPressureAveraging => 0x3615,
            Command::SampleDifferentialPressureRaw => 0x361E,
            Command::StopContinuousMeasurement => 0x3FF9,
            Command::EnterSleepMode => 0x3677,
            Command::ReadProductId0 => 0x367C,
            Command::ReadProductId1 => 0xE102,
        }
    }
}
