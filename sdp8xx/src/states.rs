//! State types for the SDP8xx

use core::marker::PhantomData;

use crate::{DifferentialPressure, MassFlow, Sdp8xx, SdpError};

/// Default idle state of the SDP8xx
#[derive(Debug)]
pub struct IdleState;

/// Triggered state of the SDP8xx
#[derive(Debug)]
pub struct TriggeredState;

/// Continuous sampling state of the SDP8xx
#[derive(Debug)]
pub struct ContinuousSamplingState<MeasurementType> {
    data_type: PhantomData<MeasurementType>,
}

/// Sleep state of the SDP8xx
#[derive(Debug)]
pub struct SleepState;

/// Transition from Idle to Mass Flow Sampling
pub type ToMassflowSampling<I2C, D> =
    Result<Sdp8xx<I2C, D, ContinuousSamplingState<MassFlow>>, SdpError<I2C, I2C>>;

/// Transition from Idle to Differential Pressure Sampling
pub type ToDifferentialPressureSampling<I2C, D> =
    Result<Sdp8xx<I2C, D, ContinuousSamplingState<DifferentialPressure>>, SdpError<I2C, I2C>>;

/// Transition from Continuous Sampling to Idle
pub type ToIdle<I2C, D> = Result<Sdp8xx<I2C, D, IdleState>, SdpError<I2C, I2C>>;

/// Transition from Idle to Sleep state
pub type ToSleep<I2C, D> = Result<Sdp8xx<I2C, D, SleepState>, SdpError<I2C, I2C>>;

/// Transition from Sleep to Idle state
pub type SleepToIdle<I2C, D> = Result<Sdp8xx<I2C, D, IdleState>, SdpError<I2C, I2C>>;
