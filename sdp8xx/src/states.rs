//! State types for the `SDP8xx`

use core::marker::PhantomData;

use crate::{DifferentialPressure, MassFlow, Sdp8xx, SdpError};

/// Default idle state of the `SDP8xx`
#[derive(Debug)]
pub struct IdleState;

/// Triggered state of the `SDP8xx`
#[derive(Debug)]
pub struct TriggeredState;

/// Continuous sampling state of the `SDP8xx`
#[derive(Debug)]
pub struct ContinuousSamplingState<MeasurementType> {
    data_type: PhantomData<MeasurementType>,
}

/// Sleep state of the `SDP8xx`
#[derive(Debug)]
pub struct SleepState;

/// Transition from Idle to Mass Flow Sampling
pub type ToMassflowSampling<I, D> =
    Result<Sdp8xx<I, D, ContinuousSamplingState<MassFlow>>, SdpError<I>>;

/// Transition from Idle to Differential Pressure Sampling
pub type ToDifferentialPressureSampling<I, D> =
    Result<Sdp8xx<I, D, ContinuousSamplingState<DifferentialPressure>>, SdpError<I>>;

/// Transition from Continuous Sampling to Idle
pub type ToIdle<I, D> = Result<Sdp8xx<I, D, IdleState>, SdpError<I>>;

/// Transition from Idle to Sleep state
pub type ToSleep<I, D> = Result<Sdp8xx<I, D, SleepState>, SdpError<I>>;

/// Transition from Sleep to Idle state
pub type SleepToIdle<I, D> = Result<Sdp8xx<I, D, IdleState>, SdpError<I>>;
