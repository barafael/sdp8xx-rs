//! State types for the SDP8xx

use core::marker::PhantomData;

/// Default idle state of the SDP8xx
pub struct IdleState {}

/// Triggered state of the SDP8xx
pub struct TriggeredState {}

/// Continuous sampling state of the SDP8xx
pub struct ContinuousSamplingState<MeasurementType> {
    data_type: PhantomData<MeasurementType>,
}

/// Sleep state of the SDP8xx
pub struct SleepState {}
