//! State types for the SDP8xx

use core::marker::PhantomData;

/// Default idle state of the SDP8xx
#[derive(Debug)]
pub struct IdleState {}

/// Triggered state of the SDP8xx
#[derive(Debug)]
pub struct TriggeredState {}

/// Continuous sampling state of the SDP8xx
#[derive(Debug)]
pub struct ContinuousSamplingState<MeasurementType> {
    data_type: PhantomData<MeasurementType>,
}

/// Sleep state of the SDP8xx
#[derive(Debug)]
pub struct SleepState {}
