use embassy_time::{Duration, Instant};

use crate::Axis;

const STALL_DETECTION_ENABLED: bool = false;
const OSCILLATION_DETECTION_ENABLED: bool = false;
const STALL_MIN_TARGET_RPM: f32 = 5.0;
const STALL_MIN_OUTPUT: f32 = 80.0;
const STALL_MAX_SPEED_RPM: f32 = 1.0;
const STALL_DETECTION_DURATION: Duration = Duration::from_millis(500);
const OSCILLATION_MIN_AMPLITUDE_RAD: f32 = 0.02;
const OSCILLATION_MAX_PERIOD: Duration = Duration::from_millis(333);
pub(crate) const OSCILLATION_DETECTION_DURATION: Duration = Duration::from_millis(500);
const OSCILLATION_MIN_SPEED_RPM: f32 = 1.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    Stalled,
    Oscillating,
    InvalidCommand,
    PrepareTimeout,
    NormalOverspeed,
    CanFailure,
    ControlTiming,
    InvalidControl,
}

#[derive(Default)]
pub(crate) struct AxisFaultMonitor {
    stall_since: Option<Instant>,
    last_speed_direction: i8,
    cycle_direction: i8,
    cycle_started_at: Option<Instant>,
    previous_angle: f32,
    unwrapped_angle: f32,
    cycle_min_angle: f32,
    cycle_max_angle: f32,
    high_frequency_since: Option<Instant>,
    angle_initialized: bool,
    pub estimated_frequency_hz: f32,
}

impl AxisFaultMonitor {
    pub(crate) fn reset(&mut self) {
        *self = Self::default();
    }

    pub(crate) fn update(
        &mut self,
        axis: &Axis,
        now: Instant,
        output_enabled: bool,
    ) -> Option<Fault> {
        let stalled = STALL_DETECTION_ENABLED
            && output_enabled
            && axis.target_rpm.abs() >= STALL_MIN_TARGET_RPM
            && axis.motor.command_voltage.abs() >= STALL_MIN_OUTPUT
            && axis.motor.speed_rpm.abs() <= STALL_MAX_SPEED_RPM;
        if stalled {
            let since = *self.stall_since.get_or_insert(now);
            if now.saturating_duration_since(since) >= STALL_DETECTION_DURATION {
                return Some(Fault::Stalled);
            }
        } else {
            self.stall_since = None;
        }

        if !OSCILLATION_DETECTION_ENABLED {
            return None;
        }

        let angle = axis.motor.angle_rad;
        if !self.angle_initialized {
            self.previous_angle = angle;
            self.unwrapped_angle = angle;
            self.angle_initialized = true;
            self.cycle_started_at = Some(now);
            self.cycle_min_angle = angle;
            self.cycle_max_angle = angle;
        } else {
            let mut delta = angle - self.previous_angle;
            if delta > core::f32::consts::PI {
                delta -= core::f32::consts::TAU;
            } else if delta < -core::f32::consts::PI {
                delta += core::f32::consts::TAU;
            }
            self.unwrapped_angle += delta;
            self.previous_angle = angle;
        }
        self.cycle_min_angle = self.cycle_min_angle.min(self.unwrapped_angle);
        self.cycle_max_angle = self.cycle_max_angle.max(self.unwrapped_angle);

        let speed_direction = if axis.motor.speed_rpm >= OSCILLATION_MIN_SPEED_RPM {
            1
        } else if axis.motor.speed_rpm <= -OSCILLATION_MIN_SPEED_RPM {
            -1
        } else {
            0
        };
        if speed_direction == 0 {
            return None;
        }

        let direction_changed =
            self.last_speed_direction != 0 && speed_direction != self.last_speed_direction;
        self.last_speed_direction = speed_direction;
        if !direction_changed {
            return None;
        }

        match self.cycle_started_at {
            Some(started_at) if self.cycle_direction == speed_direction => {
                let period = now.saturating_duration_since(started_at);
                let amplitude = self.cycle_max_angle - self.cycle_min_angle;
                if period <= OSCILLATION_MAX_PERIOD && amplitude >= OSCILLATION_MIN_AMPLITUDE_RAD {
                    self.estimated_frequency_hz = 1000.0 / period.as_millis().max(1) as f32;
                    let since = *self.high_frequency_since.get_or_insert(now);
                    if now.saturating_duration_since(since) >= OSCILLATION_DETECTION_DURATION {
                        return Some(Fault::Oscillating);
                    }
                } else {
                    self.high_frequency_since = None;
                }

                self.cycle_started_at = Some(now);
                self.cycle_min_angle = self.unwrapped_angle;
                self.cycle_max_angle = self.unwrapped_angle;
            }
            _ if self.cycle_direction == 0 => {
                self.cycle_direction = speed_direction;
                self.cycle_started_at = Some(now);
                self.cycle_min_angle = self.unwrapped_angle;
                self.cycle_max_angle = self.unwrapped_angle;
            }
            _ => {}
        }

        None
    }
}
