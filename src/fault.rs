use embassy_time::{Duration, Instant};

use crate::Axis;

const OSCILLATION_DETECTION_ENABLED: bool = true;

#[derive(Clone, Copy)]
pub(crate) struct StallConfig {
    enabled: bool,
    min_target_rpm: f32,
    min_output: f32,
    max_speed_rpm: f32,
    duration: Duration,
}

pub(crate) const YAW_STALL_CONFIG: StallConfig = StallConfig {
    enabled: true,
    min_target_rpm: 5.0,
    min_output: 1000.0,
    max_speed_rpm: 1.0,
    duration: Duration::from_millis(1000),
};

pub(crate) const PITCH_STALL_CONFIG: StallConfig = StallConfig {
    enabled: true,
    min_target_rpm: 20.0,
    min_output: 10000.0,
    max_speed_rpm: 0.01,
    duration: Duration::from_millis(1000),
};

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
        stall_config: StallConfig,
    ) -> Option<Fault> {
        let stalled = stall_config.enabled
            && output_enabled
            && axis.target_rpm.abs() >= stall_config.min_target_rpm
            && axis.motor.command_voltage.abs() >= stall_config.min_output
            && axis.motor.speed_rpm.abs() <= stall_config.max_speed_rpm;
        if stalled {
            let since = *self.stall_since.get_or_insert(now);
            if now.saturating_duration_since(since) >= stall_config.duration {
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
