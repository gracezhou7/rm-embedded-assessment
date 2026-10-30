#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};

// GM6020 电流模式组指令，适用于电机 ID 1～4。
const MOTOR_COMMAND_ID: u16 = 0x1FE;
// yaw 电机为 ID 1，pitch 电机为 ID 2。
const YAW_FEEDBACK_ID: u16 = 0x205;
const PITCH_FEEDBACK_ID: u16 = 0x206;
// 判定遥控器失联所需的连续无输入控制周期数。
const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);
const CONTROL_DT_S: f32 = 0.001;
// 遥控器调试日志的开关和输出间隔。
const REMOTE_DEBUG_ENABLED: bool = true;
const REMOTE_DEBUG_INTERVAL: Duration = Duration::from_millis(500);
// Normal 状态控制 yaw 电机的速度，pitch 保持零电流。
const NORMAL_OUTPUT_ENABLED: bool = true;
const NORMAL_SPEED_RPM: f32 = 10.0;
const NORMAL_CURRENT_A: f32 = 0.1;
const NORMAL_MAX_SPEED_RPM: f32 = 30.0;
// 临时阈值，需要结合现场反馈间隔和接收时序确认。
const NORMAL_FEEDBACK_TIMEOUT: Duration = Duration::from_millis(100);

/// 占位参数，并非实测安全值；启用回正前必须根据教具重新标定和验证。
const MOTION_CONFIG: Option<MotionConfig> = Some(MotionConfig {
    feedback_timeout: Duration::from_millis(100),
    prepare_timeout: Duration::from_secs(10),
    settle_time: Duration::from_millis(500),
    settle_angle_rad: 0.02,
    settle_speed_rpm: 2.0,
    stick_deadzone: 0.05,
    yaw: AxisConfig {
        min_angle_rad: -0.1,
        max_angle_rad: 0.1,
        stick_direction: 1.0,
        motor_direction: 1.0,
        max_target_rate_rad_s: 0.1,
        prepare_target_rate_rad_s: 0.05,
        max_speed_rpm: 10.0,
        prepare_speed_rpm: 5.0,
        max_current_a: 0.1,
        prepare_current_a: 0.1,
    },
    pitch: AxisConfig {
        min_angle_rad: -0.1,
        max_angle_rad: 0.1,
        stick_direction: 1.0,
        motor_direction: 1.0,
        max_target_rate_rad_s: 0.1,
        prepare_target_rate_rad_s: 0.05,
        max_speed_rpm: 10.0,
        prepare_speed_rpm: 5.0,
        max_current_a: 0.1,
        prepare_current_a: 0.1,
    },
});
/// TODO：只有验证反馈时序、故障检测和控制输出后才能启用正式回正。
const MOTION_IMPLEMENTED: bool = false;

#[derive(Clone, Copy)]
struct AxisConfig {
    min_angle_rad: f32,
    max_angle_rad: f32,
    stick_direction: f32,
    motor_direction: f32,
    max_target_rate_rad_s: f32,
    prepare_target_rate_rad_s: f32,
    max_speed_rpm: f32,
    prepare_speed_rpm: f32,
    max_current_a: f32,
    prepare_current_a: f32,
}

impl AxisConfig {
    fn is_valid(self) -> bool {
        self.min_angle_rad.is_finite()
            && self.max_angle_rad.is_finite()
            && self.min_angle_rad < 0.0
            && self.max_angle_rad > 0.0
            && (self.stick_direction == -1.0 || self.stick_direction == 1.0)
            && (self.motor_direction == -1.0 || self.motor_direction == 1.0)
            && self.max_target_rate_rad_s.is_finite()
            && self.max_target_rate_rad_s > 0.0
            && self.prepare_target_rate_rad_s.is_finite()
            && self.prepare_target_rate_rad_s > 0.0
            && self.prepare_target_rate_rad_s <= self.max_target_rate_rad_s
            && self.max_speed_rpm.is_finite()
            && self.max_speed_rpm > 0.0
            && self.prepare_speed_rpm.is_finite()
            && self.prepare_speed_rpm > 0.0
            && self.prepare_speed_rpm <= self.max_speed_rpm
            && self.max_current_a.is_finite()
            && self.max_current_a > 0.0
            && self.max_current_a <= 3.0
            && self.prepare_current_a.is_finite()
            && self.prepare_current_a > 0.0
            && self.prepare_current_a <= self.max_current_a
    }
}

#[derive(Clone, Copy)]
struct MotionConfig {
    feedback_timeout: Duration, //电机反馈超时时间
    prepare_timeout: Duration,
    settle_time: Duration,
    settle_angle_rad: f32, //允许回正的角度误差
    settle_speed_rpm: f32,
    stick_deadzone: f32, //摇杆输入归一化后，小于这个范围的微小偏移会被当成零处理
    yaw: AxisConfig, //yaw轴配置，包含这一轴的角度范围、方向、速度、电流限幅登
    pitch: AxisConfig,
}

impl MotionConfig {
    fn is_valid(self) -> bool {
        self.feedback_timeout > Duration::from_millis(0)
            && self.prepare_timeout > Duration::from_millis(0)
            && self.settle_time > Duration::from_millis(0)
            && self.settle_angle_rad.is_finite()
            && self.settle_angle_rad > 0.0
            && self.settle_speed_rpm.is_finite()
            && self.settle_speed_rpm > 0.0
            && self.stick_deadzone.is_finite()
            && (0.0..1.0).contains(&self.stick_deadzone)
            && self.yaw.is_valid()
            && self.pitch.is_valid()
    }
}

/// 因 CAN 发送队列已满而丢弃的指令帧数，例如没有电机应答时。
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);

/// 已配置电机的反馈帧中，长度或角度无效的帧数。
pub static INVALID_FEEDBACK: AtomicU32 = AtomicU32::new(0);

/// 单个电机的反馈数据和电流指令。
#[derive(Default)]
pub struct Motor {
    /// 电机轴单圈角度，单位为 rad；不累计圈数。
    pub angle_rad: f32,
    pub speed_rpm: f32,
    /// 接收到的反馈电流原始值；电机手册未给出换算成安培的比例。
    pub current_raw: i16,
    /// 电流指令，单位为 A，由 `Control::transmit` 编码。
    pub command_current_a: f32,
}

impl Motor {
    /// 清除内存中的电流指令；发送失败时不能保证电机收到零指令。
    pub fn disable(&mut self) {
        self.command_current_a = 0.0;
    }

    pub fn read(&mut self, data: &[u8; 8]) -> bool {
        let angle_raw = u16::from_be_bytes([data[0], data[1]]);
        if angle_raw >= 8192 {
            return false;
        }
        // TODO：在实际电机上核实转速和电流反馈的有符号编码。
        self.current_raw = i16::from_be_bytes([data[4], data[5]]);
        self.angle_rad = angle_raw as f32 / 8192.0 * core::f32::consts::TAU;
        self.speed_rpm = i16::from_be_bytes([data[2], data[3]]) as f32;
        return true;
    }
}

fn log_invalid_feedback() {
    let count = INVALID_FEEDBACK.fetch_add(1, Ordering::Relaxed) + 1;
    if count == 1 || count.is_multiple_of(1000) {
        rtt_target::rprintln!("Invalid motor feedback: {}", count);
    }
}

/// 单个控制环的 PID 增益、积分和历史数据。
#[derive(Default)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral: f32,
    pub previous_error: Option<f32>,
    pub maxout: f32,
    pub integral_limit: f32,
}

#[derive(Debug, PartialEq, Eq)]
pub enum PidError {
    InvalidInput,
    InvalidConfiguration,
    NonFiniteResult,
}

impl Pid {
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = None;
    }

    /// `target` 和 `measured` 使用相同单位；`dt_s` 的单位为秒。
    /// 调用方通过增益确定输出单位。
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> Result<f32, PidError> {
        if !target.is_finite() || !measured.is_finite() || !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(PidError::InvalidInput);
        }
        if !self.kp.is_finite()
            || !self.ki.is_finite()
            || !self.kd.is_finite()
            || !self.integral_limit.is_finite()
            || self.integral_limit < 0.0
            || !self.maxout.is_finite()
            || self.maxout < 0.0
        {
            return Err(PidError::InvalidConfiguration);
        }
        let error = target - measured;
        let integral = self.integral + error * dt_s;
        let derivative = match self.previous_error {
            Some(previous) => (error - previous) / dt_s,
            None => 0.0,
        };
        if !error.is_finite() || !integral.is_finite() || !derivative.is_finite() {
            return Err(PidError::NonFiniteResult);
        }
        let integral = integral.clamp(-self.integral_limit, self.integral_limit);
        let output = self.kp * error + self.ki * integral + self.kd * derivative;
        if !output.is_finite() {
            return Err(PidError::NonFiniteResult);
        }
        self.integral = integral;
        self.previous_error = Some(error);
        Ok(output.clamp(-self.maxout, self.maxout))
    }
}

/// 云台当前阶段，决定允许的操作和输出。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// 零输出。等待有效遥控输入和两台电机的新鲜反馈，再等待一次新的使能请求。
    Waiting,
    /// 限速回正；两轴稳定后进入 `Running`。
    Prepare,
    /// yaw 以指定目标速度旋转，pitch 保持零电流。
    Normal,
    /// 左摇杆改变各轴的 `target_angle_rad`，双环控制跟踪目标。
    Running,
    /// 保持零输出，直到人工恢复。
    Error,
}

/// 一条云台轴：包含电机、目标值和串联的 PID 控制器。
#[derive(Default)]
pub struct Axis {
    pub motor: Motor,
    /// 相对于参考姿态的标定关节角，不是电机原始角度。
    // TODO：确认零位、方向和传动关系后，根据有效反馈填写。
    pub joint_angle_rad: Option<f32>,
    /// 位置环目标，单位为 rad。
    pub target_angle_rad: f32,
    /// 速度环目标，单位为 rpm，由位置环输出设定。
    pub target_rpm: f32,
    /// 位置环：输入单位为 rad，输出单位为 rpm。
    pub position_pid: Pid,
    /// 速度环：输入单位为 rpm，输出单位为 A。
    pub speed_pid: Pid,
}

#[derive(Debug)]
enum ControlError {
    InvalidState,
    InvalidAngle,
    InvalidTarget,
    Pid(PidError),
}

fn apply_deadzone(channel: i16, deadzone: f32) -> f32 {
    let input = (channel as f32 / 660.0).clamp(-1.0, 1.0);
    if input.abs() <= deadzone {
        0.0
    } else {
        input.signum() * (input.abs() - deadzone) / (1.0 - deadzone)
    }
}

impl Axis {
    fn update_axis(
        &mut self,
        state: State,
        input: f32,
        config: AxisConfig,
    ) -> Result<(), ControlError> {
        let angle = self.joint_angle_rad.ok_or(ControlError::InvalidAngle)?;
        if !angle.is_finite()
            || angle < config.min_angle_rad
            || angle > config.max_angle_rad
        {
            return Err(ControlError::InvalidAngle);
        }
        if !self.target_angle_rad.is_finite()
            || self.target_angle_rad < config.min_angle_rad
            || self.target_angle_rad > config.max_angle_rad
        {
            return Err(ControlError::InvalidTarget);
        }

        let (speed_limit, current_limit) = match state {
            State::Prepare => {
                let step = config.prepare_target_rate_rad_s * CONTROL_DT_S;
                if self.target_angle_rad.abs() <= step {
                    self.target_angle_rad = 0.0;
                } else {
                    self.target_angle_rad -= self.target_angle_rad.signum() * step;
                }
                (config.prepare_speed_rpm, config.prepare_current_a)
            }
            State::Running => {
                let target = self.target_angle_rad
                    + input * config.stick_direction * config.max_target_rate_rad_s * CONTROL_DT_S;
                if !target.is_finite() {
                    return Err(ControlError::InvalidTarget);
                }
                self.target_angle_rad = target.clamp(config.min_angle_rad, config.max_angle_rad);
                (config.max_speed_rpm, config.max_current_a)
            }
            _ => return Err(ControlError::InvalidState),
        };

        // 位置环输出电机转速；方向系数将关节正方向转换为电机正方向。
        let target_rpm = self
            .position_pid
            .calculate(self.target_angle_rad, angle, CONTROL_DT_S)
            .map_err(ControlError::Pid)?
            * config.motor_direction;
        self.target_rpm = target_rpm.clamp(-speed_limit, speed_limit);
        self.motor.command_current_a = self
            .speed_pid
            .calculate(self.target_rpm, self.motor.speed_rpm, CONTROL_DT_S)
            .map_err(ControlError::Pid)?
            .clamp(-current_limit, current_limit);
        Ok(())
    }
}

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
/// 各控制周期共用并持续保存的状态。
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,

    pub yaw_feedback_age: Option<u16>,
    pub pitch_feedback_age: Option<u16>,
    pub fault: Option<Fault>,
    previous_right_switch: Option<u8>,
    normal_armed: bool,
    yaw_feedback_time: Option<Instant>,
    pitch_feedback_time: Option<Instant>,
    prepare_started: Option<Instant>,
    settled_since: Option<Instant>,
}

/// 初始状态为 `Waiting`，且没有遥控输入。
// TODO：在实际教具上调试后，配置 PID 增益和安全限幅。
impl Default for Control {
    fn default() -> Self {
        let speed_pid = Pid {
            kp: 10.0,
            ki: 5.0,
            kd: 1.0,
            maxout: 100.0,
            integral_limit: 50.0,
            ..Pid::default()
        };
        let yaw = Axis {
            speed_pid,
            ..Axis::default()
        };
        Self {
            state: State::Waiting,
            yaw,
            pitch: Axis::default(),
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
            yaw_feedback_age: None,
            pitch_feedback_age: None,
            fault: None,
            previous_right_switch: None,
            normal_armed: false,
            yaw_feedback_time: None,
            pitch_feedback_time: None,
            prepare_started: None,
            settled_since: None,
        }
    }
}

impl Control {
    /// 清除内存中两台电机的电流指令。
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    fn wait(&mut self) {
        self.state = State::Waiting;
        self.disable_all();
        self.prepare_started = None;
        self.settled_since = None;
    }

    fn fail(&mut self, fault: Fault) {
        self.disable_all();
        self.fault = Some(fault);
        self.state = State::Error;
    }

    /// 每 1 ms 运行一次：读取遥控器和电机反馈，更新状态，再计算并发送输出。
    /// 没有遥控输入时强制进入 `Waiting`。
    pub fn tick(&mut self, can: &mut Can1) {
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();
        self.update();
        self.transmit(can);
    }

    /// 获取遥控任务发布的最新输入。收到无效帧，或连续 `REMOTE_SILENT_TICKS`
    /// 个周期没有输入后，清除遥控数据，由状态更新逻辑清零指令并返回 `Waiting`。
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = dm_mc02::REMOTE.try_take() {
            self.remote = remote;
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
    }

    /// 非阻塞地读取队列中的所有 CAN 帧。
    // 接收时间由 CAN 驱动在中断处理帧时记录。
    pub fn read_feedback(&mut self, can: &mut Can1) {
        if let Some(age) = self.yaw_feedback_age {
            self.yaw_feedback_age = Some(age.saturating_add(1));
        }
        if let Some(age) = self.pitch_feedback_age {
            self.pitch_feedback_age = Some(age.saturating_add(1));
        }
        let receiver = can.reader();
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => {
                    let received_at = envelope.ts;
                    let frame = envelope.frame;
                    if frame.header().rtr() {
                        continue;
                    }

                    let id = match frame.header().id() {
                        embedded_can::Id::Standard(id) => id.as_raw(),
                        embedded_can::Id::Extended(_) => continue,
                    };
                    let (axis, age, time) = match id {
                        YAW_FEEDBACK_ID => (
                            &mut self.yaw,
                            &mut self.yaw_feedback_age,
                            &mut self.yaw_feedback_time,
                        ),
                        PITCH_FEEDBACK_ID => (
                            &mut self.pitch,
                            &mut self.pitch_feedback_age,
                            &mut self.pitch_feedback_time,
                        ),
                        _ => continue,
                    };
                    let data: &[u8; 8] = match frame.data().try_into() {
                        Ok(data) => data,
                        Err(_) => {
                            log_invalid_feedback();
                            continue;
                        }
                    };
                    if axis.motor.read(data) {
                        *age = Some(0);
                        *time = Some(received_at);
                    } else {
                        log_invalid_feedback();
                    }
                }
                Err(error) => {
                    dm_mc02::log_can_error(error);
                    if self.state == State::Normal {
                        self.fail(Fault::CanFailure);
                    }
                }
            }
        }
    }

    fn feedback_fresh(time: Option<Instant>, now: Instant, timeout: Duration) -> bool {
        time.is_some_and(|time| now.saturating_duration_since(time) < timeout)
    }

    fn update_normal_mode(&mut self, now: Instant, enable_requested: bool) {
        if self.state != State::Normal && !enable_requested {
            return;
        }
        if !NORMAL_OUTPUT_ENABLED {
            if enable_requested {
                rtt_target::rprintln!("Normal disabled: output switch off");
            }
            self.wait();
            return;
        }
        if !Self::feedback_fresh(self.yaw_feedback_time, now, NORMAL_FEEDBACK_TIMEOUT) {
            if enable_requested || self.state == State::Normal {
                rtt_target::rprintln!("Normal disabled: yaw feedback missing or stale");
            }
            self.wait();
            return;
        }
        if self.yaw.motor.speed_rpm.abs() > NORMAL_MAX_SPEED_RPM {
            self.fail(Fault::NormalOverspeed);
            rtt_target::rprintln!("Fault: Normal overspeed");
            return;
        }
        if self.state == State::Waiting && enable_requested {
            self.yaw.speed_pid = Pid {
                kp: 0.005,
                ki: 0.0,
                kd: 0.0,
                maxout: NORMAL_CURRENT_A,
                integral_limit: 0.0,
                ..Pid::default()
            };
            self.yaw.target_rpm = 0.0;
            self.pitch.target_rpm = 0.0;
            self.state = State::Normal;
            rtt_target::rprintln!(
                "Normal started: yaw target {} rpm, limit {} A",
                NORMAL_SPEED_RPM,
                NORMAL_CURRENT_A
            );
        }
    }

    /// 根据遥控器、反馈和故障情况选择状态。
    // TODO：检测持续堵转和振荡，并记录故障原因。
    pub fn update_mode(&mut self) {
        let now = Instant::now();
        let right_switch = self.remote.map(|remote| remote.right_switch);
        let previous_switch = self.previous_right_switch;
        let enable_requested = previous_switch == Some(2) && right_switch == Some(3);
        self.previous_right_switch = right_switch;

        if self.fault.is_some() {
            self.state = State::Error;
        }

        if self.state == State::Error {
            self.normal_armed = false;
            self.disable_all();
            self.prepare_started = None;
            self.settled_since = None;
            // TODO：确认故障条件已解除后，实现明确的人工恢复操作；
            // 遥控器断联或拨杆位置变化时不得清除 `fault`。
            return;
        }

        match right_switch {
            Some(2) => {
                self.wait();
                self.normal_armed = true;
            }
            Some(3) => {
                if self.state == State::Normal {
                    self.normal_armed = false;
                    self.wait();
                } else {
                    self.update_recenter_mode(now, enable_requested);
                    if self.state == State::Running {
                        self.normal_armed = false;
                    }
                }
            }
            Some(1) => {
                let normal_requested = self.normal_armed
                    && matches!(self.state, State::Waiting | State::Prepare);
                self.normal_armed = false;
                if self.state == State::Waiting && !normal_requested && previous_switch != Some(1) {
                    rtt_target::rprintln!("Normal disabled: move right switch down first");
                }
                if normal_requested && self.state == State::Prepare {
                    self.wait();
                }
                if matches!(self.state, State::Waiting | State::Normal) {
                    self.update_normal_mode(now, normal_requested);
                } else {
                    self.wait();
                }
            }
            _ => {
                self.normal_armed = false;
                self.wait();
            }
        }
    }

    fn update_recenter_mode(&mut self, now: Instant, enable_requested: bool) {
        let config = match MOTION_CONFIG {
            Some(config) if MOTION_IMPLEMENTED && config.is_valid() => config,
            _ => {
                if enable_requested {
                    rtt_target::rprintln!("Motion disabled: configuration or safety checks incomplete");
                }
                self.wait();
                return;
            }
        };
        // 正式回正仍需验证接收时间戳在所有停顿情况下的可靠性。
        if !Self::feedback_fresh(self.yaw_feedback_time, now, config.feedback_timeout)
            || !Self::feedback_fresh(self.pitch_feedback_time, now, config.feedback_timeout)
        {
            self.wait();
            return;
        }

        let angles_valid = self.yaw.joint_angle_rad.is_some_and(f32::is_finite)
            && self.pitch.joint_angle_rad.is_some_and(f32::is_finite);
        if !angles_valid {
            if enable_requested || self.state != State::Waiting {
                rtt_target::rprintln!("Motion disabled: joint angles not calibrated");
            }
            self.wait();
            return;
        }

        match self.state {
            State::Waiting => {
                if enable_requested {
                    let (Some(yaw_angle), Some(pitch_angle)) =
                        (self.yaw.joint_angle_rad, self.pitch.joint_angle_rad)
                    else {
                        self.disable_all();
                        return;
                    };
                    // 两轴目标从当前位置开始，回正时再逐步移动到零位。
                    self.yaw.target_angle_rad = yaw_angle;
                    self.pitch.target_angle_rad = pitch_angle;
                    self.yaw.target_rpm = 0.0;
                    self.pitch.target_rpm = 0.0;
                    self.yaw.position_pid.reset();
                    self.yaw.speed_pid = Pid {
                        kp: 10.0,
                        ki: 5.0,
                        kd: 1.0,
                        maxout: 100.0,
                        integral_limit: 50.0,
                        ..Pid::default()
                    };
                    self.pitch.position_pid.reset();
                    self.pitch.speed_pid.reset();
                    self.prepare_started = Some(now);
                    self.settled_since = None;
                    self.state = State::Prepare;
                }
            }
            State::Prepare => {
                let Some(started) = self.prepare_started else {
                    self.wait();
                    return;
                };
                if now.saturating_duration_since(started) >= config.prepare_timeout {
                    self.fail(Fault::PrepareTimeout);
                    rtt_target::rprintln!("Fault: recentering timed out");
                    return;
                }
                let axis_settled = |axis: &Axis| {
                    axis.joint_angle_rad.is_some_and(|angle| {
                        angle.is_finite()
                            && angle.abs() <= config.settle_angle_rad
                            && axis.target_angle_rad == 0.0
                    }) && axis.motor.speed_rpm.is_finite()
                        && axis.motor.speed_rpm.abs() <= config.settle_speed_rpm
                };
                if axis_settled(&self.yaw) && axis_settled(&self.pitch) {
                    let settled_since = *self.settled_since.get_or_insert(now);
                    if now.saturating_duration_since(settled_since) >= config.settle_time {
                        self.state = State::Running;
                    }
                } else {
                    self.settled_since = None;
                }
            }
            State::Running => {
                // TODO：堵转或振荡检测器触发时设置 `fault`。
            }
            State::Normal => self.wait(),
            State::Error => {
                self.disable_all();
            }
        }
    }

    /// 根据当前状态设置目标、运行各轴 PID 并写入电流指令。
    pub fn update(&mut self) {
        let switch = self.remote.map(|remote| remote.right_switch);
        let enabled = match self.state {
            State::Normal => switch == Some(1),
            State::Prepare | State::Running => switch == Some(3),
            _ => false,
        };
        if !enabled || self.fault.is_some() {
            self.disable_all();
            return;
        }

        let (yaw_result, pitch_result) = if self.state == State::Normal {
            self.pitch.target_rpm = 0.0;
            self.pitch.motor.disable();
            self.yaw.target_rpm = NORMAL_SPEED_RPM;
            let yaw_result = self.yaw
                .speed_pid
                .calculate(self.yaw.target_rpm, self.yaw.motor.speed_rpm, CONTROL_DT_S)
                .map(|current| {
                    self.yaw.motor.command_current_a =
                        current.clamp(-NORMAL_CURRENT_A, NORMAL_CURRENT_A);
                })
                .map_err(ControlError::Pid);
            (yaw_result, Ok(()))
        } else {
            let config = match MOTION_CONFIG {
                Some(config) if MOTION_IMPLEMENTED && config.is_valid() => config,
                _ => {
                    self.disable_all();
                    self.state = State::Waiting;
                    rtt_target::rprintln!("Motion disabled: configuration or safety checks incomplete");
                    return;
                }
            };
            let Some(remote) = self.remote else {
                self.disable_all();
                return;
            };
            let yaw_input = apply_deadzone(remote.channels[2], config.stick_deadzone);
            let pitch_input = apply_deadzone(remote.channels[3], config.stick_deadzone);
            (
                self.yaw.update_axis(self.state, yaw_input, config.yaw),
                self.pitch.update_axis(self.state, pitch_input, config.pitch),
            )
        };

        if let Err(error) = yaw_result.and(pitch_result) {
            self.fail(Fault::InvalidControl);
            match error {
                ControlError::Pid(error) => {
                    rtt_target::rprintln!("PID 故障：{:?}", error);
                }
                error => {
                    rtt_target::rprintln!("控制故障：{:?}", error);
                }
            }
        }
    }

    fn encode_current(current_a: f32) -> Option<[u8; 2]> {
        if !current_a.is_finite() {
            return None;
        }
        let limited = current_a.clamp(-3.0, 3.0);
        let raw = (limited / 3.0 * 16384.0) as i16;
        Some(raw.to_be_bytes())
    }

    /// 非阻塞地向 CAN1 队列写入组指令。队列满或总线故障时不能保证停止指令送达。
    pub fn transmit(&mut self, can: &mut Can1) {
        // TODO：启用输出前，确认电机 ID 和电流限幅，并处理队列中的旧指令。
        // CAN 发送失败时不能保证电机停止。
        const OUTPUT_ENABLED: bool = false;
        let mut data = [0u8; 8];
        // TODO13：按手册限幅、转换各轴的 `command_current_a`，
        // 并写入电机 ID 对应的字节；其他电机 ID 的字节保持为零。
        let normal_output = NORMAL_OUTPUT_ENABLED && self.state == State::Normal;
        if (normal_output || (OUTPUT_ENABLED
            && matches!(self.state, State::Prepare | State::Running)))
            && self.fault.is_none()
        {

            let yaw_bytes = Self::encode_current(self.yaw.motor.command_current_a);
            let pitch_bytes = Self::encode_current(self.pitch.motor.command_current_a);

            match (yaw_bytes, pitch_bytes){
                (Some(yaw), Some(pitch)) => {
                    data[0..2].copy_from_slice(&yaw);
                    data[2..4].copy_from_slice(&pitch);
                }
                _ => {
                    self.fail(Fault::InvalidCommand);
                    rtt_target::rprintln!("Invalid command detected");
                }
            }
        }

        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
            if matches!(self.state, State::Waiting | State::Normal) && self.fault.is_none() {
                self.fail(Fault::CanFailure);
                rtt_target::rprintln!("Fault: CAN transmit queue full; disconnect motor power if needed");
            }
        }
    }
}

// 初始化 RTT 和开发板，然后启动遥控器任务和控制任务。
#[embassy_executor::main]
async fn main(spawner: Spawner) { //Spawner是Embassy提供的任务启动器
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("Gimbal control started");
    rtt_target::rprintln!("hello!");
    //启动两个异步任务
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// 以 1 kHz 运行 `tick`，每次完成后喂看门狗。如果某个周期卡住，
// 看门狗会在约 100 ms 后复位 MCU。
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    let mut last_remote_debug = last;
    loop {
        ticker.next().await;
        // 发生停顿后（例如调试探针连接时暂停 20 ms），重新安排周期，
        // 避免连续补跑所有错过的周期。
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
            if control.state == State::Normal {
                control.fail(Fault::ControlTiming);
                rtt_target::rprintln!("Fault: Normal control loop delayed");
            }
        }
        last = now;
        control.tick(&mut can);
        watchdog.pet();
        if REMOTE_DEBUG_ENABLED
            && control.state != State::Normal
            && now.saturating_duration_since(last_remote_debug) >= REMOTE_DEBUG_INTERVAL
        {
            last_remote_debug = now;
            rtt_target::rprintln!(
                "Status: state={:?}, fault={:?}, yaw_age={:?}, yaw_angle={}, yaw_rpm={}, yaw_target={}, yaw_current={}, tx_failed={}, invalid_feedback={}, can_errors={}",
                control.state,
                control.fault,
                control.yaw_feedback_age,
                control.yaw.motor.angle_rad,
                control.yaw.motor.speed_rpm,
                control.yaw.target_rpm,
                control.yaw.motor.command_current_a,
                TX_FAILED.load(Ordering::Relaxed),
                INVALID_FEEDBACK.load(Ordering::Relaxed),
                dm_mc02::CAN_ERRORS.load(Ordering::Relaxed),
            );
            match control.remote {
                Some(remote) => {
                    rtt_target::rprintln!(
                        "Remote: channels={:?}, right_switch={}, left_switch={}, mouse={:?}, mouse_left={}, mouse_right={}, keys=0x{:04X}",
                        remote.channels,
                        remote.right_switch,
                        remote.left_switch,
                        remote.mouse,
                        remote.mouse_left,
                        remote.mouse_right,
                        remote.keys
                    );
                }
                None => {
                    rtt_target::rprintln!("Remote: no valid input");
                }
            }
        }
    }
}
