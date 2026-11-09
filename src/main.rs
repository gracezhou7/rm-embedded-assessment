#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;
mod fault;
mod pid;
mod pid_tuning;

use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};
use fault::{AxisFaultMonitor, Fault, OSCILLATION_DETECTION_DURATION};
use pid::{Pid, PidError};
use pid_tuning::RttPidTuner;
use tuning_studio_api::{watch, watch_mut};

// GM6020 电压模式组指令，适用于电机 ID 1～4。
const MOTOR_COMMAND_ID: u16 = 0x1FF;
// yaw 电机为 ID 1，pitch 电机为 ID 2。
const YAW_FEEDBACK_ID: u16 = 0x205;
const PITCH_FEEDBACK_ID: u16 = 0x206;
// 判定遥控器失联所需的连续无输入控制周期数。
const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);
const CONTROL_PERIOD_S: f32 = 0.001;
const PREPARE_ANGLE_TOLERANCE_RAD: f32 = 0.1;
const PREPARE_SPEED_TOLERANCE_RPM: f32 = 6.0;
const PREPARE_SETTLE_DURATION: Duration = Duration::from_millis(100);
const PREPARE_SWITCH_DWELL: Duration = Duration::from_millis(1000);
const YAW_MAX_TARGET_RATE_RAD_S: f32 = 20.0;
const PITCH_MAX_TARGET_RATE_RAD_S: f32 = 2.0;

const STICK_DEADZONE: f32 = 0.05;
// channel 2 中位静止时约偏大 100；按实际静止均值校准。
const YAW_CHANNEL_OFFSET: i16 = 100;
// 遥控器调试日志的开关和输出间隔。
const REMOTE_DEBUG_ENABLED: bool = true;
const REMOTE_DEBUG_INTERVAL: Duration = Duration::from_millis(500);
//状态使能。
const PREPARE_OUTPUT_ENABLED: bool = true;
const NORMAL_OUTPUT_ENABLED: bool = false;
const RUNNING_OUTPUT_ENABLED: bool = true;
// 电压给定值为协议原始数值，不是伏特。
const MAX_VOLTAGE_COMMAND: f32 = 25000.0;
// 临时阈值，需要结合现场反馈间隔和接收时序确认。
const NORMAL_FEEDBACK_TIMEOUT: Duration = Duration::from_millis(100);

//零位姿时电机角度
const YAW_CALIB_ANGLE_RAD: f32 = 2.5564;
const PITCH_CALIB_ANGLE_RAD: f32 = 3.1000;


/// 单个电机的反馈数据和电压指令。
#[derive(Default)]
pub struct Motor {
    /// 电机轴单圈角度，单位为 rad；不累计圈数。
    pub angle_rad: f32,
    pub speed_rpm: f32,
    /// 接收到的反馈电流原始值，换算比例尚待确认。
    pub current_raw: i16,
    /// 电压给定值（协议原始数值），由 `Control::transmit` 编码。
    pub command_voltage: f32,
}

impl Motor {
    /// 清除内存中的电压指令；发送失败时不能保证电机收到零指令。
    pub fn disable(&mut self) {
        self.command_voltage = 0.0;
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

/// 云台当前阶段，决定允许的操作和输出。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// 零输出。等待有效遥控输入和两台电机的新鲜反馈，再等待一次新的使能请求。
    Waiting,
    /// 回正模式，待实现。
    Prepare,
    /// yaw 以指定目标速度旋转，pitch 保持零电压指令。
    Normal,
    /// 遥控调试模式，待实现。
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
    /// 环绕修正后的目标角与反馈角之差，范围为 [-π, π]。
    pub angle_error_rad: f32,
    /// 速度环目标，单位为 rpm，由位置环输出设定。
    pub target_rpm: f32,
    /// 位置环：输入单位为 rad，输出单位为 rpm。
    pub position_pid: Pid,
    /// 速度环：输入单位为 rpm，输出电压给定值的协议原始数值。
    pub speed_pid: Pid,
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
    pub invalid_feedback: u32,
    pub tx_failed: u32,
    pub can_errors: u32,
    previous_right_switch: Option<u8>,
    armed: bool,
    prepare_switch_since: Option<Instant>,
    yaw_feedback_time: Option<Instant>,
    pitch_feedback_time: Option<Instant>,
    prepare_settled_since: Option<Instant>,
    yaw_fault_monitor: AxisFaultMonitor,
    pitch_fault_monitor: AxisFaultMonitor,
    previous_left_switch: Option<u8>,
    last_can_data: [u8; 8],
    last_can_queued: bool,
}

/// 初始状态为 `Waiting`，且没有遥控输入。
// TODO：在实际教具上调试后，配置 PID 增益和安全限幅。
impl Default for Control {
    fn default() -> Self {
        let speed_pid_yaw = Pid {
            kp: 100.0,
            ki: 0.01,
            kd: 0.1,
            maxout: 25000.0,
            integral_limit: 12000.0,
            ..Pid::default()
        };
        let pos_pid_yaw = Pid {
            kp: 100.0,
            ki: 0.01,
            kd: 0.5,
            maxout: 200.0,
            integral_limit: 2000.0,
            ..Pid::default()
        };
        let pos_pid_pitch: Pid = Pid {
            kp: 300.0,
            ki: 5.0,
            kd: 2.0,
            maxout: 200.0,
            integral_limit: 20.0,
            ..Pid::default()
        };
        let speed_pid_pitch = Pid {
            kp: 100.0,
            ki: 0.01,
            kd: 0.05,
            maxout: 25000.0,
            integral_limit: 250000.0,
            ..Pid::default()
        };
        let yaw = Axis {
            speed_pid: speed_pid_yaw,
            position_pid: pos_pid_yaw,
            ..Axis::default()
        };

        let pitch = Axis {
            speed_pid: speed_pid_pitch,
            position_pid: pos_pid_pitch,
            ..Axis::default()
        };

        Self {
            state: State::Waiting,
            yaw,
            pitch,
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
            yaw_feedback_age: None,
            pitch_feedback_age: None,
            fault: None,
            invalid_feedback: 0,
            tx_failed: 0,
            can_errors: 0,
            previous_right_switch: None,
            armed: false,
            prepare_switch_since: None,
            yaw_feedback_time: None,
            pitch_feedback_time: None,
            prepare_settled_since: None,
            yaw_fault_monitor: AxisFaultMonitor::default(),
            pitch_fault_monitor: AxisFaultMonitor::default(),
            previous_left_switch: None,
            last_can_data: [0; 8],
            last_can_queued: false,
        }
    }
}

impl Control {
    /// 清除内存中两台电机的电压指令。
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    fn wait(&mut self) {
        self.state = State::Waiting;
        self.prepare_switch_since = None;
        self.disable_all();
    }

    fn fail(&mut self, fault: Fault) {
        self.disable_all();
        self.fault = Some(fault);
        self.state = State::Error;
    }

    fn fail_axis(&mut self, fault: Fault, axis: &'static str) {
        self.fail(fault);
        rtt_target::rprintln!("Fault: {:?} detected on {} axis", fault, axis);
    }

    fn check_motion_faults(&mut self, now: Instant, output_enabled: bool) -> bool {
        if let Some(fault) = self
            .yaw_fault_monitor
            .update(&self.yaw, now, output_enabled)
        {
            if fault == Fault::Oscillating {
                self.fail(fault);
                rtt_target::rprintln!(
                    "Fault: {:?} detected on yaw axis at {:.2} Hz for at least {} ms",
                    fault,
                    self.yaw_fault_monitor.estimated_frequency_hz,
                    OSCILLATION_DETECTION_DURATION.as_millis()
                );
            } else {
                self.fail_axis(fault, "yaw");
            }
            return true;
        }
        if let Some(fault) = self
            .pitch_fault_monitor
            .update(&self.pitch, now, output_enabled)
        {
            if fault == Fault::Oscillating {
                self.fail(fault);
                rtt_target::rprintln!(
                    "Fault: {:?} detected on pitch axis at {:.2} Hz for at least {} ms",
                    fault,
                    self.pitch_fault_monitor.estimated_frequency_hz,
                    OSCILLATION_DETECTION_DURATION.as_millis()
                );
            } else {
                self.fail_axis(fault, "pitch");
            }
            return true;
        }
        false
    }

    fn recover_from_fault(&mut self) {
        self.fault = None;
        self.state = State::Running;
        self.prepare_switch_since = None;
        self.yaw.target_angle_rad = self.yaw.motor.angle_rad;
        self.pitch.target_angle_rad = self.pitch.motor.angle_rad;
        self.yaw.target_rpm = 0.0;
        self.pitch.target_rpm = 0.0;
        self.yaw.position_pid.reset();
        self.yaw.speed_pid.reset();
        self.pitch.position_pid.reset();
        self.pitch.speed_pid.reset();
        self.yaw_fault_monitor.reset();
        self.pitch_fault_monitor.reset();
        self.prepare_settled_since = None;
        rtt_target::rprintln!(
            "Fault recovery: Running at current angles yaw={} rad, pitch={} rad; targets reset",
            self.yaw.motor.angle_rad,
            self.pitch.motor.angle_rad
        );
    }

    fn log_invalid_feedback(&mut self) {
        self.invalid_feedback = self.invalid_feedback.wrapping_add(1);
        if self.invalid_feedback == 1 || self.invalid_feedback.is_multiple_of(1000) {
            rtt_target::rprintln!("Invalid motor feedback: {}", self.invalid_feedback);
        }
    }

    /// 每 1 ms 运行一次：读取遥控器和电机反馈，更新状态，再计算并发送输出。
    /// 没有遥控输入时强制进入 `Waiting`。
    pub fn tick(&mut self, can: &mut Can1) {
        self.read_remote();
        self.read_feedback(can);
        let previous_state = self.state;
        self.update_mode();
        self.update(previous_state);
        self.transmit(can);
    }

    /// 获取遥控任务发布的最新输入。收到无效帧，或连续 `REMOTE_SILENT_TICKS`
    /// 个周期没有输入后，清除遥控数据，由状态更新逻辑清零指令并返回 `Waiting`。
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1); //saturating表示超出数据类型能表示的边界时不会绕回去，而是保持在最大值
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
        while let Ok(received) = receiver.try_receive() { //try_receive会返回两层的Result，外层是CAN总线是否有帧，内层是帧是否有效，这里判断的是外层是否有帧
            match received  { //receive是内层的Result，表示帧是否有效，无效帧会返回Err，有效帧会返回Ok，每一种result(state)会包含一个参数
                Ok(envelope) => {
                    let received_at = envelope.ts;
                    let frame = envelope.frame;
                    if frame.header().rtr() { //Remote Transmission Request远程帧，表示请求发送数据而不是实际数据帧
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
                            self.log_invalid_feedback();
                            continue;
                        }
                    };
                    if axis.motor.read(data) {
                        *age = Some(0);
                        *time = Some(received_at);
                    } else {
                        self.log_invalid_feedback();
                    }
                }
                Err(error) => {
                    self.can_errors = self.can_errors.wrapping_add(1);
                    if self.can_errors == 1 || self.can_errors.is_multiple_of(1000) {
                        rtt_target::rprintln!("CAN1 bus error {}: {:?}", self.can_errors, error);
                    }
                }
            }
        }
    }

    fn feedback_fresh(time: Option<Instant>, now: Instant, timeout: Duration) -> bool {
        match time{
            Some(time) => {
                let elapsed = now.saturating_duration_since(time);
                return elapsed < timeout;
            }
            None => {
                return false;
            }
        }
    }

    fn update_axis_control(axis: &mut Axis) -> Result<(), PidError> {
        axis.angle_error_rad = axis.target_angle_rad - axis.motor.angle_rad;
        if axis.angle_error_rad > core::f32::consts::PI {
            axis.angle_error_rad -= core::f32::consts::TAU;
        } else if axis.angle_error_rad < -core::f32::consts::PI {
            axis.angle_error_rad += core::f32::consts::TAU;
        }
        let measured_angle = axis.target_angle_rad - axis.angle_error_rad;
        axis.target_rpm = axis
            .position_pid
            .calculate(axis.target_angle_rad, measured_angle)?;
        Self::update_speed_control(axis)?;
        Ok(())
    }

    fn update_speed_control(axis: &mut Axis) -> Result<(), PidError> {
        axis.motor.command_voltage = axis
            .speed_pid
            .calculate(axis.target_rpm, axis.motor.speed_rpm)?;
        Ok(())
    }

    fn apply_stick_deadzone(input: f32) -> f32 {
        let magnitude = input.abs();
        if magnitude <= STICK_DEADZONE {
            0.0
        } else {
            input.signum() * (magnitude - STICK_DEADZONE) / (1.0 - STICK_DEADZONE)
        }
    }

    fn corrected_channel(channel: i16, offset: i16) -> i16 {
        (i32::from(channel) - i32::from(offset)).clamp(-660, 660) as i16
    }

    fn update_normal_mode(&mut self, now: Instant, entered: bool) {
        if entered {
            self.yaw.speed_pid.reset();
            self.pitch.speed_pid.reset();
            self.yaw_fault_monitor.reset();
            self.pitch_fault_monitor.reset();
        }

        if !Self::feedback_fresh(self.yaw_feedback_time, now, NORMAL_FEEDBACK_TIMEOUT)
            || !Self::feedback_fresh(self.pitch_feedback_time, now, NORMAL_FEEDBACK_TIMEOUT)
        {
            rtt_target::rprintln!("Normal disabled: motor feedback missing or stale");
            self.wait();
            return;
        }

        self.pitch.target_rpm = 30.0;
        self.yaw.target_rpm = 30.0;
        if Self::update_speed_control(&mut self.yaw).is_err()
            || Self::update_speed_control(&mut self.pitch).is_err()
        {
            self.fail(Fault::InvalidControl);
            rtt_target::rprintln!("Normal PID configuration or calculation error");
        }
    }

    /// 根据左摇杆积分更新角度目标，再通过位置环和速度环计算输出。
    pub fn update_running_mode(&mut self, now: Instant, entered: bool) {
        if !Self::feedback_fresh(self.yaw_feedback_time, now, NORMAL_FEEDBACK_TIMEOUT)
            || !Self::feedback_fresh(self.pitch_feedback_time, now, NORMAL_FEEDBACK_TIMEOUT)
        {
            self.wait();
            return;
        }

        if entered {
            self.yaw.target_angle_rad = self.yaw.motor.angle_rad;
            self.pitch.target_angle_rad = self.pitch.motor.angle_rad;
            self.yaw.position_pid.reset();
            self.yaw.speed_pid.reset();
            self.pitch.position_pid.reset();
            self.pitch.speed_pid.reset();
            self.yaw.target_rpm = 0.0;
            self.pitch.target_rpm = 0.0;
            self.yaw_fault_monitor.reset();
            self.pitch_fault_monitor.reset();
        }

        if let Some(remote) = self.remote {
            let yaw_channel = Self::corrected_channel(remote.channels[2], YAW_CHANNEL_OFFSET);
            let yaw_input = Self::apply_stick_deadzone(yaw_channel as f32 / 660.0);
            let pitch_input = Self::apply_stick_deadzone(remote.channels[3] as f32 / 660.0);
            self.yaw.target_angle_rad += yaw_input * YAW_MAX_TARGET_RATE_RAD_S * CONTROL_PERIOD_S;
            if self.yaw.target_angle_rad >= core::f32::consts::TAU {
                self.yaw.target_angle_rad -= core::f32::consts::TAU;
            } else if self.yaw.target_angle_rad < 0.0 {
                self.yaw.target_angle_rad += core::f32::consts::TAU;
            }
            self.pitch.target_angle_rad += pitch_input * PITCH_MAX_TARGET_RATE_RAD_S * CONTROL_PERIOD_S;
            if self.pitch.target_angle_rad >= core::f32::consts::TAU {
                self.pitch.target_angle_rad -= core::f32::consts::TAU;
            } else if self.pitch.target_angle_rad < 0.0 {
                self.pitch.target_angle_rad += core::f32::consts::TAU;
            }
        }

        if Self::update_axis_control(&mut self.yaw).is_err()
            || Self::update_axis_control(&mut self.pitch).is_err()
        {
            self.fail(Fault::InvalidControl);
            rtt_target::rprintln!("Fault: invalid PID calculation in Running");
            return;
        }
        self.check_motion_faults(now, RUNNING_OUTPUT_ENABLED);
    }

    // 回正到两个电机的原始单圈角度零位。
    pub fn update_prepare_mode(&mut self, now: Instant, entered: bool) {
        if entered {
            self.yaw.target_angle_rad = YAW_CALIB_ANGLE_RAD;
            self.pitch.target_angle_rad = PITCH_CALIB_ANGLE_RAD;
            self.yaw.position_pid.reset();
            self.yaw.speed_pid.reset();
            self.pitch.position_pid.reset();
            self.pitch.speed_pid.reset();
            self.yaw.target_rpm = 0.0;
            self.pitch.target_rpm = 0.0;
            self.yaw_fault_monitor.reset();
            self.pitch_fault_monitor.reset();
            self.prepare_settled_since = None;
        }

        if !Self::feedback_fresh(self.yaw_feedback_time, now, NORMAL_FEEDBACK_TIMEOUT)
            || !Self::feedback_fresh(self.pitch_feedback_time, now, NORMAL_FEEDBACK_TIMEOUT)
        {
            self.wait();
            self.prepare_settled_since = None;
            return;
        }

        if Self::update_axis_control(&mut self.yaw).is_err()
            || Self::update_axis_control(&mut self.pitch).is_err()
        {
            self.fail(Fault::InvalidControl);
            rtt_target::rprintln!("Fault: invalid PID calculation in Prepare");
            return;
        }
        if self.check_motion_faults(now, PREPARE_OUTPUT_ENABLED) {
            return;
        }

        let settled = self.yaw.angle_error_rad.abs() <= PREPARE_ANGLE_TOLERANCE_RAD
            && self.pitch.angle_error_rad.abs() <= PREPARE_ANGLE_TOLERANCE_RAD
            && self.yaw.motor.speed_rpm.abs() <= PREPARE_SPEED_TOLERANCE_RPM
            && self.pitch.motor.speed_rpm.abs() <= PREPARE_SPEED_TOLERANCE_RPM;

        if !settled {
            self.prepare_settled_since = None;
            return;
        }

        let settled_since = *self.prepare_settled_since.get_or_insert(now);
        if now.saturating_duration_since(settled_since) >= PREPARE_SETTLE_DURATION {
            self.state = State::Running;
            self.prepare_settled_since = None;
        }
    }

    /// 根据遥控器、反馈和故障情况选择状态。
    // TODO：检测持续堵转和振荡，并记录故障原因。
    pub fn update_mode(&mut self) {
        let now = Instant::now();
        let right_switch = self.remote.map(|remote| remote.right_switch);
        let left_switch = self.remote.map(|remote| remote.left_switch);
        self.previous_right_switch = right_switch;
        let previous_left_switch = self.previous_left_switch;
        self.previous_left_switch = left_switch;

        if self.state == State::Error {
            self.armed = false;
            self.prepare_switch_since = None;
            let sticks_centered = self.remote.is_some_and(|remote| {
                Self::corrected_channel(remote.channels[2], YAW_CHANNEL_OFFSET).abs()
                    <= (STICK_DEADZONE * 660.0) as i16
                    && remote.channels[3].abs() <= (STICK_DEADZONE * 660.0) as i16
            });
            if right_switch == Some(3)
                && left_switch == Some(1)
                && previous_left_switch == Some(2)
                && sticks_centered
            {
                self.recover_from_fault();
            }
            return;
        }

        match right_switch {
            Some(2) => {
                self.state = State::Waiting;
                self.armed = true;
                self.prepare_switch_since = None;
            }
            Some(3) => {
                if self.state == State::Waiting && self.armed {
                    let since = *self.prepare_switch_since.get_or_insert(now);
                    if now.saturating_duration_since(since) >= PREPARE_SWITCH_DWELL {
                        self.state = State::Prepare;
                        self.armed = false;
                        self.prepare_switch_since = None;
                    }
                } else if self.state == State::Normal {
                    self.state = State::Waiting;
                    self.prepare_switch_since = None;
                }
            }
            //（prepare切running逻辑是在update_prepare_mode中处理的）

            Some(1) => {
                let normal_requested = self.armed;
                self.armed = false;
                self.prepare_switch_since = None;
                match self.state {
                    State::Waiting if normal_requested => self.state = State::Normal,
                    State::Prepare | State::Running => self.state = State::Waiting,
                    State::Waiting | State::Normal => {}
                    State::Error => {}
                }
            }
            _ => {
                self.armed = false;
                self.prepare_switch_since = None;
                self.state = State::Waiting;
            }
        }
    }

    /// 根据当前状态设置目标、运行各轴 PID 并写入电压指令。
    pub fn update(&mut self, previous_state: State) {
        let now = Instant::now();

        match self.state {
            State::Normal => self.update_normal_mode(now, previous_state != State::Normal),
            State::Prepare => {
                self.update_prepare_mode(now, previous_state != State::Prepare);
            }
            State::Running => {
                self.update_running_mode(now, previous_state != State::Running);
            }
            State::Waiting | State::Error => self.disable_all(),
        }
    }

    fn encode_voltage(voltage: f32) -> Option<[u8; 2]> {
        if !voltage.is_finite() {
            return None;
        }
        let raw = voltage.clamp(-MAX_VOLTAGE_COMMAND, MAX_VOLTAGE_COMMAND) as i16;
        Some(raw.to_be_bytes())
    }

    /// 非阻塞地向 CAN1 队列写入组指令。队列满或总线故障时不能保证停止指令送达。
    pub fn transmit(&mut self, can: &mut Can1) {
        // TODO：启用输出前，确认电机 ID 和电压限幅，并处理队列中的旧指令。
        // CAN 发送失败时不能保证电机停止。
        let mut data = [0u8; 8];
        let output_enabled = match self.state {
            State::Prepare => PREPARE_OUTPUT_ENABLED,
            State::Normal => NORMAL_OUTPUT_ENABLED,
            State::Running => RUNNING_OUTPUT_ENABLED,
            State::Waiting | State::Error => false,
        };
        if output_enabled && self.fault.is_none() {
            let yaw_bytes = Self::encode_voltage(self.yaw.motor.command_voltage);
            let pitch_bytes = Self::encode_voltage(self.pitch.motor.command_voltage);

            match (yaw_bytes, pitch_bytes){
                (Some(yaw), Some(pitch)) => {
                    data[0..2].copy_from_slice(&yaw);
                    data[2..4].copy_from_slice(&pitch);
                }
                _ => {
                    //self.fail(Fault::InvalidCommand);
                    rtt_target::rprintln!("Invalid command detected");
                }
            }
        }

        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        self.last_can_data = data;
        self.last_can_queued = can.writer().try_write(frame).is_ok();
        if !self.last_can_queued {
            self.tx_failed = self.tx_failed.wrapping_add(1);
            if self.tx_failed == 1 || self.tx_failed.is_multiple_of(1000) {
                rtt_target::rprintln!("CAN transmit queue full: {}", self.tx_failed);
            }
        }
    }
}

// 初始化 RTT 和开发板，然后启动遥控器任务和控制任务。
#[embassy_executor::main]
async fn main(spawner: Spawner) { //Spawner是Embassy提供的任务启动器
    let rtt_channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 1024,
                mode: rtt_target::ChannelMode::NoBlockTrim,
                name: "Terminal"
            }
        }
        down: {
            0: {
                size: 256,
                name: "PID commands"
            }
        }
    };
    rtt_target::set_print_channel(rtt_channels.up.0);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("Gimbal control started");
    rtt_target::rprintln!("Send 'help' on RTT down channel 0 for PID commands");
    let pid_tuner = RttPidTuner::new(rtt_channels.down.0);
    //启动两个异步任务
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog, pid_tuner).unwrap());
}

// 以 1 kHz 运行 `tick`，每次完成后喂看门狗。如果某个周期卡住，
// 看门狗会在约 100 ms 后复位 MCU。
#[embassy_executor::task]
async fn control_task(
    mut can: Can1,
    mut watchdog: dm_mc02::Watchdog,
    mut pid_tuner: RttPidTuner,
) {
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
        pid_tuner.poll(&mut control);
        watch_mut!("yaw.position.kp": f32 = &mut control.yaw.position_pid.kp);
        watch_mut!("yaw.position.ki": f32 = &mut control.yaw.position_pid.ki);
        watch_mut!("yaw.position.kd": f32 = &mut control.yaw.position_pid.kd);
        watch_mut!("yaw.speed.kp": f32 = &mut control.yaw.speed_pid.kp);
        watch_mut!("yaw.speed.ki": f32 = &mut control.yaw.speed_pid.ki);
        watch_mut!("yaw.speed.kd": f32 = &mut control.yaw.speed_pid.kd);
        watch_mut!("pitch.position.kp": f32 = &mut control.pitch.position_pid.kp);
        watch_mut!("pitch.position.ki": f32 = &mut control.pitch.position_pid.ki);
        watch_mut!("pitch.position.kd": f32 = &mut control.pitch.position_pid.kd);
        watch_mut!("pitch.speed.kp": f32 = &mut control.pitch.speed_pid.kp);
        watch_mut!("pitch.speed.ki": f32 = &mut control.pitch.speed_pid.ki);
        watch_mut!("pitch.speed.kd": f32 = &mut control.pitch.speed_pid.kd);
        control.tick(&mut can);
        watch!("yaw.angle.target": f32 = control.yaw.target_angle_rad, unit = "rad");
        watch!("yaw.angle.measured": f32 = control.yaw.motor.angle_rad, unit = "rad");
        watch!("yaw.angle.error": f32 = control.yaw.angle_error_rad, unit = "rad");
        watch!("yaw.speed.target": f32 = control.yaw.target_rpm, unit = "rpm");
        watch!("yaw.speed.measured": f32 = control.yaw.motor.speed_rpm, unit = "rpm");
        watch!("yaw.voltage.command": f32 = control.yaw.motor.command_voltage);
        watch!("pitch.angle.target": f32 = control.pitch.target_angle_rad, unit = "rad");
        watch!("pitch.angle.measured": f32 = control.pitch.motor.angle_rad, unit = "rad");
        watch!("pitch.angle.error": f32 = control.pitch.angle_error_rad, unit = "rad");
        watch!("pitch.speed.target": f32 = control.pitch.target_rpm, unit = "rpm");
        watch!("pitch.speed.measured": f32 = control.pitch.motor.speed_rpm, unit = "rpm");
        watch!("pitch.voltage.command": f32 = control.pitch.motor.command_voltage);
        watchdog.pet();
        if REMOTE_DEBUG_ENABLED
            && now.saturating_duration_since(last_remote_debug) >= REMOTE_DEBUG_INTERVAL
        {
            last_remote_debug = now;
            rtt_target::rprintln!(
                "Status: state={:?}, fault={:?}, yaw_age={:?}, yaw_angle={}, yaw_error={}, yaw_rpm={}, yaw_target={}, yaw_voltage={}, tx_failed={}, invalid_feedback={}, can_errors={}, armed={}",
                control.state,
                control.fault,
                control.yaw_feedback_age,
                control.yaw.motor.angle_rad,
                control.yaw.angle_error_rad,
                control.yaw.motor.speed_rpm,
                control.yaw.target_rpm,
                control.yaw.motor.command_voltage,
                control.tx_failed,
                control.invalid_feedback,
                control.can_errors,
                control.armed,
            );
            rtt_target::rprintln!(
                "Pitch feedback: age={:?}, angle_rad={}, angle_error_rad={}, speed_rpm={}",
                control.pitch_feedback_age,
                control.pitch.motor.angle_rad,
                control.pitch.angle_error_rad,
                control.pitch.motor.speed_rpm,
            );
            rtt_target::rprintln!(
                "CAN TX attempt: id=0x{:03X}, data={:02X?}, queued={}",
                MOTOR_COMMAND_ID,
                control.last_can_data,
                control.last_can_queued,
            );
            match control.remote {
                Some(remote) => {
                    rtt_target::rprintln!(
                        "Remote: channels={:?}, raw_channels={:?}, right_switch={}, left_switch={}, mouse={:?}, mouse_left={}, mouse_right={}, keys=0x{:04X}",
                        remote.channels,
                        remote.channels.map(|channel| channel + 1024),
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
