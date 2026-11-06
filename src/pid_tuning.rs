use crate::{Control, Pid};

const INPUT_BUFFER_SIZE: usize = 32;
const COMMAND_BUFFER_SIZE: usize = 128;

pub struct RttPidTuner {
    channel: rtt_target::DownChannel,
    input_buffer: [u8; INPUT_BUFFER_SIZE],
    command_buffer: [u8; COMMAND_BUFFER_SIZE],
    command_len: usize,
    discarding_command: bool,
}

impl RttPidTuner {
    pub fn new(channel: rtt_target::DownChannel) -> Self {
        Self {
            channel,
            input_buffer: [0; INPUT_BUFFER_SIZE],
            command_buffer: [0; COMMAND_BUFFER_SIZE],
            command_len: 0,
            discarding_command: false,
        }
    }

    pub fn poll(&mut self, control: &mut Control) {
        let input_len = self.channel.read(&mut self.input_buffer);
        for &byte in &self.input_buffer[..input_len] {
            match byte {
                b'\r' => {}
                b'\n' => {
                    if self.discarding_command {
                        rtt_target::rprintln!("RTT command error: line exceeds 128 bytes");
                    } else if self.command_len > 0 {
                        match core::str::from_utf8(&self.command_buffer[..self.command_len]) {
                            Ok(command) => control.handle_rtt_command(command),
                            Err(_) => {
                                rtt_target::rprintln!(
                                    "RTT command error: command must be valid UTF-8/ASCII"
                                );
                            }
                        }
                    }
                    self.command_len = 0;
                    self.discarding_command = false;
                }
                _ if self.discarding_command => {}
                _ if self.command_len < self.command_buffer.len() => {
                    self.command_buffer[self.command_len] = byte;
                    self.command_len += 1;
                }
                _ => {
                    self.command_len = 0;
                    self.discarding_command = true;
                }
            }
        }
    }
}

impl Control {
    fn pid_mut(&mut self, axis: &str, loop_name: &str) -> Option<&mut Pid> {
        match (axis, loop_name) {
            ("yaw", "position") => Some(&mut self.yaw.position_pid),
            ("yaw", "speed") => Some(&mut self.yaw.speed_pid),
            ("pitch", "position") => Some(&mut self.pitch.position_pid),
            ("pitch", "speed") => Some(&mut self.pitch.speed_pid),
            _ => None,
        }
    }

    fn show_pid(&self, axis: &str, loop_name: &str, pid: &Pid) {
        rtt_target::rprintln!(
            "{} {} kp={} ki={} kd={} maxout={} integral_limit={}",
            axis,
            loop_name,
            pid.kp,
            pid.ki,
            pid.kd,
            pid.maxout,
            pid.integral_limit
        );
    }

    fn handle_rtt_command(&mut self, line: &str) {
        let mut parts = line.split_whitespace();
        let Some(command) = parts.next() else {
            return;
        };

        match command {
            "help" if parts.next().is_none() => {
                rtt_target::rprintln!(
                    "Commands: show | set <yaw|pitch> <position|speed> <kp> <ki> <kd> <maxout> <integral_limit>"
                );
            }
            "show" if parts.next().is_none() => {
                self.show_pid("yaw", "position", &self.yaw.position_pid);
                self.show_pid("yaw", "speed", &self.yaw.speed_pid);
                self.show_pid("pitch", "position", &self.pitch.position_pid);
                self.show_pid("pitch", "speed", &self.pitch.speed_pid);
            }
            "set" => {
                let Some(axis) = parts.next() else {
                    rtt_target::rprintln!("RTT command error: missing axis");
                    return;
                };
                let Some(loop_name) = parts.next() else {
                    rtt_target::rprintln!("RTT command error: missing loop name");
                    return;
                };
                let parsed = (|| {
                    let kp = parts.next()?.parse::<f32>().ok()?;
                    let ki = parts.next()?.parse::<f32>().ok()?;
                    let kd = parts.next()?.parse::<f32>().ok()?;
                    let maxout = parts.next()?.parse::<f32>().ok()?;
                    let integral_limit = parts.next()?.parse::<f32>().ok()?;
                    parts
                        .next()
                        .is_none()
                        .then_some((kp, ki, kd, maxout, integral_limit))
                })();
                let Some((kp, ki, kd, maxout, integral_limit)) = parsed else {
                    rtt_target::rprintln!(
                        "RTT command error: expected five numeric parameters; use 'help'"
                    );
                    return;
                };
                if !kp.is_finite()
                    || !ki.is_finite()
                    || !kd.is_finite()
                    || !maxout.is_finite()
                    || maxout < 0.0
                    || !integral_limit.is_finite()
                    || integral_limit < 0.0
                {
                    rtt_target::rprintln!(
                        "RTT command error: parameters must be finite and limits non-negative"
                    );
                    return;
                }
                let Some(pid) = self.pid_mut(axis, loop_name) else {
                    rtt_target::rprintln!("RTT command error: unknown axis or loop name");
                    return;
                };
                pid.kp = kp;
                pid.ki = ki;
                pid.kd = kd;
                pid.maxout = maxout;
                pid.integral_limit = integral_limit;
                pid.reset();
                rtt_target::rprintln!("Updated {} {} PID", axis, loop_name);
            }
            _ => {
                rtt_target::rprintln!("RTT command error: unknown command; use 'help'");
            }
        }
    }
}
