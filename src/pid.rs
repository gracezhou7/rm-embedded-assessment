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

    /// `target` 和 `measured` 使用相同单位；积分和微分按每次调用更新。
    /// 调用方通过增益确定输出单位，控制周期改变后需要重新调参。
    pub fn calculate(&mut self, target: f32, measured: f32) -> Result<f32, PidError> {
        if !target.is_finite() || !measured.is_finite() {
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
        let integral = self.integral + error;
        let derivative = match self.previous_error {
            Some(previous) => error - previous,
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

#[cfg(test)]
mod tests {
    use super::Pid;

    #[test]
    fn integral_accumulates_error_per_call() {
        let mut pid = Pid {
            ki: 2.0,
            maxout: 100.0,
            integral_limit: 10.0,
            ..Pid::default()
        };

        assert_eq!(pid.calculate(3.0, 0.0), Ok(6.0));
        assert_eq!(pid.calculate(3.0, 0.0), Ok(12.0));
        assert_eq!(pid.integral, 6.0);
    }

    #[test]
    fn derivative_uses_error_difference_per_call() {
        let mut pid = Pid {
            kd: 2.0,
            maxout: 100.0,
            integral_limit: 0.0,
            ..Pid::default()
        };

        assert_eq!(pid.calculate(3.0, 0.0), Ok(0.0));
        assert_eq!(pid.calculate(5.0, 0.0), Ok(4.0));
    }
}
