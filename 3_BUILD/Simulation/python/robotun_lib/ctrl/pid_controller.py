class PIDController:
    def __init__(self, k_p, k_d, k_i, target, out_limit, u_min=0.0, deadband=1e-3):
        """
        k_p, k_d, k_i : PID gains
        target        : setpoint (e.g. upright angle in rad)
        out_limit     : max |output| (torque or PWM/voltage), used for saturation
        u_min         : static friction compensation (~70-90% of the breakaway level)
        deadband      : |error| below this value gets no friction compensation (avoids chattering)
        """
        self.Kp = k_p
        self.Kd = k_d
        self.Ki = k_i
        self.target = target
        self.out_limit = out_limit
        self.u_min = u_min
        self.deadband = deadband

        # Clamp the integral state so that Ki * integral never exceeds the output limit
        self.i_limit = out_limit / k_i if k_i > 0 else 0.0

        self.integral = 0.0
        self.last_value = None  # None until the first measurement arrives

    def reset(self):
        """Call when the robot falls or the motors are disabled, to drop stale integral state."""
        self.integral = 0.0
        self.last_value = None

    def set_target(self, target):
        self.target = target

    def get_control(self, value, dt, rate=None):
        """
        value : measured angle
        dt    : time step in seconds
        rate  : measured angular velocity (e.g. from the gyro, same sign convention as value).
                If None, the derivative is computed numerically from value.
        """
        error = self.target - value

        # Derivative on measurement (not on error) to avoid derivative kick on setpoint changes
        if rate is not None:
            derivative = -rate
        elif self.last_value is None:
            derivative = 0.0  # no spike on the first call
        else:
            derivative = -(value - self.last_value) / dt
        self.last_value = value

        # Candidate integral, clamped to its limit
        integral_new = self.integral + error * dt
        integral_new = max(-self.i_limit, min(self.i_limit, integral_new))

        u = self.Kp * error + self.Kd * derivative + self.Ki * integral_new

        # Static friction (deadzone) compensation: push past the breakaway level
        if self.u_min > 0.0 and abs(error) > self.deadband:
            direction = 1.0 if (u > 0 or (u == 0 and error > 0)) else -1.0
            u += self.u_min * direction

        # Output saturation
        u_sat = max(-self.out_limit, min(self.out_limit, u))

        # Conditional integration (anti-windup):
        # only accept the new integral if the output is not saturated,
        # or if the error is pulling the output back toward the unsaturated range
        if u == u_sat or error * u < 0:
            self.integral = integral_new

        return u_sat