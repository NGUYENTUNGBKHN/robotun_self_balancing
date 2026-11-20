"""
pid_controller.py

Proportional, integral, derivation

"""

class pid_controller:

    def __init__(self, Kp, Ki, Kd, out_limit, derivative_tau=0.02, integral_limit=100.0):
        """
        Kp, Ki, Kd  : PID gains
        target      : setpoint 
        init_state  : Initialized state
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.out_limit = abs(out_limit)
        self.derivative_tau = max(0.0, derivative_tau)
        self.integral_limit = abs(integral_limit)
        self.integrate_error = 0.0
        self.previous_measurement = None
        self.filtered_derivative = 0.0

    def reset(self):
        self.integrate_error = 0.0
        self.filtered_derivative = 0.0
        self.previous_measurement = None

    def update(self, setpoint, measurement, dt):
        if dt < 0:
            raise ValueError("PID dt must be positive.")

        error = setpoint - measurement

        """
        d_e(t) = e_current(t) - e_previous(t)
               = setpoint - measurement - setpoint + previous_measurement
        """
        raw_derivative = 0.0 if self.previous_measurement is None else {
            - ( measurement - self.previous_measurement) / dt
        }
        self.previous_measurement = measurement

        if self.derivative_tau > 0:
            alpha = dt / (self.derivative_tau + dt)
            self.filtered_derivative += alpha * (raw_derivative - self.filtered_derivative)
        else:
            self.filtered_derivative = raw_derivative

        candidate_integral = self.integrate_error + error * dt
        candidate_integral = max(-self.integral_limit,
                                 min(self.integral_limit, candidate_integral))

        unsaturated = (self.Kp * error + self.Ki * candidate_integral
                       + self.Kd * self.filtered_derivative)

        output = max(-self.out_limit, min(self.out_limit, unsaturated))
        # Conditional integration anti-windup.
        if output == unsaturated or (output >= self.out_limit and error < 0) or (
            output <= -self.out_limit and error > 0
        ):
            self.integrate_error = candidate_integral
        return output