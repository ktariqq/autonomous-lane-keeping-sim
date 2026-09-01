"""PID controller with integral anti-windup (clamped + saturation-aware)
and a low-pass filtered derivative term to avoid steering chatter from
noisy lane-error estimates."""


class PIDController:
    def __init__(self, kp, ki, kd, out_limit, i_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.i_limit = i_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_deriv = 0.0
        self.p_term = self.i_term = self.d_term = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_deriv = 0.0

    def update(self, error, dt):
        dt = max(dt, 1e-4)

        self.p_term = self.kp * error

        self._integral += error * dt
        self._integral = max(-self.i_limit, min(self.i_limit, self._integral))
        self.i_term = self.ki * self._integral

        raw_deriv = (error - self._prev_error) / dt
        alpha = 0.35
        self._filtered_deriv = alpha * raw_deriv + (1 - alpha) * self._filtered_deriv
        self.d_term = self.kd * self._filtered_deriv
        self._prev_error = error

        output = self.p_term + self.i_term + self.d_term

        if output > self.out_limit:
            output = self.out_limit
            if error > 0:
                self._integral -= error * dt
        elif output < -self.out_limit:
            output = -self.out_limit
            if error < 0:
                self._integral -= error * dt

        return output