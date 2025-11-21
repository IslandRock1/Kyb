
"""
This file is written by ChatGPT
Some very small modifications were made,
but this is almost exclusivly ChatGPT
"""

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, output_limits=(None, None), sample_time=0.0):
        """
        Initialize the PID controller.

        Kp, Ki, Kd: PID gains
        setpoint: desired target value
        output_limits: tuple (min_output, max_output) for limiting output
        sample_time: minimum time (seconds) between PID updates (0 = always update)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.setpoint = setpoint
        self.output_limits = output_limits
        self.sample_time = sample_time

        self._last_time = None
        self._last_error = 0.0
        self._integral = 0.0
        self._last_output = 0.0

    def reset(self):
        """Reset the PID controller state."""
        self._last_time = None
        self._last_error = 0.0
        self._integral = 0.0
        self._last_output = 0.0

    def update(self, measurement):
        """
        Compute new PID output based on a measurement.

        measurement: the current measured value
        current_time: optional timestamp (seconds). If None, time.time() will be used.
        """

        error = self.setpoint - measurement
        dt = self.sample_time

        # Proportional term
        P = self.Kp * error

        # Integral term (anti-windup via clamping)
        self._integral += error * dt
        I = self.Ki * self._integral

        # Derivative term
        derivative = (error - self._last_error) / dt
        D = self.Kd * derivative

        # PID output
        output = P + I + D

        # Output limits
        low, high = self.output_limits
        if low is not None:
            output = max(low, output)
        if high is not None:
            output = min(high, output)

        # Update stored values for next step
        self._last_output = output
        self._last_error = error

        return output
