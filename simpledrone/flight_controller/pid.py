class PID:
    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float = 1000.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error: float, dt: float) -> float:
        self.integral += error * dt
        self.integral = min(max(self.integral, self.integral_limit) -self.integral_limit)
        derivative = (error - self.prev_error) / dt if dt > 0.0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative