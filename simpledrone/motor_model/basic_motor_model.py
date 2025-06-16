class BasicMotorModel:
    def __init__(self):
        self.a = 16.84
        self.b = -3.48

    def compute_thrust(self, throttle: float) -> float:
        return self.a * throttle + self.b