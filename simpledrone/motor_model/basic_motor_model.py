import numpy as np

# from miniquadtestbench T-Motor F60 Pro II 2500kv

class BasicMotorModel:
    def __init__(self):
        self.throttle_values = np.array([0.00, 0.25, 0.50, 0.75, 1.00])
        self.thrust_g = np.array([46, 176, 536, 1012, 1548])
        self.rpm = np.array([5768, 10896, 18600, 25602, 31504])

    def compute_thrust(self, throttle: float) -> float:
        return self._interp(throttle, self.thrust_g) * 9.81 / 1000

    def compute_rpm(self, throttle: float) -> float:
        return self._interp(throttle, self.rpm)
    
    def _interp(self, throttle: float, curve: np.ndarray) -> float:
        return np.interp(np.clip(throttle, 0.0, 1.0), self.throttle_values, curve)
