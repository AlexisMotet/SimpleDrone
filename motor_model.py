import numpy as np
from numpy.typing import NDArray

class MotorModel:
    def __init__(self, n_motors: int, tau: float = 0.02, rpm_idle: float = 1000.0, rpm_max: float = 20000.0):
        self.rpm: NDArray[np.float64] = np.full(n_motors, rpm_idle)
        self.target: NDArray[np.float64] = self.rpm.copy()
        self.tau = tau
        self.rpm_idle = rpm_idle
        self.rpm_max = rpm_max

    def set_target(self, rpm_cmd: NDArray[np.float64]):
        self.target = np.clip(rpm_cmd, self.rpm_idle, self.rpm_max)

    def update(self, dt: float) -> NDArray[np.float64]:
        alpha = dt / (self.tau + 1e-6)
        self.rpm += alpha * (self.target - self.rpm)
        return self.rpm