import numpy as np
from ahrs.filters import EKF
from simpledrone.maths import quaternions

class AHRS_EKF:
    def __init__(self):
        self.filter = EKF(frame="NED")
        self.last_t = 0.0

    def estimate(self, t: float, prior: np.ndarray, accel: np.ndarray, gyro: np.ndarray, 
                 accel_std: np.ndarray, gyro_std: np.ndarray) -> np.ndarray: 
        dt = self.last_t - t
        self.filter.Dt = dt
        self.filter.noises = np.array([0.0, accel_std**2, 0.0])
        self.filter.g_noise = gyro_std**2
        return self.filter.update(q=prior, gyr=gyro, acc=accel)