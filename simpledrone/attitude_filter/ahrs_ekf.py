import numpy as np
from ahrs.filters import EKF
from simpledrone.maths import quaternions

class ahrsEKF:
    def __init__(self):
        self.filter = EKF(frame="NED")
        self.prev_t = 0.0
        self.prior_quat = np.array([1.0, 0.0, 0.0, 0.0]) # sensor -> world (see ahrs doc)

    def estimate(self, t: float, accel: np.ndarray, gyro: np.ndarray, 
                 accel_std: float, gyro_std: float) -> np.ndarray: # returns world -> sensor quaternion
        dt = t - self.prev_t
        assert dt >= 0.0
        self.prev_t = t
        self.filter.Dt = dt
        self.filter.noises = np.array([0.0, accel_std**2, 0.0])
        self.filter.g_noise = gyro_std**2
        self.prior_quat = self.filter.update(q=self.prior_quat, gyr=gyro, acc=accel)
    
    def get_estimated_orientation(self):
        return quaternions.conjugate(self.prior_quat)
