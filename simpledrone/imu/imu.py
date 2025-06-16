import numpy as np
from simpledrone.utils import constants
from simpledrone.maths import quaternions as quat


class IMU:
    def __init__(self, accel_noise: float = 70.0, gyro_noise: float = 3.8, 
                 bandwidth: float = 100.0, update_freq: float = 1000.0):
        
        accel_noise_density = accel_noise * 1e-6 * constants.GRAVITY
        gyro_noise_density = np.deg2rad(gyro_noise * 1e-3)

        self.accel_std = accel_noise_density * np.sqrt(bandwidth)
        self.gyro_std = gyro_noise_density * np.sqrt(bandwidth)

        self.sample_period = 1.0 / update_freq
        self.prev_t = 0.0
        
        self.accel_meas = np.zeros(3)
        self.gyro_meas = np.zeros(3)

    def get_accel_std(self) -> float:
        return self.accel_std

    def get_gyro_std(self) -> float:
        return self.gyro_std

    def read(self, t: float, accel_world: np.ndarray, quat_world2body: np.ndarray, ang_vel_body:np.ndarray):
        
        if t - self.prev_t  < self.sample_period:
            return self.accel_meas, self.gyro_meas
        self.prev_t = t

        R_world2body = quat.to_rotation_matrix(quat_world2body)
        specific_force_body = R_world2body @ (accel_world - constants.GRAVITY_VECTOR_NED)

        self.accel_meas = specific_force_body + np.random.normal(0.0, self.accel_std, 3)
        self.gyro_meas = ang_vel_body + np.random.normal(0.0, self.gyro_std, 3)

        return self.accel_meas, self.gyro_meas
