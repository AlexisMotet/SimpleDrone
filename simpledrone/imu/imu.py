import numpy as np
from simpledrone.utils import constants
from simpledrone.maths import quaternions
from typing import Tuple


class IMU:
    def __init__(self, accel_noise: float = 70.0, gyro_noise: float = 3.8, 
                 bandwidth: float = 100.0, update_freq: float = 1000.0):
        
        accel_noise_density = accel_noise * 1e-6 * constants.GRAVITY 
        gyro_noise_density = np.deg2rad(gyro_noise * 1e-3)

        self.accel_std = accel_noise_density * np.sqrt(bandwidth)
        self.gyro_std = gyro_noise_density * np.sqrt(bandwidth)

        self.sample_period = 1.0 / update_freq
        self.prev_t = 0.0
        
        self.accel_minus_gravity = np.zeros(3)
        self.gyro = np.zeros(3)

    def get_std(self) -> Tuple[float, float]:
        return self.accel_std, self.gyro_std
    
    def get_gyro(self) -> float:
        return self.gyro

    def read(self, t: float, accel_world: np.ndarray, quat_world_to_body: np.ndarray, ang_vel_body:np.ndarray):
        if t - self.prev_t  < self.sample_period:
            return self.accel_minus_gravity, self.gyro
        self.prev_t = t

        R_world_to_body = quaternions.to_rotation_matrix(quat_world_to_body)
        specific_force_body = R_world_to_body @ (accel_world - np.array([0, 0, constants.GRAVITY])) # +g because NED frame

        self.accel_minus_gravity = specific_force_body + np.random.normal(0.0, self.accel_std, 3)

        self.gyro = ang_vel_body + np.random.normal(0.0, self.gyro_std, 3)

        return self.accel_minus_gravity, self.gyro
