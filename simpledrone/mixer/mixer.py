import numpy as np
from typing import List
from simpledrone.data import Geometry

class Mixer:
    def __init__(self, geometry: Geometry, torque2thrust_coef: float):
        self.Mp, _ = Mixer.compute_mixer_matrix(geometry, torque2thrust_coef)

    def get_num_motors(self):
        return self.Mp.shape[1]

    def mix(self, thrust_cmd: np.ndarray, torque_cmd: np.ndarray) -> np.ndarray:
        mix_input = np.concatenate([[thrust_cmd], torque_cmd])
        throttles = self.Mp @ mix_input
        throttles = np.clip(throttles, 0.0, 1.0)
        return throttles
    
    @staticmethod
    def compute_mixer_matrix(geometry: Geometry, torque2thrust_coef: float):
        if geometry.frame == "4+":
            angles = [0, 90, 180, 270]
            spin = [1, -1, 1, -1]
        elif geometry.frame == "4x":
            angles = [45, 135, 225, 315]
            spin = [1, -1, 1, -1]
        elif geometry.frame == "6x":
            angles = [30, 90, 150, 210, 270, 330]
            spin = [1, -1, 1, -1, 1, -1]
        else:
            raise ValueError(f"unsupported frame \"{geometry.frame}\"")
        x = geometry.arm_length * np.cos(np.deg2rad(angles))
        y = geometry.arm_length * np.sin(np.deg2rad(angles))
        N = len(angles)
        M = np.zeros((4, N))
        M[0, :] = 1.0
        M[1, :] = -y
        M[2, :] = x
        M[3, :] = np.array(spin) * torque2thrust_coef
        Mp = np.linalg.pinv(M)
        return Mp, M