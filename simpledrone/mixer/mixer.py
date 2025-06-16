import numpy as np
from typing import List

class Mixer:
    def __init__(self, geometry: str, arm_length: float, torque2thrust_coef: float):
        self.Mp = Mixer.compute_mixer_matrix(geometry, arm_length, torque2thrust_coef)

    def mix(self, thrust_cmd: np.ndarray, torque_cmd: np.ndarray) -> np.ndarray:
        mix_input = np.concatenate([[thrust_cmd], torque_cmd])
        throttles = self.Mp @ mix_input
        throttles = np.clip(throttles, 0.0, 1.0)
        return throttles
    
    @staticmethod
    def compute_mixer_matrix(geometry: str, arm_length: float, torque2thrust_coef: float):
        if geometry == "quad+":
            angles = [0, 90, 180, 270]
            spin = [1, -1, 1, -1]
        elif geometry == "quadx":
            angles = [45, 135, 225, 315]
            spin = [1, -1, 1, -1]
        elif geometry == "hexax":
            angles = [30, 90, 150, 210, 270, 330]
            spin = [1, -1, 1, -1, 1, -1]
        else:
            raise ValueError(f"unsupported geometry \"{geometry}\"")
        x = arm_length * np.cos(np.deg2rad(angles))
        y = arm_length * np.sin(np.deg2rad(angles))
        N = len(angles)
        M = np.zeros((4, N))
        M[0, :] = 1.0
        M[1, :] = -y
        M[2, :] = x
        M[3, :] = np.array(spin) * torque2thrust_coef
        Mp = np.linalg.pinv(M)
        return Mp, M