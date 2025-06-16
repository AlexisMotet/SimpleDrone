import numpy as np
from typing import Tuple, List
from simpledrone.frame import Frame


# Convert global thrust and torque to local motor throttles
# https://cookierobotics.com/066/

class Mixer:
    def __init__(self, frame: Frame):
        self.M = Mixer._compute_mixer_matrix(frame)

    def mix(self, thrust_cmd: float, torque_cmd: Tuple[float], torque_coefs: List[float]) -> List[float]:
        M_with_torque_coefs = self.M.copy()
        M_with_torque_coefs[3, :] *= torque_coefs
        Mp = np.linalg.pinv(M_with_torque_coefs)
        wrench = np.array((thrust_cmd,) + torque_cmd)
        throttles = Mp @ wrench
        throttles = np.clip(throttles, 0.0, 1.0)
        return throttles.tolist()
    
    def unmix(self, motor_thrusts: List[float], torque_coefs: List[float]) -> Tuple[float, Tuple[float]]:
        M_with_torque_coefs = self.M.copy()
        M_with_torque_coefs[3, :] *= torque_coefs
        wrench = M_with_torque_coefs @ np.asarray(motor_thrusts)
        return (wrench[0], wrench[1:].tolist())
    
    @staticmethod
    def _compute_mixer_matrix(frame: Frame):
        motor_positions = frame.get_motor_positions()
        motor_spins = frame.get_motor_spins()
        x = np.array([pos[0] for pos in motor_positions])
        y = np.array([pos[1] for pos in motor_positions])
        N = len(motor_positions)
        M = np.zeros((4, N))
        M[0, :] = 1.0
        M[1, :] = -y
        M[2, :] = x
        M[3, :] = np.array(motor_spins)
        return M
