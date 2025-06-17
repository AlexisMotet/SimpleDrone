from typing import Tuple
import numpy as np
from simpledrone.data import RCInputs
from simpledrone.maths import quaternions
from simpledrone.pid.pid import PID

class FlightController:
    def __init__(self, roll_pid: Tuple[float] = (40.0, 0.0, 1.0), pitch_pid: Tuple[float] = (40.0, 0.0, 1.0),
                 p_pid: Tuple[float] = (0.001, 0.0, 0.001), q_pid: Tuple[float] = (0.001, 0.0, 0.001), r_pid: Tuple[float] = (0.001, 0.0, 0.001),
                 max_angle_deg: float = 45.0, max_yaw_rate_deg: float = 200.0):
        
        self.roll_pid = PID(*roll_pid)
        self.pitch_pid = PID(*pitch_pid)

        self.p_pid = PID(*p_pid)
        self.q_pid = PID(*q_pid)
        self.r_pid = PID(*r_pid)

        self.max_angle = np.deg2rad(max_angle_deg)
        self.max_yaw_rate = np.deg2rad(max_yaw_rate_deg)

        self.prev_t = 0.0

    def compute_torque_cmd(self, t: float, rc_inputs: RCInputs, estimated_orient: np.ndarray, 
                           gyro_output: np.ndarray, max_torques: Tuple[float]) -> Tuple[float]:
        dt = t - self.prev_t
        self.prev_t = t

        roll_desired = rc_inputs.roll * self.max_angle
        pitch_desired = rc_inputs.pitch * self.max_angle

        roll, pitch, _ = quaternions.to_euler_angles(estimated_orient)

        roll_rate_desired = self.roll_pid.update(roll_desired - roll, dt)
        pitch_rate_desired = self.pitch_pid.update(pitch_desired - pitch, dt)
        yaw_rate_desired = rc_inputs.yaw_rate * self.max_yaw_rate

        p, q, r = gyro_output

        torque_x = self.p_pid.update(roll_rate_desired - p, dt)
        torque_y = self.q_pid.update(pitch_rate_desired - q, dt)
        torque_z = self.r_pid.update(yaw_rate_desired - r, dt)

        print(max_torques)
        for dim, torque, max_torque in zip(["x", "y", "z"], [torque_x, torque_y, torque_z], max_torques):
            if torque >= max_torque:
                print(f"torque_{dim} ({torque}) is >= max_torque ({max_torque}), consider changing pid")

        torque_x = min(torque_x, max_torques[0])
        torque_y = min(torque_y, max_torques[1])
        torque_z = min(torque_z, max_torques[2])

        return (torque_x, torque_y, torque_z)

