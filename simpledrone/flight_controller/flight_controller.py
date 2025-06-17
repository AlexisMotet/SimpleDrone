from typing import Tuple
import numpy as np
from simpledrone.data import RCInputs
from simpledrone.maths import quaternions

class PID:
    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float = 1000.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.prev_error = None
        self.integral = 0.0

    def update(self, error: float, dt: float) -> float:
        self.integral += error * dt
        self.integral = min(max(self.integral, -self.integral_limit), self.integral_limit)
        derivative = (error - self.prev_error) / dt if self.prev_error and dt > 0.0 else 0.0
        print(derivative, error, self.prev_error, dt)
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class FlightController:
    def __init__(self, angle_pid: Tuple[float] = (5.0, 0.0, 1.0), rate_pid: Tuple[float] = (0.001, 0.0, 0.001), 
                 max_angle: int = 45, max_yaw_rate: int = 200):
        
        self.roll_pid = PID(*angle_pid)
        self.pitch_pid = PID(*angle_pid)
        self.yaw_pid = PID(*angle_pid)
        
        self.p_pid = PID(*rate_pid)
        self.q_pid = PID(*rate_pid)
        self.r_pid = PID(*rate_pid)

        self.max_angle = np.deg2rad(max_angle)
        self.max_yaw_rate = np.deg2rad(max_yaw_rate)

        self.prev_t = 0.0

    def compute_torque_cmd(self, t: float, rc_inputs: RCInputs, estimated_orient: np.ndarray, gyro_output: np.ndarray) -> Tuple[float]:
        dt = t - self.prev_t
        self.prev_t = t

        roll_desired = rc_inputs.roll * self.max_angle
        pitch_desired = rc_inputs.pitch * self.max_angle
        yaw_desired = 0.0

        roll, pitch, yaw = quaternions.to_euler_angles(estimated_orient)

        roll_rate_desired = self.roll_pid.update(roll_desired - roll, dt)
        pitch_rate_desired = self.pitch_pid.update(pitch_desired - pitch, dt)
        yaw_rate_desired = self.yaw_pid.update(yaw_desired - yaw, dt)
        yaw_rate_desired += rc_inputs.yaw_rate * self.max_yaw_rate

        p, q, r = gyro_output

        torque_x = self.p_pid.update(roll_rate_desired - p, dt)
        torque_y = self.q_pid.update(pitch_rate_desired - q, dt)
        torque_z = self.r_pid.update(yaw_rate_desired - r, dt)

        return (torque_x, torque_y, torque_z)