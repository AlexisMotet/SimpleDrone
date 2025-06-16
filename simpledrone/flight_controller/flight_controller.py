import numpy as np
from simpledrone.data import RCInputs
from simpledrone.maths import quaternions

class PID:
    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float = 1000.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error: float, dt: float) -> float:
        self.integral += error * dt
        self.integral = min(max(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.prev_error) / dt if dt > 0.0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class FlightController:
    def __init__(self, angle_pid=(4.0, 0.0, 0.0), rate_pid=(0.15, 0.0, 0.005), max_angle_deg=45.0, max_yaw_rate_deg=200.0):
        self.roll_pid = PID(*angle_pid)
        self.pitch_pid = PID(*angle_pid)
        self.yaw_pid = PID(*angle_pid)
        
        self.p_pid = PID(*rate_pid)
        self.q_pid = PID(*rate_pid)
        self.r_pid = PID(*rate_pid)

        self.max_angle = np.deg2rad(max_angle_deg)
        self.max_yaw_rate = np.deg2rad(max_yaw_rate_deg)

        self.last_t = 0.0

    def compute_torque_cmd(self, t: float, rc_inputs: RCInputs, orient_quat: np.ndarray, gyro_output: np.ndarray) -> np.ndarray:
        dt = t - self.last_t
        self.last_t = t

        roll_desired = rc_inputs.roll * self.max_angle
        pitch_desired = rc_inputs.pitch * self.max_angle
        yaw_desired = 0.0

        roll, pitch, yaw = quaternions.to_euler_angles(orient_quat)

        roll_rate_desired = self.roll_pid.update(roll_desired - roll, dt)
        pitch_rate_desired = self.pitch_pid.update(pitch_desired - pitch, dt)
        yaw_rate_desired = self.yaw_pid.update(yaw_desired - yaw, dt)

        yaw_rate_desired += rc_inputs.yaw_rate * self.max_yaw_rate

        p, q, r = gyro_output

        torque_x = self.p_pid.update(roll_rate_desired - p, dt)
        torque_y = self.q_pid.update(pitch_rate_desired - q, dt)
        torque_z = self.r_pid.update(yaw_rate_desired - r, dt)

        return np.array([torque_x, torque_y, torque_z])