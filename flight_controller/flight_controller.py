from __future__ import annotations
import math
import argparse
import random
from typing import List, Tuple, Dict, Any, Optional

import numpy as np
from data_classes import DroneState
import quaternions as quat


class PID:
    def __init__(self, kp: float, ki: float, kd: float, integ_limit: float = 1000.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integ_limit = integ_limit
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error: float, dt: float) -> float:
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integ_limit, self.integ_limit)
        derivative = (error - self.prev_error) / dt if dt > 0.0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class IMU:
    """https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf"""
    def __init__(self, accel_noise_ug_rtHz: float = 70.0, gyro_noise_mdps_rtHz: float = 3.8, 
                 bandwidth: float = 100.0, odr: float = 1000.0):
        accel_noise_density = accel_noise_ug_rtHz * 1e-6 * 9.81
        gyro_noise_density = gyro_noise_mdps_rtHz * 1e-3 * np.pi / 180
        self.gyro_std = gyro_noise_density * np.sqrt(bandwidth)
        self.accel_std = accel_noise_density * np.sqrt(bandwidth)
        self.sample_period = 1.0 / odr
        self.last_read = -np.inf
        self.accel = np.zeros(3)
        self.gyro = np.zeros(3)
        self.time = 0.0

    def read(self, dt: float, drone_state: DroneState) -> Tuple[np.ndarray, np.ndarray]:
        if self.time + dt < self.sample_period:
            return self.accel, self.gyro
        self.accel = drone_state.acceleration + np.random.normal(0.0, self.accel_std, 3)
        self.gyro = drone_state.angular_vel + np.random.normal(0.0, self.gyro_std, 3)
        return self.accel, self.gyro
    
class Mahony:
    def __init__(self, kp: float = 2.0, ki: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.integral = np.zeros(3)

    def update(self, gyro: Tuple[float, float, float], accel: Tuple[float, float, float], dt: float):
        gx, gy, gz = gyro
        ax, ay, az = accel
        q = self.quaternion

        # Normalise accelerometer
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-6:
            # free‑fall – gyro‑only
            q_dot = 0.5 * quat.multiply(q, np.array([0.0, gx, gy, gz]))
            self.quaternion = quat.normalise(q + q_dot * dt)
            return
        ax, ay, az = ax / norm, ay / norm, az / norm

        # Estimated gravity (body frame)
        vx, vy, vz = quat.rotate(np.array([0.0, 0.0, 1.0]), q)

        # Error – cross product
        error = np.array([
            ay * vz - az * vy,
            az * vx - ax * vz,
            ax * vy - ay * vx,
        ])

        # Integral term
        if self.ki > 0.0:
            self._integral += error * dt
        else:
            self._integral[:] = 0.0

        # Apply feedback
        gyro_corr = np.array([gx, gy, gz]) + self.kp * error + self.ki * self._integral

        # Integrate quaternion rate
        q_dot = 0.5 * quat.multiply(q, np.concatenate([[0.0], gyro_corr]))
        self.quaternion = quat.normalise(q + q_dot * dt)
        return quat.to_euler(self.quaternion)

# class FlightController:
#     def __init__(
#         self,
#         DroneParams,
#         angle_pid: Tuple[float, float, float] = (4.0, 0.0, 0.0),
#         rate_pid: Tuple[float, float, float] = (0.15, 0.0, 0.005),
#         max_angle_deg: float = 45.0,
#     ):
#         self.mixer = mixer.astype(float)
#         self.num_motors = mixer.shape[0]
#         assert mixer.shape[1] == 4, "Mixer must be N×4 ([throttle, roll, pitch, yaw])"
#         assert self.num_motors == len(motors), "Mixer rows must match motor count"

#         self.angle_pids = [PID(*angle_pid) for _ in range(3)]  # roll, pitch, yaw (outer)
#         self.rate_pids  = [PID(*rate_pid)  for _ in range(3)]  # roll, pitch, yaw (inner)
#         self.max_angle_rad = math.radians(max_angle_deg)

#         self.attitude_filter = Mahony()
#         self.imu = IMU()

#     def update(self, dt, drone_state: DroneState, rc_cmd: Tuple[float, float, float, float],
#         dt: float) -> np.ndarray:
#         accel, gyro = self.imu.read(drone_state, dt)
#         roll, pitch, yaw = self.attitude_filter.update(accel, gyro, dt)
#         desired_roll = rc_cmd[0] * self.max_angle_rad
#         desired_pitch = rc_cmd[1] * self.max_angle_rad
#         desired_yaw  = 0.0
#         desired_angles = (desired_roll, desired_pitch, desired_yaw)

#         rate_set = np.array([self.angle_pids[i].update(desired_angles[i] - ang, dt)
#             for i, ang in enumerate((roll, pitch, yaw))
#         ])

#         rate_set[2] += rc_cmd[2]  # yaw stick is rate command directly

#         rate_error = rate_set - np.array(gyro)
#         torque_cmd = np.array([
#             self.rate_pids[i].update(rate_error[i], dt)
#             for i in range(3)
#         ])  # roll, pitch, yaw torque demands (‑1…+1 roughly)

#         thrust_cmd = np.clip(rc_cmd[3], 0.0, 1.0)

#         # Mixer: motor_cmd = M · [thrust, roll, pitch, yaw]^T
#         mix_input = np.concatenate([[thrust_cmd], torque_cmd])
#         motor_throttle = self.mixer @ mix_input
#         motor_throttle = np.clip(motor_throttle, 0.0, 1.0)

#         pack_voltage = self.battery.voltage
#         rpm = np.array([
#             motor.rpm_from_throttle(motor_throttle[i], pack_voltage)
#             for i, motor in enumerate(self.motors)
#         ])
#         return rpm

if __name__ == "__main__":
    pass
    # motor = LinearThrustMotor(throttle_to_thrust_points=[(20.0, 200.0), (60.0, 700.0), (80.0, 1000.0)])

    # print(motor.compute_thrust_per_g(50.0))