import unittest
import numpy as np
from ahrs.filters import EKF
from simpledrone.attitude_filter.ahrs_ekf import ahrsEKF
from simpledrone.imu.imu import IMU
from simpledrone.data import State
from simpledrone.maths import quaternions


class TestAttitudeFilter(unittest.TestCase):
    def test_hover(self):
        attitude_filter = ahrsEKF()
        imu = IMU()
        state = State()

        state.orient_quat = quaternions.from_euler_angles(roll=0.0, pitch=np.deg2rad(50), yaw=0.0)

        roll_angles = []
        pitch_angles = []
        yaw_angles = []
        t = 0
        for _ in range(10000):

            accel_delta_free_fall, gyro = imu.read(t=t, accel_world=state.accel, quat_world2body=state.orient_quat, ang_vel_body=state.ang_vel)

            accel_std, gyro_std = imu.get_accel_std(), imu.get_gyro_std()

            estimate_quat = attitude_filter.estimate(t=t, accel=accel_delta_free_fall, gyro=gyro, accel_std=accel_std, gyro_std=gyro_std)

            roll, pitch, yaw = quaternions.to_euler_angles(estimate_quat)

            roll_angles.append(np.rad2deg(roll))
            pitch_angles.append(np.rad2deg(pitch))
            yaw_angles.append(np.rad2deg(yaw))

            t += 1e-3

        # WHY DO THE FILTER CONVERGE TO -10 DEGREES PITCH ???? 

        import matplotlib.pyplot as plt
        plt.plot(range(len(roll_angles)), roll_angles, label="roll")
        plt.plot(range(len(pitch_angles)), pitch_angles, label="pitch")
        plt.plot(range(len(yaw_angles)), yaw_angles, label="yaw")
        plt.grid()
        plt.legend()

        plt.show()