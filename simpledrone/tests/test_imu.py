import unittest
import numpy as np
from simpledrone.imu.imu import IMU
from simpledrone.maths import quaternions as quat
from simpledrone.utils import constants


class TestIMU(unittest.TestCase):
    def test_thrust_convention(self):
        imu = IMU()

        # apply upward acceleration in world frame (NED, up = -Z)
        accel_world = np.array([0.0, 0.0, -2.0])

        quat_world2body = np.array([1.0, 0.0, 0.0, 0.0])
        ang_vel_body = np.zeros(3)

        accel_meas, _ = imu.read(0.001, accel_world, quat_world2body, ang_vel_body)

        # Expected specific force = accel_world - gravity → -2 - 9.8 = -11.8
        expected_force = np.array([0.0, 0.0, -11.8])
        np.testing.assert_allclose(accel_meas, expected_force, atol=1e-2)