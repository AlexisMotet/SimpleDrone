import unittest
import numpy as np
from simpledrone.imu.imu import IMU
from simpledrone.data import RCInputs, State
from simpledrone.integrator.euler_integrator import EulerIntegrator
from simpledrone.frame import FPV4XSymmetric
from simpledrone.maths import quaternions

class TestIntegration(unittest.TestCase):
    def test_free_fall(self):
        frame = FPV4XSymmetric()
        inertia = frame.get_inertia()
        state = State()
        euler_integrator = EulerIntegrator()


        t = 0.0
        while t < 1.0:
            t += 1e-3
            state = euler_integrator.integrate(t, inertia, state, thrust=0.0, torque=[0.0, 0.0, 0.0])
            

            np.testing.assert_array_equal(state.ang_vel, np.zeros_like(state.ang_vel))
            np.testing.assert_array_equal(state.orient_quat, np.array([1.0, 0.0, 0.0, 0.0]))
            self.assertEqual(state.pos[0], 0.0)
            self.assertEqual(state.pos[1], 0.0)
            np.testing.assert_array_equal(state.accel, np.array([0.0, 0.0, 9.81]))

    def test_rotation(self):
        frame = FPV4XSymmetric()
        inertia = frame.get_inertia()
        state = State()
        euler_integrator = EulerIntegrator()

        state.ang_vel[0] = 0.1 # roll rotation

        t = 0.0
        while t < 1.0:
            t += 1e-3
            state = euler_integrator.integrate(t, inertia, state, thrust=0.0, torque=[0.0, 0.0, 0.0])

            roll, pitch, yaw = quaternions.to_euler_angles(state.orient_quat)

            self.assertNotEqual(roll, 0.0)
            self.assertEqual(pitch, 0.0)
            self.assertEqual(yaw, 0.0)

    def test_thrust_direction(self):
        frame = FPV4XSymmetric()
        inertia = frame.get_inertia()
        integrator = EulerIntegrator()
        state = State()

        hover_thrust = inertia.mass * 9.81
        t = 0.0

        state = integrator.integrate(t, inertia, state, thrust=hover_thrust + 2.0, torque=(0, 0, 0))
        accel_z = state.accel[2]

        self.assertLess(accel_z, 0.0) # z negative in NED convention


if __name__ == "__main__":#
    unittest.main()