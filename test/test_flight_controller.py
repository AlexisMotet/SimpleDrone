import unittest
import numpy as np
from simpledrone.data import RCInputs
from simpledrone.flight_controller.flight_controller import FlightController
from simpledrone.maths import quaternions
from simpledrone.data import State
from simpledrone.esc_motor_prop.esc_motor_prop import ESCMotorProp
from simpledrone.integrator.euler_integrator import EulerIntegrator
from simpledrone.frame import FPV4XSymmetric

class TestFlightController(unittest.TestCase):
    # def test_level_hover_zero_input(self):
    #     fc = FlightController()
    #     rc = RCInputs(throttle=0.5, roll=0.0, pitch=0.0, yaw_rate=0.0)

    #     # Orientation: perfectly level (no roll, pitch, yaw)
    #     q = quaternions.from_euler_angles(0.0, 0.0, 0.0)
    #     gyro = np.array([0.0, 0.0, 0.0])

    #     # Simulate at time t
    #     t = 0.01
    #     torque_x, torque_y, torque_z = fc.compute_torque_cmd(t, rc, q, gyro)

    #     self.assertAlmostEqual(torque_x, 0.0, delta=1e-3)
    #     self.assertAlmostEqual(torque_y, 0.0, delta=1e-3)
    #     self.assertAlmostEqual(torque_z, 0.0, delta=1e-3)

    # def test_roll_input(self):
    #     fc = FlightController()
    #     rc = RCInputs(throttle=0.5, roll=+1.0, pitch=0.0, yaw_rate=0.0)

    #     q = quaternions.from_euler_angles(0.0, 0.0, 0.0)
    #     gyro = np.array([0.0, 0.0, 0.0])

    #     t = 0.01
    #     torque_x, torque_y, torque_z = fc.compute_torque_cmd(t, rc, q, gyro)

    #     self.assertGreater(torque_x, 0.0)
    #     self.assertAlmostEqual(torque_y, 0.0, delta=1e-3)
    #     self.assertAlmostEqual(torque_z, 0.0, delta=1e-3)

    # def test_pitch_input(self):
    #     fc = FlightController()
    #     rc = RCInputs(throttle=0.5, roll=0.0, pitch=+1.0, yaw_rate=0.0)

    #     q = quaternions.from_euler_angles(0.0, 0.0, 0.0)
    #     gyro = np.array([0.0, 0.0, 0.0])

    #     t = 0.01
    #     torque_x, torque_y, torque_z = fc.compute_torque_cmd(t, rc, q, gyro)

    #     self.assertLess(torque_y, 0.0)
    #     self.assertAlmostEqual(torque_x, 0.0, delta=1e-3)
    #     self.assertAlmostEqual(torque_z, 0.0, delta=1e-3)

    # def test_yaw_input(self):
    #     fc = FlightController()
    #     rc = RCInputs(throttle=0.5, roll=0.0, pitch=0.0, yaw_rate=+1.0)

    #     q = quaternions.from_euler_angles(0.0, 0.0, 0.0)
    #     gyro = np.array([0.0, 0.0, 0.0])

    #     t = 0.01
    #     torque_x, torque_y, torque_z = fc.compute_torque_cmd(t, rc, q, gyro)

    #     self.assertGreater(torque_z, 0.0)
    #     self.assertAlmostEqual(torque_x, 0.0, delta=1e-3)
    #     self.assertAlmostEqual(torque_y, 0.0, delta=1e-3)

    # def test_roll_reaction(self):
    #     frame = FPV4XSymmetric()
    #     inertia = frame.get_inertia()

    #     fc = FlightController()
    #     rc = RCInputs(throttle=0.0, roll=0.0, pitch=0.0, yaw_rate=0.0)
    #     s = State()

    #     q = quaternions.from_euler_angles(np.deg2rad(20), 0.0, 0.0)
    #     s.orient_quat = q

    #     integrator = EulerIntegrator()

    #     angles = []

    #     t = 0.0
    #     while t < 0.01:
    #         t += 1e-3
    #         torque = fc.compute_torque_cmd(t, rc, s.orient_quat, s.ang_vel)

    #         s = integrator.integrate(t, inertia, s, 0.0, torque)


    #         angles.append(quaternions.to_euler_angles(s.orient_quat))
        
    #     import matplotlib.pyplot as plt

    #     plt.plot(range(len(angles)), [np.degrees(a[0]) for a in angles])
    #     plt.plot(range(len(angles)), [np.degrees(a[1]) for a in angles])
    #     plt.plot(range(len(angles)), [np.degrees(a[2]) for a in angles])

    #     plt.show()

    def test_roll_reaction(self):
        frame = FPV4XSymmetric()
        inertia = frame.get_inertia()
        fc = FlightController(angle_pid=(40, 0, 0), rate_pid=(0.01, 0, 0))
        rc = RCInputs(throttle=0.0, roll=1.0, pitch=0.0, yaw_rate=0.0)
        s = State()

        q = quaternions.from_euler_angles(0, 0.0, 0.0)
        s.orient_quat = q

        integrator = EulerIntegrator()

        angles = []

        times = []
        torques = []

        t = 0.0
        while t < 50:
            t += 1e-3
            thrust = 0.0
            torque = fc.compute_torque_cmd(t, rc, s.orient_quat, s.ang_vel)

            torques.append(torque[0])

            s = integrator.integrate(t, inertia, s, thrust, torque)


            angles.append(quaternions.to_euler_angles(s.orient_quat))

            times.append(t)
        
        import matplotlib.pyplot as plt

        plt.figure()

        plt.plot(times, [np.degrees(a[0]) for a in angles], label="roll")
        plt.plot(times, [np.degrees(a[1]) for a in angles], label="pitch")
        plt.plot(times, [np.degrees(a[2]) for a in angles], label="yaw")

        plt.legend()
        plt.grid()

        plt.figure()

        plt.plot(times, torques, label="torque roll")

        plt.legend()
        plt.grid()
        plt.show()

        motor = ESCMotorProp()

        print(frame.compute_torque_sanity(motor.estimate_thrust(1.0)))
        print(frame.compute_yaw_torque_sanity(motor.estimate_propeller_torque_from_rpm(1.0)))

        

if __name__ == "__main__":
    unittest.main()