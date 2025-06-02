import numpy as np
from simpledrone.transferdata import DroneState, DroneCommands


class FlightController:
    def __init__(
        self,
        mixer_matrix,
        angle_pid = (4.0, 0.0, 0.0),
        rate_pid = (0.15, 0.0, 0.005),
        max_angle_deg: float = 45.0,
    ):
        self.mixer = mixer_matrix
        self.num_motors = mixer.shape[0]
        assert mixer.shape[1] == 4, "Mixer must be N×4 ([throttle, roll, pitch, yaw])"
        assert self.num_motors == len(motors), "Mixer rows must match motor count"

        self.angle_pids = [PID(*angle_pid) for _ in range(3)]  # roll, pitch, yaw (outer)
        self.rate_pids = [PID(*rate_pid) for _ in range(3)]  # roll, pitch, yaw (inner)
        self.max_angle_rad = math.radians(max_angle_deg)

        self.attitude_filter = Mahony()
        self.imu = IMU()

    def update(self, dt: float, drone_state: DroneState, cmds: DroneCommands) -> np.ndarray:
        accel, gyro = self.imu.read(drone_state, dt)
        roll, pitch, yaw = self.attitude_filter.update(accel, gyro, dt)
        desired_roll = cmds[0] * self.max_angle_rad
        desired_pitch = cmds[1] * self.max_angle_rad
        desired_yaw = 0.0
        desired_angles = (desired_roll, desired_pitch, desired_yaw)

        rate_set = np.array([self.angle_pids[i].update(desired_angles[i] - ang, dt) for i, ang in enumerate((roll, pitch, yaw))])

        rate_set[2] += cmds[2]  # yaw stick is rate command directly

        rate_error = rate_set - np.array(gyro)
        torque_cmd = np.array([self.rate_pids[i].update(rate_error[i], dt) for i in range(3)])  # roll, pitch, yaw torque demands (‑1…+1 roughly)

        # faux
        thrust_cmd = np.clip(cmds[3], 0.0, 1.0)

        # Mixer: motor_cmd = M · [thrust, roll, pitch, yaw]^T
        mix_input = np.concatenate([[thrust_cmd], torque_cmd])
        motor_throttle = self.mixer @ mix_input
        motor_throttle = np.clip(motor_throttle, 0.0, 1.0)

        pack_voltage = self.battery.voltage
        rpm = np.array([motor.rpm_from_throttle(motor_throttle[i], pack_voltage) for i, motor in enumerate(self.motors)])
        return rpm


