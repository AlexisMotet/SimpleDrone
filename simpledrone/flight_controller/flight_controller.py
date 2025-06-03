import numpy as np
from ahrs.filters import EKF
from pid import PID
from imu import IMU
from simpledrone.transferdata import DroneConfig, DroneState, DroneCommands
from simpledrone.utils import mixer

class FlightController:
    def __init__(self, cfg: DroneConfig, imu: IMU, angle_pid=(4.0, 0.0, 0.0), rate_pid=(0.15, 0.0, 0.005), max_angle_deg: float = 45.0):
        self.cfg = cfg
        self.imu = imu
        self.mixer = mixer.compute_mixer_matrix(cfg)
        self.angle_pids = [PID(*angle_pid) for _ in range(3)]
        self.rate_pids = [PID(*rate_pid) for _ in range(3)]
        self.max_angle_rad = np.deg2rad(max_angle_deg)
        self.attitude_filter = EKF()

    def update(self, dt: float, state: DroneState, cmds: DroneCommands) -> np.ndarray:
        accel, gyro = self.imu.read(state, dt)

        roll, pitch, yaw = self.attitude_filter.update(accel, gyro, dt)
        
        desired_pitch = cmds.pitch * self.max_angle_rad
        desired_roll = cmds.roll * self.max_angle_rad
        desired_yaw = 0.0
        desired_angles = (desired_roll, desired_pitch, desired_yaw)

        rate_set = np.array([self.angle_pids[i].update(desired_angles[i] - ang, dt) for i, ang in enumerate((roll, pitch, yaw))])
        rate_set[2] += cmds[2]  # yaw stick is rate command directly

        rate_error = rate_set - np.array(gyro)
        torque_cmd = np.array([self.rate_pids[i].update(rate_error[i], dt) for i in range(3)])

        thrust_cmd = self.motor_model.estimate_thrust(cmds.throttle)

        # Mixer: motor_cmd = M · [thrust, roll, pitch, yaw]^T
        mix_input = np.concatenate([[thrust_cmd], torque_cmd])
        motor_throttle = self.mixer @ mix_input
        motor_throttle = np.clip(motor_throttle, 0.0, 1.0)

        pack_voltage = self.battery.voltage
        rpm = np.array([motor.rpm_from_throttle(motor_throttle[i], pack_voltage) for i, motor in enumerate(self.motors)])
        return rpm
