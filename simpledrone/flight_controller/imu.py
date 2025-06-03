import numpy as np
from simpledrone.utils import constants

# https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf


class IMU:
    GRAVITY_VECTOR = np.array([0, 0, -constants.GRAVITY])

    def __init__(self, accel_noise_ug_rtHz: float = 70.0, gyro_noise_mdps_rtHz: float = 3.8, bandwidth: float = 100.0, odr: float = 1000.0):
        accel_noise_density = accel_noise_ug_rtHz * 1e-6 * constants.GRAVITY
        gyro_noise_density = np.radians(gyro_noise_mdps_rtHz * 1e-3)

        self.accel_std = accel_noise_density * np.sqrt(bandwidth)
        self.gyro_std = gyro_noise_density * np.sqrt(bandwidth)

        self.sample_period = 1.0 / odr
        self.last_read_timestamp = 0.0

        self.accel_meas = np.zeros(3)
        self.gyro_meas = np.zeros(3)

    def read(self, timestamp: float, R_world_to_body, accel_world, angular_vel):
        dt = timestamp - self.last_read_timestamp
        if dt < self.sample_period:
            return self.accel_meas, self.gyro_meas
        self.last_read_timestamp = timestamp

        specific_force_body = R_world_to_body @ (accel_world - IMU.GRAVITY_VECTOR)

        self.accel_meas = specific_force_body + np.random.normal(0.0, self.accel_std, 3)
        self.gyro_meas = angular_vel + np.random.normal(0.0, self.gyro_std, 3)
        return self.accel_meas, self.gyro_meas


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    imu = IMU()
    t = 0.0
    gyro_output = []
    times = []
    for _ in range(1000):
        _, gyro = imu.read(t, np.eye(3), [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        gyro_output.append(gyro)
        times.append(t)
        t += 1.0 / 10000.0  # simulate a 10 000 Hz loop
    gyro_output = np.array(gyro_output)
    plt.figure()
    for i, label in enumerate(["x", "y", "z"]):
        plt.plot(times, gyro_output[:, i], label=f"gyro {label}")
    plt.legend()
    plt.xlabel("time (s)")
    plt.ylabel("gyro (rad/s)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()
