import numpy as np
from simpledrone.maths import quaternions as quat

class Mahony:
    def __init__(self, kp: float = 2.0, ki: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.integral = np.zeros(3)
        self.q_state = np.array([1.0, 0.0, 0.0, 0.0])
        

    def update(self, specific_force_body, gyro_body, dt):
        gravity_obs_body = specific_force_body / np.linalg.norm(specific_force_body)
        gravity_est_body = quat.rotate_vector(np.array([0.0, 0.0, 1.0]), self.q_state)
        
        error = np.cross(gravity_obs_body, gravity_est_body)

        self.integral += error * dt
        gyro_corr = gyro_body + self.kp * error + self.ki * self.integral

        q_dot = 0.5 * quat.derive(self.q_state, gyro_corr)
        self.q_state = quat.normalize(self.q_state + q_dot * dt)



if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # ---------- simulation settings -----------------------------------
    fs = 200.0  # Hz
    dt = 1.0 / fs
    duration = 5.0  # seconds
    n_steps = int(duration * fs)

    # trajectory: roll ±20°, pitch ±10° sinusoid, yaw 45 °/s ramp
    roll_amp = np.deg2rad(20.0)
    pitch_amp = np.deg2rad(10.0)
    yaw_rate = np.deg2rad(45.0)

    def euler_to_quat(r: float, p: float, y: float) -> np.ndarray:
        """ZXZ intrinsic? We'll use Z‑Y‑X (yaw‑pitch‑roll) order."""
        cy, sy = np.cos(y * 0.5), np.sin(y * 0.5)
        cp, sp = np.cos(p * 0.5), np.sin(p * 0.5)
        cr, sr = np.cos(r * 0.5), np.sin(r * 0.5)
        return np.array([
            cy * cp * cr + sy * sp * sr,  # w
            cy * cp * sr - sy * sp * cr,  # x
            cy * sp * cr + sy * cp * sr,  # y
            sy * cp * cr - cy * sp * sr,  # z
        ])

    # ---------- filter -------------------------------------------------
    filt = Mahony(kp=3.0, ki=1.0)

    # logs
    t_arr, roll_err, pitch_err, yaw_err = [], [], [], []

    # pre‑compute angular‑rate coefficients for speed
    roll_freq = 0.5  # Hz
    pitch_freq = 0.4  # Hz
    roll_omega = 2.0 * np.pi * roll_freq
    pitch_omega = 2.0 * np.pi * pitch_freq

    for k in range(n_steps):
        t = k * dt
        # true Euler angles
        roll_true = roll_amp * np.sin(roll_omega * t)
        pitch_true = pitch_amp * np.cos(pitch_omega * t)
        yaw_true = yaw_rate * t

        # Euler time derivatives
        roll_dot = roll_amp * roll_omega * np.cos(roll_omega * t)
        pitch_dot = -pitch_amp * pitch_omega * np.sin(pitch_omega * t)
        yaw_dot = yaw_rate

        # convert Euler‑rates to body angular velocity (ZYX convention)
        wx = roll_dot - np.sin(pitch_true) * yaw_dot
        wy = pitch_dot * np.cos(roll_true) + yaw_dot * np.sin(roll_true) * np.cos(pitch_true)
        wz = -pitch_dot * np.sin(roll_true) + yaw_dot * np.cos(roll_true) * np.cos(pitch_true)
        omega_b = np.array([wx, wy, wz])

        # true attitude quaternion & gravity direction
        q_true = quat.normalize(euler_to_quat(roll_true, pitch_true, yaw_true))
        R_wb = quat.to_rotation_matrix(q_true)
        g_body = R_wb @ np.array([0.0, 0.0, 9.81])

        # Mahony update
        filt.update(specific_force_body=g_body, gyro_body=omega_b, dt=dt)

        # error logging
        r_est, p_est, y_est = quat.to_euler_angles(filt.q_state)
        t_arr.append(t)
        roll_err.append(np.rad2deg(r_est - roll_true))
        pitch_err.append(np.rad2deg(p_est - pitch_true))
        yaw_err.append(np.rad2deg(y_est - yaw_true))

    # ---------- plot ---------------------------------------------------
    fig, ax = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    ax[0].plot(t_arr, roll_err)
    ax[0].set_ylabel("Roll error (°)")
    ax[1].plot(t_arr, pitch_err)
    ax[1].set_ylabel("Pitch error (°)")
    ax[2].plot(t_arr, yaw_err)
    ax[2].set_ylabel("Yaw error (°)")
    ax[2].set_xlabel("Time (s)")
    fig.suptitle("Mahony filter attitude error — noiseless sensors")
    plt.tight_layout()
    plt.show()
    
