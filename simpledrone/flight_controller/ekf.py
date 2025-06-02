"""ekf_attitude.py – Minimal quaternion Extended Kalman Filter for IMU attitude

State vector
------------
    x = [ q_w, q_x, q_y, q_z,  b_gx, b_gy, b_gz ]ᵀ
where **q** is the body→world rotation (unit quaternion) and **b_g** is the
estimated gyro bias [rad s⁻¹].  The covariance is 7×7.

Prediction step
---------------
    ω_m   : measured gyro  (body)            [rad s⁻¹]
    ω     = ω_m − b_g                        # bias‑compensated rate
    q̇     = ½ q ⊗ [0, ω]
    ḃ_g  = 0  (bias random walk)

Update step (accelerometer only)
--------------------------------
    z_acc = specific force (body)             [m s⁻²]
    h(x)  = R_bw(q) · g_world                 # predicted gravity in body frame
    y     = z_acc/‖z_acc‖ − h(x)              # innovation (unit vectors)

`to_rotation_matrix` and `derive` are imported from quaternions.py.
"""
from __future__ import annotations
import numpy as np
from numpy.typing import NDArray
from simpledrone.mahts import quaternions as quat

GRAVITY_BASIS = np.array([0.0, 0.0, 1.0])  # +Z up (unit vector)

class AttitudeEKF:
    def __init__(
        self,
        q0: NDArray[np.floating] | None = None,
        gyro_bias_var: float = 1e-6,        # (rad/s)² process noise
        gyro_noise_var: float = 1e-4,       # (rad/s)²
        accel_noise_var: float = 1e-2,      # unit‑vector variance
    ) -> None:
        self.x = np.zeros(7)
        self.x[:4] = quat.normalise(q0) if q0 is not None else np.array([1.0, 0, 0, 0])
        self.P = np.eye(7) * 1e-3
        self.Q = np.diag([*([gyro_noise_var]*3), *([gyro_bias_var]*3)])  # in error‑state order
        self.R = np.eye(3) * accel_noise_var

    # ------------------------------------------------------------ prediction
    def predict(self, gyro_meas: NDArray[np.floating], dt: float) -> None:
        q = self.x[:4]
        b = self.x[4:]
        omega = gyro_meas - b
        # quaternion propagation
        q_dot = quat.derive(q, omega)
        q_new = quat.normalise(q + q_dot * dt)
        self.x[:4] = q_new
        # bias assumed constant

        # ---- EKF covariance propagation (error‑state 6×6) ------------
        F = np.eye(6)
        # attitude part: δq̇ ≈ ½(−ω×)δq  − I δb
        Omega = np.array([
            [ 0,      -omega[0], -omega[1], -omega[2]],
            [omega[0],     0,    omega[2], -omega[1]],
            [omega[1], -omega[2],    0,    omega[0]],
            [omega[2],  omega[1], -omega[0],   0   ],
        ])  # not used fully; small‑angle approx below
        F[:3, :3] -= 0.5 * skew(omega) * dt
        F[:3, 3:] = -np.eye(3) * dt
        # process noise covariance in error state order [δθ, δb]
        self.P = F @ self.P @ F.T + self.Q * dt

    # --------------------------------------------------------------- update
    def update_accel(self, accel_meas: NDArray[np.floating]) -> None:
        # normalise measurement to unit vector
        if np.linalg.norm(accel_meas) < 1e-6:
            return  # free‑fall
        z = accel_meas / np.linalg.norm(accel_meas)
        q = self.x[:4]
        h = quat.rotate_vector(GRAVITY_BASIS, q)  # predicted gravity dir in body
        y = z - h                                 # innovation (3,)

        # Jacobian H (3×6): ∂h/∂δθ,  no bias influence
        H = np.zeros((3, 6))
        H[:, :3] = -skew(h)

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)       # 6×3

        delta_x = K @ y                           # error‑state correction
        delta_theta = delta_x[:3]
        delta_b = delta_x[3:]

        # Apply attitude correction via small‑angle quaternion
        dq = np.hstack(([1.0], 0.5 * delta_theta))
        self.x[:4] = quat.normalise(quat.multiply(self.x[:4], dq))
        self.x[4:] += delta_b

        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

    # ------------------------------------------------------------ helpers
    @property
    def quaternion(self) -> NDArray[np.floating]:
        return self.x[:4].copy()

    @property
    def euler(self):
        return quat.to_euler_angles(self.x[:4])


# ---------------------------------------------------------------------
# small util
# ---------------------------------------------------------------------

def skew(v: NDArray[np.floating]) -> NDArray[np.floating]:
    """Return 3×3 skew‑symmetric matrix of vector v."""
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
