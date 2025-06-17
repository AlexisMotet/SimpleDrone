"""quaternions.py – lightweight quaternion helpers (right‑handed, scalar‑first)

All quaternions are NumPy arrays shaped (4,), ordered [w, x, y, z].
Vectors are shape (3,).
"""
from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

# -----------------------------------------------------------------------------
# Basic quaternion algebra
# -----------------------------------------------------------------------------

def multiply(p: NDArray[np.floating], q: NDArray[np.floating]) -> NDArray[np.floating]:
    """Hamilton product **p ⊗ q**.
    Arrays `p`, `q` must be shape (4,) each.
    """
    w1, x1, y1, z1 = p
    w2, x2, y2, z2 = q
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def normalize(q: NDArray[np.floating]) -> NDArray[np.floating]:
    """Return `q` scaled to unit length."""
    return q / np.linalg.norm(q)


# -----------------------------------------------------------------------------
# Vector rotation
# -----------------------------------------------------------------------------

def rotate_vector(v: NDArray[np.floating], q: NDArray[np.floating]) -> NDArray[np.floating]:
    """Rotate 3‑vector `v` by quaternion `q` (scalar‑first).

    Uses the optimized formula `v + 2*cross(q_vec, cross(q_vec,v) + q_w*v)`.
    """
    qw, qx, qy, qz = q
    q_vec = q[1:]
    t = 2.0 * np.cross(q_vec, v)
    return v + qw * t + np.cross(q_vec, t)


# -----------------------------------------------------------------------------
# Time derivative  (q̇ = ½ q ⊗ [0, ω])
# -----------------------------------------------------------------------------

def derive(q: NDArray[np.floating], omega_body: NDArray[np.floating]) -> NDArray[np.floating]:
    """Return quaternion time‑derivative given body‑frame angular rate `omega_body`.

    Parameters
    ----------
    q : (4,) quaternion [w x y z]
    omega_body : (3,) rad/s in body frame
    """
    return 0.5 * multiply(q, np.hstack(([0.0], omega_body)))

# -----------------------------------------------------------------------------
# Rotation‑matrix and Euler utility
# -----------------------------------------------------------------------------

def to_rotation_matrix(q: NDArray[np.floating]) -> NDArray[np.floating]:
    """Convert quaternion to 3 × 3 DCM **R_bw** (body ← world)."""
    w, x, y, z = q
    ww, xx, yy, zz = w * w, x * x, y * y, z * z
    wx, wy, wz = w * x, w * y, w * z
    xy, xz, yz = x * y, x * z, y * z
    return np.array(
        [
            [ww + xx - yy - zz, 2 * (xy - wz),     2 * (xz + wy)],
            [2 * (xy + wz),     ww - xx + yy - zz, 2 * (yz - wx)],
            [2 * (xz - wy),     2 * (yz + wx),     ww - xx - yy + zz],
        ]
    )

def to_euler_angles(q: NDArray[np.floating]) -> tuple[float, float, float]:
    """Return roll‑pitch‑yaw (rad) from quaternion."""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def from_euler_angles(roll: float, pitch: float, yaw: float) -> NDArray[np.floating]:
    """Convert roll, pitch, yaw (rad) → quaternion [w, x, y, z]"""
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return normalize(np.array([w, x, y, z]))

def conjugate(q: NDArray[np.floating]) -> NDArray[np.floating]:
    """Return the conjugate of a quaternion [w, x, y, z]."""
    w, x, y, z = q
    return np.array([w, -x, -y, -z])