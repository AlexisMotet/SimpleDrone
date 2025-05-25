# === sensor_model.py ===
"""Time‑correlated IMU simulator + lightweight complementary‑filter attitude estimator.
Only Python / NumPy; no external deps.  Call IMUSim.sample(state, dt) each physics step.
"""
from __future__ import annotations
import numpy as np

_GRAV = np.array([0, 0, -9.81])

class IMUSim:
    """1st‑order Gauss‑Markov gyro & acc with bias drift.

    Parameters
    ----------
    gyro_arw  : float  – angle‑random‑walk  (rad / √s)
    gyro_tau  : float  – bias correlation time (s)
    acc_std   : float  – white noise on specific force (m/s²)
    acc_tau   : float  – bias correlation time (s)
    """
    def __init__(self,
                 gyro_arw: float = 0.002,   # ≈ 7 °/√h – cheap MEMS
                 gyro_tau: float = 100.0,
                 acc_std: float = 0.05,
                 acc_tau: float = 100.0):
        self.gyro_arw = gyro_arw
        self.gyro_tau = gyro_tau
        self.acc_std = acc_std
        self.acc_tau = acc_tau
        self._b_g = np.zeros(3)
        self._b_a = np.zeros(3)
        rng = np.random.default_rng()
        self._rng = rng

    # ------------------------------------------------------------------
    def sample(self, state, dt: float, gravity_vec=_GRAV) -> dict[str, np.ndarray]:
        # -------- bias propagation (Gauss‑Markov) ---------
        exp_g = np.exp(-dt / self.gyro_tau)
        exp_a = np.exp(-dt / self.acc_tau)
        self._b_g = exp_g * self._b_g + self._rng.normal(0, self.gyro_arw*np.sqrt(dt), 3)
        self._b_a = exp_a * self._b_a + self._rng.normal(0, self.acc_std*np.sqrt(dt), 3)

        # -------- ideal signals ----------
        gyro_true = state.angular_velocity
        # specific force in body frame
        acc_world = state.linear_acc
        acc_body = state.R_ib @ (acc_world - gravity_vec)

        # -------- add noise & bias ---------
        gyro_meas = gyro_true + self._b_g + self._rng.normal(0, self.gyro_arw/np.sqrt(dt), 3)
        acc_meas  = acc_body  + self._b_a + self._rng.normal(0, self.acc_std, 3)

        return {"gyro": gyro_meas, "acc": acc_meas}