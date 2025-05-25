"""Cascaded angle→rate PID controller with split update rates.

Call ``fc.update(rc, meas, V_batt, dt_phys)`` every physics step.  The
rate loop runs every call; the angle loop ticks at ``1/f_angle``.
"""
from __future__ import annotations
import numpy as np
from pid import PID

_DEG2RAD = np.pi / 180.0

class FlightController:
    """Betaflight‑style cascaded PID with internal scheduler.

    Parameters
    ----------
    drone      : object with fields mass, k_f, k_m, motor_pos, motor_dirs, battery
    f_rate     : rate‑loop frequency   (Hz)
    f_angle    : angle‑loop frequency  (Hz)
    max_angle  : max absolute tilt (deg) in angle mode
    max_rate   : max body‑rate (deg/s)
    """
    def __init__(self, drone, *, f_rate=1000, f_angle=200,
                 max_angle_deg=35.0, max_rate_dps=360.0):
        self.drone = drone
        # loop periods
        self.dt_rate  = 1.0 / f_rate
        self.dt_angle = 1.0 / f_angle
        # limits
        self.max_ang  = max_angle_deg * _DEG2RAD
        self.max_rate = max_rate_dps * _DEG2RAD
        # PIDs
        self.pid_angle = {
            'roll':  PID(4.5, 0.0, 0.0),
            'pitch': PID(4.5, 0.0, 0.0),
        }
        self.pid_rate = {
            'roll':  PID(0.15, 0.15, 0.001, -10, 10),
            'pitch': PID(0.15, 0.15, 0.001, -10, 10),
            'yaw':   PID(0.4,  0.10, 0.0,   -5, 5),
        }
        # mixer matrix: [Fz, τx, τy, τz]^T → thrusts per motor
        self._mixer = self._build_mixer()
        # schedule accumulators
        self._acc_angle = 0.0
        # outputs cached between loops
        self._rate_set = np.zeros(3)
        # motor limits
        self.rpm_idle = 500
        self.rpm_max_nom = 20000

    # ------------------------------------------------------------------
    def _build_mixer(self):
        k_f = self.drone.k_f
        k_m = self.drone.k_m
        arms = self.drone.motor_pos  # shape (n,3)
        dirs = np.array([1 if d == 'cw' else -1 for d in self.drone.motor_dirs])
        n = arms.shape[0]
        # collective thrust contribution
        col = np.full((n, 1), k_f)
        # torques from thrust on arms
        torques = np.cross(arms, np.array([0, 0, k_f]))
        # yaw torque from motor drag
        yaw = (k_m * dirs).reshape(-1, 1)
        mat = np.hstack([col, torques, yaw])  # (n,4)
        return np.linalg.pinv(mat)

    # ------------------------------------------------------------------
    def update(self, rc: dict[str, float], meas: dict[str, np.ndarray], V_batt: float, dt_phys: float) -> np.ndarray:
        """Compute motor RPMs for this physics step.

        ``meas`` must contain keys: 'euler' [rad], 'gyro' [rad/s]
        """
        # ---------- scheduler ----------
        self._acc_angle += dt_phys
        run_angle = False
        if self._acc_angle >= self.dt_angle:
            self._acc_angle -= self.dt_angle
            run_angle = True

        # ---------- angle loop ----------
        if run_angle:
            roll_meas, pitch_meas, _ = meas['euler']
            des_roll  = rc['roll']  * self.max_ang
            des_pitch = rc['pitch'] * self.max_ang
            des_yaw_rate = rc['yaw'] * self.max_rate  # yaw uses rate only

            self._rate_set[0] = self.pid_angle['roll'](des_roll  - roll_meas,  self.dt_angle)
            self._rate_set[1] = self.pid_angle['pitch'](des_pitch - pitch_meas, self.dt_angle)
            self._rate_set[2] = des_yaw_rate

        # ---------- rate loop ----------
        torques = np.zeros(3)
        for ax, idx in zip(('roll','pitch','yaw'), range(3)):
            err = self._rate_set[idx] - meas['gyro'][idx]
            torques[idx] = self.pid_rate[ax](err, self.dt_rate)

        # ---------- collective thrust ----------
        thr_in = np.clip(rc['throttle'], 0.0, 1.0)
        Fz = thr_in * 2.0 * self.drone.mass * 9.81  # up to 2 g

        # ---------- solve thrust per motor ----------
        thrusts = self._mixer @ np.array([Fz, *torques])
        thrusts = np.clip(thrusts, 0.0, None)

        # ---------- desaturation when near limits ----------
        max_thr_per_motor = self.drone.k_f * (self.rpm_max_nom ** 2) * (V_batt / self.drone.battery.V_nom)
        scale = max(1.0, thrusts.max() / max_thr_per_motor)
        thrusts /= scale

        # ---------- thrust → RPM ----------
        rpm = np.sqrt(thrusts / self.drone.k_f)
        rpm = np.clip(rpm, self.rpm_idle, self.rpm_max_nom * (V_batt / self.drone.battery.V_nom))
        return rpm