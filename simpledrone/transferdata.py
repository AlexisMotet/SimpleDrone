import numpy as np
from dataclasses import dataclass

@dataclass
class DroneConfig:
    geometry: str
    arm_length: float
    torque2thrust_coef: float

@dataclass
class DroneCommands:
    """Normalized commands in the range [‑1, 1] and throttle [0,1]."""

    pitch: float = 0.0
    roll: float = 0.0
    yaw_rate: float = 0.0
    throttle: float = 0.0


# from numpy.typing import NDArray

# Vec3 = NDArray[np.float64]


# @dataclass
# class DroneState:
#     """Full rigid‑body state for a simulation tick."""

#     position: Vec3  # m, world frame
#     velocity: Vec3  # m/s, world frame
#     acceleration: Vec3
#     orientation: NDArray[np.float64]  # quaternion w, x, y, z body -> world
#     angular_vel: Vec3  # rad/s, body frame

