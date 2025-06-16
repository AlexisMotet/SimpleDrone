import numpy as np
from dataclasses import dataclass, field
from typing import List
from numpy.typing import NDArray

@dataclass
class DroneConfig:
    geometry: str
    arm_length: float
    torque2thrust_coef: float

@dataclass
class RCInputs:
    throttle: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    yaw_rate: float = 0.0

@dataclass
class PulseWidthModulation:
    values: List[float]

@dataclass
class ESCOutput:
    values: List[float]

@dataclass
class RevolutionsPerMinute:
    values: List[float]

@dataclass
class DroneState:
    pos: NDArray[np.float64] = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=np.float64))
    vel: NDArray[np.float64] = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=np.float64))
    accel: NDArray[np.float64] = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=np.float64))
    orient_quat: NDArray[np.float64] = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64))
    ang_vel: NDArray[np.float64] = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=np.float64))