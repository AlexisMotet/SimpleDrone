import numpy as np
from dataclasses import dataclass, field
from typing import List

def default_ndarray(l: List, dtype):
    return field(default_factory=lambda: np.array(l, dtype=dtype))

@dataclass
class Inertia:
    mass: float
    matrix: np.ndarray
    inv_matrix: np.ndarray

@dataclass
class RCInputs:
    throttle: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw_rate: float = 0.0

@dataclass
class State:
    pos: np.ndarray = default_ndarray([0.0, 0.0, 0.0], np.float64)
    vel: np.ndarray = default_ndarray([0.0, 0.0, 0.0], np.float64)
    accel: np.ndarray = default_ndarray([0.0, 0.0, 0.0], np.float64)
    orient_quat: np.ndarray = default_ndarray([1.0, 0.0, 0.0, 0.0], np.float64) # world -> body
    ang_vel: np.ndarray = default_ndarray([0.0, 0.0, 0.0], np.float64)


@dataclass(init=False)
class MotorsRPM:
    start_date: float
    start: List[float]
    cmd: List[float]

    def __init__(self, num_motors):
        self.start_date = 0.0
        self.start = [0.0] * num_motors
        self.cmd = [0.0] * num_motors