from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray

Vec3 = NDArray[np.float64]

@dataclass
class DroneState:
    """Full rigid‑body state for a simulation tick."""
    position: Vec3              # m, world frame
    velocity: Vec3              # m/s, world frame
    acceleration: Vec3
    orientation: NDArray[np.float64]  # quaternion w,x,y,z body→world
    angular_vel: Vec3           # rad/s, body frame

@dataclass
class RadioControllerInputs:
    """Normalized RC sticks in the range [‑1, 1] and throttle [0,1]."""
    throttle: float = 0.0       # 0…1 collective thrust request
    roll:     float = 0.0       # left/right stick (‑1…1)
    pitch:    float = 0.0       # forward/back stick (‑1…1)
    yaw:      float = 0.0       # yaw rate stick (‑1…1)
    # additional aux channels can be appended later if needed

@dataclass
class FlightControllerOutputs:
    """Low‑level command packet produced by the flight controller."""
    realtime_delay: float       # seconds of loop timing slack
    rpm: NDArray[np.float64]    # per‑motor RPM command