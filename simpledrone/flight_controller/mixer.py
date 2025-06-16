"""
From: https://cookierobotics.com/066/

axis convention:

    x
    ^ 
 O  |  O
    + -- > y
 O     O

"""

import numpy as np
from simpledrone.data import DroneConfig

def compute_mixer_matrix(cfg: DroneConfig):
    if cfg.geometry == "quad+":
        angles = [0, 90, 180, 270]
        spin = [1, -1, 1, -1]
    elif cfg.geometry == "quadx":
        angles = [45, 135, 225, 315]
        spin = [1, -1, 1, -1]
    elif cfg.geometry == "hexax":
        angles = [30, 90, 150, 210, 270, 330]
        spin = [1, -1, 1, -1, 1, -1]
    else:
        raise ValueError(f"unsupported geometry '{cfg.geometry}'")
    x = cfg.arm_length * np.cos(np.deg2rad(angles))
    y = cfg.arm_length * np.sin(np.deg2rad(angles))
    N = len(angles)
    M = np.zeros((4, N))
    M[0, :] = 1.0
    M[1, :] = -y
    M[2, :] = x
    M[3, :] = np.array(spin) * cfg.torque2thrust_coef
    Mp = np.linalg.pinv(M)
    return Mp, M

if __name__ == "__main__":
    cfg = DroneConfig("quad+", arm_length=1.0, torque2thrust_coef=1.0)
    print(compute_mixer_matrix(cfg))