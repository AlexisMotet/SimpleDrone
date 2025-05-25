import numpy as np
from numpy.typing import NDArray

def quat_mult(q: NDArray[np.float64], r: NDArray[np.float64]) -> NDArray[np.float64]:
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1,
    ])

def quat_normalize(q: NDArray[np.float64]) -> NDArray[np.float64]:
    return q / np.linalg.norm(q)

omega_quat = lambda w: np.hstack(([0.0], w))

def quat_to_rotmat(q: NDArray[np.float64]) -> NDArray[np.float64]:
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)],
    ])
