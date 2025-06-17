from scipy.spatial.transform import Rotation as R
import numpy as np

r = R.from_euler("ZYX", (0.0, 20.0, 0.0), degrees=True)
M = r.as_matrix()

print(M@np.array([1, 0, 0]))
# [ 0.93969262  0.         -0.34202014]