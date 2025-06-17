import numpy as np
from ahrs.filters.ekf import EKF
from ahrs.common.orientation import q2euler   # helper that returns [roll, pitch, yaw]

ekf = EKF(frame="NED", frequency=1000)

gyr  = np.array([0.0, 0.0, 0.0])                        # rad s⁻¹
g_NED = np.array([0.0, 0.0, 9.81])
from scipy.spatial.transform import Rotation as R
r = R.from_euler("ZYX", (0.0, 20.0, 0.0), degrees=True)
M = r.as_matrix()

print(M@np.array([1, 0, 0]))

acc = M@(-g_NED)

print(acc)


# acc  = np.array([3.35407184, 0.0, -9.21523664])         # for me this is +20° 
q    = np.array([1.0, 0.0, 0.0, 0.0])                   # prior quaternion (w, x, y, z)

roll, pitch, yaw = np.degrees(q2euler(q))               # convert to deg
print(f"roll={roll:+.2f}°, pitch={pitch:+.2f}°, yaw={yaw:+.2f}°")

for _ in range(1):                                   # a second of data
    q = ekf.update(q=q, gyr=gyr, acc=acc)

roll, pitch, yaw = np.degrees(q2euler(q))               # convert to deg
print(f"roll={roll:+.2f}°, pitch={pitch:+.2f}°, yaw={yaw:+.2f}°")

# WHY PITCH IS NEGATIVE ????