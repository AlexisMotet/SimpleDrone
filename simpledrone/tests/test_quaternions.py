import numpy as np
from simpledrone.maths import quaternions

quat = quaternions.from_euler_angles(roll=0.0, pitch=np.deg2rad(10), yaw=0.0)

r_W2B = quaternions.to_rotation_matrix(quat)

print("xbody in world ned", r_W2B@np.array([1, 0, 0]))
print("ybody in world ned", r_W2B@np.array([0, 1, 0]))
print("zbody in world ned", r_W2B@np.array([0, 0, 1])) # zed is down + a bit forward

print(r_W2B@np.array([0, 0, 10]))

# print(r_W2B@np.array([0, 0, -10]))



# print(r_W2B)

# print("res:", r_W2B@np.array([1, 0, 0]))
# print("res:", r_W2B@np.array([0, 0, 1]))
# print("res:", r_W2B@np.array([0, 0, 9.81])) # this result is weird : [-1.70348862  0.         -9.66096406]
# # it should be negative because the forward axis is going up, like -gravity

# #
#.   _-> forward
#  _-  
# ------>