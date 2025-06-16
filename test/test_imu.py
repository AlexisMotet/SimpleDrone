import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# Parameters
T = 10.0
dt = 0.01
steps = int(T / dt)
t_vec = np.linspace(0, T, steps)

# 3D trajectory (helix with vertical and lateral motion)
positions = np.array([
    5 * np.cos(0.4 * t_vec),
    5 * np.sin(0.4 * t_vec),
    0.5 * t_vec + 0.5 * np.sin(1.5 * t_vec)
]).T

# Velocity and acceleration
velocities = np.gradient(positions, dt, axis=0)
accelerations = np.gradient(velocities, dt, axis=0)

# Orientation integration
g_vec = np.array([0, 0, -9.81])
quat = R.identity().as_quat()
quaternions = [quat]
angular_vels = []

for i in range(1, steps):
    acc = accelerations[i]
    thrust = -(acc - g_vec)
    thrust /= np.linalg.norm(thrust)

    # Current orientation
    R_curr = R.from_quat(quat)
    z_body = R_curr.apply([0, 0, 1])

    # Compute axis-angle to rotate z_body to thrust
    axis = np.cross(z_body, thrust)
    sin_angle = np.linalg.norm(axis)
    cos_angle = np.dot(z_body, thrust)
    if sin_angle < 1e-6:
        angular_velocity = np.zeros(3)
    else:
        axis /= sin_angle
        angle = np.arctan2(sin_angle, cos_angle)
        angular_velocity = axis * angle / dt

    angular_vels.append(angular_velocity)

    # Quaternion integration
    omega_quat = np.array([0, *angular_velocity])
    q = quat
    q_matrix = np.array([
        [0, -omega_quat[1], -omega_quat[2], -omega_quat[3]],
        [omega_quat[1], 0, omega_quat[3], -omega_quat[2]],
        [omega_quat[2], -omega_quat[3], 0, omega_quat[1]],
        [omega_quat[3], omega_quat[2], -omega_quat[1], 0]
    ])
    q_dot = 0.5 * q_matrix @ q
    quat = quat + q_dot * dt
    quat /= np.linalg.norm(quat)

    quaternions.append(quat)

# Pad angular velocity
angular_vels.insert(0, angular_vels[0])
quaternions = np.array(quaternions)

# Plot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label="Trajectory")

step_plot = 200
scale = 1.0

for i in range(0, steps, step_plot):
    rot = R.from_quat(quaternions[i])
    fwd = rot.apply([1, 0, 0])
    right = rot.apply([0, 1, 0])
    up = rot.apply([0, 0, 1])

    ax.quiver(*positions[i], *fwd, length=scale, color='blue', label='forward' if i == 0 else "")
    ax.quiver(*positions[i], *right, length=scale, color='green', label='right' if i == 0 else "")
    ax.quiver(*positions[i], *up, length=scale, color='red', label='up (thrust)' if i == 0 else "")

ax.set_title("Integrated Drone Orientation from Thrust Alignment")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.legend()
ax.grid(True)
plt.tight_layout()
plt.show()
