def dynamics(state: DroneState, force_body: np.ndarray, torque_body: np.ndarray, mass: float, inertia: np.ndarray) -> DroneState:
    """Compute time derivative of the state given force & torque in body frame."""
    R = quat_to_rotmat(state.attitude)  # world_from_body
    force_world = R @ force_body

    # Linear motion -------------------------------------------------------
    acc = force_world / mass + GRAVITY

    # Rotational motion ---------------------------------------------------
    omega = state.angular_velocity
    omega_dot = np.linalg.inv(inertia) @ (torque_body - np.cross(omega, inertia @ omega))

    # Attitude derivative -------------------------------------------------
    q_dot = 0.5 * quat_mult(state.attitude, omega_quat(omega))

    return DroneState(position=state.velocity, velocity=acc, attitude=q_dot, angular_velocity=omega_dot)
