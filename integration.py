def integrate(drone: Drone, rpm: NDArray[np.float64], dt: float, method: str = "rk4") -> DroneState:
    new_state = STEP_METHODS[method](drone, rpm, dt)

STEP_METHODS: Dict[str, Callable[[Drone, NDArray[np.float64], float], DroneState]] = {
    "euler": _euler,
    "rk4": _rk4,
}

def _euler(drone: Drone, rpm: NDArray[np.float64], dt: float) -> DroneState:
    k1 = _state_derivatives(drone, drone.state, rpm)
    return _add_state(drone.state, k1, dt)


def _rk4(drone: Drone, rpm: NDArray[np.float64], dt: float) -> DroneState:
    s = drone.state
    k1 = drone.compute_state_derivates(drone, s, rpm)
    k2 = drone.compute_state_derivates(drone, _add_state(s, k1, 0.5*dt), rpm)
    k3 = drone.compute_state_derivates(drone, _add_state(s, k2, 0.5*dt), rpm)
    k4 = drone.compute_state_derivates(drone, _add_state(s, k3, dt),     rpm)
    return DroneState(
        pos   = s.pos   + dt/6*(k1.vel   + 2*k2.vel   + 2*k3.vel   + k4.vel),
        vel   = s.vel   + dt/6*(k1.acc   + 2*k2.acc   + 2*k3.acc   + k4.acc),
        quat  = quat.normalize(
            s.quat + dt/6*(k1.quat_dot + 2*k2.quat_dot + 2*k3.quat_dot + k4.quat_dot)),
        omega = s.omega + dt/6*(k1.omega_dot + 2*k2.omega_dot + 2*k3.omega_dot + k4.omega_dot),
    )

def _add_state(s: DroneState, k: DroneStateDerivatives, h: float) -> DroneState:
    return DroneState(pos=s.pos + k.vel*h,
                      vel=s.vel + k.acc*h,
                      quat=quat.normalize(s.quat + k.quat_dot*h),
                      omega=s.omega + k.omega_dot*h)





