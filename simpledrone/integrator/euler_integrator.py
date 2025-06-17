import numpy as np
from typing import Tuple
from simpledrone.data import Inertia, State
from simpledrone.maths import quaternions


class EulerIntegrator:
    def __init__(self):
        self.prev_t = 0.0


    def integrate(self, t: float, inertia: Inertia, state: State, thrust: float, torque: Tuple[float]) -> State:
        dt = t - self.prev_t
        assert dt >= 0.0
        self.prev_t = t

        R_world2body = quaternions.to_rotation_matrix(state.orient_quat)
        R_body2world = R_world2body.T
        
        force_body = np.array([0, 0, -thrust]) # NED convention
        force_world = R_body2world @ force_body
        weight = np.array([0.0, 0.0, inertia.mass * 9.81]) # NED
        accel_world = (force_world + weight) / inertia.mass

        new_state = State()
        
        new_state.accel = accel_world
        new_state.vel = state.vel + new_state.accel * dt
        new_state.pos = state.pos + new_state.vel * dt

        coriolis = np.cross(state.ang_vel, inertia.matrix @ state.ang_vel)
        ang_acc = inertia.inv_matrix @ (np.asarray(torque) - coriolis)

        new_state.ang_vel = state.ang_vel + ang_acc * dt

        q_dot = quaternions.derive(state.orient_quat, state.ang_vel)
        
        new_state.orient_quat = quaternions.normalize(state.orient_quat + q_dot * dt)

        return new_state