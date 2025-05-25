from dataclasses import dataclass
from pathlib import Path
from typing import Any, Tuple

import numpy as np
from numpy.typing import NDArray

import quaternions as quat

Vec3 = NDArray[np.float64]

@dataclass(frozen=True)
class DroneParams:
    mass: float                      # kg
    inertia: NDArray[np.float64]     # 3x3 kg·m^2
    motors_pos: NDArray[np.float64]  # (N,3) m, body frame
    motors_dir: NDArray[np.int8]     # +1 ccw, -1 cw
    k_f: float                       # thrust coefficient (N·s^2/rad^2)
    k_m: float                       # drag‑torque coefficient (N·m·s^2/rad^2)

@dataclass
class DroneState:
    pos: Vec3
    vel: Vec3
    quat: NDArray[np.float64]  # w,x,y,z body to world
    omega: Vec3

@dataclass
class DroneStateDerivatives:
    vel: Vec3
    acc: Vec3
    quat_dot: NDArray[np.float64]
    omega_dot: NDArray[np.float64]


class Drone:
    g_vec = np.array([0.0, 0.0, -9.81])  # m/s^2, world frame

    def __init__(self, params: DroneParams, init_pos=np.array([0.0, 0.0, 0.0]), init_vel=np.array([0.0, 0.0, 0.0])):
        self.params = params
        self.state = DroneState(
            pos=init_pos,
            vel=init_vel,
            quat=np.array([1.0, 0.0, 0.0, 0.0]),
            omega=np.zeros(3),
        )
        self.Jinv = np.linalg.inv(params.inertia)

    def compute_state_derivates(self, state: DroneState, rpm: NDArray[np.float64]):
        thrust, torque = self._compute_thrust_and_torque(rpm)
        R_body_to_world = quat.quat_to_rotmat(state.quat)
        acc = (R_body_to_world @ thrust) / self.params.mass + self.g_vec
        quat_dot = 0.5 * quat.mul(s.quat, np.concatenate(([0.0], state.omega)))
        omega = state.omega
        omega_dot = self.Jinv @ (torque - np.cross(omega, self.params.inertia @ omega))
        return DroneStateDerivatives(state.vel, acc, quat_dot, omega_dot)


    def compute_accelerations(self, rpm: NDArray[np.float64]) -> Tuple[Vec3, Vec3]:
        """Return (linear_acc_world, angular_acc_body) at current state for rpm."""
        thrust, torque = self._compute_thrust_and_torque(rpm)
        R_body_to_world = quat.quat_to_rotmat(self.state.quat)
        linear_acc_world = (R_body_to_world @ thrust) / self.params.mass + self.g_vec
        omega = self.state.omega
        angular_acc_body = self.Jinv @ (torque - np.cross(omega, self.params.inertia @ omega))
        return linear_acc_world, angular_acc_body

    def _compute_thrust_and_torque(self, rpm: NDArray[np.float64]) -> Tuple[Vec3, Vec3]:
        """Return body-frame (total_thrust, total_torque) for given motor RPM."""
        omega2 = (rpm * np.pi / 30.0) ** 2  # (rad/s)^2
        thrusts = self.params.k_f * omega2
        total_thrust = np.array([0.0, 0.0, thrusts.sum()])
        total_torque = np.zeros(3)
        for motor_pos, motor_dir, om2, thrust in zip(self.params.motors_pos, self.params.motors_dir, omega2, thrusts):
            total_torque += np.cross(motor_pos, [0.0, 0.0, thrust])
            total_torque[2] += motor_dir * self.params.k_m * om2
        return total_thrust, total_torque

    @classmethod
    def from_yaml(cls, path: str | Path) -> "Drone":
        import yaml
        with Path(path).open() as f:
            return cls.from_dict(yaml.safe_load(f))

    @classmethod
    def  _from_dict(cls, cfg: dict[str, Any]) -> "Drone":
        params = cls._params_from_dict(cfg)
        return cls(params)

    @classmethod
    def _params_from_dict(cls, cfg: dict[str, Any]) -> DroneParams:
        J = np.diag([cfg["inertia"][c] for c in ("xx", "yy", "zz")])
        motors = cfg["motors"]
        motors_pos = np.array([m["position"] for m in motors])
        motors_dir = np.array([1 if m["direction"].lower() == "ccw" else -1 for m in motors])
        return DroneParams(
            mass=cfg["mass"],
            inertia=J,
            motors_pos=motors_pos,
            motors_dir=motors_dir,
            k_f=cfg["propeller"]["thrust_coefficient"],
            k_m=cfg["propeller"]["torque_coefficient"],
        )

    def plot(self):
        import matplotlib.pyplot as plt

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        # draw arms
        for pos in self.params.motors_pos:
            ax.plot([0, pos[0]], [0, pos[1]], [0, pos[2]], "k-", lw=2)
        ax.set_box_aspect([1, 1, 0.25])

        # draw rotation arrows
        for pos, dir_ in zip(self.params.motors_pos, self.params.motors_dir):
            theta = np.linspace(0, 2 * np.pi, 40)
            r = 0.05
            x = pos[0] + r * np.cos(theta)
            y = pos[1] + r * np.sin(theta)
            z = np.full_like(x, pos[2])
            ax.plot(x, y, z, color="0.6")
            # arrow indicating spin dir (+1 ccw blue, -1 cw red)
            arrow_len = 0.07
            col = "tab:blue" if dir_ > 0 else "tab:red"
            ax.quiver(
                pos[0], pos[1], pos[2],
                0, 0, arrow_len * (1 if dir_ > 0 else -1),
                color=col, linewidth=1, arrow_length_ratio=0.4,
            )

        # origin marker
        ax.scatter([0], [0], [0], c="k", s=30)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        extent = np.max(np.linalg.norm(self.params.motors_pos, axis=1)) * 1.3
        ax.set_xlim(-extent, extent)
        ax.set_ylim(-extent, extent)
        ax.set_zlim(-0.05, extent * 0.3)
        ax.set_title(f"mass={self.params.mass:.2f} kg {len(self.params.motors_pos)} motors k_f={self.params.k_f} k_m={self.params.k_m}")
        ax.view_init(elev=25, azim=135)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    import yaml
    import numpy.testing as npt

    example_yaml = """
    name: TestQuad
    mass: 1.5
    inertia: {xx: 0.02, yy: 0.02, zz: 0.04}
    propeller:
      thrust_coefficient: 2.3e-6
      torque_coefficient: 1.1e-7
    motors:
      - id: 0
        position: [ 0.225,  0.225, 0.0]
        direction: ccw
      - id: 1
        position: [-0.225,  0.225, 0.0]
        direction: cw
      - id: 2
        position: [-0.225, -0.225, 0.0]
        direction: ccw
      - id: 3
        position: [ 0.225, -0.225, 0.0]
        direction: cw
    """

    cfg = yaml.safe_load(example_yaml)
    quad = Drone._from_dict(cfg)

    quad.plot()

    def test_basic_params():
        assert quad.params.mass == 1.5
        npt.assert_allclose(np.diag(quad.params.inertia), [0.02, 0.02, 0.04])
        assert quad.params.motors_pos.shape == (4, 3)
        assert quad.params.motors_dir.tolist() == [1, -1, 1, -1]

    def test_forces_zero_rpm():
        rpm = np.zeros(4)
        F, tau = quad._compute_thrust_and_torque(rpm)
        npt.assert_allclose(F, [0, 0, 0])
        npt.assert_allclose(tau, [0, 0, 0])

    def test_simple_hover():
        thrust_per_motor = quad.params.mass * 9.81 / 4
        rpm_hover = np.sqrt(thrust_per_motor / quad.params.k_f) * 30 / np.pi
        F, tau = quad._compute_thrust_and_torque(np.full(4, rpm_hover))
        npt.assert_allclose(F[2], quad.params.mass * 9.81, rtol=1e-3)
        npt.assert_allclose(F[:2], [0, 0], atol=1e-6)
        npt.assert_allclose(tau, [0, 0, 0], atol=1e-6)

    def test_dynamics_gravity_only():
        rpm = np.zeros(4)
        acc, alpha = quad.compute_accelerations(rpm)
        npt.assert_allclose(acc, [0, 0, -9.81], atol=1e-6)
        npt.assert_allclose(alpha, [0, 0, 0], atol=1e-6)

    for fn in (test_basic_params, test_forces_zero_rpm, test_simple_hover, test_dynamics_gravity_only):
        fn()
