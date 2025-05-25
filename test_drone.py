# === test_drone.py ===
"""Unit tests for drone.py using pytest‑style assertions.
Run with  `pytest -q`  or simply `python test_drone.py`.
"""
import numpy as np
import yaml
from drone import Drone

_YAML = """
name: TestQuad
mass: 1.5
inertia: {xx: 0.02, yy: 0.02, zz: 0.04}
init_pos: [0.0, 0.0, 0.0]
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

cfg = yaml.safe_load(_YAML)
quad = Drone.from_dict(cfg)


def test_basic_params():
    assert quad.params.mass == 1.5
    np.testing.assert_allclose(np.diag(quad.p.inertia), [0.02, 0.02, 0.04])
    assert quad.params.motor_pos.shape == (4, 3)
    assert quad.p.motor_dirs.tolist() == [1, -1, 1, -1]


def test_forces_zero_rpm():
    rpm = np.zeros(4)
    F, tau = quad.forces_torques(rpm)
    np.testing.assert_allclose(F, [0, 0, 0])
    np.testing.assert_allclose(tau, [0, 0, 0])


def test_simple_hover():
    # compute hover rpm analytically: thrust = mg / 4
    thrust_per_motor = quad.p.mass * 9.81 / 4
    rpm = np.sqrt(thrust_per_motor / quad.p.k_f) * 30 / np.pi  # back to RPM
    F, tau = quad.forces_torques(np.full(4, rpm))
    np.testing.assert_allclose(F[2], quad.p.mass * 9.81, rtol=1e-3)
    np.testing.assert_allclose(F[:2], [0, 0], atol=1e-6)
    np.testing.assert_allclose(tau, [0, 0, 0], atol=1e-6)


if __name__ == "__main__":
    for fn in (test_basic_params, test_forces_zero_rpm, test_simple_hover):
        fn()
    print("All drone tests passed.")
