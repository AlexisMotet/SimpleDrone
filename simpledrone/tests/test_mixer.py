import unittest
import numpy as np
from simpledrone.flight_controller.mixer import compute_mixer_matrix
from simpledrone.data import DroneConfig


class TestMixerMatrix(unittest.TestCase):
    def test_hexarotorx(self):
        cfg = DroneConfig("hexax", arm_length=1.0, torque2thrust_coef=1.0)
        Mp, M = compute_mixer_matrix(cfg)
        # np.testing.assert_allclose(M, M_res, rtol=1e-4)
        Mp_res = np.array([
            [0.1667, -0.1667, 0.2887, 0.1667],
            
            [0.1667, 0.1667, 0.2887, -0.1667],
            [0.1667, 0.1667, -0.2887, -0.1667],

            [0.1667, -0.3333, 0.0000, -0.1667],
            
            [0.1667, 0.3333, 0.0000, 0.1667],
            
            [0.1667, -0.1667, -0.2887, 0.1667],
            
            
        ])
        np.testing.assert_allclose(Mp, Mp_res, rtol=1e-4)

