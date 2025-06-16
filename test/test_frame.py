import unittest
import numpy as np
from simpledrone.frame import FPV4XSymmetric


class TestFrames(unittest.TestCase):
    def test_fpv_4x_sym(self):
        frame = FPV4XSymmetric(core_mass_g=200, motor_mass_g=30)
        inertia = frame.get_inertia()

        I = inertia.matrix

        self.assertTrue(np.allclose(np.linalg.inv(I), frame.I_inv, atol=1e-10))
        self.assertTrue(np.allclose(I, np.diag(np.diagonal(I)), atol=1e-10))

        Ixx, Iyy, Izz = np.diagonal(I)

        self.assertTrue(8e-4 < Ixx < 1.1e-3) # ranges given by chatgpt
        self.assertTrue(7e-4 < Iyy < 1.0e-3)
        self.assertTrue(1.4e-3 < Izz < 1.7e-3)

        self.assertGreater(Izz, Ixx) # yaw inertia Izz should be larger than Ixx for X-quad layout



if __name__ == '__main__':
    unittest.main()