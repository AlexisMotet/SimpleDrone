from typing import Tuple
import numpy as np
from simpledrone.data import Inertia

class Frame:
    def get_inertia(self) -> Inertia:
        raise NotImplementedError()
    
    def get_motor_positions(self) ->Tuple[Tuple[float, float, float]]:
        raise NotImplementedError()
    
    def get_motor_spins(self) -> Tuple[int]:
        raise NotImplementedError()


class FPV4XSymmetric(Frame):
    def __init__(self, core_width_cm: float = 8.0, core_length_cm: float = 12.0, core_mass_g: float = 250.0, 
                 arm_length_cm: float = 10.0, motor_mass_g: float = 30.0):
        cm = core_mass_g * 0.001
        mm = motor_mass_g * 0.001
        self.mass = (cm + 4 * mm) * 0.001

        L = (arm_length_cm * 0.01) / np.sqrt(2)

        self.motor_positions = (
            (L,  L, 0.0),
            (-L,  L, 0.0),
            (-L, -L, 0.0),
            (L, -L, 0.0),
        )

        w = core_width_cm * 0.01
        l = core_length_cm * 0.01

        Ixx = (1 / 12) * cm * (w**2 + l**2)
        Iyy = (1 / 12) * cm * (2 * (w**2))
        Izz = (1 / 12) * cm * (w**2 + l**2)

        self.I = np.diag([Ixx, Iyy, Izz])

        for motor_pos in self.motor_positions:
            r2 = np.dot(motor_pos, motor_pos)
            self.I += mm * (r2 * np.eye(3) - np.outer(motor_pos, motor_pos))

        self.I_inv = np.linalg.inv(self.I)

    def get_num_motors(self) -> int:
        return 4

    def get_inertia(self) -> Inertia:
        return Inertia(mass=self.mass, matrix=self.I, inv_matrix=self.I_inv)

    def get_motor_positions(self) -> Tuple[Tuple[float, float, float]]:
        return self.motor_positions
    
    def get_motor_spins(self) -> Tuple[int]:
        return (1, -1, 1, -1)