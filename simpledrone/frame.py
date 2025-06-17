from typing import Tuple
import numpy as np
from simpledrone.data import Inertia
from simpledrone.esc_motor_prop.esc_motor_prop import ESCMotorProp

class Frame:
    def get_inertia(self) -> Inertia:
        raise NotImplementedError()
    
    def get_motor_positions(self) ->Tuple[Tuple[float, float, float]]:
        raise NotImplementedError()
    
    def get_motor_spins(self) -> Tuple[int]:
        raise NotImplementedError()
    
    def get_max_torques(self, motor) -> Tuple[float]:
        raise NotImplementedError()

class FPV4XSymmetric(Frame):
    def __init__(self, core_width_cm: float = 8.0, core_length_cm: float = 12.0, core_mass_g: float = 250.0, 
                 arm_length_cm: float = 10.0, motor_mass_g: float = 30.0):
        cm = core_mass_g * 0.001
        mm = motor_mass_g * 0.001
        self.mass = (cm + 4 * mm)

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
        Izz = 2 * (1 / 12) * cm * (w**2 + l**2)

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
    
    # yaw_diff_ratio -> comes from betaflight https://betaflight.com/docs/development/Mixer?utm_source=chatgpt.com
    def get_max_torques(self, motor: ESCMotorProp, yaw_diff_ratio: float = 0.3) -> Tuple[float]:
        arm_length = float(np.linalg.norm(self.get_motor_positions()[0]))
        max_thrust = motor.estimate_thrust(throttle=1.0)
        
        max_roll_torque = 2 * arm_length * max_thrust
        max_pitch_torque = max_roll_torque

        max_yaw_torque = yaw_diff_ratio * 4 * motor.estimate_propeller_torque(throttle=1.0)
        
        return (max_roll_torque, max_pitch_torque, max_yaw_torque)
