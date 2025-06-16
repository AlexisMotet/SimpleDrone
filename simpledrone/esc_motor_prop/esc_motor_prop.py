from typing import List
import numpy as np
import math


# default is: https://database.tytorobotics.com/tests/xkn6/unmartmoto-as2820-880kv-g-camfold-28x15-11x6-hub45

class ESCMotorProp:
    MIN_PWM_US = 1000.0
    MAX_PWM_US = 2000.0

    def __init__(
        self,
        pwm_frequency=50.0,
        motor_response_time=100.0,
        pwms: List[int] = [1000, 1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 1600, 1650, 1700, 1750, 1800, 1900, 2000],
        rpms: List[int] = [0, 465, 813, 1148, 1476, 1798, 2107, 2390, 2671, 2983, 3279, 3543, 3818, 4056, 4322, 4550, 5027, 5227],
        thrusts: List[float] = [0.0001, 0.01, 0.032, 0.0665, 0.1136, 0.1728, 0.2444, 0.3179, 0.4107, 0.527, 0.6194, 0.7339, 0.8289, 0.969, 1.086, 1.244, 1.504, 1.674],
        torques: List[float] = [0.0001, 0.0036, 0.0111, 0.0212, 0.0337, 0.05, 0.0671, 0.086, 0.1094, 0.1343, 0.1541, 0.1793, 0.202, 0.2356, 0.2618, 0.3029, 0.3625, 0.4061],
    ):
        self.pwm_frequency = pwm_frequency
        self.motor_response_time = motor_response_time / 1e3
        self.pwms = pwms
        self.rpms = rpms
        self.thrusts = thrusts
        self.torques = torques
       
    def get_pwm_frequency(self) -> float:
        return self.pwm_frequency

    def estimate_thrust(self, throttle: float) -> float:
        if not 0.0 <= throttle <= 1.0:
            raise ValueError()
        
        pwm = ESCMotorProp._compute_pwm(throttle)
        return float(np.interp(pwm, self.pwms, self.thrusts))
    
    def estimate_torque(self, throttle: float) -> float:
        if not 0.0 <= throttle <= 1.0:
            raise ValueError()
        
        pwm = ESCMotorProp._compute_pwm(throttle)
        return float(np.interp(pwm, self.pwms, self.torques))
    
    def estimate_thrust_from_rpm(self, rpm: float) -> float:
        return float(np.interp(rpm, self.rpms, self.thrusts))
    
    def estimate_torque_from_rpm(self, rpm: float) -> float:
        return float(np.interp(rpm, self.rpms, self.torques))
    
    def estimate_rpm_from_throttle(self, throttle: float) -> float:
        if not 0.0 <= throttle <= 1.0:
            raise ValueError()
        
        pwm = ESCMotorProp._compute_pwm(throttle)
        rpm = float(np.interp(pwm, self.pwms, self.rpms))
        return rpm
    
    def estimate_torque_coefs_from_current_rpms(self, rpms: List[float]) -> List[float]:
        torque_coefs = []
        for rpm in rpms:
            torque = self.estimate_thrust_from_rpm(rpm)
            thrust = self.estimate_torque_from_rpm(rpm)
            torque_coefs.append(torque / thrust)
        return torque_coefs
    
    def compute_current_rpms(self, dt: float, start_rpms: List[float], rpms_cmd: List[float]) -> List[float]:
        curr_rpms = []
        for start_rpm, rpm_cmd in zip(start_rpms, rpms_cmd):
            alpha = math.exp(-dt / self.motor_response_time)
            curr_rpm = alpha * start_rpm + (1.0 - alpha) * rpm_cmd
            curr_rpms.append(curr_rpm)
        return curr_rpms
    
    @staticmethod
    def _compute_pwm(throttle: float) -> float:
        span = ESCMotorProp.MAX_PWM_US - ESCMotorProp.MIN_PWM_US
        return max(ESCMotorProp.MIN_PWM_US, min(throttle * span + ESCMotorProp.MIN_PWM_US, ESCMotorProp.MAX_PWM_US))
    
    @staticmethod
    def _compute_throttle(pwm: float) -> float:
        span = ESCMotorProp.MAX_PWM_US - ESCMotorProp.MIN_PWM_US
        throttle = (pwm - ESCMotorProp.MIN_PWM_US) / span
        return max(0.0, min(throttle, 1.0))
    
    def plot_pwm_curves(self):
        pwm_interp = np.linspace(1000, 2000, 500)
        rpm_interp = np.interp(pwm_interp, self.pwms, self.rpms)
        thrust_interp = np.interp(pwm_interp, self.pwms, self.thrusts)
        torque_interp = np.interp(pwm_interp, self.pwms, self.torques)
    
        import matplotlib.pyplot as plt

        plt.figure(figsize=(8, 8))

        plt.title("ESC + Motor + Propeller PWM Curves")

        for i, (y_values, y_interp, y_name) in enumerate(zip([self.rpms, self.thrusts, self.torques], [rpm_interp, thrust_interp, torque_interp], ["RPM", "Thrust (N)", "Torque (Nm)"])):
            plt.subplot(3, 1, i + 1)
            plt.plot(self.pwms, y_values, "+", label="data", zorder=100.0)
            plt.plot(pwm_interp, y_interp,label="interpolation")
            plt.ylabel(y_name)
            plt.grid()
            plt.legend()

        plt.tight_layout()
        plt.show()
                

