from typing import List
import numpy as np
import math


# default is: T-Motor F40 PRO II Motor POPO - 2400kv motor
# https://database.tytorobotics.com/motors/yx5/t-motor-f40-pro-ii-motor-popo-2400kv

class ESCMotorProp:
    MIN_PWM_US = 1000.0
    MAX_PWM_US = 2000.0

    def __init__(
        self,
        pwm_frequency=50.0,
        motor_response_time=100.0,
        pwms: List[int] = [1000, 1300, 1325, 1350, 1375, 1400, 1425, 1450, 1475, 1500, 1550, 1600, 1650, 1700, 1750, 1800],
        rpms: List[int] = [0, 4550, 5138, 5685, 6163, 6569, 6983, 7433, 7868, 8276, 9184, 10042, 10905, 11854, 12654, 13261],
        thrusts_kgf: List[float] = [0.0002, 0.0429, 0.0544, 0.0669, 0.0788, 0.0898, 0.1022, 0.1167, 0.1317, 0.1457, 0.1804, 0.218, 0.2589, 0.308, 0.3508, 0.3866],
        torques: List[float] = [0.0001, 0.0055, 0.007, 0.0086, 0.0103, 0.0117, 0.0132, 0.0154, 0.0178, 0.0201, 0.0252, 0.0301, 0.0359, 0.0434, 0.0502, 0.0561],
    ):
        self.pwm_frequency = pwm_frequency
        self.motor_response_time = motor_response_time / 1e3
        self.pwms = pwms
        self.rpms = rpms
        self.thrusts = [thrust * 9.81 for thrust in thrusts_kgf]
        self.torques = torques
       
    def get_pwm_frequency(self) -> float:
        return self.pwm_frequency

    def estimate_thrust(self, throttle: float) -> float:
        if not 0.0 <= throttle <= 1.0:
            raise ValueError()
        
        pwm = ESCMotorProp._compute_pwm(throttle)
        return float(np.interp(pwm, self.pwms, self.thrusts))

    def estimate_propeller_torque(self, throttle: float) -> float:
        if not 0.0 <= throttle <= 1.0:
            raise ValueError()
        
        pwm = ESCMotorProp._compute_pwm(throttle)
        return float(np.interp(pwm, self.pwms, self.torques))
     
    def estimate_thrust_from_rpm(self, rpm: float) -> float:
        return float(np.interp(rpm, self.rpms, self.thrusts))
    
    def estimate_propeller_torque_from_rpm(self, rpm: float) -> float:
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
            torque = self.estimate_propeller_torque_from_rpm(rpm)
            thrust = self.estimate_thrust_from_rpm(rpm)
            torque_coef = torque / thrust
            if torque_coef >= 1:
                raise ValueError("torque/thrust coefficient is not expected to be >= 1")
            torque_coefs.append(torque_coef)
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
                

