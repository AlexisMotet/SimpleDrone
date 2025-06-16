import numpy as np
from data import DroneState, RCInputs
from mixer.mixer import Mixer

class Drone:
    def __init__(self, geometry, arm_length, torque2thrust_coef, radio_command, radio_receiver, imu, attitude_filter, flight_controller, motor_model,
                 attitude_estimation_freq: float = 250.0):
        self.mixer = Mixer(geometry, arm_length, torque2thrust_coef)
        self.radio_command = radio_command
        self.radio_receiver = radio_receiver
        self.imu = imu
        self.attitude_filter = attitude_filter
        self.rc_inputs = RCInputs()
        self.state = DroneState()
        self.attitude_estimation_freq = attitude_estimation_freq
        self.flight_controller = flight_controller
        self.motor_model = motor_model
