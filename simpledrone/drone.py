import numpy as np
from data import State, RCInputs, MotorsRPM
from simpledrone.frame import Frame
from mixer.mixer import Mixer

class Drone:
    def __init__(self, frame: Frame, radio_command, radio_receiver, imu, attitude_filter, flight_controller, esc_motor_prop, integrator):
        self.frame = frame
        self.radio_command = radio_command
        self.radio_receiver = radio_receiver
        self.imu = imu
        self.attitude_filter = attitude_filter
        self.flight_controller = flight_controller
        self.esc_motor_prop = esc_motor_prop

        self.integrator = integrator

        self.motors_rpm = MotorsRPM(self.frame.get_num_motors())
        self.inertia = frame.get_inertia()
        self.mixer = Mixer(frame)
        self.state = State()
