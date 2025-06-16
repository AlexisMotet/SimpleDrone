from events import Task
from drone import Drone



class ReceiveRCInputs(Task):
    def __init__(self, drone: Drone, next_t=0.0):
        self.drone = drone
        self.next_t = next_t

    def get_next_t(self) -> float:
        return self.next_t

    def execute(self, t):
        raw_cmds = self.drone.radio_command.read_rc_inputs(t)

        self.next_t = t + self.drone.radio_receiver.get_transmission_delay()

        self.drone.rc_inputs = self.drone.radio_receiver.receive(self.next_t, raw_cmds)


class EstimateAttitude(Task):
    def __init__(self, drone: Drone, next_t=0.0):
        self.drone = drone
        self.next_t = next_t

    def get_next_t(self) -> float:
        return self.next_t
    
    def execute(self, t):
        accel, gyro = self.drone.imu.read(t, self.drone.state.accel, 
                                          self.drone.state.orient_quat, self.drone.state.ang_vel)
        
        prior = self.drone.state.orient_quat

        accel_std, gyro_std = self.drone.imu.get_accel_std(), self.drone.imu.get_gyro_std()
        
        self.drone.state.orient_quat = self.drone.attitude_filter.estimate(t, prior, accel, gyro, accel_std, gyro_std)

        self.next_t = t + 1.0 / self.drone.attitude_estimation_freq
        

class UpdateMotorSpeeds(Task):
    def __init__(self, drone: Drone, next_t=0.0):
        self.drone = drone
        self.next_t = next_t

    def get_next_t(self) -> float:
        return self.next_t

    def execute(self, t):
        thrust_cmd = self.drone.mixer.get_num_motors() * self.drone.motor_model.compute_thrust(self.drone.rc_inputs.throttle)
        
        torque_cmd = self.drone.flight_controller.compute_torque_cmd(t, self.drone.rc_inputs, self.drone.state.orient_quat, self.drone.state.ang_vel)

        motors_throttles = self.drone.mixer.mix(thrust_cmd, torque_cmd)

        motors_rpm = []
        for throttle in motors_throttles:
            motors_rpm.append(self.drone.motor_model.compute_rpm(throttle))

        # Now i need to integrate



        self.next_t = t + 1.0 / 1000.0#