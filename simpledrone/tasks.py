from events import Task
from drone import Drone


class ReceiveRCInputs(Task):
    def __init__(self, drone: Drone, next_t: float = 0.0):
        self.drone = drone
        self.next_t = next_t

    def get_next_task_date(self) -> float:
        return self.next_t

    def execute(self, t: float):
        raw_cmds = self.drone.radio_command.read_rc_inputs()
        print(raw_cmds)

        self.drone.radio_receiver.receive(t, raw_cmds)

        self.next_t += 1.0 / min(self.drone.radio_command.get_frequency(), self.drone.radio_receiver.get_frequency())


class EstimateAttitude(Task):
    def __init__(self, drone: Drone, frequency: float, next_t: float = 0.0):
        self.drone = drone
        self.frequency = frequency
        self.next_t = next_t

    def get_next_task_date(self) -> float:
        return self.next_t
    
    def execute(self, t: float):
        integrate_drone_state(t, self.drone)

        s = self.drone.state

        accel_minus_gravity, gyro = self.drone.imu.read(t=t, accel_world=s.accel, quat_world_to_body=s.orient_quat, ang_vel_body=s.ang_vel)

        accel_std, gyro_std = self.drone.imu.get_std()
        self.drone.attitude_filter.estimate(t=t, accel=accel_minus_gravity, gyro=gyro, accel_std=accel_std, gyro_std=gyro_std)

        self.next_t = t + 1.0 / self.frequency


class CommandMotorSpeeds(Task):
    def __init__(self, drone: Drone, next_t: float = 0.0):
        self.drone = drone
        self.next_t = next_t

    def get_next_task_date(self) -> float:
        return self.next_t

    def execute(self, t: float):
        integrate_drone_state(t, self.drone)

        rc_inputs = self.drone.radio_receiver.get_rc_inputs(t)

        thrust_cmd = self.drone.frame.get_num_motors() * self.drone.esc_motor_prop.estimate_thrust(rc_inputs.throttle)

        estimated_orient = self.drone.attitude_filter.get_estimated_orientation()
        gyro_output = self.drone.imu.get_gyro()
        torque_cmd = self.drone.flight_controller.compute_torque_cmd(t, rc_inputs, estimated_orient, gyro_output)

        torque_cmd = (0, 0, 0)

        current_rpms = self.drone.esc_motor_prop.compute_current_rpms(t - self.drone.motors_rpm.start_date, self.drone.motors_rpm.start, self.drone.motors_rpm.cmd)
        torque_coefs = self.drone.esc_motor_prop.estimate_torque_coefs_from_current_rpms(current_rpms)

        self.drone.motors_rpm.start = current_rpms
        self.drone.motors_rpm.start_date = t
        self.drone.motors_rpm.cmd = []

        throttles_cmd = self.drone.mixer.mix(thrust_cmd, torque_cmd, torque_coefs)

        for throttle_cmd in throttles_cmd:
            rpm_cmd = self.drone.esc_motor_prop.estimate_rpm_from_throttle(throttle_cmd)
            self.drone.motors_rpm.cmd.append(rpm_cmd)

        self.next_t += 1 / self.drone.esc_motor_prop.get_pwm_frequency()
        

def integrate_drone_state(t: float, drone: Drone):
    current_rpms = drone.esc_motor_prop.compute_current_rpms(t - drone.motors_rpm.start_date, drone.motors_rpm.start, drone.motors_rpm.cmd)

    motor_thrusts = []

    for rpm in current_rpms:
        motor_thrust = drone.esc_motor_prop.estimate_thrust_from_rpm(rpm)
        motor_thrusts.append(motor_thrust)

    torque_coefs = drone.esc_motor_prop.estimate_torque_coefs_from_current_rpms(current_rpms)

    global_thrust, global_torque = drone.mixer.unmix(motor_thrusts, torque_coefs)

    drone.state = drone.integrator.integrate(t, drone.inertia, drone.state, global_thrust, global_torque)
