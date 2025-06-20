from simpledrone.simulation import Task
from drone import Drone
from pathlib import Path
import struct
import socket
import subprocess

class SendRCInputsTask(Task):
    NUM_RC_CHANNELS = 8
    DEST_IP = "127.0.0.1"
    DEST_PORT = 9004

    def __init__(self, drone: Drone, next_task_date: float = 0.0):
        self.drone = drone
        self.next_task_date = next_task_date

    def get_next_task_date(self) -> float:
        return self.next_task_date

    def execute(self, t: float):
        rc_inputs = self.drone.radio_receiver.get_rc_inputs(t)
        channels = [rc_inputs.roll, rc_inputs.pitch, rc_inputs.yaw_rate, rc_inputs.throttle]
        channels += [0.0] * 4
        packet = struct.pack('<d' + 'H' * SendRCInputsTask.NUM_RC_CHANNELS, t, *channels)
        self.socket.sendto(packet, (SendRCInputsTask.DEST_IP, SendRCInputsTask.DEST_PORT))

        self.next_task_date += 1.0 / min(self.drone.radio_command.get_frequency(), self.drone.radio_receiver.get_frequency())


class SendFDMAndReceiveMotorPWMTask(Task):
    DEST_IP = "127.0.0.1"
    DEST_PORT = 9004

    def __init__(self, drone: Drone, next_task_date: float = 0.0):
        self.drone = drone
        self.next_task_date = next_task_date

    def get_next_task_date(self) -> float:
        return self.next_task_date

    def execute(self, t: float):
        accel, gyro = self.drone.imu.get_last_measurement()
        # time...
        # velocity
        # pressure
        packet = None
        self.socket.sendto(packet, (SendFDMAndReceiveMotorPWMTask.DEST_IP, SendFDMAndReceiveMotorPWMTask.DEST_PORT))

        # data, addr = sock.recvfrom(1024)
        # if len(data) != packet_size:
        #     print(f"[WARN] Received unexpected packet size: {len(data)} {time.time()}")
        #     continue

        # motor_count, *pwm_values = struct.unpack(fmt, data)
        current_rpms = self.drone.esc_motor_prop.compute_current_rpms(t - self.drone.motors_rpm.start_date, self.drone.motors_rpm.start, self.drone.motors_rpm.cmd)

        self.drone.motors_rpm.start = current_rpms
        self.drone.motors_rpm.start_date = t
        self.drone.motors_rpm.cmd = []

        # update des cmds


