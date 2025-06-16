import time

from simpledrone.panda3d_gui import Panda3D_GUI
from drone import Drone
from simpledrone.radio_command.keyboard import Keyboard
from simpledrone.radio_receiver.elrs_receiver import ExpressLRSReceiver
from simpledrone.flight_controller.basic_fc import BasicFC
from simpledrone.imu.basic_imu import BasicIMU
from simpledrone.attitude_filter.ahrs_ekf import AHRS_EKF
from simpledrone.esc.basic_esc import BasicESC
from simpledrone.motors.basic_motors import BasicMotors
from simpledrone.integrator.euler_integrator import EulerIntegrator
from simpledrone.flight_controller.flight_controller import FlightController
from simpledrone.motor_model.basic_motor_model import BasicMotorModel

from simpledrone.data import DroneConfig

drone0 = Drone(geometry="quadx", arm_length=0.2, torque2thrust_coef=1.0, 
               radio_command=Keyboard(), radio_receiver=ExpressLRSReceiver(), 
               imu=BasicIMU(), attitude_filter=AHRS_EKF(),
               flight_controller=FlightController(), motor_model=BasicMotorModel())

drones = [drone0]

gui = Panda3D_GUI()

integrator = EulerIntegrator()

gui.add_entity("drone0", "models/box", drone0.state.pos.tolist())

for drone in drones:
    if isinstance(drone.radio_command, Keyboard):
        key_action_dict = drone.radio_command.get_key_action_dict()
        for key, action in key_action_dict.items():
            gui.register_action_on_key_pressed(key, action)

from simpledrone.events import EventQueue
from simpledrone.drone_tasks import ReceiveRCInputs, EstimateAttitude, UpdateMotorSpeed

simulation_queue = EventQueue()

for drone in drones:
    simulation_queue.add_task(ReceiveRCInputs(drone))
    simulation_queue.add_task(EstimateAttitude(drone))
    simulation_queue.add_task(UpdateMotorSpeeds(drone))

while not simulation_queue.empty():
    event = simulation_queue.pop()
    print(event.t)
    if event.t > 10.0:
        break
    event.execute()
    gui.step()

# simulation_time = 0.0

# while True:

#     # t = time.monotonic_ns() / 1e9

#     for drone in drones: 

#         drone_time = simulation_time
        
#         raw_cmds = drone.radio_command.read_commands(drone_time)

#         drone_time += drone.radio_receiver.get_transmission_delay()

#         cmds = drone.radio_receiver.receive(drone_time, raw_cmds)

#         drone_time += drone.radio_receiver.get_rx_to_fc_delay() # what do i put here ?

#         accel, gyro = drone.imu.read(drone_time, drone.state.accel, drone.state.orient, drone.state.ang_vel)

#         print(accel, gyro)

#         # pwm = drone.flight_controller.update(drone_time, cmds)

#         # drone_time += delay

#         # rpm = drone.esc.update(pwm)

#         # drone.motors.update(rpm)

#         # drone.state = integrator.integrate(drone.state)

#         # gui.move_entity("drone0", drone.state.pos.tolist())

#     gui.step()