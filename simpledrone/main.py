import time

from simpledrone.panda3d_gui import Panda3D_GUI
from drone import Drone
from simpledrone.radio_command.keyboard_rc import KeyboardRC
from simpledrone.radio_receiver.elrs_receiver import ExpressLRSReceiver
from simpledrone.imu.imu import IMU
from simpledrone.attitude_filter.ahrs_ekf import ahrsEKF
from simpledrone.integrator.euler_integrator import EulerIntegrator
from simpledrone.flight_controller.flight_controller import FlightController
from simpledrone.esc_motor_prop.esc_motor_prop import ESCMotorProp
from simpledrone.integrator.euler_integrator import EulerIntegrator

from simpledrone.frame import FPV4XSymmetric

drone0 = Drone(frame=FPV4XSymmetric(), radio_command=KeyboardRC(), radio_receiver=ExpressLRSReceiver(), 
               imu=IMU(), attitude_filter=ahrsEKF(), flight_controller=FlightController(), 
               esc_motor_prop=ESCMotorProp(), integrator=EulerIntegrator())

drones = [drone0]

gui = Panda3D_GUI()

gui.add_entity("drone0", "models/box", drone0.state.pos.tolist())


for drone in drones:
    if isinstance(drone.radio_command, KeyboardRC):
        gui.register_keys(drone.radio_command.get_keys_pressed())

from simpledrone.events import EventQueue
from simpledrone.tasks import ReceiveRCInputs, EstimateAttitude, CommandMotorSpeeds

simulation_queue = EventQueue()

for drone in drones:
    simulation_queue.add_task(ReceiveRCInputs(drone))
    simulation_queue.add_task(EstimateAttitude(drone, frequency=100.0))
    # simulation_queue.add_task(CommandMotorSpeeds(drone))

start = time.monotonic()

while not simulation_queue.empty():
    event = simulation_queue.pop()
    if event.t > 10.0:
        break
    event.execute()

    real_elapsed_time = time.monotonic() - start

    print(event.t, real_elapsed_time)

    time_to_sleep = event.t - real_elapsed_time
    if time_to_sleep < 0.0:
        print("!!!!!!!!")
    else:
        while time.monotonic() - start < event.t:
            pass
    
    # gui.step()
    

    
    # gui.step()

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