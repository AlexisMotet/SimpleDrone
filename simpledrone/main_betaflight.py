from simpledrone.betaflight_sitl import BetaflightSITL
from simpledrone.betafligth_task import BetaflightTask
from simpledrone.drone import BetaflightDrone
from simpledrone.simulation import Simulation


simulation = Simulation()

drone0 = BetaflightDrone(radio_command=KeyboardRC(), esc_motor_prop=ESCMotorProp(), integrator=EulerIntegrator())


simulation.add_task(BetaflightTask())


with BetaflightSITL("/Users/alexismotet/Documents/drone_simulator/betaflight_4.6.0_SITL") as b:
    





    while True:
        continue
