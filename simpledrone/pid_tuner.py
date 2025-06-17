import sys
import math
import matplotlib.pyplot as plt
from typing import List, Tuple
from simpledrone.maths import quaternions
from simpledrone.data import State
from simpledrone.integrator.euler_integrator import EulerIntegrator
from simpledrone.flight_controller.flight_controller import FlightController
from simpledrone.data import RCInputs

plt.ion() 

INTEGRATION_TIME = 3.0
INTEGRATION_STEP = 1e-3

MAX_ANGLE_DEG = 45.0
MAX_YAW_RATE_DEG = 200.0

AXES = ["roll", "pitch", "p", "q", "r"]
GAINS = {}
for axis in ["roll", "pitch"]:
    GAINS[axis] = {"P": 500.0, "I": 0.0, "D": 50.0}

for axis in ["p", "q"]:
    GAINS[axis] = {"P": 0.001, "I": 0.0, "D": 0.0}
GAINS["r"] = {"P": 0.001, "I": 0.0, "D": 0.0}

COMMANDS = {"roll": 35.0, "pitch": -20.0, "yaw": 100.0}
LINES = [None, None, None]
COMMAND_LINES = [None, None, None]


def integrate(frame, motor, rc_inputs, axis) -> Tuple[List[float], List[float]]: 
    fc = FlightController(roll_pid=(GAINS["roll"]["P"], GAINS["roll"]["I"], GAINS["roll"]["D"]), 
                          pitch_pid=(GAINS["pitch"]["P"], GAINS["pitch"]["I"], GAINS["pitch"]["D"]),
                        #   yaw_pid=(GAINS["yaw"]["P"], GAINS["yaw"]["I"], GAINS["yaw"]["D"]),
                          p_pid=(GAINS["p"]["P"], GAINS["p"]["I"], GAINS["p"]["D"]), 
                          q_pid=(GAINS["q"]["P"], GAINS["q"]["I"], GAINS["q"]["D"]),
                          r_pid=(GAINS["r"]["P"], GAINS["r"]["I"], GAINS["r"]["D"]), 
                          max_angle_deg=MAX_ANGLE_DEG, max_yaw_rate_deg=MAX_YAW_RATE_DEG)
    state = State()
    integrator = EulerIntegrator()
    inertia = frame.get_inertia()

    times = []
    angles = []
    time = 0.0
    while time < INTEGRATION_TIME:
        time += INTEGRATION_STEP
        thrust = 0.0
        print("MAX TORQUE: ", frame.get_max_torques(motor))
        torque = fc.compute_torque_cmd(time, rc_inputs, state.orient_quat, state.ang_vel, max_torques=frame.get_max_torques(motor))

        state = integrator.integrate(time, inertia, state, thrust, torque)

        if axis != 2:
            angles.append(math.degrees(quaternions.to_euler_angles(state.orient_quat)[axis]))
        else:
            angles.append(math.degrees(state.ang_vel[axis]))

        times.append(time)
    return times, angles

def plot(fig, axes, frame, motor):
    roll_input = RCInputs(throttle=0.0, roll=COMMANDS["roll"] / MAX_ANGLE_DEG, pitch=0.0, yaw_rate=0.0)
    pitch_input = RCInputs(throttle=0.0, roll=0.0, pitch=COMMANDS["pitch"] / MAX_ANGLE_DEG, yaw_rate=0.0)
    yaw_input = RCInputs(throttle=0.0, roll=0.0, pitch=0.0, yaw_rate=COMMANDS["yaw"] / MAX_YAW_RATE_DEG)
    for i, rc_inputs in enumerate([roll_input, pitch_input, yaw_input]):

        if i != 2:
            continue

        cmd = list(COMMANDS.values())[i]
        t, angles = integrate(frame, motor, rc_inputs, i)

        if LINES[i] is None:
            LINES[i], = axes[i].plot(t, angles)
            COMMAND_LINES[i], = axes[i].plot(t, [cmd] * len(t), label="cmd")
        else:
            LINES[i].set_ydata(angles)
            COMMAND_LINES[i].set_ydata([cmd] * len(t))
            fig.canvas.draw()
            fig.canvas.flush_events()
            axes[i].relim()
            axes[i].autoscale_view()


def show_table():
    print("\nPID table:")
    for axis in AXES:
        g = GAINS[axis]
        print(f"{axis:>5}: P={g['P']:7.3f}  I={g['I']:7.3f}  D={g['D']:7.3f}")
    print()
    print("Commands:")
    for axis, value in COMMANDS.items():
        print(f"{axis:>5}: {value:7.3f}")
    print()

def pid_tuner(frame, motor):
    fig, axes = plt.subplots(3, 1, figsize=(6, 8))

    show_table()
    plot(fig, axes, frame, motor)

    for i, angle in enumerate(["roll", "pitch", "yaw_rate"]):
        axes[i].set_ylabel(f"{angle} (degrees)")
        axes[i].grid()
        axes[i].legend(loc="upper right")
    axes[-1].set_xlabel(f"time (s)")

    while True:
        inpt = input("PID tuner -- <cmd>, <roll/pitch/p/q/r>: ").strip().lower()
        if inpt not in AXES + ["cmd"]:
            print("unknown command, try again")
            continue
    
        if inpt == "cmd":
            axis = input("what axis ? -- <roll/pitch/yaw>: ")
            if axis not in ["roll", "pitch", "yaw"]:
                print("unknown axis, try again")
                continue
            try:
                value = float(input("value (degrees): ").strip())
            except ValueError:
                print("not a number, try again")
                continue
            COMMANDS[axis] = value

        else:
            gain = input("what gain? -- <P/I/D>: ").strip().upper()
            if gain not in {"P", "I", "D"}:
                print("unknown gain, try again")
                continue
            try:
                value = float(input("value: ").strip())
            except ValueError:
                print("not a number, try again")
                continue

            GAINS[inpt][gain] = value
        show_table()
        plot(fig, axes, frame, motor)

if __name__ == "__main__":
    from simpledrone.frame import FPV4XSymmetric
    from simpledrone.esc_motor_prop.esc_motor_prop import ESCMotorProp

    frame = FPV4XSymmetric()
    motor = ESCMotorProp()
    pid_tuner(frame, motor)
