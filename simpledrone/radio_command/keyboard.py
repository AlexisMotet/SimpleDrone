import time
from typing import Callable
from simpledrone.data import RCInputs
from simpledrone.radio_command.keyboard_data import CommandsToKeys


class Keyboard:
    DEFAULT_REPEAT_FREQUENCY: float = 15.0
    NUM_MEAS_REPEAT_FREQUENCY: int = 10

    def __init__(self, commands_to_keys: CommandsToKeys = CommandsToKeys(), 
                 time_to_saturation: float=0.3, time_to_reset: float=0.3, expo: float = 0.35):
        
        self.commands_to_keys = commands_to_keys

        self.time_to_satur = time_to_saturation
        self.increment = 1.0 / (Keyboard.DEFAULT_REPEAT_FREQUENCY * time_to_saturation)

        self.return_rate = 1.0 / time_to_reset

        self.expo = expo

        self.pitch_axis = 0.0        # -1...1
        self.roll_axis = 0.0         # -1...1
        self.yaw_axis = 0.0          # -1...1
        self.throttle_axis = 0.0     #  0...1

        self.modified = set()

        self.last_update = 0.0

        self.repeat_freq = None
        self.measured_freqs = [] # to determine real repeat (key held) frequency
        self.last_update_axes = {"pitch_axis": None, "roll_axis": None, "yaw_axis": None, "throttle_axis": None}



    def get_key_action_dict(self) -> dict[str, Callable]:
        return {
            self.commands_to_keys.pitch_pos: lambda: self._update_axis("pitch_axis", self.increment, -1.0, 1.0),
            self.commands_to_keys.pitch_neg: lambda: self._update_axis("pitch_axis", -self.increment, -1.0, 1.0),

            self.commands_to_keys.roll_pos: lambda: self._update_axis("roll_axis", self.increment, -1.0, 1.0),
            self.commands_to_keys.roll_neg: lambda: self._update_axis("roll_axis", -self.increment, -1.0, 1.0),

            self.commands_to_keys.yaw_pos: lambda: self._update_axis("yaw_axis", self.increment, -1.0, 1.0),
            self.commands_to_keys.yaw_neg: lambda: self._update_axis("yaw_axis", -self.increment, -1.0, 1.0),

            self.commands_to_keys.throttle_pos: lambda: self._update_axis("throttle_axis", self.increment, -1.0, 1.0),
            self.commands_to_keys.throttle_neg: lambda: self._update_axis("throttle_axis", -self.increment, -1.0, 1.0),
        }
    
    def _update_axis(self, axis_name: str, incr: float, min_val: float, max_val: float):
        print(axis_name)
        axis = getattr(self, axis_name)
        setattr(self, axis_name, max(min(axis + incr, min_val), max_val))
        self.modified.add(axis)

        if self.repeat_freq is None:
            if self.last_update_axes[axis_name] is not None:
                dt = (time.monotonic_ns() - self.last_update_axes[axis_name]) / 1e9
                freq = 1.0 / dt
                self.measured_freqs.append(freq)

            self.last_update_axes[axis_name] = time.monotonic_ns()

            if len(self.measured_freqs) == Keyboard.NUM_MEAS_REPEAT_FREQUENCY:
                self.repeat_freq = sum(self.measured_freqs) / len(self.measured_freqs)
                print("repeat freq found: ", self.repeat_freq)
                self.increment = 1.0 / (freq * self.time_to_satur)


    def read_rc_inputs(self, t):
        dt = t - self.last_update
        self.last_update = t

        for axis in [self.pitch_axis, self.roll_axis, self.yaw_axis]:
            if axis not in self.modified:
                if axis > 0:
                    axis = max(0.0, axis - self.return_rate * dt)
                elif axis < 0:
                    axis = min(0.0, axis + self.return_rate * dt)

        self.modified = set()

        pitch_shaped = self._apply_expo_shape(self.pitch_axis)
        roll_shaped = self._apply_expo_shape(self.roll_axis)
        yaw_shaped = self._apply_expo_shape(self.yaw_axis)

        return RCInputs(throttle=self.throttle_axis, pitch=pitch_shaped, roll=roll_shaped, yaw_rate=yaw_shaped)

    def _apply_expo_shape(self, value: float) -> float:
        return (1 - self.expo) * value + self.expo * value**3
