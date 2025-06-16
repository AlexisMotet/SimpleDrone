import time
from dataclasses import asdict
from simpledrone.data import RCInputs
from simpledrone.radio_command.keyboard_data import CommandsToKeysAzerty


class KeyboardRC:
    KEYBOARD_FREQUENCY_HZ: float = 50.0

    def __init__(self, cmds_to_key: CommandsToKeysAzerty = CommandsToKeysAzerty(), time_to_satur: float=0.3, time_to_reset: float=0.3, 
                 expo_shaping: float = 0.35):
        
        self.keys_pressed = {}
        for key in asdict(cmds_to_key).values():
            self.keys_pressed[key] = 0
        
        self.cmds_to_key = cmds_to_key

        self.ramp_rate = 1.0 / time_to_satur
        self.return_rate = 1.0 / time_to_reset

        self.expo_shaping = expo_shaping

        self.axes = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0}

        self.prev_t = 0.0

    def get_frequency(self) -> float:
        return KeyboardRC.KEYBOARD_FREQUENCY_HZ

    def get_keys_pressed(self) -> dict[str, int]:
        return self.keys_pressed

    def read_rc_inputs(self):
        t = time.monotonic()
        dt = t - self.prev_t
        self.prev_t = t
        
        self.axes["roll"] = self._update_rc_input(self.axes["roll"],  dt, self.cmds_to_key.roll_pos, self.cmds_to_key.roll_neg)
        self.axes["pitch"] = self._update_rc_input(self.axes["pitch"], dt, self.cmds_to_key.pitch_pos, self.cmds_to_key.pitch_neg)
        self.axes["yaw"] = self._update_rc_input(self.axes["yaw"], dt, self.cmds_to_key.yaw_pos, self.cmds_to_key.yaw_neg)
        self.axes["throttle"] = self._update_rc_input(self.axes["throttle"], dt, self.cmds_to_key.throttle_pos, self.cmds_to_key.throttle_neg, 
                                                      min_val=0.0, max_val=1.0, return_=False)

        roll_shaped = self._apply_expo_shape(self.axes["roll"])
        pitch_shaped = self._apply_expo_shape(self.axes["pitch"])
        yaw_shaped = self._apply_expo_shape(self.axes["yaw"])

        return RCInputs(throttle=self.axes["throttle"], roll=roll_shaped, pitch=pitch_shaped, yaw_rate=yaw_shaped)
    
    def _update_rc_input(self, old_val: float, dt: float, pos_key: str, neg_key: str, min_val: float = -1.0, max_val: float = 1.0, return_=True):
        pos_v = self.keys_pressed[pos_key]
        neg_v = self.keys_pressed[neg_key]
        if pos_v != neg_v:
            new_val = old_val + (pos_v - neg_v) * self.ramp_rate * dt
        else:
            new_val = old_val
            if return_:
                if old_val > 0:
                    new_val = max(0.0, old_val - self.return_rate * dt)
                elif old_val < 0:
                    new_val = min(0.0, old_val + self.return_rate * dt)

        return max(min(new_val, max_val), min_val)

    def _apply_expo_shape(self, value: float) -> float:
        return (1 - self.expo_shaping) * value + self.expo_shaping * value**3
