from dataclasses import dataclass

@dataclass
class CommandsToKeysQwerty:
    throttle_pos: str = "arrow_up"
    throttle_neg: str = "arrow_down"

    yaw_pos: str = "arrow_left"
    yaw_neg: str = "arrow_right"

    pitch_pos: str = "w"
    pitch_neg: str = "s"

    roll_pos: str = "a"
    roll_neg: str = "d"

@dataclass
class CommandsToKeysAzerty:
    throttle_pos: str = "arrow_up"
    throttle_neg: str = "arrow_down"

    yaw_pos: str = "arrow_left"
    yaw_neg: str = "arrow_right"

    pitch_pos: str = "z"
    pitch_neg: str = "s"

    roll_pos: str = "q"
    roll_neg: str = "d"
