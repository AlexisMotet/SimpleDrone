import pygame
import time
from simpledrone.transferdata import DroneCommands


class KeyboardController:
    RAMP_RATE = 2.5  # fraction per second to reach full deflection
    RETURN_RATE = 3.0  # return‑to‑centre speed
    EXPO = 0.35  # 0 = linear, 1 = very soft centre

    def __init__(self, keyboard_layout="qwerty"):
        self._setup_keys(keyboard_layout)
        self.pitch_linear = 0.0  # -1...1
        self.roll_linear = 0.0  # -1...1
        self.yaw_linear = 0.0  # -1...1
        self.throttle_linear = 0.0  # 0...1
        self.last_read = time.monotonic_ns()

    def get_commands(self) -> DroneCommands:
        dt = self._update_time()
        keys = pygame.key.get_pressed()
        self._update_axes(keys, dt)
        self._update_throttle(keys, dt)
        pitch_shaped = KeyboardController._apply_exponential_shape(self.pitch_linear)
        roll_shaped = KeyboardController._apply_exponential_shape(self.roll_linear)
        yaw_shaped = KeyboardController._apply_exponential_shape(self.yaw_linear)
        return DroneCommands(pitch_shaped, roll_shaped, yaw_shaped, self.throttle_linear)

    @staticmethod
    def plot_exponential_shape():
        import matplotlib.pyplot as plt
        import numpy as np

        x = np.linspace(-1.0, 1.0, 1000)
        y = KeyboardController._apply_exponential_shape(x)
        plt.plot(x, y)
        plt.title("Exponential shaping of the commands")
        plt.show()

    def _update_time(self):
        now = time.monotonic_ns()
        dt = (now - self.last_read) / 1e9
        self.last_read = now
        return dt

    def _update_axes(self, keys, dt):
        self.pitch_linear = KeyboardController._update_axis(self.pitch_linear, keys[self.PITCH_POS], keys[self.PITCH_NEG], dt)
        self.roll_linear = KeyboardController._update_axis(self.roll_linear, keys[self.ROLL_POS], keys[self.ROLL_NEG], dt)
        self.yaw_linear = KeyboardController._update_axis(self.yaw_linear, keys[self.YAW_POS], keys[self.YAW_NEG], dt)

    @staticmethod
    def _update_axis(current: float, pos_key: int, neg_key: int, dt: float) -> float:
        if pos_key != neg_key:
            current += (pos_key - neg_key) * KeyboardController.RAMP_RATE * dt
        else:
            if current > 0:
                current = max(0.0, current - KeyboardController.RETURN_RATE * dt)
            elif current < 0:
                current = min(0.0, current + KeyboardController.RETURN_RATE * dt)
        return max(-1.0, min(1.0, current))

    def _update_throttle(self, keys, dt) -> float:
        pos_key, neg_key = keys[self.THROTTLE_POS], keys[self.THROTTLE_NEG]
        if pos_key != neg_key:
            self.throttle_linear += (pos_key - neg_key) * KeyboardController.RAMP_RATE * dt
        self.throttle_linear = max(0.0, min(1.0, self.throttle_linear))

    @staticmethod
    def _apply_exponential_shape(value: float) -> float:
        return (1 - KeyboardController.EXPO) * value + KeyboardController.EXPO * value**3

    def _setup_keys(self, keyboard_layout):
        if keyboard_layout == "qwerty":
            self.PITCH_POS = pygame.K_s
            self.PITCH_NEG = pygame.K_w
            self.ROLL_POS = pygame.K_d
            self.ROLL_NEG = pygame.K_a
            self.YAW_POS = pygame.K_e
            self.YAW_NEG = pygame.K_q
            self.THROTTLE_POS = pygame.K_UP
            self.THROTTLE_NEG = pygame.K_DOWN
        elif keyboard_layout == "azerty":
            self.PITCH_POS = pygame.K_s
            self.PITCH_NEG = pygame.K_z
            self.ROLL_POS = pygame.K_d
            self.ROLL_NEG = pygame.K_q
            self.YAW_POS = pygame.K_e
            self.YAW_NEG = pygame.K_a
            self.THROTTLE_POS = pygame.K_UP
            self.THROTTLE_NEG = pygame.K_DOWN
        else:
            raise ValueError(f"Unsupported keyboard layout: {keyboard_layout}")


if __name__ == "__main__":
    FPS = 60
    WINDOW_SIZE = (400, 250)
    BG_COLOR = (20, 20, 20)

    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("Keyboard RC Transmitter")
    font = pygame.font.SysFont(None, 24)
    clock = pygame.time.Clock()

    rc = KeyboardController(keyboard_layout="azerty")

    rc.plot_exponential_shape()

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0 # tick is in ms, dt is in s
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        screen.fill(BG_COLOR)

        commands = rc.get_commands()
        lines = [
            f"Pitch : {commands.pitch:+0.1f}",
            f"Roll : {commands.roll:+0.1f}",
            f"Yaw : {commands.yaw:+0.1f}",
            f"Throttle : {commands.throttle:0.1f}",
        ]
        for i, line in enumerate(lines):
            surf = font.render(line, True, (240, 240, 240))
            screen.blit(surf, (20, 30 + i * 30))
        pygame.display.flip()
    pygame.quit()
