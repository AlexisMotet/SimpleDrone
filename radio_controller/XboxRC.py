import sys
import pygame


class XboxRc:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.j = pygame.joystick.Joystick(0)
        self.j.init()
    def receive_rc_inputs(self):
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        pygame.event.pump()
        return {
            "throttle": -self.j.get_axis(1),
            "yaw": self.j.get_axis(0),
            "pitch": -self.j.get_axis(3),
            "roll": self.j.get_axis(2)
        }
