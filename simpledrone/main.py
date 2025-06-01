import pygame
import time
from commands import KeyboardController

pygame.init()
WINDOW_SIZE = (400, 250)
pygame.display.set_mode(WINDOW_SIZE)
controller = KeyboardController.KeyboardController(keyboard_layout="azerty")

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            running = False
    drone_commands = controller.get_commands()

    print(drone_commands)

    # time.sleep(0.01)
