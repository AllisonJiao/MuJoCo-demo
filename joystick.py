import pygame
import time
from multiprocessing import Queue

def joystick_loop(q: Queue):
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("⚠️ No joystick detected")
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"🎮 Joystick: {joystick.get_name()}")

    while True:
        pygame.event.pump()
        # Example: left stick Y-axis
        axis_val = joystick.get_axis(1)  # [-1, 1]
        q.put(axis_val)
        time.sleep(0.01)  # 100 Hz update