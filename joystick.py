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
        # Left stick
        axis_ud = joystick.get_axis(1)  # [-1, 1]
        # Left stick
        axis_lr = joystick.get_axis(2)  # [-1, 1]
        
        q.put(("updown", axis_ud))  
        q.put(("leftright", axis_lr))
        time.sleep(0.01)  # 100 Hz update