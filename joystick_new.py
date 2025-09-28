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
        axis_left = joystick.get_axis(1)  # [-1, 1]
        
        q.put(("actuator_val", axis_left))

        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} pressed")

                if event.button == 1:  # o
                    q.put(("save_step", 1))
                elif event.button == 3:  # triangle
                    q.put(("up", 1))
                elif event.button == 2:  # square
                    q.put(("export_json", 1))
                elif event.button == 0:  # x
                    q.put(("down", 1))
        time.sleep(0.01)  # 100 Hz update