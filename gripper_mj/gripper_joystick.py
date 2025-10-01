import pygame
import time
from multiprocessing import Queue

def gripper_joystick_loop(q: Queue):
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
        # Right stick
        axis_lr = joystick.get_axis(2)  # [-1, 1]
        axis_fb = joystick.get_axis(3)  # [-1, 1]
        
        q.put(("updown", axis_ud))  
        q.put(("leftright", axis_lr))
        q.put(("forwardbackward", axis_fb))

        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} pressed")

                if event.button == 1:  # o
                    q.put(("save_step", 1))
                elif event.button == 2:  # square
                    q.put(("export_np", 1))
        time.sleep(0.01)  # 100 Hz update