from multiprocessing import Process, Queue
from joystick_new import joystick_loop
from franka_emika_panda_controller import sim_loop

if __name__ == "__main__":
    q = Queue()

    # Joystick in child process
    p1 = Process(target=joystick_loop, args=(q,))
    p1.start()

    # Simulation in main process (must be here for macOS GUI)
    sim_loop(q)

    p1.join()
