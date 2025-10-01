import argparse
from multiprocessing import Process, Queue

# Import loops
from joystick_new import joystick_loop           # Franka joystick
from franka_emika_panda_controller import sim_loop
from gripper_v1_code.gripper_controller import gripper_sim_loop
from gripper_v1_code.joystick import gripper_joystick_loop   # Gripper joystick

def main():
    parser = argparse.ArgumentParser(description="Run Mujoco simulation with different controllers")

    parser.add_argument(
        "--control",
        choices=["franka", "gripper"],
        default="franka",
        help="Choose which simulation control loop to run (franka or gripper)."
    )

    args = parser.parse_args()
    q = Queue()

    # Pair controller with joystick
    if args.control == "franka":
        joystick_fn = joystick_loop
        sim_fn = sim_loop
    else:
        joystick_fn = gripper_joystick_loop
        sim_fn = gripper_sim_loop

    # Joystick in child process
    p1 = Process(target=joystick_fn, args=(q,))
    p1.start()

    # Run the selected simulation loop
    sim_fn(q)

    p1.join()

if __name__ == "__main__":
    main()
