import time
import mujoco
import mujoco.viewer
from multiprocessing import Queue

# Deadzone threshold
DEADZONE = 0.1

def sim_loop(q: Queue):
    # Load model
    m = mujoco.MjModel.from_xml_path("franka_emika_panda/mjx_panda.xml")
    d = mujoco.MjData(m)

    # Get actuator id
    curr_id = 0
    actuator_id = []
    actuator_name = ["actuator1", "actuator2", "actuator3", "actuator4", "actuator5", "actuator6", "actuator7", "actuator8"]
    for name in actuator_name:
        actuator_id.append(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name))

    with mujoco.viewer.launch_passive(m, d) as viewer:
        start = time.time()

        # Initialize control values
        axis_left = 0.0

        while viewer.is_running() and time.time() - start < 600:
            step_start = time.time()

            # Consume joystick values if available
            while not q.empty():
                target, val = q.get_nowait()

                if target == "actuator_val":
                    axis_left = val
                elif target == "up":
                    curr_id = max(0, curr_id - 1)
                elif target == "down":
                    curr_id = min(7, curr_id + 1)
            
            # Apply deadzone filter
            if abs(axis_left) < DEADZONE:
                axis_left = 0.0
            
            d.ctrl[actuator_id[curr_id]] -= axis_left * 0.05   # small step per tick

            mujoco.mj_step(m, d)
            viewer.sync()

            # Keep realtime pace
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
