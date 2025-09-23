import time
import mujoco
import mujoco.viewer
from multiprocessing import Queue

# Deadzone threshold
DEADZONE = 0.1

def sim_loop(q: Queue):
    # Load model
    m = mujoco.MjModel.from_xml_path("model/humanoid.xml")
    d = mujoco.MjData(m)

    # Get actuator id for abdomen_z
    abdomen_z_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "abdomen_z")

    with mujoco.viewer.launch_passive(m, d) as viewer:
        start = time.time()
        while viewer.is_running() and time.time() - start < 600:
            step_start = time.time()

            # Consume joystick values if available
            while not q.empty():
                axis_val = q.get_nowait()

                # Apply deadzone filter
                if abs(axis_val) < DEADZONE:
                    axis_val = 0.0

                # Incremental control instead of absolute mapping
                d.ctrl[abdomen_z_id] += axis_val * 0.01   # small step per tick

                # Optional: clamp to actuator range (e.g. [-1, 1])
                d.ctrl[abdomen_z_id] = max(-1.0, min(1.0, d.ctrl[abdomen_z_id]))


            mujoco.mj_step(m, d)
            viewer.sync()

            # Keep realtime pace
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
