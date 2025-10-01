import time
import mujoco
import mujoco.viewer
import numpy as np
from npz_exporter import save_npz_to_mongo
from multiprocessing import Queue

# Deadzone threshold
DEADZONE = 0.1

def franka_sim_loop(q: Queue):
    # Load model
    m = mujoco.MjModel.from_xml_path("model/franka_emika_panda/mjx_single_cube.xml")
    d = mujoco.MjData(m)

    # Get actuator id
    curr_id = 0
    actuator_id = []
    actuator_name = ["actuator1", "actuator2", "actuator3", "actuator4", "actuator5", "actuator6", "actuator7", "actuator8"]
    for name in actuator_name:
        actuator_id.append(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name))
    
    # Get cube body id
    cube_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cube")
    
    trajectory = []

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
                elif target == "save_step":
                    # Record state + action
                    obs = {
                        "time": d.time,
                        "qpos": np.array(d.qpos, dtype=np.float32),
                        "qvel": np.array(d.qvel, dtype=np.float32),
                        "cube_pos": np.array(d.xpos[cube_id], dtype=np.float32),
                        "cube_quat": np.array(d.xquat[cube_id], dtype=np.float32),
                        "ctrl": np.array(d.ctrl, dtype=np.float32)
                    }
                    trajectory.append(obs)
                    print(f"Saved step {len(trajectory)} at time {d.time:.2f}s")
                elif target == "export_np":
                    # Save everything into NumPy arrays
                    np.savez_compressed(
                        "trajectory.npz",
                        time=np.array([step["time"] for step in trajectory], dtype=np.float32),
                        qpos=np.array([step["qpos"] for step in trajectory], dtype=np.float32),
                        qvel=np.array([step["qvel"] for step in trajectory], dtype=np.float32),
                        cube_pos=np.array([step["cube_pos"] for step in trajectory], dtype=np.float32),
                        cube_quat=np.array([step["cube_quat"] for step in trajectory], dtype=np.float32),
                        ctrl=np.array([step["ctrl"] for step in trajectory], dtype=np.float32),
                    )
                    print("Exported trajectory with", len(trajectory), "steps to trajectory.npz")
                    save_npz_to_mongo("trajectory.npz")

            # Apply deadzone filter
            if abs(axis_left) < DEADZONE:
                axis_left = 0.0
            
            d.ctrl[actuator_id[curr_id]] -= axis_left * 0.01   # small step per tick

            mujoco.mj_step(m, d)
            viewer.sync()

            # Keep realtime pace
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
