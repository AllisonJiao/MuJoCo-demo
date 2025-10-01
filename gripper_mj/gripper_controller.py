import time
import mujoco
import mujoco.viewer
import numpy as np
import os
import datetime
from npz_exporter import save_npz_to_mongo
from multiprocessing import Queue

# Deadzone threshold
DEADZONE = 0.1

def gripper_sim_loop(q: Queue):
    # Load model
    m = mujoco.MjModel.from_xml_path("model/GripperGPT.xml")
    d = mujoco.MjData(m)

    # Get actuator id for gripper_updown, gripper_leftright
    gripper_updown_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "up/down")
    gripper_leftright_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "left/right")

    # Get cube body id
    cube_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "block")
    
    trajectory = []

    with mujoco.viewer.launch_passive(m, d) as viewer:
        start = time.time()

        # Initialize control values
        axis_ud = 0.0
        axis_lr = 0.0

        while viewer.is_running() and time.time() - start < 600:
            step_start = time.time()

            # Consume joystick values if available
            while not q.empty():
                target, val = q.get_nowait()

                if target == "updown":
                    axis_ud = val
                elif target == "leftright":
                    axis_lr = val
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
                    # Ensure dataset directory exists
                    os.makedirs("dataset", exist_ok=True)

                    # Unique timestamped filename
                    timestamp = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
                    filepath = os.path.join("dataset", f"trajectory_{timestamp}.npz")

                    # Save everything into NumPy arrays
                    np.savez_compressed(
                        filepath,
                        time=np.array([step["time"] for step in trajectory], dtype=np.float32),
                        qpos=np.array([step["qpos"] for step in trajectory], dtype=np.float32),
                        qvel=np.array([step["qvel"] for step in trajectory], dtype=np.float32),
                        cube_pos=np.array([step["cube_pos"] for step in trajectory], dtype=np.float32),
                        cube_quat=np.array([step["cube_quat"] for step in trajectory], dtype=np.float32),
                        ctrl=np.array([step["ctrl"] for step in trajectory], dtype=np.float32),
                    )

                    print(f"✅ Exported trajectory with {len(trajectory)} steps to {filepath}")

                    # Now pass the same path to Mongo, comment out if not needed
                    save_npz_to_mongo(filepath)
            
            # Apply deadzone filter
            if abs(axis_ud) < DEADZONE:
                axis_ud = 0.0
            
            if abs(axis_lr) < DEADZONE:
                axis_lr = 0.0
            
            d.ctrl[gripper_updown_id] -= axis_ud * 0.05   # small step per tick
            d.ctrl[gripper_updown_id] = max(-15.0, min(15.0, d.ctrl[gripper_updown_id]))

            d.ctrl[gripper_leftright_id] += axis_lr * 0.05
            d.ctrl[gripper_leftright_id] = max(-10.0, min(10.0, d.ctrl[gripper_leftright_id]))

            mujoco.mj_step(m, d)
            viewer.sync()

            # Keep realtime pace
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
