import time
import mujoco
import mujoco.viewer
import numpy as np
import os
import datetime
#from npz_exporter import save_npz_to_mongo
from multiprocessing import Queue

def rand_spawn(m, d):
    # Spawn gripper, block, and the target in random positions

    # Block
    block_joint = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "block_free")
    block_adr = m.jnt_qposadr[block_joint]
    # Set (x, y, z)
    d.qpos[block_adr:block_adr+3] = [
        np.random.uniform(-0.5, 0.5),   # x
        np.random.uniform(-0.5, 0.5),   # y
        0.05                            # z
    ]
    # Set orientation quaternion (w, x, y, z) = identity
    d.qpos[block_adr+3:block_adr+7] = [1, 0, 0, 0]
    # Force MuJoCo to recompute positions
    mujoco.mj_forward(m, d)

    # Target
    target_joint = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "target_free")
    target_adr = m.jnt_qposadr[target_joint]
    # Set (x, y, z)
    d.qpos[target_adr:target_adr+3] = [
        np.random.uniform(-0.5, 0.5),   # x
        np.random.uniform(-0.5, 0.5),   # y
        0.05                            # z
    ]
    # Set orientation quaternion (w, x, y, z) = identity
    d.qpos[target_adr+3:target_adr+7] = [1, 0, 0, 0]
    # Force MuJoCo to recompute positions
    mujoco.mj_forward(m, d)

# Deadzone threshold
DEADZONE = 0.1

def gripper_sim_loop(q: Queue):
    # Load model
    m = mujoco.MjModel.from_xml_path("model/GripperGPT.xml")
    d = mujoco.MjData(m)

    # Randomize spawn before simulation starts
    rand_spawn(m, d)

    # Get actuator id for gripper_updown, gripper_leftright
    gripper_updown_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "up/down")
    gripper_leftright_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "left/right")
    gripper_forwardbackward_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "forward/backward")

    # Get cube body id
    cube_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "block")
    # Get target id
    target_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "target")
    
    trajectory = []

    with mujoco.viewer.launch_passive(m, d) as viewer:
        start = time.time()

        # Initialize control values
        axis_ud = 0.0
        axis_lr = 0.0
        axis_fb = 0.0

        while viewer.is_running() and time.time() - start < 600:
            step_start = time.time()

            # Consume joystick values if available
            while not q.empty():
                target, val = q.get_nowait()

                if target == "updown":
                    axis_ud = val
                elif target == "leftright":
                    axis_lr = val
                elif target == "forwardbackward":
                    axis_fb = val
                elif target == "save_step":
                    # Record state + action
                    obs = {
                        "time": d.time,
                        "qpos": np.array(d.qpos, dtype=np.float32),
                        "qvel": np.array(d.qvel, dtype=np.float32),
                        "cube_pos": np.array(d.xpos[cube_id], dtype=np.float32),
                        "cube_quat": np.array(d.xquat[cube_id], dtype=np.float32),
                        "target_pos": np.array(d.xpos[target_id], dtype=np.float32),
                        "target_quat": np.array(d.xquat[target_id], dtype=np.float32),
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
                        target_pos= np.array([step["target_pos"] for step in trajectory], dtype=np.float32),
                        target_quat= np.array([step["target_quat"] for step in trajectory], dtype=np.float32),
                        ctrl=np.array([step["ctrl"] for step in trajectory], dtype=np.float32),
                    )

                    print(f"✅ Exported trajectory with {len(trajectory)} steps to {filepath}")

                    # Now pass the same path to Mongo, comment out if not needed
                    # save_npz_to_mongo(filepath)
            
            # Apply deadzone filter
            if abs(axis_ud) < DEADZONE:
                axis_ud = 0.0
            
            if abs(axis_lr) < DEADZONE:
                axis_lr = 0.0

            if abs(axis_fb) < DEADZONE:
                axis_fb = 0.0
            
            d.ctrl[gripper_updown_id] -= axis_ud * 0.05   # small step per tick
            d.ctrl[gripper_updown_id] = max(-15.0, min(15.0, d.ctrl[gripper_updown_id]))

            d.ctrl[gripper_leftright_id] += axis_lr * 0.05
            d.ctrl[gripper_leftright_id] = max(-10.0, min(10.0, d.ctrl[gripper_leftright_id]))

            d.ctrl[gripper_forwardbackward_id] -= axis_fb * 0.05
            d.ctrl[gripper_forwardbackward_id] = max(-10.0, min(10.0, d.ctrl[gripper_forwardbackward_id]))

            mujoco.mj_step(m, d)
            viewer.sync()

            # Keep realtime pace
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
