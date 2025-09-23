import time
import mujoco
import mujoco.viewer
from multiprocessing import Queue

# Deadzone threshold
DEADZONE = 0.1

def sim_loop(q: Queue):
    # Load model
    m = mujoco.MjModel.from_xml_path("model/GripperGPT.xml")
    d = mujoco.MjData(m)

    # Get actuator id for gripper_updown, gripper_leftright
    gripper_updown_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "up/down")
    gripper_leftright_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "left/right")
    gripper_leftfinger_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_finger")
    gripper_rightfinger_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_finger")
    # abdomen_z_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "abdomen_z")

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
                elif target == "leftup":
                    d.ctrl[gripper_leftfinger_id] += 0.01
                    d.ctrl[gripper_leftfinger_id] = min(1.0, d.ctrl[gripper_leftfinger_id])
                elif target == "rightup":
                    d.ctrl[gripper_rightfinger_id] += 0.01
                    d.ctrl[gripper_rightfinger_id] = min(1.0, d.ctrl[gripper_rightfinger_id])
                elif target == "leftdown":
                    d.ctrl[gripper_leftfinger_id] -= 0.01
                    d.ctrl[gripper_leftfinger_id] = max(-0.5, d.ctrl[gripper_leftfinger_id])
                elif target == "rightdown":
                    d.ctrl[gripper_rightfinger_id] -= 0.01
                    d.ctrl[gripper_rightfinger_id] = max(-0.5, d.ctrl[gripper_rightfinger_id])
            
            # Apply deadzone filter
            if abs(axis_ud) < DEADZONE:
                axis_ud = 0.0
            
            if abs(axis_lr) < DEADZONE:
                axis_lr = 0.0
            
            d.ctrl[gripper_updown_id] -= axis_ud * 0.05   # small step per tick
            d.ctrl[gripper_updown_id] = max(-15.0, min(15.0, d.ctrl[gripper_updown_id]))

            d.ctrl[gripper_leftright_id] += axis_lr * 0.05
            d.ctrl[gripper_leftright_id] = max(-6.0, min(6.0, d.ctrl[gripper_leftright_id]))

            mujoco.mj_step(m, d)
            viewer.sync()

            # Keep realtime pace
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
