import time

import mujoco
import mujoco.viewer

import glfw

m = mujoco.MjModel.from_xml_path('model/humanoid.xml')
d = mujoco.MjData(m)

# https://mujoco.readthedocs.io/en/stable/python.html#named-access
abdomen_z_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "abdomen_z")

paused = False
abdomen_z = False

def key_callback(keycode):
#   global paused
  global abdomen_z
#   if chr(keycode) == ' ':
#     paused = not paused
  if chr(keycode) == ' ':
    abdomen_z = not abdomen_z

with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
#   while viewer.is_running():
    step_start = time.time()

    if not paused:
      mujoco.mj_step(m, d)
      viewer.sync()
      if not abdomen_z:
        d.ctrl[abdomen_z_id] = 1.0
      else:
        d.ctrl[abdomen_z_id] = -0.5
    
    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    # mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    # with viewer.lock():
    #   viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    # viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)