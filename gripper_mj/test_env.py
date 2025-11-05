from gripper_env import GripperEnv
import numpy as np

env = GripperEnv()

# Gym reset returns (obs, info)
obs, info = env.reset(seed=0)
print("obs shape:", obs.shape, "obs:", obs)
assert obs.shape == (4,), "Obs must be a flat 4-vector (block_x, block_y, grip_x, grip_y)"

# Roll a few random steps
for t in range(99):
    a = env.action_space.sample()  # 3-d action
    obs, reward, terminated, truncated, info = env.step(a)
    print(f"t={t:02d}  a={a}  reward={reward:.3f}  done={terminated}  trunc={truncated}  dist={info['distance']:.3f}")
    if terminated or truncated:
        break
