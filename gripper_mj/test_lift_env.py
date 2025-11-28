from gripper_env_lift_improved import GripperLiftEnv
import numpy as np

print("Testing GripperLiftEnv...")

env = GripperLiftEnv()

# Test reset
obs, info = env.reset(seed=42)
print(f"Observation shape: {obs.shape}")
print(f"Expected shape: (11,)")
assert obs.shape == (11,), f"Obs must be an 11-vector, got {obs.shape}"
print(f"Initial observation: {obs}")

# Test action space
print(f"\nAction space: {env.action_space}")
assert env.action_space.shape == (4,), "Action space should be 4D (up/down, left/right, forward/back, finger)"

# Test observation space
print(f"Observation space: {env.observation_space}")
assert env.observation_space.shape == (11,), "Observation space should be 11D"

# Test a few random steps
print("\nRunning 20 random steps...")
for t in range(20):
    a = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(a)
    
    if t % 5 == 0:
        print(f"t={t:02d}  reward={reward:.3f}  term={terminated}  trunc={truncated}  "
              f"horiz_dist={info.get('horizontal_dist', 0.0):.3f}  "
              f"height={info.get('block_height', 0.0):.3f}  "
              f"grasped={info.get('grasped', False)}")
    
    if terminated or truncated:
        print(f"Episode ended at step {t}")
        break

print("\n✅ Environment test passed!")
