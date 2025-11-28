from gripper_env_release import GripperReleaseEnv
import numpy as np

print("Testing GripperReleaseEnv...")

env = GripperReleaseEnv()

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
              f"block_z={info.get('block_z', 0.0):.3f}  "
              f"finger_dist={info.get('finger_distance', 0.0):.3f}  "
              f"grasped={info.get('grasped', False)}  "
              f"released={info.get('block_released', False)}  "
              f"landed={info.get('block_landed', False)}")

    if terminated or truncated:
        print(f"Episode ended at step {t}")
        break

# Test release behavior by opening fingers
print("\nTesting release behavior (opening fingers)...")
obs, info = env.reset(seed=123)
print(f"Initial grasp: grasped={info.get('grasped', env.initial_grasp_success)}")

# Apply finger opening action
for t in range(100):
    # Action: [up/down, left/right, forward/back, finger]
    # Negative finger action opens fingers
    action = np.array([0.0, 0.0, 0.0, -1.0])  # Open fingers
    obs, reward, terminated, truncated, info = env.step(action)

    if t % 20 == 0:
        print(f"t={t:02d}  finger_dist={info.get('finger_distance', 0.0):.3f}  "
              f"block_z={info.get('block_z', 0.0):.3f}  "
              f"grasped={info.get('grasped', False)}  "
              f"released={info.get('block_released', False)}  "
              f"landed={info.get('block_landed', False)}")

    if terminated or truncated:
        print(f"Episode ended at step {t}: success={terminated}, truncated={truncated}")
        break

print("\n✅ Environment test passed!")
