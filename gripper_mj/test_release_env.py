from gripper_env_release import GripperReleaseEnv
import numpy as np

print("Testing GripperReleaseEnv (Stage 4 - Two-Phase Release)...")

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
              f"in_pos={info.get('in_release_position', False)}  "
              f"released={info.get('block_released', False)}")

    if terminated or truncated:
        print(f"Episode ended at step {t}")
        break

# Test two-phase behavior
print("\n=== Testing Two-Phase Release Behavior ===")

# Phase 1: Position adjustment - move toward target while keeping fingers closed
print("\nPhase 1: Testing position adjustment (fingers should stay closed)...")
obs, info = env.reset(seed=123)
print(f"Initial state: grasped={env.initial_grasp_success}, horiz_dist={info.get('horizontal_dist', 0.0) if isinstance(info, dict) else 'N/A'}")

# Move toward target but try to open fingers (should be penalized)
for t in range(50):
    # Action: move gripper but try to open fingers
    action = np.array([0.0, 0.0, 0.0, -0.5])  # Try to open fingers
    obs, reward, terminated, truncated, info = env.step(action)

    if t % 10 == 0:
        print(f"t={t:02d}  horiz_dist={info.get('horizontal_dist', 0.0):.3f}  "
              f"finger_dist={info.get('finger_distance', 0.0):.3f}  "
              f"in_position={info.get('in_release_position', False)}  "
              f"finger_reward={info.get('finger_reward', 0.0):.3f}  "
              f"grasped={info.get('grasped', False)}")

    if terminated or truncated:
        print(f"Episode ended at step {t}")
        break

# Phase 2: Test release when in position
print("\nPhase 2: Testing release when in correct position...")
obs, info = env.reset(seed=456)

# First, try to get into position (keep fingers closed)
print("Step 1: Getting into position...")
for t in range(100):
    # Keep fingers closed while positioning
    action = np.array([0.0, 0.0, 0.0, 0.5])  # Keep fingers closed
    obs, reward, terminated, truncated, info = env.step(action)
    
    if info.get('in_release_position', False):
        print(f"t={t:02d}  IN POSITION! horiz_dist={info.get('horizontal_dist', 0.0):.3f}  "
              f"grasped={info.get('grasped', False)}")
        break

# Now try to release
print("Step 2: Releasing block...")
for t in range(100):
    # Open fingers to release
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
