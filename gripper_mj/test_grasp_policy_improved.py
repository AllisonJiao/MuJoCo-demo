"""
Test script for improved grasping policy (Env B - Improved).

Tests a trained grasping policy on the improved grasp-only environment where
the gripper starts already positioned above the block.
Uses velocity-based observations and improved reward shaping.

Based on GripperGPT.xml model:
- Left finger: at x=-0.06m from gripper center, slides along x-axis
- Right finger: at x=0.06m from gripper center, slides along x-axis
- Finger size: 0.02 x 0.04 x 0.08m (half-extents)
- Block size: 0.05 x 0.05 x 0.05m (half-extents)
- Inner sides face each other (left finger's right side, right finger's left side)
"""

import os
import argparse
import numpy as np
import cv2
from stable_baselines3 import PPO
from gripper_grasp_env_improved import GripperGraspEnv, MAX_STEPS

# Test configuration
VALID_EPS = 100
VALID_MAX_STEPS = MAX_STEPS
CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), "checkpoints_grasp")
VIDEO_DIR = os.path.join(os.path.dirname(__file__), "videos_grasp")

# Geometry constants from GripperGPT.xml
FINGER_SEPARATION_BASE = 0.12  # 0.06m * 2 (distance between finger centers when at rest)
FINGER_WIDTH = 0.04  # Finger half-extent in x direction (0.02m * 2)
BLOCK_SIZE = 0.10  # Block half-extent (0.05m * 2)
MIN_FINGER_GAP = 0.0  # Minimum gap when fingers are fully closed
MAX_FINGER_GAP = FINGER_SEPARATION_BASE + 0.04  # Maximum gap when fully open


def test_grasp_policy(
    checkpoint_path,
    num_episodes=VALID_EPS,
    render_video=False,
    stochastic=False,
    allow_xy_adjust=False,
    debug_log=False,
):
    """
    Test an improved grasping policy on the improved grasp-only environment.
    
    Args:
        checkpoint_path: Path to the trained grasping model (.zip)
        num_episodes: Number of test episodes
        render_video: Whether to save videos
        stochastic: Use stochastic actions (for exploration)
        allow_xy_adjust: Whether to allow XY adjustments (must match training)
        debug_log: Print detailed per-step diagnostics for first episode
    """
    print("=" * 60)
    print("Testing Improved Grasping Policy (Env B - Improved)")
    print("=" * 60)
    print(f"Checkpoint: {checkpoint_path}")
    print(f"Episodes: {num_episodes}")
    print(f"Allow XY adjust: {allow_xy_adjust}")
    print(f"Stochastic: {stochastic}")
    print(f"Debug log: {debug_log}")
    print("=" * 60)
    
    # Load model first to check observation space and determine correct environment config
    print(f"\nLoading model from {checkpoint_path}...")
    try:
        # Load model without environment first to check its observation space
        model = PPO.load(checkpoint_path)
        model_obs_shape = model.observation_space.shape[0]
        print(f"Model observation space: {model.observation_space}")
        print(f"Model action space: {model.action_space}")
        
        # Determine if model was trained with XY adjust based on observation space
        # 13 dims = with XY adjust, 8 dims = without XY adjust
        if model_obs_shape == 13:
            detected_xy_adjust = True
            print(f"Detected: Model was trained with XY adjust (obs_dim={model_obs_shape})")
        elif model_obs_shape == 8:
            detected_xy_adjust = False
            print(f"Detected: Model was trained without XY adjust (obs_dim={model_obs_shape})")
        else:
            detected_xy_adjust = allow_xy_adjust  # Fallback to user setting
            print(f"Warning: Unknown observation space shape {model_obs_shape}, using user setting: allow_xy_adjust={allow_xy_adjust}")
        
        # Warn if user setting doesn't match model
        if detected_xy_adjust != allow_xy_adjust:
            print(f"\n⚠️  WARNING: Model was trained with allow_xy_adjust={detected_xy_adjust}, but you specified {allow_xy_adjust}")
            print(f"   Using detected setting: allow_xy_adjust={detected_xy_adjust}")
            allow_xy_adjust = detected_xy_adjust
        
    except Exception as e:
        print(f"Error loading model: {e}")
        raise
    
    # Create environment with correct settings (matching the model)
    render_mode = "rgb_array" if render_video else None
    if render_video:
        env = GripperGraspEnv(allow_xy_adjust=allow_xy_adjust, render_mode=render_mode, width=1280, height=720)
    else:
        env = GripperGraspEnv(allow_xy_adjust=allow_xy_adjust, render_mode=render_mode)
    
    # Note: We don't need to set_env for prediction, model.predict() works without it
    # Setting env is only needed for training/learning
    print("Model ready for testing (env not required for prediction)")
    
    # Create video directory if needed
    if render_video:
        os.makedirs(VIDEO_DIR, exist_ok=True)
        print(f"Videos will be saved to {VIDEO_DIR}")
    
    # Test statistics
    total_rewards = []
    successes = 0
    episode_lengths = []
    grasp_times = []  # Steps until successful grasp
    
    # Additional statistics from improved environment
    progress_rewards = []
    stuck_penalties = []
    precision_bonuses = []
    final_velocities = []
    ground_contact_counts = []
    finger_contact_counts = []
    
    # Finger geometry statistics (based on XML structure)
    finger_separation_distances = []
    left_finger_inner_contacts = []
    right_finger_inner_contacts = []
    
    print(f"\nRunning {num_episodes} test episodes...\n")
    
    for episode in range(num_episodes):
        obs, info = env.reset()
        total_reward = 0.0
        episode_length = 0
        grasped = False
        frames = []
        
        log_this_ep = debug_log and episode == 0
        
        # Track per-episode statistics
        ep_progress = []
        ep_stuck = []
        ep_precision = []
        ep_ground_contact = []
        ep_finger_contact = []
        ep_finger_separation = []
        ep_left_inner_contact = []
        ep_right_inner_contact = []
        
        for step in range(VALID_MAX_STEPS):
            # Get action from policy
            action, _ = model.predict(obs, deterministic=(not stochastic))
            
            # Step environment
            obs, reward, terminated, truncated, info = env.step(action)
            
            total_reward += reward
            episode_length += 1
            
            # Track detailed statistics
            ep_progress.append(info.get("progress_reward", 0.0))
            ep_stuck.append(info.get("stuck_penalty", 0.0))
            ep_precision.append(info.get("precision_bonus", 0.0))
            ep_ground_contact.append(info.get("ground_contact", False))
            ep_finger_contact.append(info.get("both_fingers_contact", False))
            
            # Track finger geometry (based on XML structure from GripperGPT.xml)
            # Get finger positions from environment data
            try:
                left_finger_pos = env.data.xpos[env.left_finger][:2]  # XY only
                right_finger_pos = env.data.xpos[env.right_finger][:2]  # XY only
                finger_separation = np.linalg.norm(right_finger_pos - left_finger_pos)
                ep_finger_separation.append(finger_separation)
            except (AttributeError, IndexError):
                # Fallback if finger positions aren't accessible
                ep_finger_separation.append(np.nan)
            
            # Track finger contacts (from info dict)
            # Note: These are any contacts, inner-side checking is done in _check_grasped()
            ep_left_inner_contact.append(info.get("left_finger_contact", False))
            ep_right_inner_contact.append(info.get("right_finger_contact", False))
            
            # Check if grasped
            if info.get("grasped", False) and not grasped:
                grasped = True
                grasp_times.append(episode_length)
            
            # Debug logging for first episode
            if log_this_ep and step % 10 == 0:
                print(f"  Step {step}: action={action} vertical_dist={info.get('vertical_dist', 0.0):.4f} "
                      f"horizontal_dist={info.get('horizontal_dist', 0.0):.4f} "
                      f"grasped={info.get('grasped', False)} "
                      f"terminated={terminated} "
                      f"ground_contact={info.get('ground_contact', False)} "
                      f"both_fingers={info.get('both_fingers_contact', False)} "
                      f"reward={reward:.4f} "
                      f"(progress={info.get('progress_reward', 0.0):.3f} "
                      f"stuck={info.get('stuck_penalty', 0.0):.3f} "
                      f"precision={info.get('precision_bonus', 0.0):.3f})")
            
            # Record frame if rendering
            if render_video:
                frame = env.render()
                if frame is not None:
                    frames.append(frame)
            
            # Episode ended
            if terminated or truncated:
                # If terminated, it means successful grasp (terminated = grasped in the env)
                # Double-check in case we missed it during the episode
                if terminated and not grasped:
                    grasped = True
                    if episode_length not in grasp_times:  # Avoid duplicate entries
                        grasp_times.append(episode_length)
                break
        
        # Update statistics
        total_rewards.append(total_reward)
        episode_lengths.append(episode_length)
        # Count success if grasped (terminated flag indicates successful grasp)
        if grasped:
            successes += 1
        
        # Aggregate per-episode statistics
        progress_rewards.append(np.sum(ep_progress))
        stuck_penalties.append(np.sum(ep_stuck))
        precision_bonuses.append(np.sum(ep_precision))
        ground_contact_counts.append(sum(ep_ground_contact))
        finger_contact_counts.append(sum(ep_finger_contact))
        
        # Finger geometry statistics
        if ep_finger_separation:
            valid_separations = [s for s in ep_finger_separation if not np.isnan(s)]
            if valid_separations:
                finger_separation_distances.append(np.mean(valid_separations))
        left_finger_inner_contacts.append(sum(ep_left_inner_contact))
        right_finger_inner_contacts.append(sum(ep_right_inner_contact))
        
        # Store final velocity if available
        final_vel = info.get("gripper_velocity", None)
        if final_vel is not None and not (isinstance(final_vel, np.ndarray) and np.isnan(final_vel).any()):
            if isinstance(final_vel, np.ndarray):
                final_velocities.append(np.linalg.norm(final_vel))
            else:
                final_velocities.append(abs(final_vel))
        
        # Print episode result
        status = "✓ GRASPED" if grasped else "✗ FAILED"
        valid_seps = [s for s in ep_finger_separation if not np.isnan(s)]
        avg_separation = np.mean(valid_seps) if valid_seps else 0.0
        print(f"Episode {episode+1:3d}: {status:12s} | "
              f"Reward: {total_reward:7.2f} | "
              f"Steps: {episode_length:3d} | "
              f"Vertical: {info.get('vertical_dist', 0):.3f} | "
              f"Finger: {info.get('finger_state', 0):.3f} | "
              f"Separation: {avg_separation:.3f}m | "
              f"Ground: {sum(ep_ground_contact)}/{episode_length} | "
              f"BothFingers: {sum(ep_finger_contact)}/{episode_length}")
        
        # Save video if requested
        if render_video and len(frames) > 0:
            video_path = os.path.join(VIDEO_DIR, f"grasp_improved_test_ep_{episode+1:03d}.mp4")
            h, w = frames[0].shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(video_path, fourcc, 30.0, (w, h))
            
            for frame in frames:
                bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                out.write(bgr_frame)
            out.release()
            if episode < 5:  # Only print for first few videos to avoid spam
                print(f"  Saved video to {video_path}")
    
    # Print summary statistics
    print("\n" + "=" * 60)
    print("TEST RESULTS SUMMARY")
    print("=" * 60)
    print(f"Total episodes: {num_episodes}")
    print(f"Successful grasps: {successes}")
    print(f"Success rate: {successes/num_episodes*100:.2f}%")
    print(f"\nReward statistics:")
    print(f"  Mean reward: {np.mean(total_rewards):.2f}")
    print(f"  Std reward: {np.std(total_rewards):.2f}")
    print(f"  Min reward: {np.min(total_rewards):.2f}")
    print(f"  Max reward: {np.max(total_rewards):.2f}")
    print(f"\nEpisode length statistics:")
    print(f"  Mean length: {np.mean(episode_lengths):.2f} steps")
    print(f"  Std length: {np.std(episode_lengths):.2f} steps")
    print(f"  Min length: {np.min(episode_lengths)} steps")
    print(f"  Max length: {np.max(episode_lengths)} steps")
    
    if grasp_times:
        print(f"\nGrasp timing (for successful episodes):")
        print(f"  Mean steps to grasp: {np.mean(grasp_times):.2f}")
        print(f"  Std steps to grasp: {np.std(grasp_times):.2f}")
        print(f"  Fastest grasp: {np.min(grasp_times)} steps")
        print(f"  Slowest grasp: {np.max(grasp_times)} steps")
    else:
        print("\nNo successful grasps recorded.")
    
    # Additional statistics from improved environment
    print(f"\nImproved environment statistics:")
    print(f"  Mean progress reward: {np.mean(progress_rewards):.2f}")
    print(f"  Mean stuck penalty: {np.mean(stuck_penalties):.2f}")
    print(f"  Mean precision bonus: {np.mean(precision_bonuses):.2f}")
    print(f"  Mean ground contact steps: {np.mean(ground_contact_counts):.2f}")
    print(f"  Mean both fingers contact steps: {np.mean(finger_contact_counts):.2f}")
    if final_velocities:
        print(f"  Mean final velocity magnitude: {np.mean(final_velocities):.4f}")
        print(f"  Std final velocity magnitude: {np.std(final_velocities):.4f}")
    
    # Finger geometry statistics (based on XML structure)
    if finger_separation_distances:
        print(f"\nFinger geometry statistics (from GripperGPT.xml):")
        print(f"  Mean finger separation: {np.mean(finger_separation_distances):.4f}m")
        print(f"  Std finger separation: {np.std(finger_separation_distances):.4f}m")
        print(f"  Min finger separation: {np.min(finger_separation_distances):.4f}m")
        print(f"  Max finger separation: {np.max(finger_separation_distances):.4f}m")
        print(f"  Expected base separation: {FINGER_SEPARATION_BASE:.4f}m (from XML)")
        print(f"  Mean left finger inner contact steps: {np.mean(left_finger_inner_contacts):.2f}")
        print(f"  Mean right finger inner contact steps: {np.mean(right_finger_inner_contacts):.2f}")
    
    print("=" * 60)
    
    env.close()
    
    return {
        "successes": successes,
        "episodes": num_episodes,
        "success_rate": successes/num_episodes*100,
        "mean_reward": np.mean(total_rewards),
        "mean_length": np.mean(episode_lengths),
        "grasp_times": grasp_times,
        "progress_rewards": progress_rewards,
        "stuck_penalties": stuck_penalties,
        "precision_bonuses": precision_bonuses,
        "ground_contact_counts": ground_contact_counts,
        "finger_contact_counts": finger_contact_counts,
        "finger_separation_distances": finger_separation_distances,
        "left_finger_inner_contacts": left_finger_inner_contacts,
        "right_finger_inner_contacts": right_finger_inner_contacts,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test improved grasping policy (Env B - Improved)")
    parser.add_argument(
        "--checkpoint",
        type=str,
        default=None,
        help="Path to grasping model checkpoint (.zip)"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=VALID_EPS,
        help=f"Number of test episodes (default: {VALID_EPS})"
    )
    parser.add_argument(
        "--render-video",
        action="store_true",
        help="Save test videos as .mp4 files"
    )
    parser.add_argument(
        "--stochastic",
        action="store_true",
        help="Use stochastic actions during testing"
    )
    parser.add_argument(
        "--allow-xy",
        action="store_true",
        help="Allow XY adjustments (must match training config)"
    )
    parser.add_argument(
        "--debug-log",
        action="store_true",
        help="Print detailed per-step diagnostics for first episode"
    )
    
    args = parser.parse_args()
    
    # Find checkpoint if not specified
    if args.checkpoint is None:
        # Look for improved grasp models
        grasp_final = os.path.join(CHECKPOINT_DIR, "ppo_grasp_model_final.zip")
        grasp_alt = os.path.join(os.path.dirname(__file__), "checkpoints", "ppo_grasp_model_final.zip")
        final_model = os.path.join(CHECKPOINT_DIR, "ppo_model_final.zip")
        
        if os.path.exists(grasp_final):
            args.checkpoint = grasp_final
            print(f"Using improved grasp model: {grasp_final}")
        elif os.path.exists(grasp_alt):
            args.checkpoint = grasp_alt
            print(f"Using improved grasp model: {grasp_alt}")
        elif os.path.exists(final_model):
            args.checkpoint = final_model
            print(f"Using final model: {final_model}")
        else:
            print("Error: No checkpoint found. Please specify --checkpoint")
            print(f"  Looked in: {CHECKPOINT_DIR}")
            print(f"  Also checked: {os.path.dirname(__file__)}/checkpoints/")
            exit(1)
    
    # Make path absolute if relative
    if not os.path.isabs(args.checkpoint):
        # If path already starts with checkpoints or checkpoints_grasp, resolve from script directory
        if args.checkpoint.startswith("checkpoints"):
            # Path like "checkpoints_grasp/model.zip" or "checkpoints/model.zip"
            # Resolve directly from script directory
            resolved_path = os.path.join(os.path.dirname(__file__), args.checkpoint)
            if os.path.exists(resolved_path):
                args.checkpoint = resolved_path
        else:
            # Path doesn't start with checkpoints, try both directories
            # Try checkpoints_grasp first (default for this script)
            if os.path.exists(os.path.join(CHECKPOINT_DIR, args.checkpoint)):
                args.checkpoint = os.path.join(CHECKPOINT_DIR, args.checkpoint)
            elif os.path.exists(os.path.join(os.path.dirname(__file__), "checkpoints", args.checkpoint)):
                args.checkpoint = os.path.join(os.path.dirname(__file__), "checkpoints", args.checkpoint)
    
    if not os.path.exists(args.checkpoint):
        print(f"Error: Checkpoint not found: {args.checkpoint}")
        exit(1)
    
    # Run test
    test_grasp_policy(
        checkpoint_path=args.checkpoint,
        num_episodes=args.episodes,
        render_video=args.render_video,
        stochastic=args.stochastic,
        allow_xy_adjust=args.allow_xy,
        debug_log=args.debug_log,
    )

