"""
Test script for grasping policy (Env B).

Tests a trained grasping policy on the grasp-only environment where
the gripper starts already positioned above the block.
"""

import os
import argparse
import numpy as np
import cv2
from stable_baselines3 import PPO
from gripper_grasp_env import GripperGraspEnv, MAX_STEPS

# Test configuration
VALID_EPS = 100
VALID_MAX_STEPS = MAX_STEPS
CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), "checkpoints")
VIDEO_DIR = os.path.join(os.path.dirname(__file__), "videos")


def test_grasp_policy(
    checkpoint_path,
    num_episodes=VALID_EPS,
    render_video=False,
    stochastic=False,
    allow_xy_adjust=False,
):
    """
    Test a grasping policy on the grasp-only environment.
    
    Args:
        checkpoint_path: Path to the trained grasping model (.zip)
        num_episodes: Number of test episodes
        render_video: Whether to save videos
        stochastic: Use stochastic actions (for exploration)
        allow_xy_adjust: Whether to allow XY adjustments (must match training)
    """
    print("=" * 60)
    print("Testing Grasping Policy (Env B)")
    print("=" * 60)
    print(f"Checkpoint: {checkpoint_path}")
    print(f"Episodes: {num_episodes}")
    print(f"Allow XY adjust: {allow_xy_adjust}")
    print(f"Stochastic: {stochastic}")
    print("=" * 60)
    
    # Create environment with render mode if needed
    render_mode = "rgb_array" if render_video else None
    env = GripperGraspEnv(allow_xy_adjust=allow_xy_adjust, render_mode=render_mode)
    
    # Load model
    print(f"\nLoading model from {checkpoint_path}...")
    try:
        model = PPO.load(checkpoint_path, env=env)
        print(f"Model loaded successfully!")
        print(f"Observation space: {model.observation_space}")
        print(f"Action space: {model.action_space}")
    except Exception as e:
        print(f"Error loading model: {e}")
        print("\nTrying to load without environment...")
        model = PPO.load(checkpoint_path)
        model.set_env(env)
        print("Model loaded and environment set!")
    
    # Create video directory if needed
    if render_video:
        os.makedirs(VIDEO_DIR, exist_ok=True)
        print(f"Videos will be saved to {VIDEO_DIR}")
    
    # Test statistics
    total_rewards = []
    successes = 0
    episode_lengths = []
    grasp_times = []  # Steps until successful grasp
    
    print(f"\nRunning {num_episodes} test episodes...\n")
    
    for episode in range(num_episodes):
        obs, info = env.reset()
        total_reward = 0.0
        episode_length = 0
        grasped = False
        frames = []
        
        for step in range(VALID_MAX_STEPS):
            # Get action from policy
            action, _ = model.predict(obs, deterministic=(not stochastic))
            
            # Step environment
            obs, reward, terminated, truncated, info = env.step(action)
            
            total_reward += reward
            episode_length += 1
            
            # Check if grasped
            if info.get("grasped", False) and not grasped:
                grasped = True
                grasp_times.append(episode_length)
            
            # Record frame if rendering
            if render_video:
                frame = env.render()
                if frame is not None:
                    frames.append(frame)
            
            # Episode ended
            if terminated or truncated:
                break
        
        # Update statistics
        total_rewards.append(total_reward)
        episode_lengths.append(episode_length)
        if grasped:
            successes += 1
        
        # Print episode result
        status = "✓ GRASPED" if grasped else "✗ FAILED"
        print(f"Episode {episode+1:3d}: {status:12s} | "
              f"Reward: {total_reward:7.2f} | "
              f"Steps: {episode_length:3d} | "
              f"Vertical: {info.get('vertical_dist', 0):.3f} | "
              f"Finger: {info.get('finger_state', 0):.3f}")
        
        # Save video if requested
        if render_video and len(frames) > 0:
            video_path = os.path.join(VIDEO_DIR, f"grasp_test_ep_{episode+1:03d}.mp4")
            h, w = frames[0].shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(video_path, fourcc, 30.0, (w, h))
            
            for frame in frames:
                bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                out.write(bgr_frame)
            out.release()
    
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
    
    print("=" * 60)
    
    env.close()
    
    return {
        "successes": successes,
        "episodes": num_episodes,
        "success_rate": successes/num_episodes*100,
        "mean_reward": np.mean(total_rewards),
        "mean_length": np.mean(episode_lengths),
        "grasp_times": grasp_times,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test grasping policy (Env B)")
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
    
    args = parser.parse_args()
    
    # Find checkpoint if not specified
    if args.checkpoint is None:
        # Look for grasp-specific models first
        grasp_final = os.path.join(CHECKPOINT_DIR, "ppo_model_grasp_final.zip")
        final_model = os.path.join(CHECKPOINT_DIR, "ppo_model_final.zip")
        
        if os.path.exists(grasp_final):
            args.checkpoint = grasp_final
            print(f"Using grasp model: {grasp_final}")
        elif os.path.exists(final_model):
            args.checkpoint = final_model
            print(f"Using final model: {final_model}")
        else:
            print("Error: No checkpoint found. Please specify --checkpoint")
            exit(1)
    
    # Make path absolute if relative
    if not os.path.isabs(args.checkpoint):
        if not args.checkpoint.startswith("checkpoints/"):
            args.checkpoint = os.path.join(CHECKPOINT_DIR, args.checkpoint)
    
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
    )

