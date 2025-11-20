"""
Test script to load and evaluate a saved checkpoint.
Usage: python test_checkpoint.py --checkpoint checkpoints/ppo_model_1000000_steps.zip
"""
import argparse
import os
import numpy as np
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize
from gripper_env import GripperEnv

CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), "checkpoints")

VALID_EPS = 100
VALID_MAX_STEPS = 100

def make():
    return GripperEnv()

def test_model(checkpoint_path, model_type="ppo", use_normalize=True, num_episodes=None, render=False):
    """
    Test a loaded checkpoint model.
    
    Args:
        checkpoint_path: Path to the checkpoint file (.zip)
        model_type: "ppo" or "sac"
        use_normalize: Whether to use VecNormalize wrapper
        num_episodes: Number of test episodes (defaults to VALID_EPS)
        render: Whether to render the environment
    """
    if num_episodes is None:
        num_episodes = VALID_EPS
    
    print(f"Loading checkpoint: {checkpoint_path}")
    
    # Determine model type from checkpoint if not specified
    if model_type is None:
        if "sac" in checkpoint_path.lower():
            model_type = "sac"
        else:
            model_type = "ppo"
    
    # Create environment
    venv = make_vec_env(make, n_envs=1, seed=123)
    
    if use_normalize:
        # Try to load normalization stats
        normalize_path = None
        if model_type == "sac":
            normalize_path = os.path.join(CHECKPOINT_DIR, "sac_vec_normalize.pkl")
        else:
            normalize_path = os.path.join(CHECKPOINT_DIR, "vec_normalize.pkl")
        
        if os.path.exists(normalize_path):
            print(f"Loading normalization stats from: {normalize_path}")
            try:
                venv = VecNormalize.load(normalize_path, venv)
                venv.training = False
                venv.norm_reward = False
                print("Normalization loaded successfully.")
            except Exception as e:
                print(f"Warning: Failed to load normalization: {e}")
                print("Continuing without normalization...")
        else:
            print(f"Warning: Normalization file not found at {normalize_path}")
            print("Testing without normalization (model may have been trained without it).")
    
    # Load the model
    print(f"Loading {model_type.upper()} model...")
    try:
        if model_type.lower() == "sac":
            model = SAC.load(checkpoint_path)
        else:
            model = PPO.load(checkpoint_path)
        # Set the environment
        model.set_env(venv)
        print("Model loaded and environment set successfully!")
    except Exception as e:
        print(f"Error loading model: {e}")
        raise
    
    print(f"Model loaded successfully! Testing for {num_episodes} episodes...")
    
    # Use non-vectorized env for testing (matches train_ppo.py logic)
    env = GripperEnv()
    obs, info = env.reset(seed=123)
    total_r = 0.0
    successes = 0
    
    for i in range(num_episodes):
        obs, info = env.reset()
        total_r_inner = 0.0
        ep_length = 0
        success = False

        for _ in range(VALID_MAX_STEPS):
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            total_r += r
            total_r_inner += r
            ep_length += 1
            success = term

            if term or trunc:
                successes += int(term)
                obs, info = env.reset()
                break

        print("Eval {}: total_reward: {:.2f}, ep_length: {}, successes: {}".format(i, total_r_inner, ep_length, success))
    
    # Print results
    print("\n" + "="*50)
    print("TEST RESULTS")
    print("="*50)
    print(f"Total episodes: {num_episodes}")
    print(f"Successes: {successes}")
    print(f"Success rate: {successes/num_episodes*100:.2f}%")
    print(f"mean_reward: {total_r / num_episodes:.2f}")
    print("="*50)
    
    return {
        "successes": successes,
        "episodes": num_episodes,
        "success_rate": successes/num_episodes*100,
        "total_reward": total_r,
        "mean_reward": total_r / num_episodes
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test a saved checkpoint")
    parser.add_argument(
        "--checkpoint",
        type=str,
        default=None,
        help="Path to checkpoint file (e.g., checkpoints/ppo_model_1000000_steps.zip)"
    )
    parser.add_argument(
        "--model-type",
        type=str,
        choices=["ppo", "sac"],
        default=None,
        help="Model type (ppo or sac). Auto-detected if not specified."
    )
    parser.add_argument(
        "--no-normalize",
        action="store_true",
        help="Don't use observation normalization"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=None,
        help=f"Number of test episodes (default: {VALID_EPS})"
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="Render the environment during testing"
    )
    
    args = parser.parse_args()
    
    # If no checkpoint specified, try to find the latest one
    if args.checkpoint is None:
        # Look for final models first
        final_ppo = os.path.join(CHECKPOINT_DIR, "ppo_model_final.zip")
        final_sac = os.path.join(CHECKPOINT_DIR, "sac_model.zip")
        
        if os.path.exists(final_sac):
            args.checkpoint = final_sac
            print(f"Using latest SAC model: {final_sac}")
        elif os.path.exists(final_ppo):
            args.checkpoint = final_ppo
            print(f"Using latest PPO model: {final_ppo}")
        else:
            # Find the highest numbered checkpoint
            checkpoints = [f for f in os.listdir(CHECKPOINT_DIR) if f.endswith("_steps.zip")]
            if checkpoints:
                # Sort by timesteps
                def get_timesteps(f):
                    try:
                        return int(f.split("_")[-2])
                    except:
                        return 0
                latest = max(checkpoints, key=get_timesteps)
                args.checkpoint = os.path.join(CHECKPOINT_DIR, latest)
                print(f"Using latest checkpoint: {args.checkpoint}")
            else:
                print("Error: No checkpoint found. Please specify --checkpoint")
                exit(1)
    
    # Make sure path is absolute or relative to checkpoint dir
    if not os.path.isabs(args.checkpoint):
        if not args.checkpoint.startswith("checkpoints/"):
            args.checkpoint = os.path.join(CHECKPOINT_DIR, args.checkpoint)
    
    if not os.path.exists(args.checkpoint):
        print(f"Error: Checkpoint not found: {args.checkpoint}")
        exit(1)
    
    # Run test
    test_model(
        checkpoint_path=args.checkpoint,
        model_type=args.model_type,
        use_normalize=not args.no_normalize,
        num_episodes=args.episodes,
        render=args.render
    )

