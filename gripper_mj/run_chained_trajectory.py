#!/usr/bin/env python3
"""
Chained trajectory execution script for the gripper task.

This script chains 4 models together to complete the full hover→grasp→lift→release task:
1. Hover: Position gripper above the block
2. Grasp: Descend and grasp the block
3. Lift: Lift block and move to target
4. Release: Release block onto target

Usage:
    python run_chained_trajectory.py \
        --hover-model path/to/hover_model.zip \
        --grasp-model path/to/grasp_model.zip \
        --lift-model path/to/lift_model.zip \
        --release-model path/to/release_model.zip \
        --rollouts 10 \
        [--render-video] \
        [--debug]
"""

import argparse
import os
import sys
import numpy as np
import cv2
from stable_baselines3 import PPO

# Add parent directory for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_env_improved import GripperEnv
from gripper_grasp_env_improved import GripperGraspEnv
from gripper_env_lift_improved import GripperLiftEnv
from gripper_env_release import GripperReleaseEnv

# Constants for stage transitions
MAX_STEPS_PER_STAGE = 500
BLOCK_DIMENSION = 0.05


def get_env_state(env):
    """Extract the current state from an environment.
    
    Returns a dict suitable for passing to env.reset(options={'initial_state': ...})
    
    Note: This assumes all environments have a 'target' geom. All gripper environments
    in this project satisfy this requirement.
    
    Important: We include 'act' (actuator states) which is critical for intvelocity
    actuators to prevent sudden movement after state transfer.
    """
    target_id = env.model.geom("target").id
    return {
        'qpos': env.data.qpos.copy(),
        'qvel': env.data.qvel.copy(),
        'ctrl': env.data.ctrl.copy(),
        'act': env.data.act.copy(),  # Actuator states - critical for intvelocity actuators
        'target_pos': env.model.geom_pos[target_id].copy()
    }


def run_stage(env, model, stage_name, max_steps, debug=False, frames_list=None):
    """
    Run a single stage of the chained trajectory.
    
    Args:
        env: The gymnasium environment for this stage
        model: The PPO model to use for this stage
        stage_name: Name of the stage (for logging)
        max_steps: Maximum steps before truncation
        debug: Print debug info
        frames_list: Optional list to append rendered frames to
        
    Returns:
        success: Whether the stage completed successfully (terminated=True)
        final_state: The final state of the environment
        info: Final info dict from environment
    """
    obs = env._get_obs()  # Get observation without reset
    
    for step in range(max_steps):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        
        if debug and step % 50 == 0:
            print(f"  [{stage_name}] Step {step}: reward={reward:.3f}, info={info}")
        
        if frames_list is not None and env.render_mode == "rgb_array":
            frame = env.render()
            if frame is not None:
                frames_list.append(frame)
        
        if terminated:
            if debug:
                print(f"  [{stage_name}] Success at step {step}")
            return True, get_env_state(env), info
        
        if truncated:
            if debug:
                print(f"  [{stage_name}] Truncated at step {step}")
            return False, get_env_state(env), info
    
    if debug:
        print(f"  [{stage_name}] Max steps reached ({max_steps})")
    return False, get_env_state(env), info


def run_chained_rollout(
    hover_model, grasp_model, lift_model, release_model,
    hover_env, grasp_env, lift_env, release_env,
    debug=False, collect_frames=False
):
    """
    Run a complete chained rollout: hover → grasp → lift → release
    
    Returns:
        success: Whether the full trajectory succeeded
        stage_results: Dict with success status for each stage
        frames: List of frames if collect_frames=True
    """
    frames = [] if collect_frames else None
    stage_results = {
        'hover': False,
        'grasp': False,
        'lift': False,
        'release': False
    }
    
    # Stage 1: Hover - position gripper above block
    if debug:
        print("Stage 1: Hover (position above block)")
    hover_env.reset()
    hover_success, hover_state, hover_info = run_stage(
        hover_env, hover_model, "Hover", MAX_STEPS_PER_STAGE, debug, frames
    )
    stage_results['hover'] = hover_success
    
    if not hover_success:
        if debug:
            print("  Hover stage failed, attempting to continue anyway...")
        # Continue even if hover doesn't meet strict success criteria, because:
        # 1. The gripper may be close enough to the block for grasp to succeed
        # 2. The grasp model is trained to handle some positioning errors
        # 3. Early termination would prevent any chance of completing the task
    
    # Stage 2: Grasp - descend and grasp the block
    if debug:
        print("Stage 2: Grasp (descend and grasp)")
    # Reset grasp env with state from hover using options
    grasp_env.reset(options={'initial_state': hover_state})
    # Reset internal tracking variables
    grasp_env.step_count = 0
    grasp_env.reached_ideal_height = False
    grasp_env.proper_descent = True
    
    grasp_success, grasp_state, grasp_info = run_stage(
        grasp_env, grasp_model, "Grasp", MAX_STEPS_PER_STAGE, debug, frames
    )
    stage_results['grasp'] = grasp_success
    
    if not grasp_success:
        if debug:
            print("  Grasp stage failed")
        return False, stage_results, frames
    
    # Stage 3: Lift - lift block and move to target
    if debug:
        print("Stage 3: Lift (lift and move to target)")
    # Reset lift env with state from grasp using options
    lift_env.reset(options={'initial_state': grasp_state})
    # Reset internal tracking variables
    lift_env.step_count = 0
    lift_env.block_dropped = False
    lift_env.initial_grasp_success = True  # We know grasp succeeded
    lift_env.prev_dist = None
    lift_env.prev_horizontal_dist = None
    lift_env.prev_block_height = None
    
    lift_success, lift_state, lift_info = run_stage(
        lift_env, lift_model, "Lift", MAX_STEPS_PER_STAGE, debug, frames
    )
    stage_results['lift'] = lift_success
    
    if not lift_success:
        if debug:
            print("  Lift stage failed, attempting release anyway...")
        # Continue even if lift doesn't meet strict success criteria, because:
        # 1. The block may be close enough to the target for release to succeed
        # 2. The release model can still attempt to position and release
        # 3. Measuring partial completion is useful for debugging
    
    # Stage 4: Release - release block onto target
    if debug:
        print("Stage 4: Release (release onto target)")
    # Reset release env with state from lift using options
    release_env.reset(options={'initial_state': lift_state})
    # Reset internal tracking variables
    release_env.step_count = 0
    release_env.block_released = False
    release_env.block_landed = False
    release_env.initial_grasp_success = True  # Assuming still grasped
    release_env.prev_finger_distance = None
    release_env.prev_horizontal_dist = None
    release_env.release_reward_given = False
    
    release_success, release_state, release_info = run_stage(
        release_env, release_model, "Release", MAX_STEPS_PER_STAGE, debug, frames
    )
    stage_results['release'] = release_success
    
    overall_success = release_success
    
    return overall_success, stage_results, frames


def save_video(frames, output_path, fps=30):
    """Save frames as a video file."""
    if not frames:
        return
    
    h, w = frames[0].shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, float(fps), (w, h))
    
    for frame in frames:
        bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        out.write(bgr_frame)
    
    out.release()
    print(f"Video saved to {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Run chained gripper trajectory: hover→grasp→lift→release"
    )
    parser.add_argument(
        "--hover-model", type=str, required=True,
        help="Path to hover/position model (.zip)"
    )
    parser.add_argument(
        "--grasp-model", type=str, required=True,
        help="Path to grasp model (.zip)"
    )
    parser.add_argument(
        "--lift-model", type=str, required=True,
        help="Path to lift model (.zip)"
    )
    parser.add_argument(
        "--release-model", type=str, required=True,
        help="Path to release model (.zip)"
    )
    parser.add_argument(
        "--rollouts", type=int, default=10,
        help="Number of rollouts to run (default: 10)"
    )
    parser.add_argument(
        "--render-video", action="store_true",
        help="Save videos of rollouts"
    )
    parser.add_argument(
        "--video-dir", type=str, default="videos_chained",
        help="Directory to save videos (default: videos_chained)"
    )
    parser.add_argument(
        "--debug", action="store_true",
        help="Print debug information"
    )
    args = parser.parse_args()
    
    # Validate model paths
    for name, path in [
        ("hover", args.hover_model),
        ("grasp", args.grasp_model),
        ("lift", args.lift_model),
        ("release", args.release_model)
    ]:
        if not os.path.exists(path):
            print(f"Error: {name} model not found at {path}")
            sys.exit(1)
    
    # Setup render mode and dimensions
    render_mode = "rgb_array" if args.render_video else None
    # Use higher resolution for video output, default for non-video mode
    if args.render_video:
        video_width, video_height = 1280, 720
    else:
        video_width, video_height = 480, 480
    
    # Create environments
    print("Creating environments...")
    hover_env = GripperEnv(
        render_mode=render_mode, 
        width=video_width, 
        height=video_height,
        enable_updown_control=True
    )
    grasp_env = GripperGraspEnv(
        render_mode=render_mode,
        width=video_width,
        height=video_height,
        allow_xy_adjust=False
    )
    lift_env = GripperLiftEnv(
        render_mode=render_mode,
        width=video_width,
        height=video_height
    )
    release_env = GripperReleaseEnv(
        render_mode=render_mode,
        width=video_width,
        height=video_height
    )
    
    # Load models
    print("Loading models...")
    hover_model = PPO.load(args.hover_model, env=hover_env)
    grasp_model = PPO.load(args.grasp_model, env=grasp_env)
    lift_model = PPO.load(args.lift_model, env=lift_env)
    release_model = PPO.load(args.release_model, env=release_env)
    print("Models loaded successfully")
    
    # Create video directory if needed
    if args.render_video:
        video_dir = os.path.join(os.path.dirname(__file__), args.video_dir)
        os.makedirs(video_dir, exist_ok=True)
    
    # Run rollouts
    successes = 0
    stage_successes = {'hover': 0, 'grasp': 0, 'lift': 0, 'release': 0}
    
    print(f"\nRunning {args.rollouts} rollout(s)...")
    print("-" * 50)
    
    for i in range(args.rollouts):
        if args.debug:
            print(f"\n=== Rollout {i+1}/{args.rollouts} ===")
        
        success, stage_results, frames = run_chained_rollout(
            hover_model, grasp_model, lift_model, release_model,
            hover_env, grasp_env, lift_env, release_env,
            debug=args.debug,
            collect_frames=args.render_video
        )
        
        if success:
            successes += 1
        
        for stage, result in stage_results.items():
            if result:
                stage_successes[stage] += 1
        
        # Save video if requested
        if args.render_video and frames:
            video_path = os.path.join(video_dir, f"rollout_{i:03d}.mp4")
            save_video(frames, video_path)
        
        # Progress indicator (only if not in debug mode)
        if not args.debug:
            status = "✓" if success else "✗"
            print(f"Rollout {i+1:3d}: {status} | "
                  f"Hover: {'✓' if stage_results['hover'] else '✗'} | "
                  f"Grasp: {'✓' if stage_results['grasp'] else '✗'} | "
                  f"Lift: {'✓' if stage_results['lift'] else '✗'} | "
                  f"Release: {'✓' if stage_results['release'] else '✗'}")
    
    # Print summary
    print("-" * 50)
    print("\n=== FINAL RESULTS ===")
    print(f"Overall success rate: {successes}/{args.rollouts} ({100.0 * successes / args.rollouts:.1f}%)")
    print(f"\nPer-stage success rates:")
    for stage, count in stage_successes.items():
        print(f"  {stage.capitalize():8s}: {count}/{args.rollouts} ({100.0 * count / args.rollouts:.1f}%)")
    
    # Cleanup
    hover_env.close()
    grasp_env.close()
    lift_env.close()
    release_env.close()
    
    return 0 if successes > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
