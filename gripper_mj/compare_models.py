"""
Model Comparison Script for Ablation Testing and Baseline Comparison.

This script compares two trained models by running validation episodes and
generating comparative histograms of the results.

Canonical environment type names (use these on CLI):
- "position" (alias: "hover"): GripperEnv from gripper_env_improved.py
- "grasp": GripperGraspEnv
- "lift": GripperLiftEnv
- "release": GripperReleaseEnv

Usage:
    python compare_models.py --model1 <path1> --model2 <path2> --env-type <type> --name1 <name1> --name2 <name2>

Or run default comparison (position stage validations):
    python compare_models.py

Output images are saved to `gripper_mj/comparison_results/`.
"""

import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3 import PPO, SAC
from typing import Dict, List, Tuple, Optional

# Import environments
from mujoco_envs.gripper_env_improved import GripperEnv
from mujoco_envs.gripper_env_grasp_improved import GripperGraspEnv
from mujoco_envs.gripper_env_lift_improved import GripperLiftEnv
from mujoco_envs.gripper_env_release import GripperReleaseEnv


# Constants
VALIDATION_EPISODES = 1000
MAX_STEPS = 500
OUTPUT_DIR = "comparison_results"  # Subdirectory for output images


def create_env(env_type: str):
    """Create environment based on type.
    
    Args:
        env_type: One of "position" (alias: "hover"), "grasp", "lift", "release"
        
    Returns:
        Gymnasium environment instance
    """
    if env_type in ("position", "hover"):
        return GripperEnv(enable_updown_control=True)
    elif env_type == "grasp":
        return GripperGraspEnv(allow_xy_adjust=False)
    elif env_type == "lift":
        return GripperLiftEnv()
    elif env_type == "release":
        return GripperReleaseEnv()
    else:
        raise ValueError(f"Unknown env_type: {env_type}. Must be one of: hover, grasp, lift, release")


def run_validation(model_path: str, env_type: str, num_episodes: int = VALIDATION_EPISODES) -> Dict[str, List[float]]:
    """Run validation episodes on a model and collect metrics.
    
    Args:
        model_path: Path to the saved model (.zip file)
        env_type: Environment type ("hover", "grasp", "lift", "release")
        num_episodes: Number of validation episodes to run
        
    Returns:
        Dictionary with lists of metrics:
        - episode_lengths: List of episode lengths
        - rewards: List of total episode rewards
        - final_horizontal_dist: List of final horizontal distances
        - final_velocity: List of final velocities (if available)
        - successes: List of success flags (1 if terminated, 0 otherwise)
    """
    # Create environment
    env = create_env(env_type)
    
    # Load model
    try:
        model = PPO.load(model_path, env=env)
    except Exception as e:
        model = SAC.load(model_path, env=env)

    
    # Initialize result containers
    results = {
        "episode_lengths": [],
        "rewards": [],
        "final_horizontal_dist": [],
        "final_velocity": [],
        "successes": []
    }
    
    print(f"Running {num_episodes} validation episodes for model: {model_path}")
    
    for ep in range(num_episodes):
        obs, info = env.reset()
        total_reward = 0.0
        ep_length = 0
        final_horiz_dist = np.nan
        final_velocity = np.nan
        success = 0
        
        for step in range(MAX_STEPS):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            
            total_reward += reward
            ep_length += 1
            
            # Extract final metrics from info
            final_horiz_dist = info.get("horizontal_dist", np.nan)
            
            # Extract velocity - different keys for different envs
            if "velocity" in info:
                vel = info["velocity"]
                if isinstance(vel, np.ndarray):
                    final_velocity = float(np.linalg.norm(vel))
                else:
                    final_velocity = float(vel) if vel is not None else np.nan
            elif "gripper_velocity" in info:
                vel = info["gripper_velocity"]
                if isinstance(vel, np.ndarray):
                    final_velocity = float(np.linalg.norm(vel))
                else:
                    final_velocity = float(vel) if vel is not None else np.nan
            
            if terminated or truncated:
                # Success is when terminated (not truncated)
                success = 1 if terminated else 0
                break
        
        # Store results
        results["episode_lengths"].append(ep_length)
        results["rewards"].append(total_reward)
        results["final_horizontal_dist"].append(final_horiz_dist)
        results["final_velocity"].append(final_velocity)
        results["successes"].append(success)
        
        # Progress indicator
        if (ep + 1) % 100 == 0:
            print(f"  Completed {ep + 1}/{num_episodes} episodes")
    
    env.close()
    return results


def generate_comparative_histograms(
    results1: Dict[str, List[float]],
    results2: Dict[str, List[float]],
    label1: str,
    label2: str,
    output_path: str,
    title_prefix: str = "",
    success_rate1: float = None,
    success_rate2: float = None
) -> None:
    """Generate and save comparative histograms for two models.
    
    Args:
        results1: Validation results from first model
        results2: Validation results from second model
        label1: Label for first model in plots
        label2: Label for second model in plots
        output_path: Path to save the histogram image
        title_prefix: Optional prefix for plot titles
        success_rate1: Success rate for first model (0-100%)
        success_rate2: Success rate for second model (0-100%)
    """
    # Determine which metrics have valid data
    metrics_to_plot = []
    metric_labels = {
        "episode_lengths": "Episode Length",
        "rewards": "Total Episode Reward",
        "final_horizontal_dist": "Final Horizontal Distance (m)",
        "final_velocity": "Final Velocity (m/s)"
    }
    
    for metric in ["episode_lengths", "rewards", "final_horizontal_dist", "final_velocity"]:
        data1 = np.array(results1[metric])
        data2 = np.array(results2[metric])
        
        # Check if data has valid (non-NaN) values
        valid1 = ~np.isnan(data1)
        valid2 = ~np.isnan(data2)
        
        if np.sum(valid1) > 0 and np.sum(valid2) > 0:
            metrics_to_plot.append(metric)
    
    if not metrics_to_plot:
        print("No valid metrics to plot!")
        return
    
    # Create figure with subplots
    num_plots = len(metrics_to_plot)
    cols = min(2, num_plots)
    rows = (num_plots + cols - 1) // cols
    
    fig, axes = plt.subplots(rows, cols, figsize=(7 * cols, 5 * rows + 0.5))
    
    # Handle single subplot case
    if num_plots == 1:
        axes = [axes]
    else:
        axes = axes.flatten() if hasattr(axes, 'flatten') else [axes]
    
    # Set main title with success rates underneath
    if title_prefix:
        main_title = f"{title_prefix} - Model Comparison"
    else:
        main_title = "Model Comparison - Validation Results Distribution"
    
    fig.suptitle(main_title, fontsize=14, fontweight='bold', y=0.99)
    
    # Add success rates as subtitle if provided
    if success_rate1 is not None and success_rate2 is not None:
        success_text = f"{label1} Success Rate: {success_rate1:.1f}%  |  {label2} Success Rate: {success_rate2:.1f}%"
        fig.text(0.5, 0.97, success_text, ha='center', va='top', fontsize=11, 
                 style='italic', color='darkgreen')
    
    # Adjust top margin to make room for title and success rates
    plt.subplots_adjust(top=0.90)
    
    # Plot each metric
    for idx, metric in enumerate(metrics_to_plot):
        ax = axes[idx]
        
        data1 = np.array(results1[metric])
        data2 = np.array(results2[metric])
        
        # Filter out NaN values
        data1 = data1[~np.isnan(data1)]
        data2 = data2[~np.isnan(data2)]
        
        # Determine bin range
        all_data = np.concatenate([data1, data2])
        min_val, max_val = np.min(all_data), np.max(all_data)
        
        # Handle case where all values are the same
        if min_val == max_val:
            # Use a single bin centered on the value with a small range
            bins = np.array([min_val - 0.5, min_val + 0.5])
        else:
            bins = np.linspace(min_val, max_val, 31)
        
        # Plot histograms with transparency
        ax.hist(data1, bins=bins, alpha=0.6, label=label1, color='blue', edgecolor='darkblue')
        ax.hist(data2, bins=bins, alpha=0.6, label=label2, color='orange', edgecolor='darkorange')
        
        # Labels and formatting
        ax.set_xlabel(metric_labels[metric], fontsize=11)
        ax.set_ylabel("Frequency", fontsize=11)
        ax.set_title(f"{metric_labels[metric]} Distribution", fontsize=12, fontweight='bold')
        ax.legend(loc='upper right', fontsize=10)
        ax.grid(True, alpha=0.3)
        
        # Add statistics annotation
        stats_text = (
            f"{label1}: μ={np.mean(data1):.3f}, σ={np.std(data1):.3f}\n"
            f"{label2}: μ={np.mean(data2):.3f}, σ={np.std(data2):.3f}"
        )
        ax.annotate(stats_text, xy=(0.02, 0.98), xycoords='axes fraction',
                    fontsize=9, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Hide unused subplots
    for idx in range(len(metrics_to_plot), len(axes)):
        axes[idx].set_visible(False)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"Saved comparative histogram to: {output_path}")


def compare_models(
    model1_path: str,
    model2_path: str,
    env_type: str,
    label1: str = "Model 1",
    label2: str = "Model 2",
    output_path: Optional[str] = None,
    num_episodes: int = VALIDATION_EPISODES
) -> Tuple[Dict[str, List[float]], Dict[str, List[float]]]:
    """Compare two models by running validation and generating histograms.
    
    This function loads two models of the same type, runs validation on each,
    and generates comparative histograms of the results.
    
    Args:
        model1_path: Path to first model (.zip file)
        model2_path: Path to second model (.zip file)
        env_type: Environment type (must be same for both models)
        label1: Label for first model in plots
        label2: Label for second model in plots
        output_path: Path to save histogram image. If None, uses default naming.
        num_episodes: Number of validation episodes (default: 1000)
        
    Returns:
        Tuple of (results1, results2) dictionaries containing validation metrics
    """
    print(f"\n{'='*60}")
    print(f"Comparing models:")
    print(f"  Model 1 ({label1}): {model1_path}")
    print(f"  Model 2 ({label2}): {model2_path}")
    print(f"  Environment: {env_type}")
    print(f"  Episodes: {num_episodes}")
    print(f"{'='*60}\n")
    
    # Run validation for both models
    print(f"Validating {label1}...")
    results1 = run_validation(model1_path, env_type, num_episodes)
    
    print(f"\nValidating {label2}...")
    results2 = run_validation(model2_path, env_type, num_episodes)
    
    # Calculate success rates
    success_rate1 = 100.0 * sum(results1["successes"]) / len(results1["successes"])
    success_rate2 = 100.0 * sum(results2["successes"]) / len(results2["successes"])
    
    # Generate output path if not provided
    if output_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, OUTPUT_DIR)
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, f"comparison_{env_type}_{label1.replace(' ', '_')}_vs_{label2.replace(' ', '_')}.png")
    
    # Generate histograms with success rates
    title_prefix = f"{env_type.capitalize()} Environment"
    generate_comparative_histograms(results1, results2, label1, label2, output_path, title_prefix,
                                    success_rate1=success_rate1, success_rate2=success_rate2)
    
    # Print summary statistics
    print(f"\n{'='*60}")
    print("Summary Statistics:")
    print(f"{'='*60}")
    
    for label, results, success_rate in [(label1, results1, success_rate1), (label2, results2, success_rate2)]:
        print(f"\n{label}:")
        print(f"  Success Rate: {success_rate:.1f}%")
        print(f"  Episode Length: mean={np.mean(results['episode_lengths']):.2f}, std={np.std(results['episode_lengths']):.2f}")
        print(f"  Reward: mean={np.mean(results['rewards']):.2f}, std={np.std(results['rewards']):.2f}")
        
        horiz_dist = np.array(results['final_horizontal_dist'])
        valid_horiz = horiz_dist[~np.isnan(horiz_dist)]
        if len(valid_horiz) > 0:
            print(f"  Final Horizontal Dist: mean={np.mean(valid_horiz):.4f}, std={np.std(valid_horiz):.4f}")
        
        velocity = np.array(results['final_velocity'])
        valid_vel = velocity[~np.isnan(velocity)]
        if len(valid_vel) > 0:
            print(f"  Final Velocity: mean={np.mean(valid_vel):.4f}, std={np.std(valid_vel):.4f}")
    
    return results1, results2


def main():
    """Main function that runs comparison on hover and lift models by default."""
    parser = argparse.ArgumentParser(description="Compare two trained models")
    parser.add_argument("--model1", type=str, default=None, help="Path to first model (.zip)")
    parser.add_argument("--model2", type=str, default=None, help="Path to second model (.zip)")
    parser.add_argument("--env-type", type=str, default=None, 
                        choices=["position", "hover", "grasp", "lift", "release"],
                        help="Environment type for validation (use 'position' as canonical name; 'hover' is accepted as alias)")
    parser.add_argument("--name1", type=str, default=None, help="Name for first model (used in legend)")
    parser.add_argument("--name2", type=str, default=None, help="Name for second model (used in legend)")
    parser.add_argument("--output", type=str, default=None, help="Output path for histogram image")
    parser.add_argument("--episodes", type=int, default=VALIDATION_EPISODES, 
                        help=f"Number of validation episodes (default: {VALIDATION_EPISODES})")
    args = parser.parse_args()
    
    # Default paths for position (hover) and lift models
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_position_path = os.path.join(script_dir, "checkpoints", "ppo_model_final.zip")
    default_lift_path = os.path.join(script_dir, "checkpoints_lift", "ppo_lift_model_final.zip")
    
    # Create output directory
    output_dir = os.path.join(script_dir, OUTPUT_DIR)
    os.makedirs(output_dir, exist_ok=True)
    
    # If both models are provided, run single comparison
    if args.model1 and args.model2 and args.env_type:
        # Use provided names or default to "Model 1" / "Model 2"
        label1 = args.name1 if args.name1 else "Model 1"
        label2 = args.name2 if args.name2 else "Model 2"
        compare_models(
            model1_path=args.model1,
            model2_path=args.model2,
            env_type=args.env_type,
            label1=label1,
            label2=label2,
            output_path=args.output,
            num_episodes=args.episodes
        )
    else:
        # Default: run individual validation tests on hover and lift stage models
        # When only one model is available, we run validation and compare two independent runs
        # This demonstrates the infrastructure and allows checking model consistency
        print("Running default validation: Testing Hover and Lift models individually")
        print("(Use --model1, --model2, --env-type to specify models for comparison)\n")
        
        # Check if default models exist
        position_exists = os.path.exists(default_position_path)
        lift_exists = os.path.exists(default_lift_path)
        
        if not position_exists:
            print(f"Warning: Position (hover) model not found at {default_position_path}")
        if not lift_exists:
            print(f"Warning: Lift model not found at {default_lift_path}")
        
        # Run position (hover) model validation (if model exists)
        if position_exists:
            print("\n" + "="*60)
            print("Validating Position (hover) model on position environment")
            print("="*60)
            # Run two independent validation runs to check consistency
            # In practice, users should provide two different model paths for meaningful comparison
            output_position = os.path.join(output_dir, "comparison_position_validation.png")
            compare_models(
                model1_path=default_position_path,
                model2_path=default_position_path,
                env_type="position",
                label1="Position Model (Run 1)",
                label2="Position Model (Run 2)",
                output_path=output_position,
                num_episodes=args.episodes
            )
        
        # Run lift model validation (if model exists)
        if lift_exists:
            print("\n" + "="*60)
            print("Validating Lift model on lift environment")
            print("="*60)
            # Run two independent validation runs to check consistency
            output_lift = os.path.join(output_dir, "comparison_lift_validation.png")
            compare_models(
                model1_path=default_lift_path,
                model2_path=default_lift_path,
                env_type="lift",
                label1="Lift Model (Run 1)",
                label2="Lift Model (Run 2)",
                output_path=output_lift,
                num_episodes=args.episodes
            )
        
        if not position_exists and not lift_exists:
            print("\nNo default models found. Please train models first or provide paths:")
            print("  python compare_models.py --model1 <path1> --model2 <path2> --env-type <type>")
            print("\nSupported environment types: position (alias: hover), grasp, lift, release")


if __name__ == "__main__":
    main()
