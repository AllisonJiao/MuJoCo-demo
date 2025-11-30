"""
Model Comparison Script for Ablation Testing and Baseline Comparison.

This script provides functions to compare two trained models by running 1000 episode
validation and generating comparative histograms of the results.

Supported environment types:
- "hover" (position): GripperEnv from gripper_env_improved.py
- "grasp": GripperGraspEnv from gripper_grasp_env_improved.py
- "lift": GripperLiftEnv from gripper_env_lift_improved.py
- "release": GripperReleaseEnv from gripper_env_release.py

Usage:
    python compare_models.py --model1 <path1> --model2 <path2> --env-type <type>
    
    Or run default comparison (hover vs lift):
    python compare_models.py
"""

import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3 import PPO
from typing import Dict, List, Tuple, Optional

# Import environments
from gripper_env_improved import GripperEnv
from gripper_grasp_env_improved import GripperGraspEnv
from gripper_env_lift_improved import GripperLiftEnv
from gripper_env_release import GripperReleaseEnv


# Constants
VALIDATION_EPISODES = 1000
MAX_STEPS = 500


def create_env(env_type: str):
    """Create environment based on type.
    
    Args:
        env_type: One of "hover", "grasp", "lift", "release"
        
    Returns:
        Gymnasium environment instance
    """
    if env_type == "hover":
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
    """
    # Create environment
    env = create_env(env_type)
    
    # Load model
    model = PPO.load(model_path, env=env)
    
    # Initialize result containers
    results = {
        "episode_lengths": [],
        "rewards": [],
        "final_horizontal_dist": [],
        "final_velocity": []
    }
    
    print(f"Running {num_episodes} validation episodes for model: {model_path}")
    
    for ep in range(num_episodes):
        obs, info = env.reset()
        total_reward = 0.0
        ep_length = 0
        final_horiz_dist = np.nan
        final_velocity = np.nan
        
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
                break
        
        # Store results
        results["episode_lengths"].append(ep_length)
        results["rewards"].append(total_reward)
        results["final_horizontal_dist"].append(final_horiz_dist)
        results["final_velocity"].append(final_velocity)
        
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
    title_prefix: str = ""
) -> None:
    """Generate and save comparative histograms for two models.
    
    Args:
        results1: Validation results from first model
        results2: Validation results from second model
        label1: Label for first model in plots
        label2: Label for second model in plots
        output_path: Path to save the histogram image
        title_prefix: Optional prefix for plot titles
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
    
    fig, axes = plt.subplots(rows, cols, figsize=(7 * cols, 5 * rows))
    
    # Handle single subplot case
    if num_plots == 1:
        axes = [axes]
    else:
        axes = axes.flatten() if hasattr(axes, 'flatten') else [axes]
    
    # Set main title
    if title_prefix:
        fig.suptitle(f"{title_prefix} - Model Comparison", fontsize=14, fontweight='bold')
    else:
        fig.suptitle("Model Comparison - Validation Results Distribution", fontsize=14, fontweight='bold')
    
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
    print(f"  Model 1: {model1_path}")
    print(f"  Model 2: {model2_path}")
    print(f"  Environment: {env_type}")
    print(f"  Episodes: {num_episodes}")
    print(f"{'='*60}\n")
    
    # Run validation for both models
    print("Validating Model 1...")
    results1 = run_validation(model1_path, env_type, num_episodes)
    
    print("\nValidating Model 2...")
    results2 = run_validation(model2_path, env_type, num_episodes)
    
    # Generate output path if not provided
    if output_path is None:
        output_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(output_dir, f"comparison_{env_type}_{label1.replace(' ', '_')}_vs_{label2.replace(' ', '_')}.png")
    
    # Generate histograms
    title_prefix = f"{env_type.capitalize()} Environment"
    generate_comparative_histograms(results1, results2, label1, label2, output_path, title_prefix)
    
    # Print summary statistics
    print(f"\n{'='*60}")
    print("Summary Statistics:")
    print(f"{'='*60}")
    
    for label, results in [(label1, results1), (label2, results2)]:
        print(f"\n{label}:")
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
                        choices=["hover", "grasp", "lift", "release"],
                        help="Environment type for validation")
    parser.add_argument("--label1", type=str, default="Model 1", help="Label for first model")
    parser.add_argument("--label2", type=str, default="Model 2", help="Label for second model")
    parser.add_argument("--output", type=str, default=None, help="Output path for histogram image")
    parser.add_argument("--episodes", type=int, default=VALIDATION_EPISODES, 
                        help=f"Number of validation episodes (default: {VALIDATION_EPISODES})")
    args = parser.parse_args()
    
    # Default paths for hover and lift models
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_hover_path = os.path.join(script_dir, "checkpoints", "ppo_model_final.zip")
    default_lift_path = os.path.join(script_dir, "checkpoints_lift", "ppo_lift_model_final.zip")
    
    # If both models are provided, run single comparison
    if args.model1 and args.model2 and args.env_type:
        compare_models(
            model1_path=args.model1,
            model2_path=args.model2,
            env_type=args.env_type,
            label1=args.label1,
            label2=args.label2,
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
        hover_exists = os.path.exists(default_hover_path)
        lift_exists = os.path.exists(default_lift_path)
        
        if not hover_exists:
            print(f"Warning: Hover model not found at {default_hover_path}")
        if not lift_exists:
            print(f"Warning: Lift model not found at {default_lift_path}")
        
        # Run hover model validation (if model exists)
        if hover_exists:
            print("\n" + "="*60)
            print("Validating Hover model on hover environment")
            print("="*60)
            # Run two independent validation runs to check consistency
            # In practice, users should provide two different model paths for meaningful comparison
            output_hover = os.path.join(script_dir, "comparison_hover_validation.png")
            compare_models(
                model1_path=default_hover_path,
                model2_path=default_hover_path,
                env_type="hover",
                label1="Hover Model (Run 1)",
                label2="Hover Model (Run 2)",
                output_path=output_hover,
                num_episodes=args.episodes
            )
        
        # Run lift model validation (if model exists)
        if lift_exists:
            print("\n" + "="*60)
            print("Validating Lift model on lift environment")
            print("="*60)
            # Run two independent validation runs to check consistency
            output_lift = os.path.join(script_dir, "comparison_lift_validation.png")
            compare_models(
                model1_path=default_lift_path,
                model2_path=default_lift_path,
                env_type="lift",
                label1="Lift Model (Run 1)",
                label2="Lift Model (Run 2)",
                output_path=output_lift,
                num_episodes=args.episodes
            )
        
        if not hover_exists and not lift_exists:
            print("\nNo default models found. Please train models first or provide paths:")
            print("  python compare_models.py --model1 <path1> --model2 <path2> --env-type <type>")
            print("\nSupported environment types: hover, grasp, lift, release")


if __name__ == "__main__":
    main()
