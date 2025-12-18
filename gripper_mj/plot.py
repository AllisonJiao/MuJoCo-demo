import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Plot training metrics for PPO and SAC')
parser.add_argument('--position', action='store_true', help='Plot position stage metrics')
parser.add_argument('--grasp', action='store_true', help='Plot grasp stage metrics')
parser.add_argument('--lift', action='store_true', help='Plot lift stage metrics')
parser.add_argument('--release', action='store_true', help='Plot release stage metrics')
parser.add_argument('--reward', action='store_true', help='Plot episode reward')
parser.add_argument('--episode-length', action='store_true', help='Plot episode length')
parser.add_argument('--entropy-loss', action='store_true', help='Plot entropy loss (separate plots for PPO and SAC)')
parser.add_argument('--value-loss', action='store_true', help='Plot value loss (separate plots for PPO and SAC)')
parser.add_argument('--all', action='store_true', help='Plot all metrics (default if no specific metric is selected)')
parser.add_argument('--window', type=int, default=10, help='Smoothing window size (default: 10)')
args = parser.parse_args()

# Determine which stage to plot
stages = [args.position, args.grasp, args.lift, args.release]
if sum(stages) == 0:
    # Default to position if no stage is specified
    args.position = True
    stage = 'position'
elif sum(stages) > 1:
    print("Error: Please specify only one stage (--position, --grasp, --lift, or --release)")
    exit(1)
else:
    if args.position:
        stage = 'position'
    elif args.grasp:
        stage = 'grasp'
    elif args.lift:
        stage = 'lift'
    else:
        stage = 'release'

# If no specific metric is selected, plot all
if not any([args.reward, args.episode_length, args.entropy_loss, args.value_loss]):
    args.all = True

# 1) Load CSVs based on selected stage (handle missing files gracefully)
ppo = None
sac = None

# Define CSV paths based on stage
if stage == 'position':
    ppo_path = "checkpoints/reward_log_position.csv"
    sac_path = "checkpoints_sac_position/reward_log_position.csv"
elif stage == 'grasp':
    ppo_path = "checkpoints_grasp/reward_log_grasp.csv"
    sac_path = "checkpoints_sac_grasp/reward_log_grasp.csv"
elif stage == 'lift':
    ppo_path = "checkpoints_lift/reward_log_lift.csv"
    sac_path = "checkpoints_sac_lift/reward_log_lift.csv"
else:  # release
    ppo_path = "checkpoints_release/reward_log_release.csv"
    sac_path = "checkpoints_sac_release/reward_log_release.csv"

if os.path.exists(ppo_path):
    ppo = pd.read_csv(ppo_path)
    print(f"Loaded PPO data: {len(ppo)} rollouts")
else:
    print(f"Warning: {ppo_path} not found")

if os.path.exists(sac_path):
    sac = pd.read_csv(sac_path)
    print(f"Loaded SAC data: {len(sac)} rollouts")
else:
    print(f"Warning: {sac_path} not found")

if ppo is None and sac is None:
    print("Error: No data files found!")
    exit(1)

# 2) Smoothing window for plots
window = args.window

# Helper function to plot with smoothing
def plot_metric(ax, data, metric_name, ylabel, title, label, window=window):
    """Plot a metric with optional smoothing"""
    if data is None:
        return
    
    # Filter out NaN values for plotting
    valid_data = data[data[metric_name].notna()].copy()
    
    if len(valid_data) == 0:
        return
    
    # Apply smoothing
    valid_data[f"{metric_name}_smooth"] = valid_data[metric_name].rolling(window, min_periods=1).mean()
    
    # Plot smoothed line
    ax.plot(valid_data["timesteps"], valid_data[f"{metric_name}_smooth"], 
            label=label, linewidth=2, alpha=0.8)
    
    # Also plot raw data with lower opacity for reference
    ax.scatter(valid_data["timesteps"], valid_data[metric_name], 
               alpha=0.15, s=8, marker='o')
    
    ax.set_xlabel("Environment Steps", fontsize=10)
    ax.set_ylabel(ylabel, fontsize=10)
    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)

# Determine which plots to create
plots_to_create = []
if args.all or args.reward:
    plots_to_create.append('reward')
if args.all or args.episode_length:
    plots_to_create.append('episode_length')
if args.all or args.entropy_loss:
    plots_to_create.append('entropy_loss')
if args.all or args.value_loss:
    plots_to_create.append('value_loss')

# Create figure with appropriate layout
num_plots = len(plots_to_create)
if num_plots == 0:
    print("No plots to create!")
    exit(1)

# For loss plots, we want separate subplots for PPO and SAC
# Count how many subplots we need
num_subplots = 0
for plot_type in plots_to_create:
    if plot_type in ['entropy_loss', 'value_loss']:
        # Separate plots for PPO and SAC
        if ppo is not None:
            num_subplots += 1
        if sac is not None:
            num_subplots += 1
    else:
        # Combined plots
        num_subplots += 1

# Create subplots
if num_subplots <= 2:
    rows, cols = 1, num_subplots
elif num_subplots <= 4:
    rows, cols = 2, 2
else:
    rows = (num_subplots + 1) // 2
    cols = 2

fig, axes = plt.subplots(rows, cols, figsize=(14, 5*rows))
stage_title = stage.capitalize()
fig.suptitle(f"PPO vs SAC – {stage_title} Stage Training Metrics Comparison", fontsize=16, fontweight='bold')

# Flatten axes if needed
if num_subplots == 1:
    axes = [axes]
elif rows == 1 or cols == 1:
    axes = axes.flatten() if hasattr(axes, 'flatten') else [axes]
else:
    axes = axes.flatten()

ax_idx = 0

# Plot each metric
for plot_type in plots_to_create:
    if plot_type == 'reward':
        ax = axes[ax_idx]
        if ppo is not None:
            plot_metric(ax, ppo, "episode_reward", "Average Episode Return", "Episode Reward", "PPO")
        if sac is not None:
            plot_metric(ax, sac, "episode_reward", "Average Episode Return", "Episode Reward", "SAC")
        if ppo is not None or sac is not None:
            ax.legend()
        ax_idx += 1
    
    elif plot_type == 'episode_length':
        ax = axes[ax_idx]
        if ppo is not None:
            plot_metric(ax, ppo, "episode_length", "Average Episode Length", "Episode Length", "PPO")
        if sac is not None:
            plot_metric(ax, sac, "episode_length", "Average Episode Length", "Episode Length", "SAC")
        if ppo is not None or sac is not None:
            ax.legend()
        ax_idx += 1
    
    elif plot_type == 'entropy_loss':
        # Separate plots for PPO and SAC
        if ppo is not None:
            ax = axes[ax_idx]
            plot_metric(ax, ppo, "entropy_loss", "Entropy Loss", "Entropy Loss - PPO\n(Policy Gradient Loss)", "PPO")
            ax.legend()
            ax_idx += 1
        
        if sac is not None:
            ax = axes[ax_idx]
            plot_metric(ax, sac, "entropy_loss", "Entropy Loss", "Entropy Loss - SAC\n(Actor Loss)", "SAC")
            ax.legend()
            ax_idx += 1
    
    elif plot_type == 'value_loss':
        # Separate plots for PPO and SAC
        if ppo is not None:
            ax = axes[ax_idx]
            plot_metric(ax, ppo, "value_loss", "Value Loss", "Value Loss - PPO", "PPO")
            ax.legend()
            ax_idx += 1
        
        if sac is not None:
            ax = axes[ax_idx]
            plot_metric(ax, sac, "value_loss", "Critic Loss", "Value Loss - SAC\n(Critic Loss)", "SAC")
            ax.legend()
            ax_idx += 1

# Hide unused subplots
for i in range(ax_idx, len(axes)):
    axes[i].set_visible(False)

plt.tight_layout()
plt.show()

# Print summary statistics
print("\n=== Summary Statistics ===")
if ppo is not None:
    print("\nPPO:")
    print(f"  Final reward: {ppo['episode_reward'].iloc[-1]:.2f}")
    print(f"  Mean reward: {ppo['episode_reward'].mean():.2f}")
    print(f"  Mean episode length: {ppo['episode_length'].mean():.1f}")
    if ppo['entropy_loss'].notna().any():
        print(f"  Mean entropy loss: {ppo['entropy_loss'].dropna().mean():.2f}")
    if ppo['value_loss'].notna().any():
        print(f"  Mean value loss: {ppo['value_loss'].dropna().mean():.2f}")

if sac is not None:
    print("\nSAC:")
    print(f"  Final reward: {sac['episode_reward'].iloc[-1]:.2f}")
    print(f"  Mean reward: {sac['episode_reward'].mean():.2f}")
    print(f"  Mean episode length: {sac['episode_length'].mean():.1f}")
    if sac['entropy_loss'].notna().any():
        print(f"  Mean entropy loss: {sac['entropy_loss'].dropna().mean():.2f}")
    if sac['value_loss'].notna().any():
        print(f"  Mean value loss: {sac['value_loss'].dropna().mean():.2f}")
