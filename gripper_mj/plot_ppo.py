import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
import numpy as np
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Plot training metrics for two PPO models (ablation study)')
parser.add_argument('--ppo1', type=str, required=True, help='Path to first PPO model log CSV file')
parser.add_argument('--ppo2', type=str, required=True, help='Path to second PPO model log CSV file')
parser.add_argument('--label1', type=str, default='PPO Model 1', help='Label for first model in plots')
parser.add_argument('--label2', type=str, default='PPO Model 2', help='Label for second model in plots')
parser.add_argument('--reward', action='store_true', help='Plot episode reward')
parser.add_argument('--episode-length', action='store_true', help='Plot episode length')
parser.add_argument('--entropy-loss', action='store_true', help='Plot entropy loss (combined for both models)')
parser.add_argument('--value-loss', action='store_true', help='Plot value loss (combined for both models)')
parser.add_argument('--all', action='store_true', help='Plot all metrics (default if no specific metric is selected)')
parser.add_argument('--window', type=int, default=10, help='Smoothing window size (default: 10)')
parser.add_argument('--title', type=str, default='PPO Ablation Study', help='Custom title for the plot')
args = parser.parse_args()

# If no specific metric is selected, plot all
if not any([args.reward, args.episode_length, args.entropy_loss, args.value_loss]):
    args.all = True

# 1) Load CSVs (handle missing files gracefully)
ppo1 = None
ppo2 = None

if os.path.exists(args.ppo1):
    ppo1 = pd.read_csv(args.ppo1)
    print(f"Loaded {args.label1} data: {len(ppo1)} rollouts from {args.ppo1}")
else:
    print(f"Error: {args.ppo1} not found")
    sys.exit(1)

if os.path.exists(args.ppo2):
    ppo2 = pd.read_csv(args.ppo2)
    print(f"Loaded {args.label2} data: {len(ppo2)} rollouts from {args.ppo2}")
else:
    print(f"Error: {args.ppo2} not found")
    sys.exit(1)

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
    sys.exit(1)

# For PPO vs PPO comparison, all plots are combined (no separate plots needed)
num_subplots = num_plots

# Create subplots
if num_subplots <= 2:
    rows, cols = 1, num_subplots
elif num_subplots <= 4:
    rows, cols = 2, 2
else:
    rows = (num_subplots + 1) // 2
    cols = 2

fig, axes = plt.subplots(rows, cols, figsize=(14, 5*rows))
fig.suptitle(f"{args.title} – Training Metrics Comparison", fontsize=16, fontweight='bold')

# Flatten axes if needed
if num_subplots == 1:
    axes = [axes]
elif rows == 1 or cols == 1:
    axes = axes.flatten() if hasattr(axes, 'flatten') else [axes]
else:
    axes = axes.flatten()

ax_idx = 0

# Plot each metric (all combined for PPO vs PPO comparison)
for plot_type in plots_to_create:
    ax = axes[ax_idx]
    
    if plot_type == 'reward':
        plot_metric(ax, ppo1, "episode_reward", "Average Episode Return", "Episode Reward", args.label1)
        plot_metric(ax, ppo2, "episode_reward", "Average Episode Return", "Episode Reward", args.label2)
        ax.legend()
        ax_idx += 1
    
    elif plot_type == 'episode_length':
        plot_metric(ax, ppo1, "episode_length", "Average Episode Length", "Episode Length", args.label1)
        plot_metric(ax, ppo2, "episode_length", "Average Episode Length", "Episode Length", args.label2)
        ax.legend()
        ax_idx += 1
    
    elif plot_type == 'entropy_loss':
        # Combined plot for both PPO models (same properties)
        plot_metric(ax, ppo1, "entropy_loss", "Entropy Loss", "Entropy Loss (Policy Gradient Loss)", args.label1)
        plot_metric(ax, ppo2, "entropy_loss", "Entropy Loss", "Entropy Loss (Policy Gradient Loss)", args.label2)
        ax.legend()
        ax_idx += 1
    
    elif plot_type == 'value_loss':
        # Combined plot for both PPO models (same properties)
        plot_metric(ax, ppo1, "value_loss", "Value Loss", "Value Loss", args.label1)
        plot_metric(ax, ppo2, "value_loss", "Value Loss", "Value Loss", args.label2)
        ax.legend()
        ax_idx += 1

# Hide unused subplots
for i in range(ax_idx, len(axes)):
    axes[i].set_visible(False)

plt.tight_layout()
plt.show()

# Print summary statistics
print("\n=== Summary Statistics ===")

print(f"\n{args.label1}:")
print(f"  Final reward: {ppo1['episode_reward'].iloc[-1]:.2f}")
print(f"  Mean reward: {ppo1['episode_reward'].mean():.2f}")
print(f"  Mean episode length: {ppo1['episode_length'].mean():.1f}")
if ppo1['entropy_loss'].notna().any():
    print(f"  Mean entropy loss: {ppo1['entropy_loss'].dropna().mean():.2f}")
if ppo1['value_loss'].notna().any():
    print(f"  Mean value loss: {ppo1['value_loss'].dropna().mean():.2f}")

print(f"\n{args.label2}:")
print(f"  Final reward: {ppo2['episode_reward'].iloc[-1]:.2f}")
print(f"  Mean reward: {ppo2['episode_reward'].mean():.2f}")
print(f"  Mean episode length: {ppo2['episode_length'].mean():.1f}")
if ppo2['entropy_loss'].notna().any():
    print(f"  Mean entropy loss: {ppo2['entropy_loss'].dropna().mean():.2f}")
if ppo2['value_loss'].notna().any():
    print(f"  Mean value loss: {ppo2['value_loss'].dropna().mean():.2f}")
