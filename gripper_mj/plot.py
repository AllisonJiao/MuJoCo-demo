import pandas as pd
import matplotlib.pyplot as plt
import os

# 1) Load CSVs (handle missing files gracefully)
ppo = None
sac = None

ppo_path = "checkpoints/reward_log_position.csv"
sac_path = "checkpoints_sac_position/reward_log_position.csv"

if os.path.exists(ppo_path):
    ppo = pd.read_csv(ppo_path)
    print(f"Loaded PPO data: {len(ppo)} episodes")
else:
    print(f"Warning: {ppo_path} not found")

if os.path.exists(sac_path):
    sac = pd.read_csv(sac_path)
    print(f"Loaded SAC data: {len(sac)} episodes")
else:
    print(f"Warning: {sac_path} not found")

if ppo is None and sac is None:
    print("Error: No data files found!")
    exit(1)

# 2) Optionally smooth the rewards (rolling mean)
window = 10  # increase if super noisy

# 3) Plot like the mock example
plt.figure(figsize=(10, 6))

if ppo is not None:
    # The CSV has columns: timesteps, episode_reward, episode_length
    ppo["reward_smooth"] = ppo["episode_reward"].rolling(window, min_periods=1).mean()
    plt.plot(ppo["timesteps"], ppo["reward_smooth"], label="PPO", linewidth=2)

if sac is not None:
    sac["reward_smooth"] = sac["episode_reward"].rolling(window, min_periods=1).mean()
    plt.plot(sac["timesteps"], sac["reward_smooth"], label="SAC", linewidth=2)

plt.xlabel("Environment Steps")
plt.ylabel("Average Episode Return")
plt.title("PPO vs SAC – Learning Curve")
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()
