import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
import torch
import os
from gripper_env import GripperEnv  # your env

TRAIN_EPS = 100000
VALID_EPS = 100
VALID_MAX_STEPS = 100

# Checkpoint configuration
CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), "checkpoints")
CHECKPOINT_INTERVAL = 25000  # Save checkpoint every 25000 timesteps
N_ENVS = 4

# Render mode configuration
RENDER_MODE = os.getenv("RENDER_MODE", None)  # Set RENDER_MODE=human to enable rendering

# Create checkpoint directory
os.makedirs(CHECKPOINT_DIR, exist_ok=True)

def make():
    if RENDER_MODE:
        return GripperEnv(render_mode=RENDER_MODE)
    return GripperEnv()

# Use fewer environments when rendering to avoid performance issues
n_envs = 1 if RENDER_MODE == "human" else N_ENVS
venv = make_vec_env(make, n_envs=n_envs, seed=0)

model = PPO(
    "MlpPolicy",
    venv,
    verbose=1,
    n_steps=1024,
    batch_size=2048,
    learning_rate=3e-4,
    gamma=0.99,
    clip_range=0.2,
)

# Custom callback to save PyTorch .pt checkpoints
class PyTorchCheckpointCallback(BaseCallback):
    """Callback to save PyTorch model weights as .pt files"""
    def __init__(self, save_path: str, save_freq: int, verbose=1):
        super().__init__(verbose)
        self.save_path = save_path
        self.save_freq = save_freq
        self.last_save = 0
        
    def _on_step(self) -> bool:
        # Check if we should save based on timesteps
        if self.num_timesteps - self.last_save >= self.save_freq:
            # Save PyTorch state dict
            checkpoint_path = os.path.join(
                self.save_path, 
                f"ppo_model_{self.num_timesteps}.pt"
            )
            # Save policy network state dict and optimizer state
            torch.save({
                'policy_state_dict': self.model.policy.state_dict(),
                'optimizer_state_dict': self.model.policy.optimizer.state_dict(),
                'timesteps': self.num_timesteps,
            }, checkpoint_path)
            self.last_save = self.num_timesteps
            if self.verbose > 0:
                print(f"Saved PyTorch checkpoint to {checkpoint_path} (timesteps: {self.num_timesteps})")
        return True

# Set up callbacks
# save_freq is in terms of calls to env.step(), not timesteps
# With n_envs=4, each step() call = 4 timesteps, so divide by n_envs
checkpoint_callback = CheckpointCallback(
    save_freq=max(CHECKPOINT_INTERVAL // N_ENVS, 1),  # Adjust for vectorized env
    save_path=CHECKPOINT_DIR,
    name_prefix="ppo_model",
    save_replay_buffer=False,
)

pytorch_callback = PyTorchCheckpointCallback(
    save_path=CHECKPOINT_DIR,
    save_freq=CHECKPOINT_INTERVAL,
    verbose=1
)

# Train with callbacks
model.learn(
    total_timesteps=TRAIN_EPS,
    callback=[checkpoint_callback, pytorch_callback]
)

# Save final model
final_model_path = os.path.join(CHECKPOINT_DIR, "ppo_model_final.zip")
model.save(final_model_path)
print(f"Saved final model to {final_model_path}")

# Save final PyTorch checkpoint
final_pt_path = os.path.join(CHECKPOINT_DIR, "ppo_model_final.pt")
torch.save({
    'policy_state_dict': model.policy.state_dict(),
    'optimizer_state_dict': model.policy.optimizer.state_dict(),
    'timesteps': TRAIN_EPS,
}, final_pt_path)
print(f"Saved final PyTorch checkpoint to {final_pt_path}")

env = GripperEnv(render_mode=RENDER_MODE) if RENDER_MODE else GripperEnv()
obs, info = env.reset(seed=123)
total_r, successes = 0.0, 0

for i in range(VALID_EPS):
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

print("mean_reward:", total_r / VALID_EPS, "successes:", successes)
