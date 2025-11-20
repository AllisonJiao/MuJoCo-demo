from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize
import numpy as np
import os
from gripper_env import GripperEnv  # your env

TRAIN_EPS = 500000
VALID_EPS = 100
VALID_MAX_STEPS = 100

# Create checkpoints directory
CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), "checkpoints")
os.makedirs(CHECKPOINT_DIR, exist_ok=True)

def make():
    return GripperEnv()

# SAC works with single or vectorized envs, but single is often better
# Using vectorized for faster data collection
venv = make_vec_env(make, n_envs=1, seed=0)  # SAC doesn't need many parallel envs
# Normalize observations - important for SAC
venv = VecNormalize(venv, norm_obs=True, norm_reward=False, clip_obs=10.0)

model = SAC(
    "MlpPolicy",
    venv,
    verbose=1,
    # Network architecture - larger for better learning
    policy_kwargs=dict(
        net_arch=[256, 256, 128]  # SAC uses shared network architecture
    ),
    # SAC-specific hyperparameters
    learning_rate=3e-4,
    buffer_size=100000,      # Replay buffer size (SAC is off-policy)
    learning_starts=1000,    # Start learning after collecting this many steps
    batch_size=256,          # Batch size for training
    tau=0.005,               # Soft update coefficient (target network update)
    gamma=0.99,              # Discount factor
    train_freq=(1, "step"),   # Train every step
    gradient_steps=1,         # Gradient steps per training call
    ent_coef="auto",          # Automatic entropy coefficient tuning
    target_entropy="auto",    # Automatic target entropy
)

# Train the model
model.learn(total_timesteps=TRAIN_EPS)

# Save the model and normalization stats
model.save(os.path.join(CHECKPOINT_DIR, "sac_model"))
venv.save(os.path.join(CHECKPOINT_DIR, "sac_vec_normalize.pkl"))
print(f"Saved model to {CHECKPOINT_DIR}")

# Test the trained model
venv.training = False  # Disable training mode for evaluation
venv.norm_reward = False

env = GripperEnv()
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