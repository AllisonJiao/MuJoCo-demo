import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from gripper_env import GripperEnv  # your env

def make():
    return GripperEnv()

venv = make_vec_env(make, n_envs=4, seed=0)

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

model.learn(total_timesteps=100000)

env = GripperEnv()
obs, info = env.reset(seed=123)
total_r, successes = 0.0, 0
for _ in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, r, term, trunc, info = env.step(action)
    total_r += r
    if term or trunc:
        successes += int(term)
        obs, info = env.reset()
print("total_reward:", total_r, "successes:", successes)
