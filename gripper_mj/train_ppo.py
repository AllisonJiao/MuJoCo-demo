import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from gripper_env import GripperEnv  # your env

TRAIN_EPS = 100000
VALID_EPS = 100
VALID_MAX_STEPS = 100

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

model.learn(total_timesteps=TRAIN_EPS)

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
