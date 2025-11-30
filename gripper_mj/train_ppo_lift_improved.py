import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
import torch
import os
import argparse
import numpy as np
from pathlib import Path
import cv2
from gripper_env_lift_improved import GripperLiftEnv  # Lift environment
from callbacks import RewardLoggingCallback

TRAIN_EPS = 100000
VALID_EPS = 10
VALID_MAX_STEPS = 500

# Parse command-line arguments
parser = argparse.ArgumentParser()
parser.add_argument("--eval-only", action="store_true", help="Skip training and only run validation")
parser.add_argument("--render-video", action="store_true", help="Save validation videos as .mp4 files")
parser.add_argument("--model-path", type=str, default=None, help="Path to a saved model (.zip) to load for evaluation")
parser.add_argument("--stochastic", action="store_true", help="Use stochastic actions during evaluation")
parser.add_argument("--debug-log", action="store_true", help="Print per-step diagnostics")
parser.add_argument("--action-scale", type=float, default=1.0, help="Multiply actions by this scale during evaluation")
parser.add_argument("--train-timesteps", type=int, default=TRAIN_EPS, help="Total timesteps for training")
parser.add_argument("--ent-coef", type=float, default=0.01, help="Entropy coefficient (higher = more exploration)")
parser.add_argument("--learning-rate", type=float, default=3e-4, help="Learning rate")
parser.add_argument("--eval-episodes", type=int, default=VALID_EPS, help="Number of evaluation episodes")
parser.add_argument("--ablation", action="store_true", help="Run ablation study")
args = parser.parse_args()

# Render mode configuration
RENDER_MODE = "human" if args.render_video and args.eval_only else None
ABLATION_TAG = "_ablation" if args.ablation else ""

# Checkpoint configuration
CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), f"checkpoints_lift{ABLATION_TAG}")
CHECKPOINT_INTERVAL = 25000
N_ENVS = 4

# Create checkpoint directory
os.makedirs(CHECKPOINT_DIR, exist_ok=True)

def make():
    if RENDER_MODE:
        return GripperLiftEnv(render_mode=RENDER_MODE, ablation=args.ablation)
    return GripperLiftEnv(ablation=args.ablation)

# Skip env creation and training if eval-only mode
if not args.eval_only:
    n_envs = 1 if RENDER_MODE == "human" else N_ENVS
    venv = make_vec_env(make, n_envs=n_envs, seed=0)

    print(f"PPO training for lift and hover task{' with ablation' if args.ablation else ''}")
    print(f"ent_coef={args.ent_coef}, learning_rate={args.learning_rate}, train_timesteps={args.train_timesteps}")
    
    # PPO hyperparameters optimized for lift task
    model = PPO(
        "MlpPolicy",
        venv,
        verbose=1,
        n_steps=2048,  # Longer rollouts for better value estimation
        batch_size=256,  # Smaller batches for more gradient updates
        n_epochs=10,  # More epochs per update for better learning
        learning_rate=float(args.learning_rate),
        gamma=0.99,
        clip_range=0.2,
        use_sde=True,  # State-dependent exploration
        sde_sample_freq=4,
        ent_coef=float(args.ent_coef),
        # Larger network with more capacity
        policy_kwargs=dict(
            net_arch=[dict(pi=[256, 256, 128], vf=[256, 256, 128])],
            activation_fn=torch.nn.Tanh,  # Tanh activation for smoother outputs
        ),
        # Value function clipping for stability
        clip_range_vf=10.0,
        # Normalize observations (important for precision)
        normalize_advantage=True,
    )
else:
    model = None
    venv = None

# Custom callback to save PyTorch .pt checkpoints
class PyTorchCheckpointCallback(BaseCallback):
    """Callback to save PyTorch model weights as .pt files"""
    def __init__(self, save_path: str, save_freq: int, verbose=1):
        super().__init__(verbose)
        self.save_path = save_path
        self.save_freq = save_freq
        self.last_save = 0
        
    def _on_step(self) -> bool:
        if self.num_timesteps - self.last_save >= self.save_freq:
            checkpoint_path = os.path.join(
                self.save_path, 
                f"ppo_lift_model_{self.num_timesteps}.pt"
            )
            torch.save({
                'policy_state_dict': self.model.policy.state_dict(),
                'optimizer_state_dict': self.model.policy.optimizer.state_dict(),
                'timesteps': self.num_timesteps,
            }, checkpoint_path)
            self.last_save = self.num_timesteps
            if self.verbose > 0:
                print(f"Saved PyTorch checkpoint to {checkpoint_path} (timesteps: {self.num_timesteps})")
        return True

if not args.eval_only:
    checkpoint_callback = CheckpointCallback(
        save_freq=max(CHECKPOINT_INTERVAL // N_ENVS, 1),
        save_path=CHECKPOINT_DIR,
        name_prefix=f"ppo_lift_model{ABLATION_TAG}",
        save_replay_buffer=False,
    )

    pytorch_callback = PyTorchCheckpointCallback(
        save_path=CHECKPOINT_DIR,
        save_freq=CHECKPOINT_INTERVAL,
        verbose=1
    )

    reward_log_path = os.path.join(CHECKPOINT_DIR, f"reward_log_lift{ABLATION_TAG}.csv")
    reward_logger = RewardLoggingCallback(log_path=reward_log_path, verbose=1)

    # Train with callbacks
    model.learn(
        total_timesteps=int(args.train_timesteps),
        callback=[checkpoint_callback, pytorch_callback, reward_logger]
    )

    # Save final model
    final_model_path = os.path.join(CHECKPOINT_DIR, f"ppo_lift_model_final{ABLATION_TAG}.zip")
    model.save(final_model_path)
    print(f"Saved final model to {final_model_path}")

    final_pt_path = os.path.join(CHECKPOINT_DIR, f"ppo_lift_model_final{ABLATION_TAG}.pt")
    torch.save({
        'policy_state_dict': model.policy.state_dict(),
        'optimizer_state_dict': model.policy.optimizer.state_dict(),
        'timesteps': args.train_timesteps,
    }, final_pt_path)
    print(f"Saved final PyTorch checkpoint to {final_pt_path}")
else:
    print("Eval-only mode: skipping training")

# Validation
validation_render_mode = "rgb_array" if args.render_video else RENDER_MODE
video_width, video_height = (1280, 720) if args.render_video else (480, 480)
env = GripperLiftEnv(render_mode=validation_render_mode, width=video_width, height=video_height, ablation=args.ablation)
obs, info = env.reset(seed=123)
total_r, successes = 0.0, 0

# Load model for eval-only mode
if args.eval_only:
    if args.model_path:
        print(f"Loading model from {args.model_path}")
        model = PPO.load(args.model_path, env=env)
    else:
        final_model_path = os.path.join(CHECKPOINT_DIR, f"ppo_lift_model_final{ABLATION_TAG}.zip")
        if os.path.exists(final_model_path):
            print(f"Loading model from {final_model_path}")
            model = PPO.load(final_model_path, env=env)
        else:
            print("Error: No model found for eval-only mode. Provide --model-path or train first.")
            exit(1)

# Create video directory if rendering videos
VIDEO_DIR = None
if args.render_video:
    VIDEO_DIR = os.path.join(os.path.dirname(__file__), f"videos_lift{ABLATION_TAG}")
    os.makedirs(VIDEO_DIR, exist_ok=True)
    print(f"Videos will be saved to {VIDEO_DIR}")

for i in range(args.eval_episodes):
    obs, info = env.reset()
    total_r_inner = 0.0
    ep_length = 0
    success = False
    frames = []
    
    log_this_ep = args.debug_log and i == 0
    ep_actions = []
    ep_horiz = []
    ep_height = []
    ep_vel = []
    ep_grasped = []

    for step in range(VALID_MAX_STEPS):
        action, _ = model.predict(obs, deterministic=(not args.stochastic))
        action = action * float(args.action_scale)
        obs, r, term, trunc, info = env.step(action)
        
        ep_actions.append(np.array(action).ravel())
        ep_horiz.append(info.get("horizontal_dist", np.nan))
        ep_height.append(info.get("block_height", np.nan))
        ep_vel.append(info.get("velocity", np.nan))
        ep_grasped.append(info.get("grasped", False))
        total_r += r
        total_r_inner += r
        ep_length += 1
        success = term
        
        if log_this_ep and step % 10 == 0:
            print(f"  Step {step}: action={action} horiz_dist={info.get('horizontal_dist', 0.0):.4f} "
                  f"height={info.get('block_height', 0.0):.4f} grasped={info.get('grasped', False)} reward={r:.4f}")

        if args.render_video:
            frame = env.render()
            if frame is not None:
                frames.append(frame)

        if term or trunc:
            successes += int(term)
            obs, info = env.reset()
            break

    # Save video if frames were collected
    if args.render_video and len(frames) > 0:
        video_path = os.path.join(VIDEO_DIR, f"validation_lift{ABLATION_TAG}_ep_{i:03d}.mp4")
        h, w = frames[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(video_path, fourcc, 30.0, (w, h))
        
        for frame in frames:
            bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            out.write(bgr_frame)
        out.release()
        print(f"Saved video to {video_path}")

    # Print diagnostics
    if len(ep_actions) > 0:
        acts = np.stack(ep_actions)
        act_mean = acts.mean(axis=0)
        act_std = acts.std(axis=0)
        print(f"Eval {i}: total_reward: {total_r_inner:.2f}, ep_length: {ep_length}, success: {success}")
        print(f"  action mean: {act_mean} std: {act_std}")
        print(f"  horiz dist mean: {np.nanmean(ep_horiz):.4f} std: {np.nanstd(ep_horiz):.4f}")
        print(f"  height mean: {np.nanmean(ep_height):.4f} std: {np.nanstd(ep_height):.4f}")
        print(f"  grasped: {sum(ep_grasped)}/{len(ep_grasped)} steps")
        print(f"  final dist: {ep_horiz[-1]:.4f} height: {ep_height[-1]:.4f} vel: {ep_vel[-1]}")
    else:
        print(f"Eval {i}: total_reward: {total_r_inner:.2f}, ep_length: {ep_length}, success: {success}")

print(f"\n=== FINAL RESULTS ===")
print(f"Mean reward: {total_r / args.eval_episodes:.2f}")
print(f"Success rate: {successes}/{args.eval_episodes} ({100.0 * successes / args.eval_episodes:.1f}%)")
print(f"Deterministic mode: {not args.stochastic}")

# Clean up
env.close()
