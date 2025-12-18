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
from mujoco_envs.gripper_env_improved import GripperEnv  # Improved positioning environment
from mujoco_envs.gripper_env_grasp_improved import GripperGraspEnv, MAX_STEPS as GRASP_MAX_STEPS  # Improved grasping environment
from mujoco_envs.gripper_env_lift_improved import GripperLiftEnv, MAX_STEPS as LIFT_MAX_STEPS  # Improved lift environment
from mujoco_envs.gripper_env_release import GripperReleaseEnv, MAX_STEPS as RELEASE_MAX_STEPS  # Release environment
from callbacks import RewardLoggingCallback

TRAIN_EPS = 100000
VALID_EPS = 10
VALID_MAX_STEPS_POSITION = 500
VALID_MAX_STEPS_GRASP = 500  # Shorter episodes for grasping task

# Checkpoint configuration
CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), "checkpoints")
CHECKPOINT_INTERVAL = 25000
N_ENVS = 4

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
parser.add_argument("--enable-updown", action="store_true", help="Enable learnable up/down control (for positioning env)")
parser.add_argument("--allow-xy-adjust", action="store_true", help="Allow small XY adjustments (for grasping env)")
parser.add_argument("--learning-rate", type=float, default=3e-4, help="Learning rate")
parser.add_argument("--env-type", type=str, default="position", choices=["position", "grasp", "lift", "release"], 
                    help="Environment type: 'position' for positioning, 'grasp' for grasping, 'lift' for lift, or 'release' for release")
parser.add_argument("--ablation", action="store_true", help="Run ablation study (for lift environment)")
parser.add_argument("--eval-episodes", type=int, default=VALID_EPS, help="Number of evaluation episodes")
args = parser.parse_args()

# Render mode configuration
RENDER_MODE = "human" if args.render_video and args.eval_only else None

# Adjust checkpoint directory and max steps based on environment type
ABLATION_TAG = "_ablation" if args.ablation and args.env_type == "lift" else ""
if args.env_type == "grasp":
    CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), "checkpoints_grasp")
    VALID_MAX_STEPS = VALID_MAX_STEPS_GRASP
elif args.env_type == "lift":
    CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), f"checkpoints_lift{ABLATION_TAG}")
    VALID_MAX_STEPS = LIFT_MAX_STEPS
elif args.env_type == "release":
    CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), "checkpoints_release")
    VALID_MAX_STEPS = RELEASE_MAX_STEPS
else:
    VALID_MAX_STEPS = VALID_MAX_STEPS_POSITION

# Create checkpoint directory
os.makedirs(CHECKPOINT_DIR, exist_ok=True)

def make():
    """Create environment - can be positioning (Env A) or grasping (Env B)"""
    if args.env_type == "grasp":
        # Env B: Grasp-only environment
        if RENDER_MODE:
            return GripperGraspEnv(render_mode=RENDER_MODE, allow_xy_adjust=args.allow_xy_adjust)
        return GripperGraspEnv(allow_xy_adjust=args.allow_xy_adjust)
    elif args.env_type == "lift":
        # Env C: Lift and move to target position
        if RENDER_MODE:
            return GripperLiftEnv(render_mode=RENDER_MODE, ablation=args.ablation)
        return GripperLiftEnv(ablation=args.ablation)
    elif args.env_type == "release":
        # Env D: Release the block
        if RENDER_MODE:
            return GripperReleaseEnv(render_mode=RENDER_MODE)
        return GripperReleaseEnv()
    else:
        # Env A: Positioning environment (default)
        if RENDER_MODE:
            return GripperEnv(render_mode=RENDER_MODE, enable_updown_control=args.enable_updown)
        return GripperEnv(enable_updown_control=args.enable_updown)

# Skip env creation and training if eval-only mode
if not args.eval_only:
    n_envs = 1 if RENDER_MODE == "human" else N_ENVS
    venv = make_vec_env(make, n_envs=n_envs, seed=0)

    if args.env_type == "grasp":
        print(f"PPO training for grasping with XY adjust: {args.allow_xy_adjust}")
    elif args.env_type == "lift":
        print(f"PPO training for lift and hover task{' with ablation' if args.ablation else ''}")
    elif args.env_type == "release":
        print(f"PPO training for release task")
    else:
        print(f"PPO training for positioning with up/down control: {args.enable_updown}")
    print(f"ent_coef={args.ent_coef}, learning_rate={args.learning_rate}, train_timesteps={args.train_timesteps}")
    
    # Improved PPO hyperparameters for deterministic precision
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
            if args.env_type == "grasp":
                model_prefix = "ppo_grasp_model"
            elif args.env_type == "lift":
                model_prefix = f"ppo_lift_model{ABLATION_TAG}"
            elif args.env_type == "release":
                model_prefix = "ppo_release_model"
            else:
                model_prefix = "ppo_position_model"
            checkpoint_path = os.path.join(
                self.save_path, 
                f"{model_prefix}_{self.num_timesteps}.pt"
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
    if args.env_type == "grasp":
        model_prefix = "ppo_grasp_model"
    elif args.env_type == "lift":
        model_prefix = f"ppo_lift_model{ABLATION_TAG}"
    elif args.env_type == "release":
        model_prefix = "ppo_release_model"
    else:
        model_prefix = "ppo_position_model"
    checkpoint_callback = CheckpointCallback(
        save_freq=max(CHECKPOINT_INTERVAL // N_ENVS, 1),
        save_path=CHECKPOINT_DIR,
        name_prefix=model_prefix,
        save_replay_buffer=False,
    )

    pytorch_callback = PyTorchCheckpointCallback(
        save_path=CHECKPOINT_DIR,
        save_freq=CHECKPOINT_INTERVAL,
        verbose=1
    )

    # Set reward log path based on environment type
    if args.env_type == "grasp":
        reward_log_path = os.path.join(CHECKPOINT_DIR, "reward_log_grasp.csv")
    elif args.env_type == "lift":
        reward_log_path = os.path.join(CHECKPOINT_DIR, f"reward_log_lift{ABLATION_TAG}.csv")
    elif args.env_type == "release":
        reward_log_path = os.path.join(CHECKPOINT_DIR, "reward_log_release.csv")
    else:
        reward_log_path = os.path.join(CHECKPOINT_DIR, "reward_log_position.csv")

    reward_logger = RewardLoggingCallback(log_path=reward_log_path, verbose=1)

    # Train with callbacks
    model.learn(
        total_timesteps=int(args.train_timesteps),
        callback=[checkpoint_callback, pytorch_callback, reward_logger]
    )

    # Save final model
    if args.env_type == "grasp":
        model_prefix = "ppo_grasp_model"
    elif args.env_type == "lift":
        model_prefix = f"ppo_lift_model{ABLATION_TAG}"
    elif args.env_type == "release":
        model_prefix = "ppo_release_model"
    else:
        model_prefix = "ppo_position_model"
    final_model_path = os.path.join(CHECKPOINT_DIR, f"{model_prefix}_final.zip")
    model.save(final_model_path)
    print(f"Saved final model to {final_model_path}")

    final_pt_path = os.path.join(CHECKPOINT_DIR, f"{model_prefix}_final.pt")
    torch.save({
        'policy_state_dict': model.policy.state_dict(),
        'optimizer_state_dict': model.policy.optimizer.state_dict(),
        'timesteps': TRAIN_EPS,
    }, final_pt_path)
    print(f"Saved final PyTorch checkpoint to {final_pt_path}")
else:
    print("Eval-only mode: skipping training")

# Validation
validation_render_mode = "rgb_array" if args.render_video else RENDER_MODE
if args.render_video:
    video_width, video_height = (1280, 720)
else:
    video_width, video_height = (480, 480)

# Create appropriate environment for validation
if args.env_type == "grasp":
    if args.render_video:
        env = GripperGraspEnv(render_mode=validation_render_mode, width=video_width, height=video_height, allow_xy_adjust=args.allow_xy_adjust)
    else:
        env = GripperGraspEnv(render_mode=validation_render_mode, allow_xy_adjust=args.allow_xy_adjust)
elif args.env_type == "lift":
    if args.render_video:
        env = GripperLiftEnv(render_mode=validation_render_mode, width=video_width, height=video_height, ablation=args.ablation)
    else:
        env = GripperLiftEnv(render_mode=validation_render_mode, ablation=args.ablation)
elif args.env_type == "release":
    if args.render_video:
        env = GripperReleaseEnv(render_mode=validation_render_mode, width=video_width, height=video_height)
    else:
        env = GripperReleaseEnv(render_mode=validation_render_mode)
else:
    env = GripperEnv(render_mode=validation_render_mode, width=video_width, height=video_height, enable_updown_control=args.enable_updown)

obs, info = env.reset(seed=123)
total_r, successes = 0.0, 0

# Load model for eval-only mode
if args.eval_only:
    if args.model_path:
        print(f"Loading model from {args.model_path}")
        model = PPO.load(args.model_path, env=env)
    else:
        if args.env_type == "grasp":
            model_prefix = "ppo_grasp_model"
        elif args.env_type == "lift":
            model_prefix = f"ppo_lift_model{ABLATION_TAG}"
        elif args.env_type == "release":
            model_prefix = "ppo_release_model"
        else:
            model_prefix = "ppo_position_model"
        final_model_path = os.path.join(CHECKPOINT_DIR, f"{model_prefix}_final.zip")
        if os.path.exists(final_model_path):
            print(f"Loading model from {final_model_path}")
            model = PPO.load(final_model_path, env=env)
        else:
            print("Error: No model found for eval-only mode. Provide --model-path or train first.")
            exit(1)

# Create video directory if rendering videos
VIDEO_DIR = None
if args.render_video:
    if args.env_type == "grasp":
        video_dir_name = "videos_grasp"
    elif args.env_type == "lift":
        video_dir_name = f"videos_lift{ABLATION_TAG}"
    elif args.env_type == "release":
        video_dir_name = "videos_release"
    else:
        video_dir_name = "videos"
    VIDEO_DIR = os.path.join(os.path.dirname(__file__), video_dir_name)
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
    
    # Track different metrics based on environment type
    if args.env_type == "grasp":
        ep_vertical_dist = []
        ep_horizontal_dist = []
        ep_grasped = []
        ep_ground_contact = []
        ep_vel = []
    elif args.env_type == "lift":
        ep_horiz = []
        ep_height = []
        ep_vel = []
        ep_grasped = []
    elif args.env_type == "release":
        ep_horiz = []
        ep_block_z = []
        ep_finger_dist = []
        ep_grasped = []
        ep_released = []
        ep_landed = []
    else:
        ep_horiz = []
        ep_dz = []
        ep_vel = []

    for step in range(VALID_MAX_STEPS):
        action, _ = model.predict(obs, deterministic=(not args.stochastic))
        action = action * float(args.action_scale)
        obs, r, term, trunc, info = env.step(action)
        
        ep_actions.append(np.array(action).ravel())
        total_r += r
        total_r_inner += r
        ep_length += 1
        success = term
        
        # Track environment-specific metrics
        if args.env_type == "grasp":
            ep_vertical_dist.append(info.get("vertical_dist", np.nan))
            ep_horizontal_dist.append(info.get("horizontal_dist", np.nan))
            ep_grasped.append(info.get("grasped", False))
            ep_ground_contact.append(info.get("ground_contact", False))
            ep_vel.append(info.get("gripper_velocity", np.nan))
            
            if log_this_ep and step % 10 == 0:
                print(f"  Step {step}: action={action} vertical_dist={info.get('vertical_dist', 0.0):.4f} "
                      f"horizontal_dist={info.get('horizontal_dist', 0.0):.4f} grasped={info.get('grasped', False)} "
                      f"ground_contact={info.get('ground_contact', False)} reward={r:.4f}")
        elif args.env_type == "lift":
            ep_horiz.append(info.get("horizontal_dist", np.nan))
            ep_height.append(info.get("block_height", np.nan))
            ep_vel.append(info.get("velocity", np.nan))
            ep_grasped.append(info.get("grasped", False))
            if log_this_ep and step % 10 == 0:
                print(f"  Step {step}: action={action} horiz_dist={info.get('horizontal_dist', 0.0):.4f} "
                      f"height={info.get('block_height', 0.0):.4f} grasped={info.get('grasped', False)} reward={r:.4f}")
        elif args.env_type == "release":
            ep_horiz.append(info.get("horizontal_dist", np.nan))
            ep_block_z.append(info.get("block_z", np.nan))
            ep_finger_dist.append(info.get("finger_distance", np.nan))
            ep_grasped.append(info.get("grasped", False))
            ep_released.append(info.get("block_released", False))
            ep_landed.append(info.get("block_landed", False))
            if log_this_ep and step % 10 == 0:
                print(f"  Step {step}: action={action} horiz_dist={info.get('horizontal_dist', 0.0):.4f} "
                      f"block_z={info.get('block_z', 0.0):.4f} finger={info.get('finger_distance', 0.0):.4f} "
                      f"grasped={info.get('grasped', False)} released={info.get('block_released', False)} "
                      f"landed={info.get('block_landed', False)} reward={r:.4f}")
        else:
            ep_horiz.append(info.get("horizontal_dist", np.nan))
            ep_dz.append(info.get("dz", np.nan))
            ep_vel.append(info.get("velocity", np.nan))

        if args.render_video:
            frame = env.render()
            if frame is not None:
                frames.append(frame)

        if term or trunc:
            # Success conditions vary by environment type
            if args.env_type == "grasp":
                successes += int(info.get("grasped", False))
            elif args.env_type == "lift":
                successes += int(term)  # Lift env sets term=True on success
            elif args.env_type == "release":
                successes += int(term)  # Release env sets term=True on success
            else:
                successes += int(term)  # Position env sets term=True on success
            obs, info = env.reset()
            break

    # Save video if frames were collected
    if args.render_video and len(frames) > 0:
        if args.env_type == "grasp":
            video_prefix = "validation_grasp_ep"
        elif args.env_type == "lift":
            video_prefix = f"validation_lift{ABLATION_TAG}_ep"
        elif args.env_type == "release":
            video_prefix = "validation_release_ep"
        else:
            video_prefix = "validation_ep"
        
        video_path = os.path.join(VIDEO_DIR, f"{video_prefix}_{i:03d}.mp4")
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
        
        if args.env_type == "grasp":
            print(f"  vertical dist mean: {np.nanmean(ep_vertical_dist):.4f} std: {np.nanstd(ep_vertical_dist):.4f}")
            print(f"  horizontal dist mean: {np.nanmean(ep_horizontal_dist):.4f} std: {np.nanstd(ep_horizontal_dist):.4f}")
            print(f"  grasped: {sum(ep_grasped)}/{len(ep_grasped)} steps, ground_contact: {sum(ep_ground_contact)}/{len(ep_ground_contact)} steps")
            print(f"  final vertical_dist: {ep_vertical_dist[-1]:.4f} horizontal_dist: {ep_horizontal_dist[-1]:.4f}")
            if len(ep_vel) > 0 and not (isinstance(ep_vel[-1], np.ndarray) and np.isnan(ep_vel[-1]).any()):
                print(f"  final velocity: {ep_vel[-1]}")
        elif args.env_type == "lift":
            print(f"  horiz dist mean: {np.nanmean(ep_horiz):.4f} std: {np.nanstd(ep_horiz):.4f}")
            print(f"  height mean: {np.nanmean(ep_height):.4f} std: {np.nanstd(ep_height):.4f}")
            print(f"  grasped: {sum(ep_grasped)}/{len(ep_grasped)} steps")
            print(f"  final dist: {ep_horiz[-1]:.4f} height: {ep_height[-1]:.4f} vel: {ep_vel[-1]}")
        elif args.env_type == "release":
            print(f"  horiz dist mean: {np.nanmean(ep_horiz):.4f} std: {np.nanstd(ep_horiz):.4f}")
            print(f"  block_z mean: {np.nanmean(ep_block_z):.4f} std: {np.nanstd(ep_block_z):.4f}")
            print(f"  finger_dist mean: {np.nanmean(ep_finger_dist):.4f} std: {np.nanstd(ep_finger_dist):.4f}")
            print(f"  grasped: {sum(ep_grasped)}/{len(ep_grasped)} steps")
            print(f"  released: {any(ep_released)}, landed: {any(ep_landed)}")
            print(f"  final dist: {ep_horiz[-1]:.4f} block_z: {ep_block_z[-1]:.4f} finger: {ep_finger_dist[-1]:.4f}")
        else:
            print(f"  horiz dist mean: {np.nanmean(ep_horiz):.4f} std: {np.nanstd(ep_horiz):.4f}")
            print(f"  dz mean: {np.nanmean(ep_dz):.4f} std: {np.nanstd(ep_dz):.4f}")
            print(f"  final vertical_dist: {ep_dz[-1]:.4f} horizontal_dist: {ep_horiz[-1]:.4f}")
            if len(ep_vel) > 0 and not (isinstance(ep_vel[-1], np.ndarray) and np.isnan(ep_vel[-1]).any()):
                print(f"  final velocity: {ep_vel[-1]}")
    else:
        print(f"Eval {i}: total_reward: {total_r_inner:.2f}, ep_length: {ep_length}, success: {success}")

print(f"\n=== FINAL RESULTS ===")
print(f"Mean reward: {total_r / args.eval_episodes:.2f}")
print(f"Success rate: {successes}/{args.eval_episodes} ({100.0 * successes / args.eval_episodes:.1f}%)")
print(f"Deterministic mode: {not args.stochastic}")

# Clean up
env.close()

