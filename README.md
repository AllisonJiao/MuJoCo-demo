# MuJoCo Gripper Training

This project implements a multi-stage curriculum learning approach for training a gripper to manipulate a block using MuJoCo and Stable-Baselines3. All training is done through unified scripts that support all four stages:
- `train_ppo_improved.py` - Proximal Policy Optimization (PPO) training
- `train_sac_improved.py` - Soft Actor-Critic (SAC) training

## Setup

1. Create a virtual environment with Python 3.9
```bash
conda create -n mujoco python=3.9
conda activate mujoco
```

2. Install MuJoCo and dependencies:
   - [Build MuJoCo from source](https://mujoco.readthedocs.io/en/latest/programming/#building-mujoco-from-source)
   - [Install the Python bindings](https://mujoco.readthedocs.io/en/stable/python.html)

3. Install Prerequisite Libraries
```bash
pip install -r requirements.txt
```

4. Connect to docker and MongoDB (optional, comment out in code if not needed)
```bash
docker compose up -d
```

## Training

All training is done through unified scripts. Use the `--env-type` argument to select which stage to train:
- **PPO**: `train_ppo_improved.py` - On-policy algorithm, good for stable learning
- **SAC**: `train_sac_improved.py` - Off-policy algorithm, sample-efficient

### Stage 1: Position Training (Env A)

Train the gripper to position itself horizontally over the block.

**Basic training:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type position
```

**With learnable up/down control:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type position --enable-updown
```

**Training with custom parameters:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type position --train-timesteps 200000 --learning-rate 3e-4 --ent-coef 0.01
```

**Training parameters:**
- Default training timesteps: 100,000
- Checkpoints saved every 25,000 timesteps
- Model saved to: `gripper_mj/checkpoints/ppo_position_model_final.zip`
- Reward logs: `gripper_mj/checkpoints/reward_log_position.csv`

### Stage 2: Grasp Training (Env B)

Train the gripper to descend and grasp the block. The gripper starts already positioned above the block.

**Basic training (no XY adjustments):**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type grasp
```

**Training with small XY adjustments:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type grasp --allow-xy-adjust
```

**Training with custom parameters:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type grasp --train-timesteps 200000 --learning-rate 3e-4
```

**Training parameters:**
- Default training timesteps: 100,000
- Episodes: 500 steps max
- Model saved to: `gripper_mj/checkpoints_grasp/ppo_grasp_model_final.zip`
- Reward logs: `gripper_mj/checkpoints_grasp/reward_log_grasp.csv`

### Stage 3: Lift and Hover Training (Env C)

Train the gripper to raise a grasped block and hover over a target position. The gripper starts with the block already grasped.

**Basic training:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type lift
```

**Training with ablation study:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type lift --ablation
```

**Training with custom parameters:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type lift --train-timesteps 200000 --learning-rate 3e-4
```

**Training parameters:**
- Default training timesteps: 100,000
- Episodes: 500 steps max
- Checkpoints saved every 25,000 timesteps
- Model saved to: `gripper_mj/checkpoints_lift/ppo_lift_model_final.zip` (or `checkpoints_lift_ablation/` with `--ablation`)
- Reward logs: `gripper_mj/checkpoints_lift/reward_log_lift.csv`
- Videos saved to: `gripper_mj/videos_lift/`

### Stage 4: Release Training (Env D)

Train the gripper to release the block at the target position. The gripper starts with the block already grasped and positioned above a target.

**Basic training:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type release
```

**Training with custom parameters:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type release --train-timesteps 200000 --learning-rate 3e-4
```

**Training parameters:**
- Default training timesteps: 100,000
- Episodes: 500 steps max
- Checkpoints saved every 25,000 timesteps
- Model saved to: `gripper_mj/checkpoints_release/ppo_release_model_final.zip`
- Reward logs: `gripper_mj/checkpoints_release/reward_log_release.csv`
- Videos saved to: `gripper_mj/videos_release/`

## Evaluation

All environments support evaluation mode using the unified training scripts. Both PPO and SAC scripts support the same evaluation arguments.

### Evaluation Only (Load Trained Model)

**Basic evaluation (PPO):**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type [position|grasp|lift|release] --eval-only
```

**Basic evaluation (SAC):**
```bash
cd gripper_mj
python train_sac_improved.py --env-type [position|grasp|lift|release] --eval-only
```

**Evaluation with video rendering:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type [position|grasp|lift|release] --eval-only --render-video
# or
python train_sac_improved.py --env-type [position|grasp|lift|release] --eval-only --render-video
```

**Evaluation with specific model:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type [position|grasp|lift|release] --eval-only --model-path checkpoints/ppo_position_model_final.zip --render-video
# or for SAC
python train_sac_improved.py --env-type [position|grasp|lift|release] --eval-only --model-path checkpoints_sac_position/sac_position_model_final.zip --render-video
```

**Evaluation with stochastic actions (for exploration):**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type [position|grasp|lift|release] --eval-only --stochastic --render-video
```

**Evaluation with custom action scaling:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type [position|grasp|lift|release] --eval-only --action-scale 0.5 --render-video
```

**Evaluation with debug logging:**
```bash
cd gripper_mj
python train_ppo_improved.py --env-type [position|grasp|lift|release] --eval-only --debug-log --render-video
```

### Evaluation Output

The evaluation provides:
- Per-episode results (success/failure, reward, steps, etc.)
- Summary statistics:
  - Success rate
  - Mean reward
  - Episode length statistics
  - Environment-specific metrics (distances, heights, velocities, etc.)

Videos are saved to environment-specific directories:
- Position: `gripper_mj/videos/` (PPO) or `gripper_mj/videos_sac/` (SAC)
- Grasp: `gripper_mj/videos_grasp/` (PPO) or `gripper_mj/videos_sac_grasp/` (SAC)
- Lift: `gripper_mj/videos_lift/` (PPO) or `gripper_mj/videos_sac_lift/` (SAC)
  - With `--ablation`: `videos_lift_ablation/` (PPO) or `videos_sac_lift_ablation/` (SAC)
- Release: `gripper_mj/videos_release/` (PPO) or `gripper_mj/videos_sac_release/` (SAC)

## SAC Training (Alternative Algorithm)

The project also supports Soft Actor-Critic (SAC) training for all stages using the unified `train_sac_improved.py` script. SAC is an off-policy algorithm that can be more sample-efficient than PPO.

**Position:**
```bash
cd gripper_mj
python train_sac_improved.py --env-type position
```

**Grasp:**
```bash
cd gripper_mj
python train_sac_improved.py --env-type grasp
```

**Lift:**
```bash
cd gripper_mj
python train_sac_improved.py --env-type lift
```

**Lift with ablation study:**
```bash
cd gripper_mj
python train_sac_improved.py --env-type lift --ablation
```

**Release:**
```bash
cd gripper_mj
python train_sac_improved.py --env-type release
```

**Training with custom parameters:**
```bash
cd gripper_mj
python train_sac_improved.py --env-type [position|grasp|lift|release] --train-timesteps 200000 --learning-rate 3e-4 --buffer-size 200000
```

**SAC-specific arguments:**
- `--buffer-size`: Replay buffer size (default: 100000)
- `--learning-starts`: Number of steps before learning starts (default: 1000)
- `--batch-size`: Batch size for training (default: 256)
- `--tau`: Soft update coefficient for target network (default: 0.005)
- `--ent-coef`: Entropy coefficient - `"auto"` (default) or float value

SAC checkpoints are saved to `checkpoints_sac_*/` directories, and reward logs follow the same naming convention. SAC also saves VecNormalize statistics (`.pkl` files) for observation normalization.

## Plotting Training Metrics

Use `plot.py` to visualize and compare training metrics between PPO and SAC:

**Plot all metrics for a stage:**
```bash
cd gripper_mj
python plot.py --position --all    # Position stage
python plot.py --grasp --all       # Grasp stage
python plot.py --lift --all        # Lift stage
python plot.py --release --all     # Release stage
```

**Plot specific metrics:**
```bash
cd gripper_mj
python plot.py --grasp --reward --episode-length
python plot.py --lift --entropy-loss --value-loss
```

**Available metrics:**
- `--reward`: Episode reward
- `--episode-length`: Episode length
- `--entropy-loss`: Entropy loss (separate plots for PPO and SAC)
- `--value-loss`: Value loss (separate plots for PPO and SAC)
- `--all`: All metrics (default if no specific metric is selected)

**Smoothing window:**
```bash
cd gripper_mj
python plot.py --grasp --reward --window 20  # Use window size of 20 for smoothing
```

## Environment Details

### Env A: Position Environment (`GripperEnv`)
- **Goal**: Position gripper horizontally over the block
- **Action space**: 3D `[up/down, left/right, forward/back]` (up/down can be learnable with `--enable-updown`)
- **Observation space**: 8D `[relative_x, relative_y, dist_x, dist_y, gripper_x, gripper_y, gripper_vel_x, gripper_vel_y]`
- **Success**: Distance ≤ 0.01m (SUCCESS_THRESHOLD)
- **Max steps**: 500

### Env B: Grasp Environment (`GripperGraspEnv`)
- **Goal**: Descend and grasp the block (gripper starts positioned above block)
- **Action space**: 
  - 2D `[up/down, finger]` (default, no XY movement)
  - 4D `[up/down, xy_x, xy_y, finger]` (with `--allow-xy-adjust`)
- **Observation space**: 
  - 6D `[rel_z, vertical_dist, finger_state, grasped, block_z, gripper_vel_z]` (without XY)
  - 11D `[rel_x, rel_y, rel_z, vertical_dist, horizontal_dist, finger_state, grasped, block_z, gripper_vel_x, gripper_vel_y, gripper_vel_z]` (with XY)
- **Success**: Successful grasp detected via contact
- **Max steps**: 500

### Env C: Lift and Hover Environment (`GripperLiftEnv`)
- **Goal**: Raise grasped block and hover over target position
- **Action space**: 4D `[up/down, left/right, forward/back, finger]`
- **Observation space**: 11D `[rel_x, rel_y, rel_z, gripper_z, horizontal_dist, block_height, finger_dist, vel_x, vel_y, block_z, grasped]`
  - Relative position is from block to target
  - `block_height` is height of block above target
  - `grasped` is binary indicator (1.0 if block is grasped, 0.0 if dropped)
- **Success**: Block horizontally aligned with target, at correct height, with minimal velocity, and still grasped
- **Max steps**: 500
- **Initial condition**: Gripper starts with block already grasped

### Env D: Release Environment (`GripperReleaseEnv`)
- **Goal**: Release the block at the target position (gripper stays fixed, target is randomized below)
- **Action space**: 4D `[up/down, left/right, forward/back, finger]` (gripper position is fixed, actions control positioning)
- **Observation space**: 10D `[rel_gripper_to_target_dx, rel_gripper_to_target_dy, gripper_height_above_target, horizontal_dist_gripper_to_target, block_height_above_ground, finger_distance, gripper_vel_x, gripper_vel_y, block_vel_z, grasped]`
  - All positions are relative to simplify learning
  - Gripper position is fixed to avoid initial velocity drift
- **Success**: Block lands on ground within threshold of target (both `block_released` and `block_landed` must be True)
- **Max steps**: 500
- **Initial condition**: Gripper starts with block already grasped at fixed position

## Checkpoints

Checkpoints are saved in environment-specific directories:

### Position (Stage 1)
- Directory: `gripper_mj/checkpoints/`
- Files:
  - `ppo_position_model_final.zip` - Final model
  - `ppo_position_model_<timesteps>_steps.zip` - Intermediate checkpoints
  - `ppo_position_model_<timesteps>.pt` - PyTorch state dicts
  - `reward_log_position.csv` - Training metrics log

### Grasp (Stage 2)
- Directory: `gripper_mj/checkpoints_grasp/`
- Files:
  - `ppo_grasp_model_final.zip` - Final model
  - `ppo_grasp_model_<timesteps>_steps.zip` - Intermediate checkpoints
  - `ppo_grasp_model_<timesteps>.pt` - PyTorch state dicts
  - `reward_log_grasp.csv` - Training metrics log

### Lift (Stage 3)
- Directory: `gripper_mj/checkpoints_lift/` (or `checkpoints_lift_ablation/` with `--ablation`)
- Files:
  - `ppo_lift_model_final.zip` - Final model (or `ppo_lift_model_ablation_final.zip`)
  - `ppo_lift_model_<timesteps>_steps.zip` - Intermediate checkpoints
  - `ppo_lift_model_<timesteps>.pt` - PyTorch state dicts
  - `reward_log_lift.csv` - Training metrics log (or `reward_log_lift_ablation.csv`)

### Release (Stage 4)
- Directory: `gripper_mj/checkpoints_release/`
- Files:
  - `ppo_release_model_final.zip` - Final model
  - `ppo_release_model_<timesteps>_steps.zip` - Intermediate checkpoints
  - `ppo_release_model_<timesteps>.pt` - PyTorch state dicts
  - `reward_log_release.csv` - Training metrics log

### SAC Checkpoints
SAC models are saved in environment-specific directories:
- Position: `gripper_mj/checkpoints_sac_position/`
- Grasp: `gripper_mj/checkpoints_sac_grasp/`
- Lift: `gripper_mj/checkpoints_sac_lift/` (or `checkpoints_sac_lift_ablation/` with `--ablation`)
- Release: `gripper_mj/checkpoints_sac_release/`

Files follow the same naming convention as PPO:
- `sac_[env]_model_final.zip` - Final model
- `sac_[env]_model_<timesteps>_steps.zip` - Intermediate checkpoints
- `sac_[env]_model_<timesteps>.pt` - PyTorch state dicts
- `sac_[env]_model_vec_normalize.pkl` - VecNormalize statistics (SAC-specific)
- `reward_log_[env].csv` - Training metrics log

## Training Metrics Logging

All training scripts log metrics to CSV files using `RewardLoggingCallback`:
- `timesteps`: Current training timesteps
- `episode_reward`: Average episode reward for the rollout
- `episode_length`: Average episode length for the rollout
- `entropy_loss`: Entropy loss (policy gradient loss for PPO, actor loss for SAC)
- `value_loss`: Value loss (for PPO) or critic loss (for SAC)
- `explained_variance`: Explained variance (PPO only, NaN for SAC)

These logs can be visualized using `plot.py` to compare PPO and SAC performance.

## Command-Line Arguments

### Common Arguments (All Environments)
- `--env-type`: Environment type - `position`, `grasp`, `lift`, or `release` (default: `position`)
- `--train-timesteps`: Total timesteps for training (default: 100000)
- `--learning-rate`: Learning rate (default: 3e-4)
- `--ent-coef`: Entropy coefficient (default: 0.01 for PPO, `"auto"` for SAC)
- `--eval-only`: Skip training and only run validation
- `--render-video`: Save validation videos as .mp4 files
- `--model-path`: Path to a saved model (.zip) to load for evaluation
- `--stochastic`: Use stochastic actions during evaluation
- `--debug-log`: Print per-step diagnostics
- `--action-scale`: Multiply actions by this scale during evaluation (default: 1.0)
- `--eval-episodes`: Number of evaluation episodes (default: 10)

### Environment-Specific Arguments
- `--enable-updown`: Enable learnable up/down control (for position environment)
- `--allow-xy-adjust`: Allow small XY adjustments (for grasp environment)
- `--ablation`: Run ablation study (for lift environment)

### SAC-Specific Arguments
- `--buffer-size`: Replay buffer size (default: 100000)
- `--learning-starts`: Number of steps before learning starts (default: 1000)
- `--batch-size`: Batch size for training (default: 256)
- `--tau`: Soft update coefficient for target network (default: 0.005)
- `--normalize-path`: Path to VecNormalize stats (.pkl) to load for evaluation

## Running the MuJoCo Simulation (Original)

For the original simulation interface:
```bash
mjpython main.py --control [model_name]
```
