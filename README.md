# MuJoCo Gripper Training

This project implements a two-stage curriculum learning approach for training a gripper to position over and grasp a block using MuJoCo and Stable-Baselines3.

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

### Stage 1: Position Training (Env A)

Train the gripper to position itself horizontally over the block.

**Basic training:**
```bash
cd gripper_mj
python train_ppo.py
```

**With rendering (visualize training):**
```bash
cd gripper_mj
RENDER_MODE=human python train_ppo.py
```

**Training parameters:**
- Default training timesteps: 100,000
- Checkpoints saved every 25,000 timesteps
- Model saved to: `gripper_mj/checkpoints/ppo_model_final.zip`

### Stage 2: Grasp Training (Env B)

Train the gripper to descend and grasp the block. The gripper starts already positioned above the block.

**Option 1: Train from scratch**
```bash
cd gripper_mj
ENV_TYPE=grasp python train_ppo.py
```

**Option 2: Train with small XY adjustments**
```bash
cd gripper_mj
ENV_TYPE=grasp ALLOW_XY_ADJUST=true python train_ppo.py
```

**Option 3: Initialize from Stage 1 weights (transfer learning)**
```bash
cd gripper_mj
ENV_TYPE=grasp python train_ppo.py --load-checkpoint checkpoints/ppo_model_final.zip
```

**Option 4: Use helper script**
```bash
cd gripper_mj
./train_grasp_env.sh                    # Train from scratch
./train_grasp_env.sh --load-env-a       # Load from Stage 1
```

**Training parameters:**
- Default training timesteps: 100,000
- Shorter episodes (200 steps max) since gripper starts close
- Model saved to: `gripper_mj/checkpoints/ppo_model_final.zip` (or `ppo_model_grasp_final.zip`)

## Testing

### Test Position Policy (Env A)

Test the positioning policy:

**Basic test:**
```bash
cd gripper_mj
python train_ppo.py --eval-only
```

**Test with video rendering:**
```bash
cd gripper_mj
python train_ppo.py --eval-only --render-video
```

**Test specific checkpoint:**
```bash
cd gripper_mj
python train_ppo.py --eval-only --model-path checkpoints/ppo_model_final.zip --render-video
```

**Test with stochastic actions (for exploration):**
```bash
cd gripper_mj
python train_ppo.py --eval-only --stochastic --render-video
```

### Test Grasp Policy (Env B)

Test the grasping policy using the dedicated test script:

**Basic test:**
```bash
cd gripper_mj
python test_grasp_policy.py --checkpoint checkpoints/ppo_model_final.zip
```

**Test with video rendering:**
```bash
cd gripper_mj
python test_grasp_policy.py --checkpoint checkpoints/ppo_model_final.zip --render-video
```

**Test with XY adjustments (if trained with it):**
```bash
cd gripper_mj
python test_grasp_policy.py --checkpoint checkpoints/ppo_model_final.zip --allow-xy --render-video
```

**Custom number of episodes:**
```bash
cd gripper_mj
python test_grasp_policy.py --checkpoint checkpoints/ppo_model_final.zip --episodes 50 --render-video
```

**Auto-detect checkpoint:**
```bash
cd gripper_mj
python test_grasp_policy.py --render-video
```

### Test Output

The test scripts provide:
- Per-episode results (success/failure, reward, steps, etc.)
- Summary statistics:
  - Success rate
  - Mean/std/min/max rewards
  - Episode length statistics
  - Grasp timing (for grasping policy)

Videos are saved to: `gripper_mj/videos/`

## Environment Details

### Env A: Position Environment (`GripperEnv`)
- **Goal**: Position gripper horizontally over the block
- **Action space**: 3D `[up/down, left/right, forward/back]` (up/down fixed)
- **Observation space**: 8D `[relative_x, relative_y, dist_x, dist_y, gripper_x, gripper_y, gripper_vel_x, gripper_vel_y]`
- **Success**: Distance ≤ 0.01m (SUCCESS_THRESHOLD)
- **Max steps**: 500

### Env B: Grasp Environment (`GripperGraspEnv`)
- **Goal**: Descend and grasp the block (gripper starts positioned above block)
- **Action space**: 
  - 2D `[up/down, finger]` (default, no XY movement)
  - 4D `[up/down, xy_x, xy_y, finger]` (with `ALLOW_XY_ADJUST=true`)
- **Observation space**: 
  - 4D `[vertical_dist, finger_state, grasped, gripper_z]` (without XY)
  - 6D `[vertical_dist, horizontal_dist, finger_state, grasped, gripper_z, block_z]` (with XY)
- **Success**: Successful grasp detected via contact
- **Max steps**: 200

## Checkpoints

Checkpoints are saved in `gripper_mj/checkpoints/`:
- `ppo_model_final.zip` - Final model (latest training)
- `ppo_model_<timesteps>_steps.zip` - Intermediate checkpoints
- `ppo_model_<timesteps>.pt` - PyTorch state dicts
- `ppo_model_grasp_final.zip` - Grasp-specific final model

## Running the MuJoCo Simulation (Original)

For the original simulation interface:
```bash
mjpython main.py --control [model_name]
```