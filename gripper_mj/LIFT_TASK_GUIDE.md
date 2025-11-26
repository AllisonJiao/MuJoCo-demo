# Lift and Hover Task - Usage Guide

This guide explains how to use the newly implemented "lift and hover" task (Stage 3) for the gripper RL training pipeline.

## Overview

The lift and hover task is the third stage in the gripper manipulation curriculum:
1. **Stage 1**: Position gripper over block
2. **Stage 2**: Lower and grasp block
3. **Stage 3**: Raise grasped block and hover over target ← **This implementation**

## Environment: `GripperLiftEnv`

### Key Characteristics

- **Initial State**: Gripper starts with block already grasped between fingers
- **Randomization**: Both block and target positions are randomized at each episode
- **Goal**: Move the grasped block to hover above a target position at the correct height
- **Difficulty**: Agent must maintain grasp while navigating in 3D space

### Observation Space (11-dimensional)

```python
[
    rel_x, rel_y, rel_z,        # Normalized relative position (block to target)
    gripper_z,                   # Absolute gripper height
    horizontal_dist,             # Horizontal distance (block to target)
    block_height,                # Block height above target
    finger_distance,             # Distance between fingers
    gripper_vel_x, gripper_vel_y,  # Gripper velocities
    block_z,                     # Absolute block height
    grasped                      # 1.0 if grasped, 0.0 if dropped
]
```

### Action Space (4-dimensional)

```python
[
    up/down,          # Vertical movement (-1 to 1)
    left/right,       # Lateral movement (-1 to 1)
    forward/backward, # Forward/backward movement (-1 to 1)
    finger            # Finger control (-1=open, 1=close)
]
```

### Success Criteria

An episode is successful when ALL conditions are met:
- ✓ Horizontal distance to target ≤ 0.0075m
- ✓ Block height above target within 0.05m of target height (0.25m)
- ✓ Gripper velocity < 0.0015 m/s
- ✓ Block is still grasped

## Training

### Basic Training

```bash
cd gripper_mj
python train_ppo_lift_improved.py
```

This will:
- Train for 100,000 timesteps by default
- Save checkpoints every 25,000 steps to `checkpoints_lift/`
- Run 10 validation episodes after training
- Display training progress and statistics

### Advanced Training Options

**Custom training duration:**
```bash
python train_ppo_lift_improved.py --train-timesteps 200000
```

**Adjust learning rate and exploration:**
```bash
python train_ppo_lift_improved.py --learning-rate 5e-4 --ent-coef 0.02
```

**Control number of evaluation episodes:**
```bash
python train_ppo_lift_improved.py --eval-episodes 20
```

## Evaluation

### Load and Test Trained Model

```bash
cd gripper_mj
python train_ppo_lift_improved.py --eval-only
```

This loads `checkpoints_lift/ppo_lift_model_final.zip` and runs evaluation episodes.

### Test Specific Checkpoint

```bash
python train_ppo_lift_improved.py --eval-only --model-path checkpoints_lift/ppo_lift_model_50000_steps.zip
```

### Generate Videos

```bash
python train_ppo_lift_improved.py --eval-only --render-video --eval-episodes 5
```

Videos will be saved to `videos_lift/` directory.

### Debug Mode

```bash
python train_ppo_lift_improved.py --eval-only --debug-log
```

Prints detailed per-step diagnostics for the first evaluation episode.

## Reward Function

The environment uses a sophisticated reward function to encourage learning:

1. **Horizontal Alignment** (-horizontal_dist): Penalize distance from target
2. **Progress Reward**: Reward for getting closer to target
3. **Height Maintenance**: Reward for maintaining appropriate height above target
4. **Finger Control**: Reward for keeping fingers closed (maintaining grasp)
5. **Precision Bonus**: Exponential reward when very close to target
6. **Stuck Penalty**: Small penalty if agent isn't making progress
7. **Success Bonus**: Large reward (+20) when all success criteria are met
8. **Drop Penalty**: Large penalty (-50) if block is dropped

## Training Tips

1. **Start with default parameters**: The default hyperparameters are well-tuned
2. **Monitor grasp**: Check evaluation logs to ensure grasp is maintained
3. **Checkpoint frequently**: Use intermediate checkpoints if training diverges
4. **Visualization**: Use `--render-video` to understand agent behavior
5. **Long training**: This task may require 200k+ timesteps for good performance

## File Structure

```
gripper_mj/
├── gripper_env_lift_improved.py   # Environment implementation
├── train_ppo_lift_improved.py     # Training script
├── test_lift_env.py                # Basic environment tests
├── checkpoints_lift/               # Saved models (gitignored)
│   ├── ppo_lift_model_final.zip
│   ├── ppo_lift_model_25000_steps.zip
│   └── ppo_lift_model_*.pt
└── videos_lift/                    # Evaluation videos (gitignored)
    └── validation_lift_ep_*.mp4
```

## Common Issues

### Block Drops Immediately

If the block drops right after reset:
- Check that fingers are set to closed position in initialization
- Verify the settling period in `reset_model()` is sufficient
- Increase finger control strength during reset

### Agent Can't Reach Target

- Increase training timesteps (try 200k-300k)
- Adjust learning rate or entropy coefficient
- Check if reward function is providing proper gradients

### Training Unstable

- Reduce learning rate (try 1e-4)
- Increase batch size
- Enable more frequent checkpointing

## Next Steps

After successfully training the lift task, potential next stages could include:
- **Stage 4**: Lower block to target and release
- **Stage 5**: Full pick-and-place task combining all stages
- **Transfer Learning**: Use lift policy as initialization for subsequent tasks

## Testing

Run basic environment tests:
```bash
cd gripper_mj
python test_lift_env.py
```

Expected output should show:
- ✓ Environment creates successfully
- ✓ Observation shape is (11,)
- ✓ Block is grasped initially (grasped=True)
- ✓ Grasp maintained during random actions
