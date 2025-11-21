#!/bin/bash
# Script to train Env B (grasp-only environment)

# Configuration
ENV_A_CHECKPOINT="checkpoints/ppo_model_final.zip"  # Optional: load from Env A
ALLOW_XY_ADJUST="false"  # Set to "true" to allow small XY adjustments

echo "=========================================="
echo "Training Env B: Grasp-only Environment"
echo "=========================================="
echo "Gripper starts already positioned above block"
echo "Agent learns: descend, close fingers, lift"
echo ""

# Check if we should load from Env A
if [ -f "$ENV_A_CHECKPOINT" ] && [ "$1" == "--load-env-a" ]; then
    echo "Loading weights from Env A: $ENV_A_CHECKPOINT"
    ENV_TYPE=grasp ALLOW_XY_ADJUST=$ALLOW_XY_ADJUST python train_ppo.py --load-checkpoint "$ENV_A_CHECKPOINT"
else
    echo "Training from scratch"
    ENV_TYPE=grasp ALLOW_XY_ADJUST=$ALLOW_XY_ADJUST python train_ppo.py
fi

echo ""
echo "Training complete!"
echo "Model saved as: checkpoints/ppo_model_final.zip"

