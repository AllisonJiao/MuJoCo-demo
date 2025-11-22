import os
import gymnasium as gym
from gymnasium import spaces

from gymnasium import utils
from gymnasium.spaces import Box
from mujoco_py_env import MuJocoPyEnv

import numpy as np
import mujoco
from gripper_controller import rand_spawn

BLOCK_DIMENSION = 0.05
SUCCESS_THRESHOLD = 0.15 * BLOCK_DIMENSION  # Slightly more lenient for deterministic policy
MAX_STEPS = 500
# Height constraint (meters) — gripper should hang at least this far above block
MIN_ABOVE = 0.2  # ~2 cm above block
TARGET_HEIGHT = BLOCK_DIMENSION + MIN_ABOVE  # Target height above block (5 cm)

"""
Improved GripperEnv with better reward shaping for deterministic precision and learnable up/down control.

Key improvements:
1. Separate reward components for horizontal and vertical positioning
2. Height maintenance reward to prevent drift
3. Precision reward that encourages getting very close
4. Action smoothing to reduce oscillations
5. Normalized observations for better learning
6. Curriculum-style reward that becomes more precise as distance decreases
"""

class GripperEnv(MuJocoPyEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human", 
            "rgb_array",
            "depth_array"
        ],
        "render_fps": 60,
    }

    def __init__(self, render_mode=None, width=480, height=480, enable_updown_control=True, **kwargs):
        utils.EzPickle.__init__(self, render_mode=render_mode, width=width, height=height, enable_updown_control=enable_updown_control, **kwargs)
        
        # Observation: [rel_dx, rel_dy, rel_dz, gripper_z, horizontal_dist, vertical_dist, gripper_vel_x, gripper_vel_y]
        # Adding distance components and velocities helps with precision
        observation_space = Box(low=-np.inf, high=np.inf, shape=(8,), dtype=np.float64)

        folder_path = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(folder_path, os.pardir, "model", "GripperGPT.xml")

        MuJocoPyEnv.__init__(
            self,
            model_path=model_path,
            frame_skip=1,
            observation_space=observation_space,
            render_mode=render_mode,
            width=width,
            height=height,
            **kwargs
        )

        self.step_count = 0
        self.max_steps = MAX_STEPS
        self.enable_updown_control = enable_updown_control
        
        # actuator ids
        self.updown = self.model.actuator("up/down").id
        self.leftright = self.model.actuator("left/right").id
        self.forwardback = self.model.actuator("forward/backward").id
        self.actuators = np.array([self.updown, self.leftright, self.forwardback], dtype=int)
        
        # body ids
        self.body = self.model.body("block").id
        self.target = self.model.body("target").id
        self.gripper = self.model.body("gripper").id

        # action = 3 motors (or 2 if up/down is fixed)
        if enable_updown_control:
            self.action_space = spaces.Box(low=-1, high=1, shape=(3,), dtype=np.float32)
        else:
            self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(8,), dtype=np.float32)

        # Control scaling: smaller for up/down to encourage fine control
        # Horizontal axes can be larger for faster movement
        if enable_updown_control:
            self.ctrl_scale = np.array([15, 15, 15], dtype=float)  # [up/down, left/right, forward/back]
        else:
            self.ctrl_scale = np.array([15, 15], dtype=float)  # [left/right, forward/back]
            self.fixed_updown_lift = 5.0  # Fixed upward lift when up/down is disabled
        
        # Reward shaping parameters
        self.gamma = 0.99
        self.prev_dist = None
        self.prev_horizontal_dist = None
        self.prev_dz = None
        
        # Action smoothing for deterministic policy (exponential moving average)
        # Stronger smoothing to reduce vibrations
        self.action_ema_alpha = 0.15 if not enable_updown_control else 0.2  # Lower = more smoothing
        self.prev_action = None
        
        # Track action history for drift detection (when up/down enabled)
        self.action_history = []
        self.action_history_size = 10

    def step(self, action):
        self.step_count += 1
        
        # Clip and scale action
        action_clipped = np.clip(action, -1, 1)
        
        # Store action for drift detection
        self.action_history.append(action_clipped.copy())
        if len(self.action_history) > self.action_history_size:
            self.action_history.pop(0)
        
        # Capture previous smoothed action (if any) before applying smoothing
        prev_action_before = self.prev_action.copy() if (hasattr(self, 'prev_action') and self.prev_action is not None) else None

        # Action smoothing for deterministic policy (reduces oscillations)
        if prev_action_before is not None:
            action_clipped = self.action_ema_alpha * action_clipped + (1 - self.action_ema_alpha) * prev_action_before
        
        # Adaptive action scaling: reduce action magnitude when close to target
        # This helps prevent vibrations near equilibrium
        # Use previous step's distance for preview
        horizontal_dist_preview = self.prev_horizontal_dist if self.prev_horizontal_dist is not None else 1.0
        '''
        # Reduce action scale when close (helps with convergence)
        if horizontal_dist_preview < 0.1:  # Within 10cm
            action_scale_factor = 0.3 + 0.7 * (horizontal_dist_preview / 0.1)  # Scale from 0.3 to 1.0
            action_clipped = action_clipped * action_scale_factor
        '''
        scaled_action = action_clipped * self.ctrl_scale
        
        # Apply actions to actuators
        if self.enable_updown_control:
            self.data.ctrl[self.updown] = scaled_action[0]
            self.data.ctrl[self.leftright] = scaled_action[1]
            self.data.ctrl[self.forwardback] = scaled_action[2]
        else:
            self.data.ctrl[self.updown] = self.fixed_updown_lift
            self.data.ctrl[self.leftright] = scaled_action[0]
            self.data.ctrl[self.forwardback] = scaled_action[1]

        # advance physics
        for i in range(1, 10):
            mujoco.mj_step(self.model, self.data)

        block_pos = self.data.xpos[self.body][:3]
        gripper_pos = self.data.xpos[self.gripper][:3]
        
        # Relative position vector
        rel = block_pos - gripper_pos
        
        # Distance metrics
        dist = np.linalg.norm(gripper_pos - block_pos)
        horizontal_dist = np.linalg.norm((gripper_pos - block_pos)[:2])
        vertical_dist = abs(rel[2])
        
        # Vertical offset (positive = gripper above block)
        dz = float(gripper_pos[2] - block_pos[2])
        
        # Get velocities for observation
        gripper_vel = self.data.qvel[:2] if len(self.data.qvel) >= 2 else np.array([0.0, 0.0])
        
        # Observation: [rel_dx, rel_dy, rel_dz, gripper_z, horizontal_dist, vertical_dist, gripper_vel_x, gripper_vel_y]
        # Normalize relative positions by a typical scale (0.5m) for better learning
        obs = np.concatenate([
            rel / 0.5,  # Normalized relative position
            np.array([gripper_pos[2]]),  # Absolute z for height reference
            np.array([horizontal_dist, vertical_dist]),  # Distance components
            gripper_vel  # Velocities
        ])
        
        # ========== REWARD SHAPING ==========
        
        # 1. Base reward: negative distance (encourages getting closer)
        base_reward = -dist * 2.0
        
        # 2. Horizontal precision reward (most important for "move over" task)
        # Use exponential decay to heavily reward being close horizontally
        horizontal_precision = -horizontal_dist * 8.0  # Increased weight
        # Bonus for being very close (encourages precision)
        if horizontal_dist < 0.05:  # Within 5cm
            horizontal_precision += 5.0 * (0.05 - horizontal_dist) / 0.05  # Stronger bonus
        
        # 2b. Settling reward: reward for being still when close (reduces vibrations)
        velocity_magnitude = np.linalg.norm(gripper_vel)
        settling_reward = 0.0
        if horizontal_dist < 0.02:  # Very close
            # Reward low velocity (being still)
            settling_reward = 2.0 * (1.0 - min(velocity_magnitude / 0.1, 1.0))
        
        # 2c. Velocity penalty: discourage unnecessary movement
        velocity_penalty = -0.5 * velocity_magnitude if horizontal_dist < 0.05 else 0.0
        
        # 3. Height maintenance reward (critical when up/down is learnable)
        if self.enable_updown_control:
            # Detect horizontal drift: if actions are consistently in one direction
            drift_penalty = 0.0
            if len(self.action_history) >= self.action_history_size:
                # Check if horizontal actions have consistent bias
                # When up/down enabled, actions are [up/down, left/right, forward/back]
                # Extract horizontal components (indices 1 and 2)
                horiz_actions = []
                for a in self.action_history:
                    if len(a) >= 3:
                        horiz_actions.append(a[1:3])  # left/right, forward/back
                    else:
                        horiz_actions.append(a[:2])  # fallback for 2D actions
                
                if len(horiz_actions) > 0:
                    horiz_actions = np.array(horiz_actions)
                    action_bias = np.mean(horiz_actions, axis=0)
                    bias_magnitude = np.linalg.norm(action_bias)
                    
                    # If there's strong bias and we're not making progress, penalize
                    if bias_magnitude > 0.7 and horizontal_dist > 0.1:
                        drift_penalty = -2.0 * bias_magnitude
            
            # Target height above block
            target_dz = TARGET_HEIGHT
            height_error = abs(dz - target_dz)
            
            # Prioritize horizontal movement: only apply height reward when close
            if horizontal_dist > 0.15:  # Far from block: prioritize horizontal movement
                height_reward = -height_error * 1.0  # Weak height reward
            else:  # Close: maintain height
                height_reward = -height_error * 4.0  # Stronger height reward
            
            # Bonus for maintaining good height
            if MIN_ABOVE <= dz <= TARGET_HEIGHT + 0.02:
                height_reward += 1.5
            
            # Strong penalty for going below block
            if dz < 0:
                height_reward -= 8.0 * abs(dz)
            
            height_reward += drift_penalty
        else:
            height_reward = 0.0
            # Small penalty if somehow below block (shouldn't happen with fixed lift)
            if dz < 0:
                height_reward = -2.0
        
        # 4. Progress reward (potential-based shaping)
        progress_reward = 0.0
        if self.prev_horizontal_dist is not None:
            horizontal_progress = self.prev_horizontal_dist - horizontal_dist
            # Stronger reward for progress, especially when far
            if horizontal_dist > 0.1:
                progress_reward = horizontal_progress * 15.0  # Very strong when far
            else:
                progress_reward = horizontal_progress * 8.0  # Moderate when close
        
        # 5. Action-change penalty: penalize drastic changes in the commanded target
        # Since actuators are position targets (not forces), penalizing absolute magnitude is
        # less appropriate than penalizing sudden changes (which cause oscillation).
        # Compute delta relative to previous smoothed action (prev_action_before captured earlier)
        if prev_action_before is None:
            action_delta = np.zeros_like(action_clipped)
        else:
            action_delta = action_clipped - prev_action_before

        # Now update stored smoothed action for next step
        self.prev_action = action_clipped.copy()

        if len(action_delta) >=3:
            action_delta = action_delta[1:]  # Only horizontal components when up/down enabled
        # Stronger penalty when close to target to encourage settling
        action_penalty_weight = 100. * max(0.0, 1.0 - horizontal_dist / (SUCCESS_THRESHOLD * 10.))
        action_penalty = -action_penalty_weight * float(np.linalg.norm(action_delta))
        
        # 6. Precision bonus (heavily rewards being very close)
        precision_bonus = 0.0
        if horizontal_dist < SUCCESS_THRESHOLD * 2:  # Within 2x success threshold
            precision_bonus = 5.0 * (1.0 - horizontal_dist / (SUCCESS_THRESHOLD * 2))
        
        # 7. Success bonus
        success_bonus = 0.0
        terminated = (horizontal_dist <= SUCCESS_THRESHOLD) and (dz >= MIN_ABOVE)
        if terminated:
            success_bonus = 20.0
        
        # Compose final reward
        reward = (base_reward + 
                 horizontal_precision + 
                 settling_reward +
                 velocity_penalty +
                 height_reward + 
                 progress_reward + 
                 action_penalty + 
                 precision_bonus + 
                 success_bonus)
        
        # Update previous values
        self.prev_dist = dist
        self.prev_horizontal_dist = horizontal_dist
        self.prev_dz = dz
        
        truncated = self.step_count >= self.max_steps

        # Rich info for debugging
        info = {
            "distance": dist,
            "horizontal_dist": horizontal_dist,
            "vertical_dist": vertical_dist,
            "dz": dz,
            "base_reward": base_reward,
            "horizontal_precision": horizontal_precision,
            "settling_reward": settling_reward,
            "velocity_penalty": velocity_penalty,
            "height_reward": height_reward,
            "progress_reward": progress_reward,
            "precision_bonus": precision_bonus,
            "success_bonus": success_bonus,
            "action_penalty": action_penalty,
            "action_delta": action_delta.tolist(),
        }

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, info

    def reset_model(self):
        """Reset the robot degrees of freedom (qpos and qvel) and randomize block/target positions."""
        self.step_count = 0
        self.prev_action = None
        self.action_history = []
        
        # Note: _reset_simulation() is already called by base class reset()
        rand_spawn(self.model, self.data)  # randomize block/target
        mujoco.mj_forward(self.model, self.data)  # propagate physics

        block_pos = self.data.xpos[self.body][:3]
        gripper_pos = self.data.xpos[self.gripper][:3]
        rel = block_pos - gripper_pos
        
        # Distance metrics
        dist = np.linalg.norm(gripper_pos - block_pos)
        horizontal_dist = np.linalg.norm((gripper_pos - block_pos)[:2])
        vertical_dist = abs(rel[2])
        
        # Get velocities
        gripper_vel = self.data.qvel[:2] if len(self.data.qvel) >= 2 else np.array([0.0, 0.0])
        
        # Observation (same format as step)
        obs = np.concatenate([
            rel / 0.5,  # Normalized relative position
            np.array([gripper_pos[2]]),
            np.array([horizontal_dist, vertical_dist]),
            gripper_vel
        ])

        # Initialize previous values for reward shaping
        self.prev_dist = dist
        self.prev_horizontal_dist = horizontal_dist
        self.prev_dz = float(gripper_pos[2] - block_pos[2])

        return obs
    
    def _get_obs(self):
        """Get current observation (same format as step/reset)"""
        block_pos = self.data.xpos[self.body][:3]
        gripper_pos = self.data.xpos[self.gripper][:3]
        rel = block_pos - gripper_pos
        
        horizontal_dist = np.linalg.norm((gripper_pos - block_pos)[:2])
        vertical_dist = abs(rel[2])
        gripper_vel = self.data.qvel[:2] if len(self.data.qvel) >= 2 else np.array([0.0, 0.0])
        
        return np.concatenate([
            rel / 0.5,
            np.array([gripper_pos[2]]),
            np.array([horizontal_dist, vertical_dist]),
            gripper_vel
        ])

    def viewer_setup(self):
        assert self.viewer is not None
        self.viewer.cam.trackbodyid = 0
        self.viewer.cam.distance = self.model.stat.extent

