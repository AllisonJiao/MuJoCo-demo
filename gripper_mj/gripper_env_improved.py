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
SUCCESS_RELAXATION_FACTOR = 1.0  # Success threshold relaxation factor
MAX_STEPS = 500
# Height constraint (meters) — gripper should hang at least this far above block
MIN_ABOVE = 0.2  # ~20 cm above block
TARGET_HEIGHT = BLOCK_DIMENSION + MIN_ABOVE  # Target height above block (5 cm)

STUCK_THRESHOLD = 0.005  # Distance change threshold to consider as "stuck"
STUCK_PENALTY = -0.05  # Penalty for being stuck

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
            self.ctrl_scale = np.array([1, 1, 1], dtype=float)  # [up/down, left/right, forward/back]
        else:
            self.ctrl_scale = np.array([1, 1], dtype=float)  # [left/right, forward/back]
            self.fixed_updown_lift = 1.0  # Fixed upward lift when up/down is disabled
        
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
        
        # Get gripper linear velocity (world frame) for observation — use body xvel instead of joint qvel
        try:
            gripper_lin = self.data.xvel[self.gripper]
            gripper_vel = np.array(gripper_lin[:2], dtype=float)
        except Exception:
            gripper_vel = np.array([0.0, 0.0], dtype=float)
        
        obs = np.concatenate([
            rel / 0.5,  # Normalized relative position
            np.array([gripper_pos[2]]),  # Absolute z for height reference
            np.array([horizontal_dist, vertical_dist]),  # Distance components
            gripper_vel  # Velocities
        ])
        
        # Base reward: negative horizontal distance, scaled down to reduce variance
        reward = -horizontal_dist  # Scale down to keep rewards moderate

        # Progress reward & stuck penalty: compute from previous distance BEFORE updating state
        progress_reward = 0.0
        stuck_penalty = 0.0
        prev = self.prev_dist if hasattr(self, 'prev_dist') else None
        if prev is not None:
            distance_change = prev - dist
            progress_reward = distance_change * 1.0  # Reward for getting closer (scaled down)
            # If distance hasn't changed much since last step, apply small stuck penalty
            if abs(dist - prev) < STUCK_THRESHOLD:
                stuck_penalty = STUCK_PENALTY
        # update stored previous distance for next step
        self.prev_dist = dist

        # Precision bonus: exponential reward as agent gets very close (encourages exact convergence)
        precision_bonus = 0.0
        if horizontal_dist < 5 * SUCCESS_THRESHOLD:
            # Exponential bonus: e^(-20*d) peaks at d=0
            precision_bonus = 5.0 * np.exp(-20.0 * horizontal_dist)
        
        # Height penalty/reward
        if self.enable_updown_control:
            # Target height above block
            target_dz = TARGET_HEIGHT
            height_error = abs(dz - target_dz)
            
            # Prioritize horizontal movement: only apply height reward when close
            if horizontal_dist > BLOCK_DIMENSION * 3.0:  # Far from block: prioritize horizontal movement
                height_reward = -height_error * 1.0  # Weak height reward
            else:  # Close: maintain height
                height_reward = -height_error * 4.0  # Stronger height reward
            
            # Bonus for maintaining good height
            if MIN_ABOVE <= dz <= TARGET_HEIGHT + 0.02:
                height_reward += 1.5
            
            # Strong penalty for going below block (collision)
            if dz < 0:
                height_reward -= 15.0 * abs(dz)  # Increased collision penalty
            # Also penalize being too close to block even from above (risk of collision)
            if 0 <= dz < 0.01:  # Within 1 cm (collision risk)
                height_reward -= 5.0 * (0.01 - dz)  # Penalty scales with proximity
            
            #height_reward += drift_penalty
        else:
            height_reward = 0.0
            # Small penalty if somehow below block (shouldn't happen with fixed lift)
            if dz < 0:
                height_reward = -2.0
        
        reward = reward + progress_reward + stuck_penalty + height_reward + precision_bonus
        
        # Success only when horizontally close and above minimum height (if up/down is learnable)
        terminated = (horizontal_dist <= SUCCESS_THRESHOLD * SUCCESS_RELAXATION_FACTOR) and ((dz >= MIN_ABOVE) if self.enable_updown_control else True)
        if terminated:
            reward += 10.0  # Large success bonus
        truncated = self.step_count >= self.max_steps
        info = {
            "distance": dist,
            "horizontal_dist": horizontal_dist,
            "dz": dz,
            "progress": progress_reward,
            "stuck_penalty": stuck_penalty,
            "height_reward": height_reward,
            "precision_bonus": precision_bonus,
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
        
        # Get gripper linear velocity for observation at reset
        try:
            gripper_lin = self.data.xvel[self.gripper]
            gripper_vel = np.array(gripper_lin[:2], dtype=float)
        except Exception:
            gripper_vel = np.array([0.0, 0.0], dtype=float)
        
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
        try:
            gripper_lin = self.data.xvel[self.gripper]
            gripper_vel = np.array(gripper_lin[:2], dtype=float)
        except Exception:
            gripper_vel = np.array([0.0, 0.0], dtype=float)
        
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

