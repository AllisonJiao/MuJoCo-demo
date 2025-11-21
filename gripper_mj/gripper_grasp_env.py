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
SUCCESS_THRESHOLD = 0.2 * BLOCK_DIMENSION
MAX_STEPS = 200  # Shorter episodes since we start close
GRASP_HEIGHT_ABOVE = 0.1  # Start gripper this height above block

"""
Env B: Grasp-only environment
- Gripper starts already positioned above the block (like successful Env A outcome)
- Agent only needs to learn: descend, close fingers, lift slightly
- Reward is only about grasping, not XY movement
- Action space: [up/down, finger] or [up/down, small_xy_adjust, finger]
"""

class GripperGraspEnv(MuJocoPyEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human", 
            "rgb_array",
            "depth_array"
        ],
        "render_fps": 60,
    }

    def __init__(self, allow_xy_adjust=True, **kwargs):
        """
        Args:
            allow_xy_adjust: If True, allow small XY adjustments (3D action space)
                            If False, only up/down and finger (2D action space)
        """
        utils.EzPickle.__init__(self, allow_xy_adjust=allow_xy_adjust, **kwargs)
        
        # Observation: vertical distance, finger state, contact info, maybe XY error
        obs_dim = 6 if allow_xy_adjust else 4
        observation_space = Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float64)
        
        MuJocoPyEnv.__init__(
            self,
            model_path=os.path.join("..", "model", "GripperGPT.xml"),
            frame_skip=1,
            observation_space=observation_space,
            **kwargs
        )

        self.step_count = 0
        self.max_steps = MAX_STEPS
        self.allow_xy_adjust = allow_xy_adjust
        
        # actuator ids
        self.updown = self.model.actuator("up/down").id
        self.leftright = self.model.actuator("left/right").id
        self.forwardback = self.model.actuator("forward/backward").id
        self.finger = self.model.actuator("finger").id
        
        # body ids
        self.body = self.model.body("block").id
        self.target = self.model.body("target").id
        self.gripper = self.model.body("gripper").id
        self.left_finger = self.model.body("left_finger").id
        self.right_finger = self.model.body("right_finger").id

        # Action space: [up/down, finger] or [up/down, xy_adjust_x, xy_adjust_y, finger]
        if allow_xy_adjust:
            self.action_space = spaces.Box(low=-1, high=1, shape=(4,), dtype=np.float32)
            self.ctrl_scale = np.array([15.0, 2.0, 2.0, 3.0], dtype=float)  # Small XY adjustments
        else:
            self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
            self.ctrl_scale = np.array([15.0, 3.0], dtype=float)  # [up/down, finger]
        
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, 
            shape=(obs_dim,), dtype=np.float32
        )
        
        # Finger control range
        self.finger_open = 0.0
        self.finger_close = 3.0
        
        # Initial height above block (will be set in reset)
        self.initial_height = GRASP_HEIGHT_ABOVE

    def _check_grasped(self) -> bool:
        """Check if block is grasped by detecting contact between fingers and block."""
        # Check for contacts between block and fingers
        left_finger_contact = False
        right_finger_contact = False
        
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1 = contact.geom1
            geom2 = contact.geom2
            
            # Get body IDs for the geoms
            body1 = self.model.geom_bodyid[geom1]
            body2 = self.model.geom_bodyid[geom2]
            
            # Check if contact involves block and a finger
            if body1 == self.body:
                if body2 == self.left_finger:
                    left_finger_contact = True
                elif body2 == self.right_finger:
                    right_finger_contact = True
            elif body2 == self.body:
                if body1 == self.left_finger:
                    left_finger_contact = True
                elif body1 == self.right_finger:
                    right_finger_contact = True
        
        # Consider grasped if both fingers are in contact with block
        if left_finger_contact and right_finger_contact:
            return True
        
        # Fallback: check finger joint position and proximity
        left_slide_joint_id = self.model.joint("left_slide").id
        if left_slide_joint_id < len(self.data.qpos):
            finger_position = self.data.qpos[left_slide_joint_id]
            fingers_closed = finger_position > 0.05
            
            block_xy = self.data.xpos[self.body][:2]
            gripper_xy = self.data.xpos[self.gripper][:2]
            dist = np.linalg.norm(block_xy - gripper_xy)
            
            if fingers_closed and dist <= 0.5 * SUCCESS_THRESHOLD:
                return True
        
        return False

    def step(self, action):
        self.step_count += 1
        
        # Clip + scale action
        action = np.clip(action, -1, 1)
        action_scaled = action * self.ctrl_scale
        
        # Apply actions
        if self.allow_xy_adjust:
            # [up/down, xy_x, xy_y, finger]
            self.data.ctrl[self.updown] = action_scaled[0]
            # Small XY adjustments (reduced scale)
            self.data.ctrl[self.leftright] = action_scaled[1]
            self.data.ctrl[self.forwardback] = action_scaled[2]
            # Finger control
            finger_control = (action[3] + 1.0) / 2.0  # Map to [0, 1]
            finger_control = self.finger_open + finger_control * (self.finger_close - self.finger_open)
            self.data.ctrl[self.finger] = finger_control
        else:
            # [up/down, finger] - no XY movement
            self.data.ctrl[self.updown] = action_scaled[0]
            self.data.ctrl[self.leftright] = 0.0  # Keep centered
            self.data.ctrl[self.forwardback] = 0.0  # Keep centered
            # Finger control
            finger_control = (action[1] + 1.0) / 2.0  # Map to [0, 1]
            finger_control = self.finger_open + finger_control * (self.finger_close - self.finger_open)
            self.data.ctrl[self.finger] = finger_control

        # Advance physics
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)

        # Get positions
        block_xyz = self.data.xpos[self.body][:3]
        gripper_xyz = self.data.xpos[self.gripper][:3]
        block_xy = block_xyz[:2]
        gripper_xy = gripper_xyz[:2]
        
        # Vertical distance (positive = gripper above block)
        vertical_dist = gripper_xyz[2] - block_xyz[2]
        
        # Horizontal distance (should be small since we start aligned)
        horizontal_dist = np.linalg.norm(block_xy - gripper_xy)
        
        # Get finger state
        left_slide_joint_id = self.model.joint("left_slide").id
        finger_state = self.data.qpos[left_slide_joint_id] if left_slide_joint_id < len(self.data.qpos) else 0.0
        
        # Check if grasped
        grasped = self._check_grasped()
        
        # Build observation
        if self.allow_xy_adjust:
            obs = np.array([
                vertical_dist,           # Height above block
                horizontal_dist,         # XY alignment error
                finger_state,           # Finger opening/closing state
                float(grasped),         # Contact indicator
                gripper_xyz[2],         # Absolute gripper height
                block_xyz[2],           # Block height
            ], dtype=np.float32)
        else:
            obs = np.array([
                vertical_dist,           # Height above block
                finger_state,           # Finger opening/closing state
                float(grasped),         # Contact indicator
                gripper_xyz[2],         # Absolute gripper height
            ], dtype=np.float32)

        # Reward: only about grasping
        reward = 0.0
        
        # Small penalty for being too high (encourages descent)
        if vertical_dist > 0.15:
            reward -= 0.1
        
        # Reward for being at good grasping height (0.02 to 0.08 above block)
        if 0.02 <= vertical_dist <= 0.08:
            reward += 1.0
            
            # Reward closing fingers when at good height
            finger_closing = max(0, finger_state)
            reward += finger_closing * 2.0
        
        # Large reward for successful grasp
        if grasped:
            reward += 10.0
        
        # Small penalty for being too low (below block)
        if vertical_dist < 0:
            reward -= 0.5
        
        # Small penalty for horizontal misalignment (if allowed)
        if self.allow_xy_adjust and horizontal_dist > SUCCESS_THRESHOLD:
            reward -= 0.1 * horizontal_dist
        
        # Step penalty to encourage efficiency
        reward -= 0.01
        
        # Termination
        terminated = grasped
        truncated = self.step_count >= self.max_steps
        
        info = {
            "grasped": grasped,
            "vertical_dist": vertical_dist,
            "horizontal_dist": horizontal_dist,
            "finger_state": finger_state,
        }

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, info

    def reset_model(self):
        """Reset: position gripper above block (like successful Env A outcome)."""
        self.step_count = 0
        
        # Randomize block position (but keep it on ground)
        rand_spawn(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        
        # Get block position
        block_xyz = self.data.xpos[self.body][:3]
        block_xy = block_xyz[:2]
        
        # Position gripper directly above block at initial height
        gripper_joint_lr = self.model.joint("gripper_leftright").id
        gripper_joint_fb = self.model.joint("gripper_forwardbackward").id
        gripper_joint_ud = self.model.joint("gripper_updown").id
        
        lr_adr = self.model.jnt_qposadr[gripper_joint_lr]
        fb_adr = self.model.jnt_qposadr[gripper_joint_fb]
        ud_adr = self.model.jnt_qposadr[gripper_joint_ud]
        
        # Set gripper XY to match block XY (aligned above)
        self.data.qpos[lr_adr] = block_xy[0]  # x position
        self.data.qpos[fb_adr] = block_xy[1]  # y position
        
        # Set gripper height to be above block
        self.data.qpos[ud_adr] = -self.initial_height  # Negative because joint range is negative
        
        # Open fingers
        finger_joint = self.model.joint("left_slide").id
        finger_adr = self.model.jnt_qposadr[finger_joint]
        self.data.qpos[finger_adr] = self.finger_open
        
        # Forward kinematics
        mujoco.mj_forward(self.model, self.data)
        
        # Build observation (after forward kinematics)
        gripper_xyz = self.data.xpos[self.gripper][:3]
        gripper_xy = gripper_xyz[:2]
        vertical_dist = gripper_xyz[2] - block_xyz[2]
        horizontal_dist = np.linalg.norm(block_xy - gripper_xy)
        finger_state = self.data.qpos[finger_adr]
        grasped = False
        
        if self.allow_xy_adjust:
            obs = np.array([
                vertical_dist,
                horizontal_dist,
                finger_state,
                float(grasped),
                gripper_xyz[2],
                block_xyz[2],
            ], dtype=np.float32)
        else:
            obs = np.array([
                vertical_dist,
                finger_state,
                float(grasped),
                gripper_xyz[2],
            ], dtype=np.float32)
        
        return obs
    
    def _get_obs(self):
        """Get current observation."""
        block_xyz = self.data.xpos[self.body][:3]
        gripper_xyz = self.data.xpos[self.gripper][:3]
        block_xy = block_xyz[:2]
        gripper_xy = gripper_xyz[:2]
        
        vertical_dist = gripper_xyz[2] - block_xyz[2]
        horizontal_dist = np.linalg.norm(block_xy - gripper_xy)
        
        left_slide_joint_id = self.model.joint("left_slide").id
        finger_state = self.data.qpos[left_slide_joint_id] if left_slide_joint_id < len(self.data.qpos) else 0.0
        grasped = self._check_grasped()
        
        if self.allow_xy_adjust:
            return np.array([
                vertical_dist,
                horizontal_dist,
                finger_state,
                float(grasped),
                gripper_xyz[2],
                block_xyz[2],
            ], dtype=np.float32)
        else:
            return np.array([
                vertical_dist,
                finger_state,
                float(grasped),
                gripper_xyz[2],
            ], dtype=np.float32)

    def viewer_setup(self):
        assert self.viewer is not None
        self.viewer.cam.trackbodyid = 0
        self.viewer.cam.distance = self.model.stat.extent

