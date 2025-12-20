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
        
        # Observation: vertical distance, finger state, contact info, ground contact, maybe XY error
        obs_dim = 7 if allow_xy_adjust else 5  # Added ground contact
        observation_space = Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float64)
        
        MuJocoPyEnv.__init__(
            self,
            model_path=os.path.join("..", "../model", "GripperGPT.xml"),
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
        
        # Find ground/floor body (geom named "floor")
        try:
            floor_geom_id = self.model.geom("floor").id
            # Get the body ID for this geom
            self.floor_body = self.model.geom_bodyid[floor_geom_id]
        except:
            # Fallback: assume body 0 is the world/ground
            self.floor_body = 0

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
        # Start with fingers more open (like gripper raised higher in position stage)
        self.finger_open = -0.05  # More open (negative = more open based on joint range)
        self.finger_close = 3.0
        
        # Initial height above block (will be set in reset)
        self.initial_height = GRASP_HEIGHT_ABOVE

    def _check_grasped(self) -> bool:
        """
        Check if block is successfully grasped.
        
        Success condition (ALL must be true):
        1. BOTH fingers must be in contact with the block
        2. Gripper/block must be in contact with ground
        
        This ensures a proper grasp with both fingers and that the gripper has 
        descended enough to touch the ground.
        """
        # Check for contacts
        left_finger_contact = False
        right_finger_contact = False
        ground_contact = False
        
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1 = contact.geom1
            geom2 = contact.geom2
            
            # Get body IDs for the geoms
            body1 = self.model.geom_bodyid[geom1]
            body2 = self.model.geom_bodyid[geom2]
            
            # Check for finger-block contact (BOTH fingers required)
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
            
            # Check for ground contact (block or gripper/fingers touching ground)
            if body1 == self.floor_body:
                if body2 in [self.body, self.gripper, self.left_finger, self.right_finger]:
                    ground_contact = True
            elif body2 == self.floor_body:
                if body1 in [self.body, self.gripper, self.left_finger, self.right_finger]:
                    ground_contact = True
        
        # Success: BOTH fingers must contact block AND ground contact required
        if left_finger_contact and right_finger_contact and ground_contact:
            return True
        
        # No fallback - strict requirement for both fingers
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
        
        # Check ground contact for observation
        ground_contact_obs = False
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1 = contact.geom1
            geom2 = contact.geom2
            body1 = self.model.geom_bodyid[geom1]
            body2 = self.model.geom_bodyid[geom2]
            if body1 == self.floor_body:
                if body2 in [self.body, self.gripper, self.left_finger, self.right_finger]:
                    ground_contact_obs = True
            elif body2 == self.floor_body:
                if body1 in [self.body, self.gripper, self.left_finger, self.right_finger]:
                    ground_contact_obs = True
        
        # Build observation
        if self.allow_xy_adjust:
            obs = np.array([
                vertical_dist,           # Height above block
                horizontal_dist,         # XY alignment error
                finger_state,           # Finger opening/closing state
                float(grasped),         # Block contact indicator
                float(ground_contact_obs),  # Ground contact indicator
                gripper_xyz[2],         # Absolute gripper height
                block_xyz[2],           # Block height
            ], dtype=np.float32)
        else:
            obs = np.array([
                vertical_dist,           # Height above block
                finger_state,           # Finger opening/closing state
                float(grasped),         # Block contact indicator
                float(ground_contact_obs),  # Ground contact indicator
                gripper_xyz[2],         # Absolute gripper height
            ], dtype=np.float32)

        # Reward: only about grasping
        reward = 0.0
        
        # Get contact status for rewards
        ground_contact = False
        left_finger_block_contact = False
        right_finger_block_contact = False
        
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1 = contact.geom1
            geom2 = contact.geom2
            body1 = self.model.geom_bodyid[geom1]
            body2 = self.model.geom_bodyid[geom2]
            
            # Check for finger-block contact
            if body1 == self.body:
                if body2 == self.left_finger:
                    left_finger_block_contact = True
                elif body2 == self.right_finger:
                    right_finger_block_contact = True
            elif body2 == self.body:
                if body1 == self.left_finger:
                    left_finger_block_contact = True
                elif body1 == self.right_finger:
                    right_finger_block_contact = True
            
            # Check for ground contact
            if body1 == self.floor_body:
                if body2 in [self.body, self.gripper, self.left_finger, self.right_finger]:
                    ground_contact = True
            elif body2 == self.floor_body:
                if body1 in [self.body, self.gripper, self.left_finger, self.right_finger]:
                    ground_contact = True
        
        # Get absolute heights
        gripper_height = gripper_xyz[2]
        block_height = block_xyz[2]
        ground_height = 0.0
        
        # Small penalty for being too high (encourages descent)
        if vertical_dist > 0.15:
            reward -= 0.1
        
        # Reward for descending closer to ground (especially important for grasping)
        # Reward getting block closer to ground
        if block_height < 0.06:  # Block is near ground
            reward += 2.0 * (0.06 - block_height) / 0.06  # More reward the closer to ground
        
        # Reward for gripper being close to ground when grasping
        if gripper_height < 0.1:  # Gripper is low
            reward += 1.0 * (0.1 - gripper_height) / 0.1
        
        # Reward for ground contact (critical for success)
        if ground_contact:
            reward += 3.0  # Strong reward for touching ground
        
        # Reward for being at good grasping height (0.01 to 0.05 above block - lower range)
        # This encourages getting closer to the ground
        if 0.01 <= vertical_dist <= 0.05:
            reward += 1.5
            
            # Reward closing fingers when at good height
            finger_closing = max(0, finger_state)
            reward += finger_closing * 2.0
            
            # Reward for finger-block contact (encourage both fingers)
            if left_finger_block_contact:
                reward += 1.5
            if right_finger_block_contact:
                reward += 1.5
            # Bonus if BOTH fingers are contacting
            if left_finger_block_contact and right_finger_block_contact:
                reward += 3.0  # Extra bonus for both fingers
            
            # Extra reward if also touching ground
            if ground_contact:
                reward += 2.0
        
        # Large reward for successful grasp (requires both block and ground contact)
        if grasped:
            reward += 15.0  # Increased from 10.0
        
        # Small penalty for being too low (below block) - but only if not touching ground
        if vertical_dist < 0 and not ground_contact:
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
            "ground_contact": ground_contact,
            "left_finger_contact": left_finger_block_contact,
            "right_finger_contact": right_finger_block_contact,
            "both_fingers_contact": left_finger_block_contact and right_finger_block_contact,
            "block_height": block_xyz[2],
            "gripper_height": gripper_xyz[2],
        }

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, info

    def reset_model(self, initial_state=None):
        """Reset: position gripper above block (like successful Env A outcome).
        
        Args:
            initial_state: Optional dict containing state to copy instead of random init.
                - 'qpos': Joint positions to copy
                - 'qvel': Joint velocities to copy
                - 'ctrl': Control values to copy
                - 'act': Actuator states to copy (critical for intvelocity actuators)
                - 'target_pos': Target position to copy
        """
        self.step_count = 0
        
        if initial_state is not None:
            # Copy state from provided initial_state
            self.data.qpos[:] = initial_state['qpos']
            self.data.qvel[:] = initial_state['qvel']
            self.data.ctrl[:] = initial_state['ctrl']
            # Copy actuator states - critical for intvelocity actuators to prevent sudden movement
            if 'act' in initial_state:
                self.data.act[:] = initial_state['act']
            if 'target_pos' in initial_state:
                target_id = self.model.geom("target").id
                self.model.geom_pos[target_id] = initial_state['target_pos']
            mujoco.mj_forward(self.model, self.data)
        else:
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
        block_xyz = self.data.xpos[self.body][:3]
        block_xy = block_xyz[:2]
        gripper_xyz = self.data.xpos[self.gripper][:3]
        gripper_xy = gripper_xyz[:2]
        vertical_dist = gripper_xyz[2] - block_xyz[2]
        horizontal_dist = np.linalg.norm(block_xy - gripper_xy)
        finger_joint = self.model.joint("left_slide").id
        finger_adr = self.model.jnt_qposadr[finger_joint]
        finger_state = self.data.qpos[finger_adr]
        grasped = False
        
        # Check ground contact for observation (at reset, should be False)
        ground_contact_obs = False
        
        if self.allow_xy_adjust:
            obs = np.array([
                vertical_dist,
                horizontal_dist,
                finger_state,
                float(grasped),
                float(ground_contact_obs),
                gripper_xyz[2],
                block_xyz[2],
            ], dtype=np.float32)
        else:
            obs = np.array([
                vertical_dist,
                finger_state,
                float(grasped),
                float(ground_contact_obs),
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
        
        # Check ground contact for observation
        ground_contact_obs = False
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1 = contact.geom1
            geom2 = contact.geom2
            body1 = self.model.geom_bodyid[geom1]
            body2 = self.model.geom_bodyid[geom2]
            if body1 == self.floor_body:
                if body2 in [self.body, self.gripper, self.left_finger, self.right_finger]:
                    ground_contact_obs = True
            elif body2 == self.floor_body:
                if body1 in [self.body, self.gripper, self.left_finger, self.right_finger]:
                    ground_contact_obs = True
        
        if self.allow_xy_adjust:
            return np.array([
                vertical_dist,
                horizontal_dist,
                finger_state,
                float(grasped),
                float(ground_contact_obs),
                gripper_xyz[2],
                block_xyz[2],
            ], dtype=np.float32)
        else:
            return np.array([
                vertical_dist,
                finger_state,
                float(grasped),
                float(ground_contact_obs),
                gripper_xyz[2],
            ], dtype=np.float32)

    def viewer_setup(self):
        assert self.viewer is not None
        self.viewer.cam.trackbodyid = 0
        self.viewer.cam.distance = self.model.stat.extent

