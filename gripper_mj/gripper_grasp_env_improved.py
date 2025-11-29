import os
import gymnasium as gym
from gymnasium import spaces
from gymnasium import utils
from gymnasium.spaces import Box
from mujoco_py_env import MuJocoPyEnv

import numpy as np
import mujoco

BLOCK_DIMENSION = 0.05
SUCCESS_THRESHOLD = 0.2 * BLOCK_DIMENSION
MAX_STEPS = 500
# Height above block - from stage 1 success condition (MIN_ABOVE = 0.2)
MIN_ABOVE = 0.2  # ~20 cm above block (from stage 1)
GRASP_HEIGHT_ABOVE = MIN_ABOVE  # Start gripper at this height above block

STUCK_THRESHOLD = 0.001  # Distance change threshold to consider as "stuck"
STUCK_PENALTY = 0.05  # Penalty for being stuck

# Geometry constants from GripperGPT.xml
FINGER_BASE_SEPARATION = 0.12  # 0.06m * 2 (distance between finger centers at rest)
FINGER_WIDTH = 0.04  # Finger half-extent in x direction (0.02m * 2)
FINGER_HEIGHT = 0.16  # Finger half-extent in z direction (0.08m * 2)

# Initialization parameters - block offset range (gripper stays fixed)
BLOCK_OFFSET_RANGE = SUCCESS_THRESHOLD * 2.0  # Horizontal offset range for block from gripper center

# Default gripper height (from XML model - gripper starts at z=0.3)
DEFAULT_GRIPPER_HEIGHT = 0.3  # Gripper default height above ground

"""
Improved GripperGraspEnv with velocity-based observations and better reward shaping.

NEW APPROACH: Fixed gripper position, randomized block below
- Gripper stays at default position (no joint position changes that cause velocity drift)
- Block is placed below gripper with horizontal offset noise
- Target is randomly placed on the ground plane
- Observation uses RELATIVE displacements (gripper-to-block)

Based on GripperGPT.xml model structure:
- Left finger: at x=-0.06m from gripper center, slides along x-axis
- Right finger: at x=0.06m from gripper center, slides along x-axis
- Finger size: 0.02 x 0.04 x 0.08m (half-extents)
- Block size: 0.05 x 0.05 x 0.05m (half-extents)
- Inner sides face each other (left finger's right side, right finger's left side)

Key improvements:
1. Velocity-based observations using mj_objectVelocity
2. Normalized observations for better learning
3. Progress rewards and stuck penalties
4. Precision bonuses that encourage exact convergence
5. Velocity-based rewards (low velocity when close to grasping)
6. Better reward shaping with separate components
7. Geometric inner-side contact detection based on finger positions
8. Fixed gripper position to avoid initial velocity from intvelocity actuators
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
            allow_xy_adjust: If True, allow small XY adjustments (4D action space)
                            If False, only up/down and finger (2D action space)
        """
        utils.EzPickle.__init__(self, allow_xy_adjust=allow_xy_adjust, **kwargs)
        
        # Observation: normalized relative position, distances, finger state, contact info, velocities
        # Note: Position 8 (allow_xy=True) / Position 5 (allow_xy=False) uses vertical_dist instead of
        # absolute gripper_z to use relative displacement. This creates redundancy with position 3/1 
        # but maintains the same observation shape for compatibility.
        #
        # With XY adjust: [rel_x, rel_y, rel_z, vertical_dist, horizontal_dist, finger_state, 
        #                  grasped, ground_contact, vertical_dist (replaces gripper_z), block_z, 
        #                  gripper_vel_x, gripper_vel_y, gripper_vel_z]
        # Without XY adjust: [rel_z, vertical_dist, finger_state, grasped, ground_contact, 
        #                     vertical_dist (replaces gripper_z), block_z, gripper_vel_z]
        if allow_xy_adjust:
            obs_dim = 13
        else:
            obs_dim = 8
        observation_space = Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float64)
        
        folder_path = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(folder_path, os.pardir, "model", "GripperGPT.xml")
        
        MuJocoPyEnv.__init__(
            self,
            model_path=model_path,
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
            self.ctrl_scale = np.array([1, 1, 1, 1], dtype=float)  # Small XY adjustments
        else:
            self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
            self.ctrl_scale = np.array([1, 1], dtype=float)  # [up/down, finger]
        
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, 
            shape=(obs_dim,), dtype=np.float32
        )
        
        # Finger control range
        # Start with fingers more open (like gripper raised higher in position stage)
        self.finger_open = -0.05  # More open (negative = more open based on joint range)
        self.finger_close = 3.0
        
        # Note: Gripper now stays at default position (no initial_height used)
        # Block is placed below gripper with slight horizontal offset
        
        # Reward shaping parameters
        self.prev_vertical_dist = None
        self.prev_horizontal_dist = None
        self.prev_gripper_height = None
        self.prev_block_height = None

    def _check_grasped(self) -> bool:
        """
        Check if block is successfully grasped.
        
        Success condition (ALL must be true):
        1. BOTH fingers must be in contact with the block
        2. The INNER sides of the fingers must be in contact (not outer sides)
        3. Gripper/block must be close to the ground (within threshold)
        
        Based on GripperGPT.xml structure:
        - Left finger: at x=-0.06m from gripper center
        - Right finger: at x=0.06m from gripper center
        - Inner sides face each other (left finger's right side, right finger's left side)
        
        This ensures a proper grasp with both fingers on the correct sides and that 
        the gripper has descended enough to hold the block.
        """
        # Threshold for being "close to ground" (in meters)
        GROUND_PROXIMITY_THRESHOLD = 0.08  # 8cm from ground
        
        # Check for contacts
        left_finger_inner_contact = False
        right_finger_inner_contact = False
        
        # Get finger positions for reference (from XML: left at -0.06m, right at +0.06m)
        left_finger_pos = self.data.xpos[self.left_finger]
        right_finger_pos = self.data.xpos[self.right_finger]
        
        # Calculate direction vector from left finger to right finger (inner direction)
        # This defines which side is "inner" for each finger
        finger_dir = right_finger_pos - left_finger_pos
        finger_dir_xy = finger_dir[:2]  # Use XY plane only (ignore z)
        finger_dir_norm = np.linalg.norm(finger_dir_xy)
        
        # Normalize direction vector (if fingers are separated)
        if finger_dir_norm > 1e-6:
            finger_dir_xy_normalized = finger_dir_xy / finger_dir_norm
        else:
            # Fallback: use default x-axis direction if fingers are too close
            finger_dir_xy_normalized = np.array([1.0, 0.0])
        
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1 = contact.geom1
            geom2 = contact.geom2
            
            # Get body IDs for the geoms
            body1 = self.model.geom_bodyid[geom1]
            body2 = self.model.geom_bodyid[geom2]
            
            # Check for finger-block contact on INNER sides
            # Inner side is defined as the side facing toward the other finger
            if body1 == self.body:
                if body2 == self.left_finger:
                    # Contact point in world frame
                    contact_pos = contact.pos
                    # Vector from left finger center to contact point (XY plane)
                    rel_pos_xy = (contact_pos - left_finger_pos)[:2]
                    # Project onto finger direction vector
                    # Positive dot product means contact is toward right finger (inner side)
                    dot_product = np.dot(rel_pos_xy, finger_dir_xy_normalized)
                    if dot_product > 0:  # Contact is toward right finger (inner side)
                        left_finger_inner_contact = True
                elif body2 == self.right_finger:
                    # Vector from right finger center to contact point (XY plane)
                    contact_pos = contact.pos
                    rel_pos_xy = (contact_pos - right_finger_pos)[:2]
                    # Project onto finger direction vector (reversed for right finger)
                    # Negative dot product means contact is toward left finger (inner side)
                    dot_product = np.dot(rel_pos_xy, finger_dir_xy_normalized)
                    if dot_product < 0:  # Contact is toward left finger (inner side)
                        right_finger_inner_contact = True
            elif body2 == self.body:
                if body1 == self.left_finger:
                    # Contact point in world frame
                    contact_pos = contact.pos
                    # Vector from left finger center to contact point (XY plane)
                    rel_pos_xy = (contact_pos - left_finger_pos)[:2]
                    # Project onto finger direction vector
                    dot_product = np.dot(rel_pos_xy, finger_dir_xy_normalized)
                    if dot_product > 0:  # Contact is toward right finger (inner side)
                        left_finger_inner_contact = True
                elif body1 == self.right_finger:
                    # Vector from right finger center to contact point (XY plane)
                    contact_pos = contact.pos
                    rel_pos_xy = (contact_pos - right_finger_pos)[:2]
                    # Project onto finger direction vector (reversed for right finger)
                    dot_product = np.dot(rel_pos_xy, finger_dir_xy_normalized)
                    if dot_product < 0:  # Contact is toward left finger (inner side)
                        right_finger_inner_contact = True
        
        # Check if block is close to ground
        block_pos = self.data.xpos[self.body]
        
        # Get the bottom of the block
        # In MuJoCo, box size is half-extent, so size=0.05 means box extends 0.05m from center
        # Block bottom is at block_pos[2] - 0.05
        block_bottom = block_pos[2] - 0.05
        
        # Check if block bottom is close to ground (ground is at z=0)
        close_to_ground = block_bottom <= GROUND_PROXIMITY_THRESHOLD
        
        # Success: BOTH fingers must contact block on INNER sides AND close to ground
        if left_finger_inner_contact and right_finger_inner_contact and close_to_ground:
            return True
        
        # No fallback - strict requirement for both fingers on inner sides
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
            finger_control = action_scaled[3]
            finger_control = self.finger_open + finger_control * (self.finger_close - self.finger_open)
            self.data.ctrl[self.finger] = finger_control
        else:
            # [up/down, finger] - no XY movement
            self.data.ctrl[self.updown] = action_scaled[0]
            self.data.ctrl[self.leftright] = 0.0  # Keep centered
            self.data.ctrl[self.forwardback] = 0.0  # Keep centered
            # Finger control
            finger_control = action_scaled[1]
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
        
        # Relative position vector (normalized)
        rel = block_xyz - gripper_xyz
        
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
        
        # Get gripper linear velocity (world frame) for observation
        try:
            gripper_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.gripper, gripper_vel_all, False)
            gripper_vel = np.array(gripper_vel_all[3:6], dtype=float)  # Linear velocity [vx, vy, vz]
        except Exception:
            gripper_vel = np.array([0.0, 0.0, 0.0], dtype=float)
        
        # Build observation with normalized values and velocities
        # Use RELATIVE displacement between gripper and block
        # Position 8 uses vertical_dist (relative height) instead of absolute gripper_z
        # This avoids the issue where gripper position varies but relative geometry is what matters
        if self.allow_xy_adjust:
            obs = np.concatenate([
                rel / 0.5,  # Normalized relative position [x, y, z] (block - gripper)
                np.array([vertical_dist]),  # Vertical distance (gripper above block)
                np.array([horizontal_dist]),  # Horizontal distance
                np.array([finger_state]),  # Finger state
                np.array([float(grasped)]),  # Grasped indicator
                np.array([float(ground_contact_obs)]),  # Ground contact indicator
                np.array([vertical_dist]),  # Relative height (replaces absolute gripper_z)
                np.array([block_xyz[2]]),  # Block height (for ground detection)
                gripper_vel  # Gripper velocity [vx, vy, vz]
            ], dtype=np.float32)
        else:
            obs = np.concatenate([
                np.array([rel[2] / 0.5]),  # Normalized relative z
                np.array([vertical_dist]),  # Vertical distance
                np.array([finger_state]),  # Finger state
                np.array([float(grasped)]),  # Grasped indicator
                np.array([float(ground_contact_obs)]),  # Ground contact indicator
                np.array([vertical_dist]),  # Relative height (replaces absolute gripper_z)
                np.array([block_xyz[2]]),  # Block height (for ground detection)
                np.array([gripper_vel[2]])  # Gripper vertical velocity
            ], dtype=np.float32)

        # Reward: improved reward shaping with progress, precision, and velocity components
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
        
        # Progress reward: reward for getting closer to ground (descending)
        progress_reward = 0.0
        stuck_penalty = 0.0
        if self.prev_gripper_height is not None:
            height_change = self.prev_gripper_height - gripper_height  # Positive = descending
            progress_reward = height_change * 2.0  # Reward descending
            
            # Stuck penalty: if gripper height hasn't changed much
            if abs(gripper_height - self.prev_gripper_height) < STUCK_THRESHOLD:
                if not ground_contact:  # Only penalize if not touching ground yet
                    stuck_penalty = -STUCK_PENALTY
        
        # Update previous values
        self.prev_gripper_height = gripper_height
        self.prev_block_height = block_height
        self.prev_vertical_dist = vertical_dist
        self.prev_horizontal_dist = horizontal_dist
        
        # Base reward: encourage getting closer to ground
        if block_height < 0.06:  # Block is near ground
            reward += 2.0 * (0.06 - block_height) / 0.06
        
        if gripper_height < 0.1:  # Gripper is low
            reward += 1.0 * (0.1 - gripper_height) / 0.1
        
        # Precision bonus: exponential reward as agent gets very close to grasping position
        precision_bonus = 0.0
        if 0.01 <= vertical_dist <= 0.05:  # Good grasping height range
            # Exponential bonus for being in the right height range
            precision_bonus = 3.0 * np.exp(-10.0 * abs(vertical_dist - 0.03))  # Peak at 0.03
            
            # Extra bonus for low velocity when at good height (stable approach)
            vel_magnitude = np.linalg.norm(gripper_vel)
            if vel_magnitude < 0.01:  # Very slow/stable
                precision_bonus += 2.0 * np.exp(-100.0 * vel_magnitude)
        
        # Reward for ground contact (critical for success)
        if ground_contact:
            reward += 3.0
        
        # Reward for being at good grasping height
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
        
        # Large reward for successful grasp
        if grasped:
            reward += 15.0
        
        # Penalties
        # Small penalty for being too high (encourages descent)
        if vertical_dist > 0.15:
            reward -= 0.1
        
        # Small penalty for being too low (below block) - but only if not touching ground
        if vertical_dist < 0 and not ground_contact:
            reward -= 0.5
        
        # Small penalty for horizontal misalignment (if allowed)
        if self.allow_xy_adjust and horizontal_dist > SUCCESS_THRESHOLD:
            reward -= 0.1 * horizontal_dist
        
        # Combine all reward components
        reward = reward + progress_reward + stuck_penalty + precision_bonus
        
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
            "progress_reward": progress_reward,
            "stuck_penalty": stuck_penalty,
            "precision_bonus": precision_bonus,
            "gripper_velocity": gripper_vel
        }

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, info

    def reset_model(self):
        """Reset the environment with FIXED gripper position, block below with horizontal offset.
        
        NEW APPROACH (following gripper_env_release.py):
        - Gripper stays at default position (no joint changes that cause velocity drift)
        - Block is placed below gripper with random horizontal offset
        - Target is randomly placed on the ground plane
        - All observations use relative displacements
        """
        self.step_count = 0
        
        # Reset previous values
        self.prev_vertical_dist = None
        self.prev_horizontal_dist = None
        self.prev_gripper_height = None
        self.prev_block_height = None

        # Reset simulation to initial state (this gives us default gripper position)
        mujoco.mj_resetData(self.model, self.data)
        
        # Get joint addresses
        block_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "block_free")
        block_adr = self.model.jnt_qposadr[block_joint]

        left_finger_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_slide")
        left_adr = self.model.jnt_qposadr[left_finger_joint]
        
        # Forward pass to get gripper position from default qpos
        mujoco.mj_forward(self.model, self.data)
        
        # Get default gripper position
        gripper_pos = self.data.xpos[self.gripper][:3].copy()
        
        # Randomize target position normally (on ground plane)
        target_id = self.model.geom("target").id
        self.model.geom_pos[target_id] = np.array([
            np.random.uniform(-0.5, 0.5),   # x
            np.random.uniform(-0.5, 0.5),   # y
            0.001                           # z (on ground)
        ])
        
        # Place block below gripper with random horizontal offset
        block_offset_x = np.random.uniform(-BLOCK_OFFSET_RANGE, BLOCK_OFFSET_RANGE)
        block_offset_y = np.random.uniform(-BLOCK_OFFSET_RANGE, BLOCK_OFFSET_RANGE)
        
        # Position block at ground level below gripper (with offset)
        self.data.qpos[block_adr:block_adr+3] = [
            gripper_pos[0] + block_offset_x,
            gripper_pos[1] + block_offset_y,
            BLOCK_DIMENSION  # z = block half-size (resting on ground)
        ]
        self.data.qpos[block_adr+3:block_adr+7] = [1, 0, 0, 0]  # Identity quaternion

        # Set fingers to open position
        self.data.qpos[left_adr] = self.finger_open

        # Set controls to neutral to avoid initial velocity
        self.data.ctrl[self.updown] = 0.0
        self.data.ctrl[self.leftright] = 0.0
        self.data.ctrl[self.forwardback] = 0.0
        self.data.ctrl[self.finger] = self.finger_open
        
        # Propagate physics
        mujoco.mj_forward(self.model, self.data)

        # Get updated positions
        block_xyz = self.data.xpos[self.body][:3]
        gripper_xyz = self.data.xpos[self.gripper][:3]
        block_xy = block_xyz[:2]
        gripper_xy = gripper_xyz[:2]
        
        vertical_dist = gripper_xyz[2] - block_xyz[2]
        horizontal_dist = np.linalg.norm(block_xy - gripper_xy)
        finger_state = self.data.qpos[left_adr]
        grasped = False
        
        # Check ground contact for observation (at reset, should be False)
        ground_contact_obs = False
        
        # Get gripper velocity at reset
        try:
            gripper_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.gripper, gripper_vel_all, False)
            gripper_vel = np.array(gripper_vel_all[3:6], dtype=float)
        except Exception:
            gripper_vel = np.array([0.0, 0.0, 0.0], dtype=float)
        
        # Relative position vector (block - gripper)
        rel = block_xyz - gripper_xyz
        
        # Initialize previous values
        self.prev_gripper_height = gripper_xyz[2]
        self.prev_block_height = block_xyz[2]
        self.prev_vertical_dist = vertical_dist
        self.prev_horizontal_dist = horizontal_dist
        
        # Use RELATIVE displacement between gripper and block
        # Position 8/5 uses vertical_dist (relative height) instead of absolute gripper_z
        if self.allow_xy_adjust:
            obs = np.concatenate([
                rel / 0.5,  # Normalized relative position (block - gripper)
                np.array([vertical_dist]),  # Vertical distance (gripper above block)
                np.array([horizontal_dist]),
                np.array([finger_state]),
                np.array([float(grasped)]),
                np.array([float(ground_contact_obs)]),
                np.array([vertical_dist]),  # Relative height (replaces absolute gripper_z)
                np.array([block_xyz[2]]),  # Block height (for ground detection)
                gripper_vel
            ], dtype=np.float32)
        else:
            obs = np.concatenate([
                np.array([rel[2] / 0.5]),  # Normalized relative z
                np.array([vertical_dist]),  # Vertical distance (gripper above block)
                np.array([finger_state]),
                np.array([float(grasped)]),
                np.array([float(ground_contact_obs)]),
                np.array([vertical_dist]),  # Relative height (replaces absolute gripper_z)
                np.array([block_xyz[2]]),  # Block height (for ground detection)
                np.array([gripper_vel[2]])  # Vertical velocity only
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
        
        # Get gripper velocity
        try:
            gripper_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.gripper, gripper_vel_all, False)
            gripper_vel = np.array(gripper_vel_all[3:6], dtype=float)
        except Exception:
            gripper_vel = np.array([0.0, 0.0, 0.0], dtype=float)
        
        # Relative position vector
        rel = block_xyz - gripper_xyz
        
        # Use RELATIVE displacement between gripper and block
        # Position 8/5 uses vertical_dist (relative height) instead of absolute gripper_z
        if self.allow_xy_adjust:
            return np.concatenate([
                rel / 0.5,  # Normalized relative position (block - gripper)
                np.array([vertical_dist]),  # Vertical distance (gripper above block)
                np.array([horizontal_dist]),
                np.array([finger_state]),
                np.array([float(grasped)]),
                np.array([float(ground_contact_obs)]),
                np.array([vertical_dist]),  # Relative height (replaces absolute gripper_z)
                np.array([block_xyz[2]]),  # Block height (for ground detection)
                gripper_vel
            ], dtype=np.float32)
        else:
            return np.concatenate([
                np.array([rel[2] / 0.5]),  # Normalized relative z
                np.array([vertical_dist]),  # Vertical distance (gripper above block)
                np.array([finger_state]),
                np.array([float(grasped)]),
                np.array([float(ground_contact_obs)]),
                np.array([vertical_dist]),  # Relative height (replaces absolute gripper_z)
                np.array([block_xyz[2]]),  # Block height (for ground detection)
                np.array([gripper_vel[2]])  # Vertical velocity only
            ], dtype=np.float32)

    def viewer_setup(self):
        assert self.viewer is not None
        self.viewer.cam.trackbodyid = 0
        self.viewer.cam.distance = self.model.stat.extent

