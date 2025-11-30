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
FINGER_GAP_CLOSED = 2 * FINGER_WIDTH  # Gap when fingers closed
FINGER_GAP_OPEN = 2 * BLOCK_DIMENSION + 2 * FINGER_WIDTH + 0.02  # Gap when fingers fully open

# Initialization parameters - block offset range (gripper stays fixed)
BLOCK_OFFSET_RANGE = SUCCESS_THRESHOLD * 2.0  # Horizontal offset range for block from gripper center

# Default gripper height (from XML model - gripper starts at z=0.3)
DEFAULT_GRIPPER_HEIGHT = 0.3  # Gripper default height above ground

MIN_HEIGHT = 0.08    # Minimum safe height (below this, fingers hit ground)
MAX_HEIGHT = 0.10    # Maximum height in ideal range
IDEAL_HEIGHT = 0.09  # Center of ideal gripper height for grasping

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
        #                  grasped, block_z, 
        #                  gripper_vel_x, gripper_vel_y, gripper_vel_z]
        # Without XY adjust: [rel_z, vertical_dist, finger_state, grasped, 
        #                     block_z, gripper_vel_z]
        if allow_xy_adjust:
            obs_dim = 11
        else:
            obs_dim = 6
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
            # XY adjustments should be MUCH smaller than vertical movement
            # This encourages the gripper to focus on descending first, then small corrections
            self.ctrl_scale = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)  # [up/down, xy_x (small), xy_y (small), finger]
        else:
            self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
            self.ctrl_scale = np.array([1.0, 1.0], dtype=float)  # [up/down, finger]
        
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, 
            shape=(obs_dim,), dtype=np.float32
        )
        
        
        # Note: Gripper now stays at default position (no initial_height used)
        # Block is placed below gripper with slight horizontal offset
        
        # Reward shaping parameters
        self.prev_vertical_dist = None
        self.prev_horizontal_dist = None
        self.prev_gripper_height = None
        
        # Track if gripper descended properly (fingers open during descent)
        self.proper_descent = True  # Becomes False if fingers close before grasp height

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
        
        # Check for contacts
        left_finger_inner_contact = False
        right_finger_inner_contact = False
        
        # Get finger positions for reference (from XML: left at -0.06m, right at +0.06m)
        left_finger_pos = self.data.xpos[self.left_finger]
        right_finger_pos = self.data.xpos[self.right_finger]
        gripper_pos = self.data.xpos[self.gripper][:3]

        finger_dist = np.linalg.norm((right_finger_pos - left_finger_pos)[:2])
        
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
        
        
        # Get the bottom of the block
        # In MuJoCo, box size is half-extent, so size=0.05 means box extends 0.05m from center
        
        # Check if block bottom is close to ground (ground is at z=0)
        close_to_ground = gripper_pos[2] <= MAX_HEIGHT and gripper_pos[2] >= MIN_HEIGHT
        
        # Success: BOTH fingers must contact block on INNER sides AND close to ground
        if left_finger_inner_contact and right_finger_inner_contact and \
            close_to_ground and (finger_dist > (BLOCK_DIMENSION * 2.0) * 0.9):
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
            # Small XY adjustments (scale already applied in ctrl_scale)
            self.data.ctrl[self.leftright] = action_scaled[1]
            self.data.ctrl[self.forwardback] = action_scaled[2]
            self.data.ctrl[self.finger] = action_scaled[3]
        else:
            # [up/down, finger] - no XY movement
            self.data.ctrl[self.updown] = action_scaled[0]
            self.data.ctrl[self.leftright] = 0.0  # Keep centered
            self.data.ctrl[self.forwardback] = 0.0  # Keep centered
            self.data.ctrl[self.finger] = action_scaled[1]

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
        vertical_dist = gripper_xyz[2] - IDEAL_HEIGHT
        
        # Horizontal distance (should be small since we start aligned)
        horizontal_dist = np.linalg.norm(block_xy - gripper_xy)
        
        # Get finger state - use left_slide qpos directly
        # qpos = -0.05 means OPEN (gap ~0.21), qpos = +0.02 means CLOSED (gap ~0.07)
        left_slide_joint_id = self.model.joint("left_slide").id
        left_slide_adr = self.model.jnt_qposadr[left_slide_joint_id]
        finger_qpos = self.data.qpos[left_slide_adr]
        
        # Calculate finger openness: 1.0 = open (qpos=-0.05), 0.0 = closed (qpos=0.02)
        FINGER_QPOS_OPEN = -0.05
        FINGER_QPOS_CLOSED = 0.02
        finger_openness = (FINGER_QPOS_CLOSED - finger_qpos) / (FINGER_QPOS_CLOSED - FINGER_QPOS_OPEN)
        finger_openness = np.clip(finger_openness, 0.0, 1.0)
        
        # Check if grasped
        grasped = self._check_grasped()
        
        # Get gripper linear velocity (world frame) for observation
        try:
            gripper_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.gripper, gripper_vel_all, False)
            gripper_vel = np.array(gripper_vel_all[3:6], dtype=float)  # Linear velocity [vx, vy, vz]
        except Exception:
            gripper_vel = np.array([0.0, 0.0, 0.0], dtype=float)
        
        # Build observation with normalized values and velocities
        # Use RELATIVE displacement between gripper and block
        if self.allow_xy_adjust:
            obs = np.concatenate([
                rel / 0.5,  # Normalized relative position [x, y, z] (block - gripper)
                np.array([vertical_dist]),  # Vertical distance (gripper above block)
                np.array([horizontal_dist]),  # Horizontal distance
                np.array([finger_openness]),  # Finger openness (1=open, 0=closed)
                np.array([float(grasped)]),  # Grasped indicator
                np.array([block_xyz[2]]),  # Block height (for ground detection)
                gripper_vel  # Gripper velocity [vx, vy, vz]
            ], dtype=np.float32)
        else:
            obs = np.concatenate([
                np.array([rel[2] / 0.5]),  # Normalized relative z
                np.array([vertical_dist]),  # Vertical distance
                np.array([finger_openness]),  # Finger openness (1=open, 0=closed)
                np.array([float(grasped)]),  # Grasped indicator
                np.array([block_xyz[2]]),  # Block height (for ground detection)
                np.array([gripper_vel[2]])  # Gripper vertical velocity
            ], dtype=np.float32)

        # ============================================================
        # SIMPLE REWARD FUNCTION
        # Focus on 3 tasks:
        # 1. Maintain horizontal precision (stay centered over block)
        # 2. Lower gripper to proper height
        # 3. Open fingers above ideal height, close at ideal height
        # ============================================================
        
        reward = 0.0
        
        # Get contact status for termination check
        ground_contact = False
        finger_ground_contact = False
        left_finger_block_contact = False
        right_finger_block_contact = False
        
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1 = contact.geom1
            geom2 = contact.geom2
            body1 = self.model.geom_bodyid[geom1]
            body2 = self.model.geom_bodyid[geom2]
            
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
            
            if body1 == self.floor_body:
                if body2 in [self.left_finger, self.right_finger]:
                    finger_ground_contact = True
                if body2 in [self.body, self.gripper, self.left_finger, self.right_finger]:
                    ground_contact = True
            elif body2 == self.floor_body:
                if body1 in [self.left_finger, self.right_finger]:
                    finger_ground_contact = True
                if body1 in [self.body, self.gripper, self.left_finger, self.right_finger]:
                    ground_contact = True
        
        gripper_height = gripper_xyz[2]
        both_fingers_contact = left_finger_block_contact and right_finger_block_contact
        
        # Height thresholds
        
        # Check if at ideal grasping height
        at_ideal_height = gripper_height <= MAX_HEIGHT and gripper_height >= MIN_HEIGHT
        
        # finger_openness is already calculated above (1.0 = open, 0.0 = closed)
        
        # Update previous values for next step
        self.prev_gripper_height = gripper_height
        self.prev_vertical_dist = vertical_dist
        self.prev_horizontal_dist = horizontal_dist
        
        # ============================================================
        # REWARD 1: Horizontal precision (scale: 0 to +1)
        # Reward being centered over block
        # ============================================================
        horizontal_reward = 2.0 *np.exp(-50.0 * horizontal_dist)  # 1.0 when centered, ~0.37 at 0.1m offset
        reward += horizontal_reward

        # Small velocity penalty to discourage swinging
        velocity_penalty = -1 * (np.exp(4.0*np.linalg.norm(gripper_vel[:2]))-1.0)
        reward += velocity_penalty
        
        # ============================================================
        # REWARD 2: Height reward (scale: -2 to +2)
        # Uses a "sweet spot" design centered at IDEAL_HEIGHT
        # Maximum reward at ideal height, decreasing linearly as you move away
        # ============================================================
        if gripper_height >= MIN_HEIGHT and gripper_height <= MAX_HEIGHT:
            # In the ideal range: reward = +2.0
            height_reward = 2.0
        elif gripper_height > MAX_HEIGHT:
            # Above ideal: penalty proportional to how far above
            # Starts at +1.0 at MAX_HEIGHT, decreases to -1.0 far above
            height_reward = 1.0 - (gripper_height - MAX_HEIGHT) * 10.0
            height_reward = max(height_reward, -1.0)  # Cap at -1
        else:
            # Below minimum: STRONG penalty to discourage going too low
            # Starts at 0.0 at MIN_HEIGHT, goes to -2.0 quickly
            height_reward = -(MIN_HEIGHT - gripper_height) * 100.0
            height_reward = max(height_reward, -2.0)  # Cap at -2
        
        reward += height_reward
        
        # ============================================================
        # REWARD 3: Finger behavior (scale: -10 to +2)
        # CRITICAL: Open fingers during descent, only close at proper position
        # This prevents "squeezing" behavior where gripper closes early
        # ============================================================
        # Fingers should be open (openness > 0.8) during descent
        fingers_open = finger_openness > 0.8
        fingers_closed = finger_openness < 0.2
        
        if at_ideal_height and horizontal_dist < 2.0 * SUCCESS_THRESHOLD:
            # At ideal height AND centered: reward CLOSING fingers
            if fingers_closed:
                finger_reward = 2.0  # Good: closed at the right time
            elif fingers_open:
                finger_reward = -1.0  # Still open, should be closing
            else:
                finger_reward = 0.0  # Transitioning
        else:
            # NOT at ideal position: MUST keep fingers OPEN
            if fingers_open:
                finger_reward = 1.0  # Good: fingers open during descent
            else:
                # STRONG penalty for closing fingers early
                # This is the key fix - prevent "squeezing" behavior
                finger_reward = -10.0 * (1.0 - finger_openness)  # -10 when fully closed
        reward += finger_reward
        
        # ============================================================
        # REWARD 4: Success bonus (+10)
        # ============================================================
        if grasped:
            reward += 10.0
        
        # Small step penalty
        reward -= 0.01
        
        # Termination
        # Truncate if gripper goes too low (fingers would hit ground)
        CRASH_HEIGHT = 0.06  # Below this, fingers definitely hit ground
        terminated = grasped
        truncated = self.step_count >= self.max_steps or gripper_height < CRASH_HEIGHT
        
        info = {
            "grasped": grasped,
            "vertical_dist": vertical_dist,
            "horizontal_dist": horizontal_dist,
            "finger_qpos": finger_qpos,
            "finger_openness": finger_openness,
            "at_ideal_height": at_ideal_height,
            "ground_contact": ground_contact,
            "finger_ground_contact": finger_ground_contact,
            "left_finger_contact": left_finger_block_contact,
            "right_finger_contact": right_finger_block_contact,
            "both_fingers_contact": both_fingers_contact,
            "gripper_height": gripper_height,
            "horizontal_reward": horizontal_reward,
            "height_reward": height_reward,
            "finger_reward": finger_reward,
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
        
        # Reset proper descent flag
        self.proper_descent = True

        # Reset simulation to initial state (this gives us default gripper position)
        mujoco.mj_resetData(self.model, self.data)
        
        # Get joint addresses
        block_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "block_free")
        block_adr = self.model.jnt_qposadr[block_joint]

        left_finger_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_slide")
        left_adr = self.model.jnt_qposadr[left_finger_joint]
        right_finger_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "right_slide")
        right_adr = self.model.jnt_qposadr[right_finger_joint]
        
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
        self.data.qpos[left_adr] = np.random.uniform(-0.06, -0.05)
        self.data.qpos[right_adr] = self.data.qpos[left_adr]

        # Set controls to neutral to avoid initial velocity
        self.data.ctrl[self.updown] = 0.0
        self.data.ctrl[self.leftright] = 0.0
        self.data.ctrl[self.forwardback] = 0.0
        self.data.ctrl[self.finger] = 0.0
        
        # Propagate physics
        mujoco.mj_forward(self.model, self.data)

        # Get updated positions
        block_xyz = self.data.xpos[self.body][:3]
        gripper_xyz = self.data.xpos[self.gripper][:3]
        block_xy = block_xyz[:2]
        gripper_xy = gripper_xyz[:2]
        
        vertical_dist = gripper_xyz[2] - block_xyz[2]
        horizontal_dist = np.linalg.norm(block_xy - gripper_xy)
        
        # Get finger openness
        left_slide_joint_id = self.model.joint("left_slide").id
        left_slide_adr = self.model.jnt_qposadr[left_slide_joint_id]
        finger_qpos = self.data.qpos[left_slide_adr]
        FINGER_QPOS_OPEN = -0.05
        FINGER_QPOS_CLOSED = 0.02
        finger_openness = (FINGER_QPOS_CLOSED - finger_qpos) / (FINGER_QPOS_CLOSED - FINGER_QPOS_OPEN)
        finger_openness = np.clip(finger_openness, 0.0, 1.0)
        
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
        self.prev_vertical_dist = vertical_dist
        self.prev_horizontal_dist = horizontal_dist
        
        # Use RELATIVE displacement between gripper and block
        if self.allow_xy_adjust:
            obs = np.concatenate([
                rel / 0.5,  # Normalized relative position (block - gripper)
                np.array([vertical_dist]),  # Vertical distance (gripper above block)
                np.array([horizontal_dist]),
                np.array([finger_openness]),  # Finger openness (1=open, 0=closed)
                np.array([float(grasped)]),
                np.array([block_xyz[2]]),  # Block height (for ground detection)
                gripper_vel
            ], dtype=np.float32)
        else:
            obs = np.concatenate([
                np.array([rel[2] / 0.5]),  # Normalized relative z
                np.array([vertical_dist]),  # Vertical distance (gripper above block)
                np.array([finger_openness]),  # Finger openness (1=open, 0=closed)
                np.array([float(grasped)]),
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
        
        # Get finger openness
        left_slide_joint_id = self.model.joint("left_slide").id
        left_slide_adr = self.model.jnt_qposadr[left_slide_joint_id]
        finger_qpos = self.data.qpos[left_slide_adr]
        FINGER_QPOS_OPEN = -0.05
        FINGER_QPOS_CLOSED = 0.02
        finger_openness = (FINGER_QPOS_CLOSED - finger_qpos) / (FINGER_QPOS_CLOSED - FINGER_QPOS_OPEN)
        finger_openness = np.clip(finger_openness, 0.0, 1.0)
        
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
        if self.allow_xy_adjust:
            return np.concatenate([
                rel / 0.5,  # Normalized relative position (block - gripper)
                np.array([vertical_dist]),  # Vertical distance (gripper above block)
                np.array([horizontal_dist]),
                np.array([finger_openness]),  # Finger openness (1=open, 0=closed)
                np.array([float(grasped)]),
                np.array([block_xyz[2]]),  # Block height (for ground detection)
                gripper_vel
            ], dtype=np.float32)
        else:
            return np.concatenate([
                np.array([rel[2] / 0.5]),  # Normalized relative z
                np.array([vertical_dist]),  # Vertical distance (gripper above block)
                np.array([finger_openness]),  # Finger openness (1=open, 0=closed)
                np.array([float(grasped)]),
                np.array([block_xyz[2]]),  # Block height (for ground detection)
                np.array([gripper_vel[2]])  # Vertical velocity only
            ], dtype=np.float32)

    def viewer_setup(self):
        assert self.viewer is not None
        self.viewer.cam.trackbodyid = 0
        self.viewer.cam.distance = self.model.stat.extent

