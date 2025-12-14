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
SUCCESS_THRESHOLD = 0.5 * BLOCK_DIMENSION  # Slightly more lenient for deterministic policy
SUCCESS_RELAXATION_FACTOR = 1.0  # Success threshold relaxation factor
MAX_STEPS = 500
# Height constraint (meters) — gripper should hover at this height above target
MIN_ABOVE_TARGET = 0.2  # ~20 cm above target
TARGET_HEIGHT_ABOVE_TARGET = BLOCK_DIMENSION + MIN_ABOVE_TARGET  # Target height above target

STUCK_THRESHOLD = SUCCESS_THRESHOLD * 0.8  # Distance change threshold to consider as "stuck"
STUCK_PENALTY = 0.05  # Penalty for being stuck

FINGER_WIDTH = 0.02  # Width of each finger (meters)
FINGER_GAP_CLOSED = 2 * FINGER_WIDTH  # Total gap between fingers when block is grasped (fingers closed)

"""
Lift and Hover Environment - Stage 3 of the gripper task.

This environment assumes:
1. Gripper is already holding the block tight (fingers closed around block)
2. Block and target positions are randomized
3. Goal: Raise the block and hover over the target at appropriate height

Key features:
1. Initialization with gripper grasping the block
2. Reward for moving block toward target (horizontal alignment)
3. Reward for maintaining appropriate height above target
4. Reward for keeping fingers closed (maintaining grasp)
5. Penalty for dropping the block
6. Success when hovering over target at correct height with minimal velocity

Reward structure (balanced to prevent exploitation):
- Base reward: -2.0 * horizontal_distance (encourages moving toward target)
- Progress reward: 50.0 * (prev_horiz_dist - curr_horiz_dist) (very strong incentive for progress)
- Velocity penalty: -3.0 * (speed - 0.3) only when speed > 0.3 m/s (allows normal movement)
- Height reward: -3.0 * height_error (stronger when far, prevents ground gliding)
- Downward velocity penalty: -5.0 * vertical_speed (prevents rapid descent)
- Low height penalty: -15.0 * (MIN_HEIGHT - height) when below minimum
- Finger reward: normalized and scaled to prevent domination (~0.5 magnitude)
- Stuck penalty: increased to encourage movement
- Precision bonus: exponential bonus when very close to target
- Drop penalty: -50.0 for dropping the block
"""

class GripperLiftEnv(MuJocoPyEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human", 
            "rgb_array",
            "depth_array"
        ],
        "render_fps": 60,
    }

    def __init__(self, render_mode=None, width=480, height=480, ablation = False, **kwargs):
        utils.EzPickle.__init__(self, render_mode=render_mode, width=width, height=height, **kwargs)
        
        # Observation: [rel_dx, rel_dy, rel_dz, gripper_z, horizontal_dist, vertical_dist, 
        #               finger_distance, gripper_vel_x, gripper_vel_y, block_z, grasped]
        # Note: rel_* is relative to target (not block), vertical_dist is block height above target
        observation_space = Box(low=-np.inf, high=np.inf, shape=(11,), dtype=np.float64)

        folder_path = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(folder_path, os.pardir, "../model", "GripperGPT.xml")

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
        self.ablation = ablation
        
        # actuator ids
        self.updown = self.model.actuator("up/down").id
        self.leftright = self.model.actuator("left/right").id
        self.forwardback = self.model.actuator("forward/backward").id
        self.finger = self.model.actuator("finger").id
        self.actuators = np.array([self.updown, self.leftright, self.forwardback, self.finger], dtype=int)
        
        # body ids
        self.body = self.model.body("block").id
        self.target_geom = self.model.geom("target").id
        self.gripper = self.model.body("gripper").id
        self.left_finger = self.model.body("left_finger").id
        self.right_finger = self.model.body("right_finger").id
        
        # Find ground/floor body
        try:
            floor_geom_id = self.model.geom("floor").id
            self.floor_body = self.model.geom_bodyid[floor_geom_id]
        except:
            self.floor_body = 0

        # action = 4 motors [up/down, left/right, forward/back, finger]
        self.action_space = spaces.Box(low=-1, high=1, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(11,), dtype=np.float32)

        # Control scaling
        self.ctrl_scale = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)
        
        # Reward shaping parameters
        self.gamma = 0.99
        self.prev_dist = None
        self.prev_horizontal_dist = None
        self.prev_block_height = None
        
        # Track if block was grasped initially
        self.initial_grasp_success = False
        self.block_dropped = False

    def _check_grasped(self) -> bool:
        """
        Check if block is successfully grasped.
        Success: BOTH fingers must be in contact with the block.
        """
        left_finger_contact = False
        right_finger_contact = False
        
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1 = contact.geom1
            geom2 = contact.geom2
            
            body1 = self.model.geom_bodyid[geom1]
            body2 = self.model.geom_bodyid[geom2]
            
            # Check for finger-block contact
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
        
        return left_finger_contact and right_finger_contact

    def _get_target_pos(self):
        """Helper method to get target position."""
        return self.model.geom_pos[self.target_geom][:3]

    def step(self, action):
        self.step_count += 1
        
        # Clip and scale action
        action_clipped = np.clip(action, -1, 1)
        scaled_action = action_clipped * self.ctrl_scale
        
        # Apply actions to actuators
        self.data.ctrl[self.updown] = scaled_action[0]
        self.data.ctrl[self.leftright] = scaled_action[1]
        self.data.ctrl[self.forwardback] = scaled_action[2]
        self.data.ctrl[self.finger] = scaled_action[3]

        # advance physics
        for i in range(1, 10):
            mujoco.mj_step(self.model, self.data)

        block_pos = self.data.xpos[self.body][:3]
        gripper_pos = self.data.xpos[self.gripper][:3]
        target_pos = self._get_target_pos()
        left_finger_xy = self.data.xpos[self.left_finger][:2]
        right_finger_xy = self.data.xpos[self.right_finger][:2]

        finger_distance = np.linalg.norm(left_finger_xy - right_finger_xy)
        
        # Check if block is still grasped
        grasped = self._check_grasped()
        if not grasped and self.initial_grasp_success:
            self.block_dropped = True
        
        # Relative position from block to target (goal: align block over target)
        rel_to_target = target_pos - block_pos
        
        # Distance metrics (block to target)
        dist_block_to_target = np.linalg.norm(block_pos - target_pos)
        horizontal_dist = np.linalg.norm((block_pos - target_pos)[:2])
        
        # Height of block above target
        block_height_above_target = float(block_pos[2] - target_pos[2])
        
        # Get gripper linear velocity (including Z component for vertical velocity penalty)
        try:
            gripper_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.gripper, gripper_vel_all, False)
            gripper_vel = np.array(gripper_vel_all[3:5], dtype=float)  # XY for observation
            gripper_vel_z = float(gripper_vel_all[5])  # Z velocity for height penalty
        except Exception:
            gripper_vel = np.array([0.0, 0.0], dtype=float)
            gripper_vel_z = 0.0
        
        obs = np.concatenate([
            rel_to_target / 0.5,  # Normalized relative position (block to target)
            np.array([gripper_pos[2]]),  # Absolute gripper z
            np.array([horizontal_dist, block_height_above_target, finger_distance]),
            gripper_vel,  # Velocities
            np.array([block_pos[2]]),  # Absolute block z
            np.array([float(grasped)])  # Grasp status
        ])
        
        # === REWARD SHAPING ===
        
        # Base reward: negative horizontal distance to target (scaled down to prevent domination)
        # Scale by 2 to keep reward in reasonable range
        reward = -horizontal_dist * 2.0
        
        # Progress reward: getting closer to target horizontally (use horizontal distance for consistency)
        progress_reward = 0.0
        stuck_penalty = 0.0
        if self.prev_horizontal_dist is not None:
            # Use horizontal distance for progress to be consistent with base reward
            distance_change = self.prev_horizontal_dist - horizontal_dist
            progress_reward = distance_change * 50.0  # Increased from 20.0 to strongly encourage target approach
            
            # Stuck penalty if not making progress
            if abs(horizontal_dist - self.prev_horizontal_dist) < STUCK_THRESHOLD:
                if horizontal_dist > SUCCESS_THRESHOLD:
                    stuck_penalty = -5.0*STUCK_PENALTY
                else:
                    stuck_penalty = 2.0 * STUCK_PENALTY
        self.prev_horizontal_dist = horizontal_dist
        self.prev_dist = dist_block_to_target

        if self.ablation:
            progress_reward = 0.0
            stuck_penalty = 0.0
        
        # Precision bonus when very close to target
        precision_bonus = 0.0
        if horizontal_dist < 5 * SUCCESS_THRESHOLD:
            precision_bonus = 5.0 * np.exp(-20.0 * horizontal_dist)
            
            if horizontal_dist <= 2.0 * SUCCESS_THRESHOLD:
                # Extra bonus for staying stable when close
                precision_bonus += np.exp(-3.5 * np.linalg.norm(gripper_vel)) * 2.0

        # Height reward: maintain appropriate height above target
        height_reward = 0.0
        target_height = TARGET_HEIGHT_ABOVE_TARGET
        height_error = abs(block_height_above_target - target_height)
        
        if horizontal_dist > BLOCK_DIMENSION * 3.0:
            # Far from target: still maintain height (increased penalty)
            height_reward = -height_error * 3.0  # Increased from 1.0 to prevent ground gliding
        else:
            # Close to target: stronger height reward
            height_reward = -height_error * 5.0
            
            # Bonus for being at good height
            if abs(block_height_above_target - target_height) < 0.03:
                height_reward += 2.0
        
        # Penalty for being too low (prevents ground gliding)
        if block_height_above_target < MIN_ABOVE_TARGET:
            # Strong penalty for being below minimum height
            height_reward -= 15.0 * (MIN_ABOVE_TARGET - block_height_above_target)
        
        # Penalty for downward velocity (prevents rapid descent)
        if gripper_vel_z < -0.1:  # Moving down faster than 0.1 m/s
            height_reward -= 5.0 * abs(gripper_vel_z)  # Penalize fast descents
        
        # Finger reward: keep fingers closed to maintain grasp
        finger_reward = 0.0
        if grasped:
            # Small reward for maintaining closed fingers (don't let this dominate)
            # Normalize by expected finger distance when grasping
            normalized_finger_dist = finger_distance / (FINGER_GAP_CLOSED * 1.25)
            finger_reward = -normalized_finger_dist * 0.5  # Reduced scaling
            
            # Bonus for keeping fingers tight
            if finger_distance < FINGER_GAP_CLOSED * 1.25:
                finger_reward += 1.0  # Reduced from 2.0
        else:
            # Strong penalty for dropping the block
            finger_reward = -20.0
        
        # Velocity penalty: only penalize EXCESSIVE speed (prevents circling without hindering normal movement)
        velocity_penalty = 0.0
        gripper_speed = np.linalg.norm(gripper_vel)
        if horizontal_dist > SUCCESS_THRESHOLD:
            # Only penalize speeds above 0.15 m/s to allow normal movement toward target
            excessive_speed = max(0.0, gripper_speed - 0.15)
            velocity_penalty = -excessive_speed * 3.0  # Penalize only excessive speed
        else:
            #velocity_penalty = -gripper_speed * 6.0
            velocity_penalty = np.exp(-2.0 * np.linalg.norm(gripper_vel)) * 3.0#-gripper_speed * 3.0
        
        # Total reward
        reward = reward + progress_reward + stuck_penalty + height_reward + finger_reward + precision_bonus + velocity_penalty
        
        # Penalty for dropping block
        if self.block_dropped:
            reward -= 50.0
        
        # Success criteria
        horizontal_success = horizontal_dist <= SUCCESS_THRESHOLD * SUCCESS_RELAXATION_FACTOR
        height_success = abs(block_height_above_target - target_height) < 0.05  # Within 5cm of target height
        velocity_success = np.linalg.norm(gripper_vel) < SUCCESS_THRESHOLD * 2.0
        grasp_success = grasped
        
        terminated = horizontal_success and height_success and velocity_success and grasp_success
        
        if terminated:
            reward += 20.0  # Large success bonus
        
        truncated = self.step_count >= self.max_steps or self.block_dropped
        
        info = {
            "distance": dist_block_to_target,
            "horizontal_dist": horizontal_dist,
            "block_height": block_height_above_target,
            "progress": progress_reward,
            "stuck_penalty": stuck_penalty,
            "height_reward": height_reward,
            "finger_reward": finger_reward,
            "precision_bonus": precision_bonus,
            "velocity_penalty": velocity_penalty,
            "velocity": gripper_vel,
            "grasped": grasped,
            "block_dropped": self.block_dropped
        }

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, info

    def reset_model(self):
        """Reset the environment with gripper grasping the block."""
        self.step_count = 0
        self.block_dropped = False
        self.initial_grasp_success = False
        
        # Randomize block and target positions
        rand_spawn(self.model, self.data)
        
        # Get block and target positions
        block_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "block_free")
        block_adr = self.model.jnt_qposadr[block_joint]
        block_pos = self.data.qpos[block_adr:block_adr+3].copy()
        target_pos = self._get_target_pos().copy()
        
        # Position gripper above block, holding it
        # Set gripper to be at block position (holding it tight)
        gripper_height = np.random.uniform(0.085, 0.1)  # Start with gripper raised above ground
        
        # Set gripper position joints to align with block
        gripper_lr_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_leftright")
        gripper_fb_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_forwardbackward")
        gripper_ud_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_updown")
        
        lr_adr = self.model.jnt_qposadr[gripper_lr_joint]
        fb_adr = self.model.jnt_qposadr[gripper_fb_joint]
        ud_adr = self.model.jnt_qposadr[gripper_ud_joint]
        
        # Align gripper XY with block
        self.data.qpos[lr_adr] = block_pos[0]
        self.data.qpos[fb_adr] = block_pos[1]
        # Position gripper slightly above block
        self.data.qpos[ud_adr] = -(0.3 - gripper_height)  # Negative because joint range is -1 to 0
        
        # Close fingers to grasp block
        left_finger_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_slide")
        right_finger_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "right_slide")
        
        left_adr = self.model.jnt_qposadr[left_finger_joint]
        right_adr = self.model.jnt_qposadr[right_finger_joint]
        
        # Set fingers to closed position (grasping the block)
        # Joint range is -0.05 to 0.02, closed position should grip the block tightly
        # Block dimension is 0.05m, finger width is 0.02m each side
        # For proper grasp, fingers should be close but not fully compressed
        finger_close_pos = 0.015  # Positive value to close fingers
        self.data.qpos[left_adr] = finger_close_pos
        self.data.qpos[right_adr] = finger_close_pos
        
        # Update block position to be at gripper height, aligned with gripper XY
        self.data.qpos[block_adr:block_adr+3] = [block_pos[0], block_pos[1], gripper_height]
        
        # Set finger actuators to maintain closed position with strong force
        self.data.ctrl[self.finger] = 0.8  # Strong positive to keep fingers closed
        
        # Zero out velocities
        self.data.qvel[:] = 0.0
        
        # Propagate physics
        mujoco.mj_forward(self.model, self.data)
        
        # Let physics settle for a few steps to ensure grasp
        # Keep strong finger closure and hold gripper position
        for _ in range(20):
            self.data.ctrl[self.finger] = 0.8  # Keep fingers tight
            self.data.ctrl[self.updown] = 0.0  # Hold vertical position
            self.data.ctrl[self.leftright] = 0.0  # Hold horizontal position
            self.data.ctrl[self.forwardback] = 0.0
            mujoco.mj_step(self.model, self.data)
        
        # Check if grasp was successful
        self.initial_grasp_success = self._check_grasped()
        
        # Get updated positions after settling
        block_pos = self.data.xpos[self.body][:3]
        gripper_pos = self.data.xpos[self.gripper][:3]
        target_pos = self._get_target_pos()
        left_finger_xy = self.data.xpos[self.left_finger][:2]
        right_finger_xy = self.data.xpos[self.right_finger][:2]

        finger_distance = np.linalg.norm(left_finger_xy - right_finger_xy)
        
        rel_to_target = target_pos - block_pos
        dist_block_to_target = np.linalg.norm(block_pos - target_pos)
        horizontal_dist = np.linalg.norm((block_pos - target_pos)[:2])
        block_height_above_target = float(block_pos[2] - target_pos[2])
        
        # Get gripper linear velocity (consistent with step method)
        try:
            gripper_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.gripper, gripper_vel_all, False)
            gripper_vel = np.array(gripper_vel_all[3:5], dtype=float)
        except Exception:
            gripper_vel = np.array([0.0, 0.0], dtype=float)
        
        obs = np.concatenate([
            rel_to_target / 0.5,
            np.array([gripper_pos[2]]),
            np.array([horizontal_dist, block_height_above_target, finger_distance]),
            gripper_vel,
            np.array([block_pos[2]]),
            np.array([float(self.initial_grasp_success)])
        ])

        # Initialize previous values for reward shaping
        self.prev_dist = dist_block_to_target
        self.prev_horizontal_dist = horizontal_dist
        self.prev_block_height = block_height_above_target

        return obs
    
    def _get_obs(self):
        """Get current observation"""
        block_pos = self.data.xpos[self.body][:3]
        gripper_pos = self.data.xpos[self.gripper][:3]
        target_pos = self._get_target_pos()
        left_finger_xy = self.data.xpos[self.left_finger][:2]
        right_finger_xy = self.data.xpos[self.right_finger][:2]

        finger_distance = np.linalg.norm(left_finger_xy - right_finger_xy)
        grasped = self._check_grasped()

        rel_to_target = target_pos - block_pos
        horizontal_dist = np.linalg.norm((block_pos - target_pos)[:2])
        block_height_above_target = float(block_pos[2] - target_pos[2])
        
        # Get gripper linear velocity (consistent with step method)
        try:
            gripper_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.gripper, gripper_vel_all, False)
            gripper_vel = np.array(gripper_vel_all[3:5], dtype=float)
        except Exception:
            gripper_vel = np.array([0.0, 0.0], dtype=float)
        
        return np.concatenate([
            rel_to_target / 0.5,
            np.array([gripper_pos[2]]),
            np.array([horizontal_dist, block_height_above_target, finger_distance]),
            gripper_vel,
            np.array([block_pos[2]]),
            np.array([float(grasped)])
        ])

    def viewer_setup(self):
        assert self.viewer is not None
        self.viewer.cam.trackbodyid = 0
        self.viewer.cam.distance = self.model.stat.extent
