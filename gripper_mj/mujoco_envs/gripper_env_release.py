import os
import gymnasium as gym
from gymnasium import spaces

from gymnasium import utils
from gymnasium.spaces import Box
from mujoco_py_env import MuJocoPyEnv

import numpy as np
import mujoco

BLOCK_DIMENSION = 0.05
SUCCESS_THRESHOLD = BLOCK_DIMENSION * 0.5  # Horizontal distance threshold for block landing on target
SUCCESS_RELAXATION_FACTOR = 2.0
MAX_STEPS = 500
# Height constraint (meters) — gripper should hover at this height above target
MIN_ABOVE_TARGET = 0.2  # ~20 cm above target
TARGET_HEIGHT_ABOVE_TARGET = BLOCK_DIMENSION + MIN_ABOVE_TARGET

# Position tolerance for allowing release (must be close to target to release)
POSITION_TOLERANCE = SUCCESS_THRESHOLD * 1.5  # Must be within this distance to release
HEIGHT_TOLERANCE = 0.05  # Must be within 5cm of target height to release
VELOCITY_TOLERANCE = SUCCESS_THRESHOLD * 2.0  # Must have low velocity to release

FINGER_WIDTH = 0.02  # Width of each finger (meters)
FINGER_GAP_CLOSED = 2 * FINGER_WIDTH  # Gap when fingers closed
FINGER_GAP_OPEN = BLOCK_DIMENSION + 2 * FINGER_WIDTH + 0.02  # Gap when fingers fully open
FINGER_CLOSE_COMMAND = 0.8  # Actuator command to close fingers
FINGER_CLOSE_POSITION = 0.015  # Joint position for closed fingers

# Initialization parameters - target offset range (gripper stays fixed)
TARGET_OFFSET_RANGE = SUCCESS_THRESHOLD * 2.0  # Horizontal offset range for target from gripper center

# Block landing detection
BLOCK_ON_GROUND_HEIGHT = BLOCK_DIMENSION + 0.01  # Block resting on ground (with small tolerance)

# Default gripper height (from XML model - gripper starts at z=0.3)
DEFAULT_GRIPPER_HEIGHT = TARGET_HEIGHT_ABOVE_TARGET  # Gripper height above ground

"""
Release Environment - Stage 4 of the gripper task.

NEW APPROACH: Fixed gripper position, randomized target below
- Gripper stays at default position (no joint position changes that cause velocity)
- Target is placed below gripper with horizontal offset noise
- Block is held by gripper at fixed position
- Observation uses RELATIVE displacements (gripper-to-target)

SIMPLIFIED reward structure:
1. Position reward: Move toward target horizontally (using relative displacement)
2. Height reward: Maintain proper height
3. Finger reward: Open when in position, keep closed otherwise  
4. Landing reward: Block landing close to target

Success: Block lands on ground within threshold of target.
"""


class GripperReleaseEnv(MuJocoPyEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array"
        ],
        "render_fps": 60,
    }

    def __init__(self, render_mode=None, width=480, height=480, **kwargs):
        utils.EzPickle.__init__(self, render_mode=render_mode, width=width, height=height, **kwargs)

        # Observation: [rel_gripper_to_target_dx, rel_gripper_to_target_dy, 
        #               gripper_height_above_target, horizontal_dist_gripper_to_target,
        #               block_height_above_ground, finger_distance, 
        #               gripper_vel_x, gripper_vel_y, block_vel_z, grasped]
        # All positions are RELATIVE to simplify learning
        observation_space = Box(low=-np.inf, high=np.inf, shape=(10,), dtype=np.float32)

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

        # action = 4 motors [up/down, left/right, forward/back, finger]
        self.action_space = spaces.Box(low=-1, high=1, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(10,), dtype=np.float32)

        # Control scaling
        self.ctrl_scale = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)

        # Track release state
        self.block_released = False
        self.block_landed = False
        self.initial_grasp_success = False
        self.prev_finger_distance = None
        self.prev_horizontal_dist = None  # For progress tracking
        self.release_reward_given = False  # Track if release reward has been given

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

    def _check_block_on_ground(self) -> bool:
        """Check if block has landed on the ground/target plane."""
        block_pos = self.data.xpos[self.body][:3]
        # Block is on ground when its z position is approximately at rest height
        return block_pos[2] <= BLOCK_ON_GROUND_HEIGHT

    def _get_target_pos(self):
        """Helper method to get target position."""
        return self.model.geom_pos[self.target_geom][:3]

    def step(self, action):
        self.step_count += 1

        # Clip and scale action
        action_clipped = np.clip(action, -1, 1)
        scaled_action = action_clipped * self.ctrl_scale

        # Apply actions to actuators (agent has full control)
        self.data.ctrl[self.updown] = scaled_action[0]
        self.data.ctrl[self.leftright] = scaled_action[1]
        self.data.ctrl[self.forwardback] = scaled_action[2]
        self.data.ctrl[self.finger] = scaled_action[3]
        
        # Advance physics
        for i in range(1, 10):
            mujoco.mj_step(self.model, self.data)

        block_pos = self.data.xpos[self.body][:3]
        gripper_pos = self.data.xpos[self.gripper][:3]
        target_pos = self._get_target_pos()
        left_finger_xy = self.data.xpos[self.left_finger][:2]
        right_finger_xy = self.data.xpos[self.right_finger][:2]

        finger_distance = np.linalg.norm(left_finger_xy - right_finger_xy)

        # Check grasp status
        grasped = self._check_grasped()

        # Track release
        if self.initial_grasp_success and not grasped and not self.block_released:
            self.block_released = True

        # Check if block has landed
        block_on_ground = self._check_block_on_ground()
        if self.block_released and block_on_ground:
            self.block_landed = True

        # RELATIVE distance metrics (gripper to target for positioning)
        rel_gripper_to_target = target_pos - gripper_pos  # Relative displacement vector
        horizontal_dist = np.linalg.norm((gripper_pos - target_pos)[:2])  # Horizontal distance gripper-to-target
        
        # Height above target (for maintaining proper hover height)
        gripper_height_above_target = float(gripper_pos[2] - target_pos[2])
        block_height_above_ground = float(block_pos[2])  # For tracking block landing

        # Get velocities
        try:
            gripper_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.gripper, gripper_vel_all, False)
            gripper_vel = np.array(gripper_vel_all[3:5], dtype=float)
            gripper_vel_z = float(gripper_vel_all[5])
        except Exception:
            gripper_vel = np.array([0.0, 0.0], dtype=float)
            gripper_vel_z = 0.0

        try:
            block_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.body, block_vel_all, False)
            block_vel_z = float(block_vel_all[5])
        except Exception:
            block_vel_z = 0.0

        gripper_speed = np.linalg.norm(gripper_vel)

        # Observation: ALL RELATIVE positions
        obs = np.concatenate([
            rel_gripper_to_target[:2] / 0.5,  # Normalized relative XY displacement (gripper to target)
            np.array([gripper_height_above_target]),  # Height of gripper above target
            np.array([horizontal_dist]),  # Horizontal distance to target
            np.array([block_height_above_ground]),  # Block absolute height (for landing detection)
            np.array([finger_distance]),  # Finger distance
            gripper_vel,  # Gripper XY velocity
            np.array([block_vel_z]),  # Block vertical velocity
            np.array([float(grasped)])  # Grasp status
        ])

        # === SIMPLIFIED REWARD SHAPING ===
        # 3 main components: position, height, finger control
        # Plus landing bonus
        
        reward = 0.0
        
        # Check if in release position (using gripper height since we're positioning gripper over target)
        position_ok = horizontal_dist <= POSITION_TOLERANCE
        height_error = abs(gripper_height_above_target - TARGET_HEIGHT_ABOVE_TARGET)
        height_ok = height_error < HEIGHT_TOLERANCE
        velocity_ok = gripper_speed < VELOCITY_TOLERANCE
        in_release_position = position_ok and height_ok and velocity_ok and grasped
        
        # 1. POSITION REWARD (like lift_improved)
        position_reward = -horizontal_dist * 2.0  # Base: closer is better
        
        # Progress bonus for getting closer
        if self.prev_horizontal_dist is not None and grasped:
            distance_change = self.prev_horizontal_dist - horizontal_dist
            position_reward += distance_change * 50.0
        self.prev_horizontal_dist = horizontal_dist
        
        # Precision bonus when very close
        if horizontal_dist < 3 * SUCCESS_THRESHOLD and grasped:
            position_reward += 4.0 * np.exp(-20.0 * horizontal_dist)
        
        # 2. HEIGHT REWARD (using gripper height above target)
        height_reward = 0.0
        if grasped:
            height_reward = -height_error * 5.0
            
            # Penalty for too low
            if gripper_height_above_target < MIN_ABOVE_TARGET:
                height_reward -= 100.0 * (MIN_ABOVE_TARGET - gripper_height_above_target)
        
        # 3. FINGER REWARD - simple: open when in position, closed otherwise
        finger_reward = 0.0
        finger_change = 0.0
        if self.prev_finger_distance is not None:
            finger_change = finger_distance - self.prev_finger_distance
        self.prev_finger_distance = finger_distance
        
        if grasped:
            if in_release_position:
                # In position -> CONTINUOUSLY reward open fingers to encourage release
                # This counteracts the closed bonus from earlier steps
                if finger_change > 0:
                    finger_reward = finger_change * 50.0  # Reward for opening
                # Add continuous bonus based on finger openness (the more open, the better)
                openness = max(0, finger_distance - FINGER_GAP_CLOSED) / (FINGER_GAP_OPEN - FINGER_GAP_CLOSED)
                finger_reward += openness * 5.0  # Continuous bonus for being open
            else:
                # Not in position -> punish opening, reward closing
                if finger_change > 0:
                    finger_reward = -finger_change * 100.0  # Strong penalty for opening
                else:
                    finger_reward = 0.5  # Small bonus for keeping closed
        
        # 4. LANDING REWARD (one-time) - based on where block lands relative to target
        landing_reward = 0.0
        block_to_target_dist = np.linalg.norm((block_pos - target_pos)[:2])
        if self.block_landed:
            # Distance-based reward for block landing close to target
            landing_reward = 50.0 * np.exp(-10.0 * block_to_target_dist)
            if block_to_target_dist <= SUCCESS_THRESHOLD:
                landing_reward += 100.0  # Big bonus for accurate landing
        
        # Total reward
        reward = position_reward + height_reward + finger_reward + landing_reward

        # Success criteria - block must land within threshold of target
        terminated = self.block_landed and block_to_target_dist <= SUCCESS_THRESHOLD * SUCCESS_RELAXATION_FACTOR

        if terminated:
            reward += 50.0  # Success bonus

        truncated = self.step_count >= self.max_steps or (self.block_landed and block_to_target_dist > SUCCESS_THRESHOLD * SUCCESS_RELAXATION_FACTOR)

        info = {
            "horizontal_dist": horizontal_dist,  # Gripper to target
            "block_to_target_dist": block_to_target_dist,  # Block to target (for landing)
            "block_z": block_pos[2],
            "gripper_height_above_target": gripper_height_above_target,
            "gripper_z": gripper_pos[2],
            "finger_distance": finger_distance,
            "grasped": grasped,
            "in_release_position": in_release_position,
            "block_released": self.block_released,
            "block_landed": self.block_landed,
            "position_reward": position_reward,
            "height_reward": height_reward,
            "finger_reward": finger_reward,
            "landing_reward": landing_reward,
            "velocity": gripper_vel,
            "gripper_speed": gripper_speed
        }

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, info

    def reset_model(self):
        """Reset the environment with FIXED gripper position, target below with horizontal offset.
        
        NEW APPROACH:
        - Gripper stays at default position (no joint changes that cause velocity drift)
        - Target is placed below gripper with random horizontal offset
        - Block is positioned at gripper location, held by closed fingers
        - All observations use relative displacements
        """
        self.step_count = 0
        self.block_released = False
        self.block_landed = False
        self.initial_grasp_success = False
        self.prev_finger_distance = None
        self.prev_horizontal_dist = None
        self.release_reward_given = False

        # Reset simulation to initial state (this gives us default gripper position)
        mujoco.mj_resetData(self.model, self.data)
        
        # Get joint addresses
        block_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "block_free")
        block_adr = self.model.jnt_qposadr[block_joint]

        left_finger_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_slide")
        right_finger_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "right_slide")
        left_adr = self.model.jnt_qposadr[left_finger_joint]
        right_adr = self.model.jnt_qposadr[right_finger_joint]

        # Forward pass to get gripper position from default qpos
        mujoco.mj_forward(self.model, self.data)
        
        # Get default gripper position
        gripper_pos = self.data.xpos[self.gripper][:3].copy()
        
        # Place target below gripper with random horizontal offset
        target_offset_x = np.random.uniform(-TARGET_OFFSET_RANGE, TARGET_OFFSET_RANGE)
        target_offset_y = np.random.uniform(-TARGET_OFFSET_RANGE, TARGET_OFFSET_RANGE)
        
        target_id = self.model.geom("target").id
        self.model.geom_pos[target_id] = np.array([
            gripper_pos[0] + target_offset_x,
            gripper_pos[1] + target_offset_y,
            0.001  # Target on ground
        ])
        
        # Set fingers to closed position
        self.data.qpos[left_adr] = FINGER_CLOSE_POSITION
        self.data.qpos[right_adr] = FINGER_CLOSE_POSITION

        # Position block at gripper location (will be held by closed fingers)
        self.data.qpos[block_adr:block_adr+3] = [
            gripper_pos[0],
            gripper_pos[1],
            gripper_pos[2]  # Same height as gripper
        ]
        self.data.qpos[block_adr+3:block_adr+7] = [1, 0, 0, 0]  # Identity quaternion

        # Set finger actuator to maintain closed position
        self.data.ctrl[self.finger] = FINGER_CLOSE_COMMAND
        self.data.ctrl[self.updown] = np.random.uniform(-SUCCESS_THRESHOLD, SUCCESS_THRESHOLD) * 2.0
        self.data.ctrl[self.leftright] = np.random.uniform(-SUCCESS_THRESHOLD, SUCCESS_THRESHOLD) * 2.0
        self.data.ctrl[self.forwardback] = np.random.uniform(-SUCCESS_THRESHOLD, SUCCESS_THRESHOLD) * 2.0

        # Propagate physics to get proper contact
        mujoco.mj_forward(self.model, self.data)

        # Let physics settle for grasp
        for _ in range(20):
            self.data.ctrl[self.finger] = FINGER_CLOSE_COMMAND
            #self.data.ctrl[self.updown] = 0.0
            #self.data.ctrl[self.leftright] = 0.0
            #self.data.ctrl[self.forwardback] = 0.0
            mujoco.mj_step(self.model, self.data)
        
        # Set finger actuator to maintain closed position
        self.data.ctrl[self.finger] = FINGER_CLOSE_COMMAND
        '''
        self.data.ctrl[self.updown] = np.random.uniform(-SUCCESS_THRESHOLD, SUCCESS_THRESHOLD) * 0.0
        self.data.ctrl[self.leftright] = np.random.uniform(-SUCCESS_THRESHOLD, SUCCESS_THRESHOLD) * 0.0
        self.data.ctrl[self.forwardback] = np.random.uniform(-SUCCESS_THRESHOLD, SUCCESS_THRESHOLD) * 0.0
        '''

        # Check if grasp was successful
        self.initial_grasp_success = self._check_grasped()

        # Get updated positions
        block_pos = self.data.xpos[self.body][:3]
        gripper_pos = self.data.xpos[self.gripper][:3]
        target_pos = self._get_target_pos()
        left_finger_xy = self.data.xpos[self.left_finger][:2]
        right_finger_xy = self.data.xpos[self.right_finger][:2]

        finger_distance = np.linalg.norm(left_finger_xy - right_finger_xy)
        self.prev_finger_distance = finger_distance

        # Calculate relative displacements
        rel_gripper_to_target = target_pos - gripper_pos
        horizontal_dist = np.linalg.norm((gripper_pos - target_pos)[:2])
        self.prev_horizontal_dist = horizontal_dist
        
        gripper_height_above_target = float(gripper_pos[2] - target_pos[2])
        block_height_above_ground = float(block_pos[2])

        # Get velocities
        try:
            gripper_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.gripper, gripper_vel_all, False)
            gripper_vel = np.array(gripper_vel_all[3:5], dtype=float)
        except Exception:
            gripper_vel = np.array([0.0, 0.0], dtype=float)

        try:
            block_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.body, block_vel_all, False)
            block_vel_z = float(block_vel_all[5])
        except Exception:
            block_vel_z = 0.0

        # Observation with RELATIVE displacements
        obs = np.concatenate([
            rel_gripper_to_target[:2] / 0.5,  # Normalized relative XY
            np.array([gripper_height_above_target]),  # Height above target
            np.array([horizontal_dist]),  # Horizontal distance
            np.array([block_height_above_ground]),  # Block absolute height
            np.array([finger_distance]),
            gripper_vel,
            np.array([block_vel_z]),
            np.array([float(self.initial_grasp_success)])
        ])

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

        # RELATIVE displacements
        rel_gripper_to_target = target_pos - gripper_pos
        horizontal_dist = np.linalg.norm((gripper_pos - target_pos)[:2])
        gripper_height_above_target = float(gripper_pos[2] - target_pos[2])
        block_height_above_ground = float(block_pos[2])

        try:
            gripper_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.gripper, gripper_vel_all, False)
            gripper_vel = np.array(gripper_vel_all[3:5], dtype=float)
        except Exception:
            gripper_vel = np.array([0.0, 0.0], dtype=float)

        try:
            block_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.body, block_vel_all, False)
            block_vel_z = float(block_vel_all[5])
        except Exception:
            block_vel_z = 0.0

        return np.concatenate([
            rel_gripper_to_target[:2] / 0.5,  # Normalized relative XY
            np.array([gripper_height_above_target]),  # Height above target
            np.array([horizontal_dist]),  # Horizontal distance
            np.array([block_height_above_ground]),  # Block absolute height
            np.array([finger_distance]),
            gripper_vel,
            np.array([block_vel_z]),
            np.array([float(grasped)])
        ])

    def viewer_setup(self):
        assert self.viewer is not None
        self.viewer.cam.trackbodyid = 0
        self.viewer.cam.distance = self.model.stat.extent
