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

STUCK_THRESHOLD = SUCCESS_THRESHOLD * 0.8
STUCK_PENALTY = 0.05

FINGER_WIDTH = 0.02  # Width of each finger (meters)
FINGER_GAP_CLOSED = 2 * FINGER_WIDTH  # Gap when fingers closed
FINGER_GAP_OPEN = BLOCK_DIMENSION + 2 * FINGER_WIDTH + 0.02  # Gap when fingers fully open
FINGER_GAP_TOLERANCE_MULTIPLIER = 1.25  # Multiplier for finger gap tolerance
FINGER_CLOSE_COMMAND = 0.8  # Actuator command to close fingers
FINGER_CLOSE_POSITION = 0.015  # Joint position for closed fingers

# Velocity control parameters
SPEED_THRESHOLD = 0.15  # Speed (m/s) above which velocity penalty applies
VELOCITY_REWARD_DECAY = 2.0  # Decay constant for velocity reward exponential

# Initialization parameters
INITIAL_POSITION_NOISE_RANGE = SUCCESS_THRESHOLD  # Range for initial position noise
INITIAL_HEIGHT_NOISE = 0.01  # Range for initial height noise

# Block landing detection
BLOCK_ON_GROUND_HEIGHT = BLOCK_DIMENSION + 0.01  # Block resting on ground (with small tolerance)

"""
Release Environment - Stage 4 of the gripper task.

This environment implements a two-phase approach with HARD CONSTRAINTS:

PHASE 1 - POSITION ADJUSTMENT:
- Gripper must first move to ideal position above target
- Fingers are FORCED to stay CLOSED (finger action is overridden)
- Reward for moving toward target horizontally
- Reward for maintaining proper height

PHASE 2 - RELEASE:
- Only allowed when gripper is at ideal position (within tolerance)
- Finger action is now allowed
- Reward for block landing close to target

The key difference is that finger opening is physically prevented until in position,
not just penalized. This ensures the agent learns to position first.

Initialization:
- Gripper starts grasping block with small position noise
- Velocity is set to zero to allow clean learning (Stage 3 should stabilize before transition)

Success condition:
- Block landed on ground within horizontal threshold of target
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

        # Observation: [rel_block_to_target_dx, rel_block_to_target_dy, rel_block_to_target_dz,
        #               gripper_z, horizontal_dist_block_to_target, block_z,
        #               finger_distance, gripper_vel_x, gripper_vel_y, block_vel_z, grasped]
        observation_space = Box(low=-np.inf, high=np.inf, shape=(11,), dtype=np.float32)

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
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(11,), dtype=np.float32)

        # Control scaling
        self.ctrl_scale = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)

        # Track release state
        self.block_released = False
        self.block_landed = False
        self.initial_grasp_success = False
        self.prev_finger_distance = None
        self.prev_horizontal_dist = None  # For progress tracking
        self.release_reward_given = False  # Track if release reward has been given
        self.in_release_position = False  # Track if gripper has reached ideal position

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

        # Get current state for position check
        block_pos_before = self.data.xpos[self.body][:3]
        target_pos = self._get_target_pos()
        gripper_pos_before = self.data.xpos[self.gripper][:3]
        
        horizontal_dist_before = np.linalg.norm((block_pos_before - target_pos)[:2])
        gripper_height_above_target = float(gripper_pos_before[2] - target_pos[2])
        
        # Get velocity for position check
        try:
            gripper_vel_all = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.gripper, gripper_vel_all, False)
            gripper_speed_before = np.linalg.norm(gripper_vel_all[3:5])
        except Exception:
            gripper_speed_before = 0.0
        
        grasped_before = self._check_grasped()
        
        # Check if gripper is in ideal release position BEFORE applying action
        position_ok = horizontal_dist_before <= POSITION_TOLERANCE
        height_ok = abs(gripper_height_above_target - TARGET_HEIGHT_ABOVE_TARGET) < HEIGHT_TOLERANCE
        velocity_ok = gripper_speed_before < (2.0 * VELOCITY_TOLERANCE)
        
        self.in_release_position = position_ok and height_ok and velocity_ok and grasped_before

        # Apply actions to actuators
        self.data.ctrl[self.updown] = scaled_action[0]
        self.data.ctrl[self.leftright] = scaled_action[1]
        self.data.ctrl[self.forwardback] = scaled_action[2]
        self.data.ctrl[self.finger] = scaled_action[3]
        '''
        # CRITICAL: Override finger action if NOT in release position
        # This is a HARD CONSTRAINT - fingers stay closed until in position
        if self.in_release_position:
            # Allow finger control when in position
            self.data.ctrl[self.finger] = scaled_action[3]
        else:
            # Force fingers to stay closed - override any open command
            self.data.ctrl[self.finger] = FINGER_CLOSE_COMMAND
        '''
        # advance physics
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

        # Distance metrics (block to target)
        rel_to_target = target_pos - block_pos
        horizontal_dist = np.linalg.norm((block_pos - target_pos)[:2])
        
        # Height of block above target
        block_height_above_target = float(block_pos[2] - target_pos[2])

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

        obs = np.concatenate([
            rel_to_target / 0.5,  # Normalized relative position (block to target)
            np.array([gripper_pos[2]]),  # Absolute gripper z
            np.array([horizontal_dist]),  # Horizontal distance to target
            np.array([block_pos[2]]),  # Block z
            np.array([finger_distance]),  # Finger distance
            gripper_vel,  # Gripper XY velocity
            np.array([block_vel_z]),  # Block vertical velocity
            np.array([float(grasped)])  # Grasp status
        ])

        # === SIMPLIFIED REWARD SHAPING ===
        reward = 0.0
        
        # 1. Base reward: negative horizontal distance (encourages moving toward target)
        base_reward = -horizontal_dist * 2.0
        
        # 2. Progress reward: getting closer to target
        progress_reward = 0.0
        if self.prev_horizontal_dist is not None and grasped:
            distance_change = self.prev_horizontal_dist - horizontal_dist
            progress_reward = distance_change * 30.0  # Reward for getting closer
        self.prev_horizontal_dist = horizontal_dist
        
        # 3. Height maintenance reward (similar to lift_improved)
        height_reward = 0.0
        target_height = TARGET_HEIGHT_ABOVE_TARGET
        height_error = abs(block_height_above_target - target_height)
        
        # Penalty for incorrect height
        height_reward = -height_error * 3.0
        
        # Bonus for good height
        if height_error < 0.03:
            height_reward += 1.0
        
        # Strong penalty for being too low
        if block_height_above_target < MIN_ABOVE_TARGET:
            height_reward -= 10.0 * (MIN_ABOVE_TARGET - block_height_above_target)
        
        # Penalty for fast downward velocity
        if gripper_vel_z < -0.1:
            height_reward -= 3.0 * abs(gripper_vel_z)
        
        # 4. Velocity penalty: encourage moving at reasonable speed toward target
        velocity_penalty = 0.0
        if grasped and horizontal_dist > POSITION_TOLERANCE:
            # Penalize excessive speed
            excessive_speed = max(0.0, gripper_speed - 0.2)
            velocity_penalty = -excessive_speed * 2.0
        elif grasped and horizontal_dist <= POSITION_TOLERANCE:
            # When close, reward low velocity to stabilize
            velocity_penalty = np.exp(-5.0 * gripper_speed) * 2.0
        
        # 5. Precision bonus when close to target
        # Precision bonus when very close to target
        precision_bonus = 0.0
        if horizontal_dist < 5 * SUCCESS_THRESHOLD:
            precision_bonus = 5.0 * np.exp(-20.0 * horizontal_dist)
            
            if horizontal_dist <= 2.0 * SUCCESS_THRESHOLD:
                # Extra bonus for staying stable when close
                precision_bonus += np.exp(-2.0 * np.linalg.norm(gripper_vel)) * 2.0
            
        # 6. In-position reward: bonus for reaching release position
        position_reward = 0.0
        if self.in_release_position:
            position_reward = 2.0  # Bonus for being in release position
        
        # 7. Release reward (one-time)
        release_reward = 0.0
        if self.block_released and not self.release_reward_given:
            if horizontal_dist < POSITION_TOLERANCE and velocity_ok:
                release_reward = 15.0  # Good release
            else:
                release_reward = -30.0  # Bad release (shouldn't happen with hard constraint)
            self.release_reward_given = True
        
        # 8. Landing reward
        landing_reward = 0.0
        if self.block_landed:
            landing_reward = 20.0 * np.exp(-10.0 * horizontal_dist)
            if horizontal_dist <= SUCCESS_THRESHOLD:
                landing_reward += 30.0
        
        # 9. Finger reward in release phase
        finger_reward = 0.0
        if self.prev_finger_distance is not None:
            finger_change = finger_distance - self.prev_finger_distance
            if self.in_release_position and velocity_ok and grasped:
                # Encourage opening fingers when in position
                finger_reward = finger_change * 5.0
            else:
                finger_reward = -finger_change * 5.0

        self.prev_finger_distance = finger_distance
        
        # Total reward
        reward = (base_reward + progress_reward + height_reward + velocity_penalty + 
                  precision_bonus + position_reward + release_reward + landing_reward + finger_reward)

        # Success criteria: block has landed on ground within threshold distance of target
        terminated = self.block_landed and horizontal_dist <= SUCCESS_THRESHOLD * SUCCESS_RELAXATION_FACTOR

        if terminated:
            reward += 50.0  # Large success bonus

        truncated = self.step_count >= self.max_steps or (self.block_landed and horizontal_dist > SUCCESS_THRESHOLD * SUCCESS_RELAXATION_FACTOR)

        info = {
            "horizontal_dist": horizontal_dist,
            "block_z": block_pos[2],
            "block_height": block_height_above_target,
            "gripper_z": gripper_pos[2],
            "finger_distance": finger_distance,
            "grasped": grasped,
            "in_release_position": self.in_release_position,
            "block_released": self.block_released,
            "block_landed": self.block_landed,
            "progress_reward": progress_reward,
            "height_reward": height_reward,
            "velocity_penalty": velocity_penalty,
            "precision_bonus": precision_bonus,
            "position_reward": position_reward,
            "release_reward": release_reward,
            "landing_reward": landing_reward,
            "finger_reward": finger_reward,
            "velocity": gripper_vel,
            "block_vel_z": block_vel_z
        }

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, info

    def reset_model(self):
        """Reset the environment with gripper above target, grasping the block.
        
        Initialization: Gripper starts above target with small position noise.
        Velocity is zero for clean learning - Stage 3 should stabilize before transition.
        """
        self.step_count = 0
        self.block_released = False
        self.block_landed = False
        self.initial_grasp_success = False
        self.prev_finger_distance = None
        self.prev_horizontal_dist = None
        self.release_reward_given = False
        self.in_release_position = False

        # Randomize target position
        rand_spawn(self.model, self.data)

        # Get target position after randomization
        target_pos = self._get_target_pos().copy()

        # Get joint addresses
        block_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "block_free")
        block_adr = self.model.jnt_qposadr[block_joint]

        gripper_lr_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_leftright")
        gripper_fb_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_forwardbackward")
        gripper_ud_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_updown")

        lr_adr = self.model.jnt_qposadr[gripper_lr_joint]
        fb_adr = self.model.jnt_qposadr[gripper_fb_joint]
        ud_adr = self.model.jnt_qposadr[gripper_ud_joint]

        left_finger_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_slide")
        right_finger_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "right_slide")
        left_adr = self.model.jnt_qposadr[left_finger_joint]
        right_adr = self.model.jnt_qposadr[right_finger_joint]

        # Position gripper above target with small noise
        gripper_height = TARGET_HEIGHT_ABOVE_TARGET + np.random.uniform(-INITIAL_HEIGHT_NOISE, INITIAL_HEIGHT_NOISE)

        # Small position noise - much smaller than before to help initial learning
        pos_noise_x = np.random.uniform(-INITIAL_POSITION_NOISE_RANGE, INITIAL_POSITION_NOISE_RANGE)
        pos_noise_y = np.random.uniform(-INITIAL_POSITION_NOISE_RANGE, INITIAL_POSITION_NOISE_RANGE)

        # Set gripper position aligned above target with noise
        self.data.qpos[lr_adr] = target_pos[0] + pos_noise_x
        self.data.qpos[fb_adr] = target_pos[1] + pos_noise_y
        self.data.qpos[ud_adr] = -(0.3 - gripper_height)

        # Set fingers to closed position
        self.data.qpos[left_adr] = FINGER_CLOSE_POSITION
        self.data.qpos[right_adr] = FINGER_CLOSE_POSITION

        # Position block at gripper location
        self.data.qpos[block_adr:block_adr+3] = [
            target_pos[0] + pos_noise_x,
            target_pos[1] + pos_noise_y,
            gripper_height
        ]
        self.data.qpos[block_adr+3:block_adr+7] = [1, 0, 0, 0]

        # Set finger actuator to maintain closed position
        self.data.ctrl[self.finger] = FINGER_CLOSE_COMMAND

        self.data.qvel[:] = np.random.uniform(-INITIAL_POSITION_NOISE_RANGE, INITIAL_POSITION_NOISE_RANGE)

        # Propagate physics
        mujoco.mj_forward(self.model, self.data)

        # Let physics settle
        for _ in range(20):
            self.data.ctrl[self.finger] = FINGER_CLOSE_COMMAND
            self.data.ctrl[self.updown] = 0.0
            self.data.ctrl[self.leftright] = 0.0
            self.data.ctrl[self.forwardback] = 0.0
            mujoco.mj_step(self.model, self.data)

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

        rel_to_target = target_pos - block_pos
        horizontal_dist = np.linalg.norm((block_pos - target_pos)[:2])
        self.prev_horizontal_dist = horizontal_dist

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

        obs = np.concatenate([
            rel_to_target / 0.5,
            np.array([gripper_pos[2]]),
            np.array([horizontal_dist]),
            np.array([block_pos[2]]),
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

        rel_to_target = target_pos - block_pos
        horizontal_dist = np.linalg.norm((block_pos - target_pos)[:2])

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
            rel_to_target / 0.5,
            np.array([gripper_pos[2]]),
            np.array([horizontal_dist]),
            np.array([block_pos[2]]),
            np.array([finger_distance]),
            gripper_vel,
            np.array([block_vel_z]),
            np.array([float(grasped)])
        ])

    def viewer_setup(self):
        assert self.viewer is not None
        self.viewer.cam.trackbodyid = 0
        self.viewer.cam.distance = self.model.stat.extent
