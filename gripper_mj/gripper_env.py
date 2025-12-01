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
MAX_STEPS = 500
MIN_ABOVE = 0.2

"""
Goal: Move gripper on top of the block (don't care about y coordinate)
State: Coordinate of Gripper, Coordinate of Block
Actions: Up/left, Right/Left
Reward: - (distance between Block and Gripper)
Success: Gripper directly above the block
Fail: Over 100 iterations (?)
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

    def __init__(self, **kwargs):
        utils.EzPickle.__init__(self, **kwargs)
        observation_space = Box(low=-np.inf, high=np.inf, shape=(8,), dtype=np.float64)
        MuJocoPyEnv.__init__(
            self,
            model_path=os.path.join("..", "model", "GripperGPT.xml"),
            frame_skip=1,
            observation_space=observation_space,
            **kwargs
        )

        self.step_count = 0
        self.max_steps = MAX_STEPS
        self.prev_distance = None  # Track previous distance for progress detection
        
        # # load xml model
        # model_path = os.path.join("../model", "GripperGPT.xml")
        # self.model = mujoco.MjModel.from_xml_path(model_path)
        # self.data = mujoco.MjData(self.model)
        
        # actuator ids
        self.updown = self.model.actuator("up/down").id
        self.leftright = self.model.actuator("left/right").id
        self.forwardback = self.model.actuator("forward/backward").id
        self.actuators = np.array([self.updown, self.leftright, self.forwardback], dtype=int)
        
        # body ids
        self.body = self.model.body("block").id
        self.target = self.model.body("target").id
        self.gripper = self.model.body("gripper").id

        # action = 3 motors, obs = [relative_pos(2), dist_x, dist_y, gripper_xy(2), gripper_vel(2)] = 8D
        self.action_space = spaces.Box(low=-1, high=1, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(8,), dtype=np.float32)

        # in __init__
        self.ctrl_scale = np.array([15.0, 10.0, 10.0], dtype=float)  # [up/down, left/right, forward/back]
        # Fixed upward lift to avoid collision with block (adjust this value as needed)
        self.fixed_updown_lift = 5.0  # Positive value lifts gripper up


    def step(self, action):
        self.step_count += 1
        action = np.clip(action, -1, 1)
        action *= self.ctrl_scale
        
        # Apply actions to actuators
        # Set fixed upward lift for up/down to avoid collision with block
        self.data.ctrl[self.updown] = self.fixed_updown_lift
        # Left/right and forward/back are controlled by actions
        self.data.ctrl[self.leftright] = action[1]
        self.data.ctrl[self.forwardback] = action[2]

        # advance physics
        for i in range(1,10):
            mujoco.mj_step(self.model, self.data)
        '''
        block_xyz   = self.data.xpos[self.body][:3]
        gripper_xyz = self.data.xpos[self.gripper][:3]
        obs = np.concatenate([block_xyz, gripper_xyz])
        
        target_pos = block_xyz.copy()
        target_pos[2] += MIN_ABOVE
        # distance between block and gripper
        dist = np.linalg.norm(gripper_xyz - target_pos)
        '''
        block_xy   = self.data.xpos[self.body][:2]
        gripper_xy = self.data.xpos[self.gripper][:2]
        target_xy = self.data.xpos[self.target][:2]
        
        # Use relative position (more informative for learning direction)
        relative_pos = block_xy - gripper_xy
        # Distance components (x and y separately)
        dist_x = relative_pos[0]
        dist_y = relative_pos[1]
        # Include velocity to help model understand movement
        gripper_vel = self.data.qvel[:2] if len(self.data.qvel) >= 2 else np.array([0.0, 0.0])
        
        # Observation: [relative_x, relative_y, dist_x, dist_y, gripper_x, gripper_y, gripper_vel_x, gripper_vel_y]
        obs = np.concatenate([relative_pos, np.array([dist_x, dist_y]), gripper_xy, gripper_vel[:2]])
        
        # distance between block and gripper
        dist = np.linalg.norm(block_xy - gripper_xy)
        # distance between block and target
        # dist = np.linalg.norm(block_xy - target_xy)

        # Base reward: negative distance
        reward = -dist
        
        # Progress reward: encourage making progress
        progress_reward = 0.0
        if self.prev_distance is not None:
            distance_change = self.prev_distance - dist
            progress_reward = distance_change * 2.0  # Reward for getting closer
        self.prev_distance = dist
        
        # Penalty for being stuck (no progress for many steps)
        stuck_penalty = 0.0
        if self.prev_distance is not None and abs(dist - self.prev_distance) < 0.001:
            # If distance hasn't changed much, add small penalty
            stuck_penalty = -0.01
        
        reward = reward + progress_reward + stuck_penalty
        
        terminated = dist <= SUCCESS_THRESHOLD   # success threshold
        if terminated:
            reward += 10.0  # Large success bonus
        truncated = self.step_count >= self.max_steps
        info = {"distance": dist, "progress": progress_reward}

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, info

    def reset_model(self, initial_state=None):
        """Reset the robot degrees of freedom (qpos and qvel) and randomize block/target positions.
        
        Args:
            initial_state: Optional dict containing state to copy instead of random init.
                - 'qpos': Joint positions to copy
                - 'qvel': Joint velocities to copy
                - 'ctrl': Control values to copy
                - 'act': Actuator states to copy (critical for intvelocity actuators)
                - 'target_pos': Target position to copy
        """
        self.step_count = 0
        self.prev_distance = None  # Reset progress tracking
        
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
            # Note: _reset_simulation() is already called by base class reset()
            # So we just need to randomize positions and forward the physics
            rand_spawn(self.model, self.data)  # randomize block/target
            mujoco.mj_forward(self.model, self.data)  # propagate physics

        block_xy   = self.data.xpos[self.body][:2]
        gripper_xy = self.data.xpos[self.gripper][:2]
        
        # Use relative position and velocity (consistent with step())
        relative_pos = block_xy - gripper_xy
        # Distance components (x and y separately)
        dist_x = relative_pos[0]
        dist_y = relative_pos[1]
        gripper_vel = self.data.qvel[:2] if len(self.data.qvel) >= 2 else np.array([0.0, 0.0])
        obs = np.concatenate([relative_pos, np.array([dist_x, dist_y]), gripper_xy, gripper_vel[:2]])

        return obs
    
    def _get_obs(self):
        return np.concatenate([self.data.qpos, self.data.qvel]).ravel()

    def viewer_setup(self):
        assert self.viewer is not None
        self.viewer.cam.trackbodyid = 0
        self.viewer.cam.distance = self.model.stat.extent
