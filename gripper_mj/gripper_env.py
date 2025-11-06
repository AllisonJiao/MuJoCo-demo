import os
import gymnasium as gym
from gymnasium import spaces

from gymnasium import utils
from gymnasium.spaces import Box
from mujoco_py_env import MuJocoPyEnv

import numpy as np
import mujoco
from gripper_controller import rand_spawn

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
        observation_space = Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float64)
        MuJocoPyEnv.__init__(
            self,
            model_path=os.path.join("..", "model", "GripperGPT.xml"),
            frame_skip=1,
            observation_space=observation_space,
            **kwargs
        )

        self.step_count = 0
        self.max_steps = 100
        
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

        # action = 3 motors, obs = [block_xy, gripper_xy]
        self.action_space = spaces.Box(low=-1, high=1, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)

        # in __init__
        self.ctrl_scale = np.array([15.0, 10.0, 10.0], dtype=float)  # [up/down, left/right, forward/back]


    def step(self, action):
        self.step_count += 1
        action = np.clip(action, -1, 1)
        action *= self.ctrl_scale
        self.data.ctrl[self.updown] = action[0]
        self.data.ctrl[self.leftright] = action[1]
        self.data.ctrl[self.forwardback] = action[2]

        # advance physics
        for i in range(1,10):
            mujoco.mj_step(self.model, self.data)

        block_xy   = self.data.xpos[self.body][:2]
        gripper_xy = self.data.xpos[self.gripper][:2]
        obs = np.concatenate([block_xy, gripper_xy])
        
        # distance between block and gripper
        dist = np.linalg.norm(gripper_xy - block_xy)

        reward = -dist
        terminated = dist <= 0.05   # success threshold
        if dist <= 0.05:
            reward += 1
        truncated = self.step_count >= self.max_steps
        info = {"distance": dist}

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, info

    def reset_model(self):
        """Reset the robot degrees of freedom (qpos and qvel) and randomize block/target positions."""
        self.step_count = 0
        # Note: _reset_simulation() is already called by base class reset()
        # So we just need to randomize positions and forward the physics
        rand_spawn(self.model, self.data)  # randomize block/target
        mujoco.mj_forward(self.model, self.data)  # propagate physics

        block_xy   = self.data.xpos[self.body][:2]
        gripper_xy = self.data.xpos[self.gripper][:2]
        obs = np.concatenate([block_xy, gripper_xy])

        return obs
    
    def _get_obs(self):
        return np.concatenate([self.data.qpos, self.data.qvel]).ravel()

    def viewer_setup(self):
        assert self.viewer is not None
        self.viewer.cam.trackbodyid = 0
        self.viewer.cam.distance = self.model.stat.extent
