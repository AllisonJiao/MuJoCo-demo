from os import path
from typing import Any, Dict, Optional, Tuple, Union

import gymnasium as gym
import numpy as np
from gymnasium import error, logger, spaces
from gymnasium.spaces import Space
from numpy.typing import NDArray

try:
    import mujoco
    import mujoco.viewer
except ImportError as e:
    MUJOCO_IMPORT_ERROR = e
else:
    MUJOCO_IMPORT_ERROR = None

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


# NOTE: duplication of analogous code in mujoco_env.py
# Support for mujoco-py based envs is deprecated, so this module will no longer be maintained
# The code is duplicated instead of imported for it to be working in standalone and independent
# of the further development of the maintained mujoco_env.py
# noinspection DuplicatedCode
DEFAULT_SIZE = 480


# noinspection DuplicatedCode
def expand_model_path(model_path: str) -> str:
    """Expands the `model_path` to a full path if it starts with '~' or '.' or '/'."""
    if model_path.startswith(".") or model_path.startswith("/"):
        fullpath = model_path
    elif model_path.startswith("~"):
        fullpath = path.expanduser(model_path)
    else:
        fullpath = path.join(path.dirname(__file__), "assets", model_path)
    if not path.exists(fullpath):
        raise OSError(f"File {fullpath} does not exist")

    return fullpath


# noinspection DuplicatedCode
class BaseMujocoPyEnv(gym.Env[NDArray[np.float64], NDArray[np.float32]]):
    """Superclass for all MuJoCo environments."""

    def __init__(
        self,
        model_path,
        frame_skip,
        observation_space: Optional[Space],
        render_mode: Optional[str] = None,
        width: int = DEFAULT_SIZE,
        height: int = DEFAULT_SIZE,
        camera_id: Optional[int] = None,
        camera_name: Optional[str] = None,
    ):
        """Base abstract class for mujoco based environments.

        Args:
            model_path: Path to the MuJoCo Model.
            frame_skip: Number of MuJoCo simulation steps per gym `step()`.
            observation_space: The observation space of the environment.
            render_mode: The `render_mode` used.
            width: The width of the render window.
            height: The height of the render window.
            camera_id: The camera ID used.
            camera_name: The name of the camera used (can not be used in conjunction with `camera_id`).

        Raises:
            OSError: when the `model_path` does not exist.
            error.DependencyNotInstalled: When `mujoco` is not installed.
        """
        self.fullpath = expand_model_path(model_path)

        self.width = width
        self.height = height
        # may use width and height
        self.model, self.data = self._initialize_simulation()

        self.init_qpos = self.data.qpos.ravel().copy()
        self.init_qvel = self.data.qvel.ravel().copy()

        self.frame_skip = frame_skip

        assert self.metadata["render_modes"] == [
            "human",
            "rgb_array",
            "depth_array",
        ], self.metadata["render_modes"]
        # Note: render_fps in metadata is a target value, not necessarily equal to 1/dt
        # The actual simulation timestep may differ based on frame_skip and model timestep
        if observation_space is not None:
            self.observation_space = observation_space
        self._set_action_space()

        self.render_mode = render_mode
        self.camera_name = camera_name
        self.camera_id = camera_id

    def _set_action_space(self):
        bounds = self.model.actuator_ctrlrange.copy().astype(np.float32)
        low, high = bounds.T
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        return self.action_space

    # methods to override:
    # ----------------------------
    def step(
        self, action: NDArray[np.float32]
    ) -> Tuple[NDArray[np.float64], np.float64, bool, bool, Dict[str, np.float64]]:
        raise NotImplementedError

    def reset_model(self) -> NDArray[np.float64]:
        """
        Reset the robot degrees of freedom (qpos and qvel).
        Implement this in each subclass.
        """
        raise NotImplementedError

    def _initialize_simulation(self) -> Tuple[Any, Any]:
        """
        Initialize MuJoCo simulation data structures mjModel and mjData.
        """
        raise NotImplementedError

    def _reset_simulation(self) -> None:
        """
        Reset MuJoCo simulation data structures, mjModel and mjData.
        """
        raise NotImplementedError

    def _step_mujoco_simulation(self, ctrl, n_frames) -> None:
        """
        Step over the MuJoCo simulation.
        """
        raise NotImplementedError

    def render(self) -> Union[NDArray[np.float64], None]:
        """
        Render a frame from the MuJoCo simulation as specified by the render_mode.
        """
        raise NotImplementedError

    # -----------------------------
    def _get_reset_info(self) -> Dict[str, float]:
        """Function that generates the `info` that is returned during a `reset()`."""
        return {}

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ):
        super().reset(seed=seed)

        self._reset_simulation()

        ob = self.reset_model()
        info = self._get_reset_info()

        if self.render_mode == "human":
            self.render()
        return ob, info

    def set_state(self, qpos, qvel) -> None:
        """
        Set the joints position qpos and velocity qvel of the model. Override this method depending on the MuJoCo bindings used.
        """
        assert qpos.shape == (self.model.nq,) and qvel.shape == (self.model.nv,)

    @property
    def dt(self) -> float:
        return self.model.opt.timestep * self.frame_skip

    def do_simulation(self, ctrl, n_frames) -> None:
        """
        Step the simulation n number of frames and applying a control action.
        """
        # Check control input is contained in the action space
        if np.array(ctrl).shape != (self.model.nu,):
            raise ValueError(
                f"Action dimension mismatch. Expected {(self.model.nu,)}, found {np.array(ctrl).shape}"
            )
        self._step_mujoco_simulation(ctrl, n_frames)

    def close(self):
        """Close all processes like rendering contexts"""
        raise NotImplementedError

    def get_body_com(self, body_name) -> NDArray[np.float64]:
        """Return the cartesian position of a body frame"""
        raise NotImplementedError

    def state_vector(self) -> NDArray[np.float64]:
        """Return the position and velocity joint states of the model"""
        return np.concatenate([self.data.qpos.flat, self.data.qvel.flat])


class MuJocoPyEnv(BaseMujocoPyEnv):
    def __init__(
        self,
        model_path: str,
        frame_skip: int,
        observation_space: Space,
        render_mode: Optional[str] = None,
        width: int = DEFAULT_SIZE,
        height: int = DEFAULT_SIZE,
        camera_id: Optional[int] = None,
        camera_name: Optional[str] = None,
    ):
        if MUJOCO_IMPORT_ERROR is not None:
            raise error.DependencyNotInstalled(
                f"{MUJOCO_IMPORT_ERROR}. "
                "Could not import mujoco, which is needed for MuJoCo environments.",
            )

        self.viewer = None
        self._viewers = {}
        self._renderer = None

        super().__init__(
            model_path,
            frame_skip,
            observation_space,
            render_mode,
            width,
            height,
            camera_id,
            camera_name,
        )

    def _initialize_simulation(self):
        model = mujoco.MjModel.from_xml_path(self.fullpath)
        data = mujoco.MjData(model)
        return model, data

    def _reset_simulation(self):
        mujoco.mj_resetData(self.model, self.data)

    def set_state(self, qpos, qvel):
        super().set_state(qpos, qvel)
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        mujoco.mj_forward(self.model, self.data)

    def get_body_com(self, body_name):
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        return self.data.xpos[body_id]

    def _step_mujoco_simulation(self, ctrl, n_frames):
        self.data.ctrl[:] = ctrl

        for _ in range(n_frames):
            mujoco.mj_step(self.model, self.data)

    def render(self):
        if self.render_mode is None:
            assert self.spec is not None
            gym.logger.warn(
                "You are calling render method without specifying any render mode. "
                "You can specify the render_mode at initialization, "
                f'e.g. gym.make("{self.spec.id}", render_mode="rgb_array")'
            )
            return

        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)

        # Determine camera ID
        camera_id = self.camera_id
        if camera_id is None and self.camera_name is not None:
            camera_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name
            )
        if camera_id is None:
            camera_id = -1  # Use default camera

        # Update renderer with current state
        self._renderer.update_scene(self.data)
        # Set camera if specified
        if camera_id >= 0:
            self._renderer.scene.camera.fixedcamid = camera_id

        if self.render_mode == "human":
            # For human mode, render and display frames using OpenCV
            frame = self._renderer.render()
            # Display using OpenCV if available (non-blocking)
            if CV2_AVAILABLE:
                # Convert RGB to BGR for OpenCV
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("MuJoCo Training", frame_bgr)
                cv2.waitKey(1)  # Non-blocking wait (1ms)
            else:
                print("Warning: OpenCV not available. Install with: pip install opencv-python")
        elif self.render_mode == "rgb_array":
            return self._renderer.render()
        elif self.render_mode == "depth_array":
            depth = self._renderer.render(depth=True)
            return depth

    def close(self):
        if self._renderer is not None:
            self._renderer = None
        if self.viewer is not None:
            self.viewer = None
            self._viewers = {}
        if CV2_AVAILABLE and self.render_mode == "human":
            cv2.destroyAllWindows()

    def viewer_setup(self):
        """
        This method is called when the viewer is initialized.
        Optionally implement this method, if you need to tinker with camera position and so forth.
        """
        raise NotImplementedError