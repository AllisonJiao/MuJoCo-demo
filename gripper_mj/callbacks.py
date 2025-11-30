from stable_baselines3.common.callbacks import BaseCallback
import os
import numpy as np


class _MetricCatcher:
    """
    Minimal logger output format that intercepts dumps from SB3's logger.
    It is invoked right after a training update (when logger.dump is called),
    so we can grab the freshly computed losses/explained variance.
    """

    def __init__(self, on_dump_fn):
        self._on_dump_fn = on_dump_fn

    def write(self, key_values, key_excluded, step):
        # Forward the values so the callback can store them
        self._on_dump_fn(key_values)

    def close(self):
        pass

class RewardLoggingCallback(BaseCallback):
    """
    Logs episode rewards, training metrics (entropy loss, value loss, explained variance, episode length)
    to a CSV file during training.
    Requires the env to be wrapped with Monitor (make_vec_env does this).
    
    Note: Explained variance is only available for on-policy algorithms (e.g., PPO) that use value functions.
    For off-policy algorithms like SAC that use Q-functions, explained_variance will be NaN.
    """
    def __init__(self, log_path: str, verbose=0):
        super().__init__(verbose)
        self.log_path = log_path
        self._training_data = []  # Store all training metrics
        self._last_rollout_metrics = {}  # Store metrics from last rollout
        self._episodes_in_rollout = []  # Episodes that finished during the current rollout
        self._metric_catcher = None  # Attached to SB3 logger to intercept dumps

    def _init_callback(self) -> None:
        """
        Attach a lightweight output format to the SB3 logger so we can
        intercept the metrics exactly when logger.dump is called
        (right after each training update).
        """
        super()._init_callback()
        if self.logger is not None and self._metric_catcher is None:
            self._metric_catcher = _MetricCatcher(self._on_metrics_dumped)
            # SB3 logger calls every output format in order; this is non-destructive
            self.logger.output_formats.append(self._metric_catcher)

    def _get_metrics_from_logger(self):
        """Helper method to extract metrics from logger"""
        metrics = {"entropy_loss": np.nan, "value_loss": np.nan, "explained_variance": np.nan}
        
        if self.logger is None:
            return metrics
        
        # Try multiple ways to access logged values
        name_to_value = {}
        
        # Method 1: Direct access to name_to_value (populated after dump)
        try:
            if hasattr(self.logger, 'name_to_value') and self.logger.name_to_value:
                name_to_value.update(self.logger.name_to_value)
        except (AttributeError, TypeError):
            pass
        
        # Method 2: Check kv_mean (running averages, populated earlier)
        try:
            if hasattr(self.logger, 'kv_mean') and self.logger.kv_mean:
                name_to_value.update(self.logger.kv_mean)
        except (AttributeError, TypeError):
            pass
        
        # Method 3: Check output formats
        try:
            if hasattr(self.logger, 'output_formats'):
                for output_format in self.logger.output_formats:
                    if hasattr(output_format, 'name_to_value') and output_format.name_to_value:
                        name_to_value.update(output_format.name_to_value)
                    if hasattr(output_format, 'kv_mean') and output_format.kv_mean:
                        name_to_value.update(output_format.kv_mean)
        except (AttributeError, TypeError):
            pass
        
        if not name_to_value:
            return metrics
        
        # Try to get entropy-related metrics (priority order)
        if "train/entropy_loss" in name_to_value:
            metrics["entropy_loss"] = name_to_value["train/entropy_loss"]
        elif "train/actor_loss" in name_to_value:
            # For SAC, actor loss includes entropy component
            metrics["entropy_loss"] = name_to_value["train/actor_loss"]
        elif "train/policy_gradient_loss" in name_to_value:
            # For PPO, policy gradient loss includes entropy
            metrics["entropy_loss"] = name_to_value["train/policy_gradient_loss"]
        elif "train/ent_coef" in name_to_value:
            # For SAC, this is the entropy coefficient (not loss, but related metric)
            metrics["entropy_loss"] = name_to_value["train/ent_coef"]
        
        # Try to get value loss
        if "train/value_loss" in name_to_value:
            metrics["value_loss"] = name_to_value["train/value_loss"]
        elif "train/critic_loss" in name_to_value:
            metrics["value_loss"] = name_to_value["train/critic_loss"]
        
        # Try to get explained variance
        if "train/explained_variance" in name_to_value:
            metrics["explained_variance"] = name_to_value["train/explained_variance"]
        
        return metrics

    def _on_metrics_dumped(self, key_values: dict):
        """
        Called right after SB3 dumps training metrics. Store the losses so we
        can attach them to any episodes that finished during the just-completed rollout.
        """
        metrics = {
            "entropy_loss": key_values.get("train/entropy_loss", np.nan),
            "value_loss": key_values.get("train/value_loss", key_values.get("train/critic_loss", np.nan)),
            "explained_variance": key_values.get("train/explained_variance", np.nan),
        }

        # Try PPO-specific metrics for entropy loss
        if np.isnan(metrics["entropy_loss"]) and "train/policy_gradient_loss" in key_values:
            metrics["entropy_loss"] = key_values["train/policy_gradient_loss"]
        
        # Try SAC-specific metrics for entropy loss
        if np.isnan(metrics["entropy_loss"]) and "train/actor_loss" in key_values:
            metrics["entropy_loss"] = key_values["train/actor_loss"]
        if np.isnan(metrics["entropy_loss"]) and "train/ent_coef" in key_values:
            metrics["entropy_loss"] = key_values["train/ent_coef"]

        # Update stored metrics if at least one value is present
        if not all(np.isnan(v) for v in metrics.values()):
            self._last_rollout_metrics.update({k: v for k, v in metrics.items() if not np.isnan(v)})

    def _on_step(self) -> bool:
        # infos is a list (one per env) in VecEnv
        infos = self.locals.get("infos", [])
        for info in infos:
            # Monitor wrapper puts episode info under "episode"
            if "episode" in info:
                ep_reward = info["episode"]["r"]
                ep_len = info["episode"]["l"]
                self._episodes_in_rollout.append({
                    "episode_reward": ep_reward,
                    "episode_length": ep_len,
                })
                if self.verbose > 1:
                    print(f"[RewardLogger] collected ep t={self.num_timesteps} "
                          f"R={ep_reward:.2f} len={ep_len}")
        return True

    def _write_rollout_row(self):
        """Write a row for the current rollout with episodes and metrics"""
        if not self._episodes_in_rollout:
            return
        
        # Calculate average reward and length for episodes in this rollout
        rewards = [ep["episode_reward"] for ep in self._episodes_in_rollout]
        lengths = [ep["episode_length"] for ep in self._episodes_in_rollout]
        ep_reward = float(np.mean(rewards))
        ep_len = float(np.mean(lengths))

        # Get metrics - try from stored metrics first, then from logger directly
        entropy_loss = self._last_rollout_metrics.get("entropy_loss", np.nan)
        value_loss = self._last_rollout_metrics.get("value_loss", np.nan)
        explained_variance = self._last_rollout_metrics.get("explained_variance", np.nan)
        
        # If metrics are still NaN, try to get them from logger directly
        if np.isnan(entropy_loss) or np.isnan(value_loss) or np.isnan(explained_variance):
            current_metrics = self._get_metrics_from_logger()
            if not np.isnan(current_metrics["entropy_loss"]):
                entropy_loss = current_metrics["entropy_loss"]
                self._last_rollout_metrics["entropy_loss"] = entropy_loss
            if not np.isnan(current_metrics["value_loss"]):
                value_loss = current_metrics["value_loss"]
                self._last_rollout_metrics["value_loss"] = value_loss
            if not np.isnan(current_metrics["explained_variance"]):
                explained_variance = current_metrics["explained_variance"]
                self._last_rollout_metrics["explained_variance"] = explained_variance

        row = {
            "timesteps": self.num_timesteps,
            "episode_reward": ep_reward,
            "episode_length": ep_len,
            "entropy_loss": entropy_loss,
            "value_loss": value_loss,
            "explained_variance": explained_variance,
        }
        self._training_data.append(row)
        
        if self.verbose > 0:
            print(f"[RewardLogger] rollout@{self.num_timesteps} "
                  f"R={ep_reward:.2f} len={ep_len:.1f} "
                  f"ent_loss={entropy_loss:.4f} "
                  f"val_loss={value_loss:.4f} "
                  f"exp_var={explained_variance:.4f}")

        # Reset rollout buffers
        self._episodes_in_rollout.clear()

    def _on_rollout_end(self) -> None:
        """
        Called after each rollout/update. Write a row immediately with the episodes
        that finished during this rollout and the most recent training metrics.
        """
        # Write the row for this rollout
        self._write_rollout_row()

    def _on_training_end(self) -> None:
        # Flush any pending episodes if training ended
        if self._episodes_in_rollout:
            self._write_rollout_row()

        if len(self._training_data) == 0:
            return
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        with open(self.log_path, "w") as f:
            # Write header
            f.write("timesteps,episode_reward,episode_length,entropy_loss,value_loss,explained_variance\n")
            # Write data
            for data in self._training_data:
                f.write(f"{data['timesteps']},{data['episode_reward']},{data['episode_length']},"
                       f"{data['entropy_loss']},{data['value_loss']},{data['explained_variance']}\n")
        if self.verbose > 0:
            print(f"[RewardLogger] Saved training log to {self.log_path} ({len(self._training_data)} entries)")
