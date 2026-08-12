#!/usr/bin/env python3
"""Publish a converted motion-capture target trajectory as ROS 2 odometry."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty


class TargetTrajectoryReplay(Node):
    """Play one finite NPZ trajectory in the same task-reference frame as TrackingEnv."""

    REQUIRED_KEYS = (
        "time_s",
        "root_linear_velocity_world_mps",
        "task_position_local_m",
    )

    def __init__(self) -> None:
        super().__init__("target_trajectory_replay")
        trajectory_file = Path(
            self.declare_parameter("trajectory_file", "").value
        ).expanduser()
        if not trajectory_file.is_file():
            raise FileNotFoundError(
                "trajectory_file must name an existing converted NPZ: "
                f"{trajectory_file}"
            )

        self._frame_id = str(self.declare_parameter("frame_id", "world").value)
        self._playback_rate = float(self.declare_parameter("playback_rate", 1.0).value)
        self._publish_rate = float(self.declare_parameter("publish_rate", 50.0).value)
        self._auto_trigger = bool(self.declare_parameter("auto_trigger", True).value)
        self._trigger_delay_s = float(
            self.declare_parameter("trigger_delay_s", 0.5).value
        )
        self._offset = np.array(
            [
                float(self.declare_parameter("position_offset_x", 0.0).value),
                float(self.declare_parameter("position_offset_y", 0.0).value),
                float(self.declare_parameter("position_offset_z", 0.0).value),
            ],
            dtype=np.float64,
        )
        if self._playback_rate <= 0.0 or self._publish_rate <= 0.0:
            raise ValueError("playback_rate and publish_rate must both be positive")

        with np.load(trajectory_file, allow_pickle=False) as data:
            missing = [key for key in self.REQUIRED_KEYS if key not in data]
            if missing:
                raise ValueError(f"trajectory is missing required arrays: {missing}")
            self._time_s = np.asarray(data["time_s"], dtype=np.float64)
            self._position = np.asarray(
                data["task_position_local_m"], dtype=np.float64
            )
            self._linear_velocity = np.asarray(
                data["root_linear_velocity_world_mps"], dtype=np.float64
            )
        self._validate(trajectory_file)

        odometry_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        trigger_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        heartbeat_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._odometry_pub = self.create_publisher(Odometry, "target_odometry", odometry_qos)
        self._trigger_pub = self.create_publisher(PoseStamped, "trigger", trigger_qos)
        self._heartbeat_sub = self.create_subscription(
            Empty, "heartbeat", self._on_heartbeat, heartbeat_qos
        )
        self._planner_ready = False
        self._triggered = False
        self._start_time_ns: int | None = None
        self._finished_logged = False
        self._timer = self.create_timer(1.0 / self._publish_rate, self._tick)
        self.get_logger().info(
            f"Loaded target truth trajectory {trajectory_file}: "
            f"{self._time_s.size} samples, "
            f"{self._time_s[-1] - self._time_s[0]:.3f} s at "
            f"{1.0 / np.median(np.diff(self._time_s)):.3f} Hz"
        )

    def _validate(self, trajectory_file: Path) -> None:
        arrays = (
            self._position,
            self._linear_velocity,
        )
        count = self._time_s.size
        expected_shapes = ((count, 3), (count, 3))
        if count < 2 or self._time_s.ndim != 1:
            raise ValueError(f"time_s must be a one-dimensional sequence: {trajectory_file}")
        if any(array.shape != shape for array, shape in zip(arrays, expected_shapes)):
            raise ValueError(f"trajectory arrays have incompatible shapes: {trajectory_file}")
        if not np.isfinite(self._time_s).all() or any(
            not np.isfinite(array).all() for array in arrays
        ):
            raise ValueError(f"trajectory contains NaN or Inf: {trajectory_file}")
        if np.any(np.diff(self._time_s) <= 0.0):
            raise ValueError(f"time_s must be strictly increasing: {trajectory_file}")

    def _on_heartbeat(self, _: Empty) -> None:
        self._planner_ready = True

    def _sample(self, elapsed_s: float) -> tuple[np.ndarray, np.ndarray]:
        sample_time = np.clip(
            self._time_s[0] + elapsed_s * self._playback_rate,
            self._time_s[0],
            self._time_s[-1],
        )
        upper = int(np.searchsorted(self._time_s, sample_time, side="right"))
        upper = min(max(upper, 1), self._time_s.size - 1)
        lower = upper - 1
        alpha = float(
            (sample_time - self._time_s[lower])
            / (self._time_s[upper] - self._time_s[lower])
        )
        position = (1.0 - alpha) * self._position[lower] + alpha * self._position[upper]
        velocity = (
            (1.0 - alpha) * self._linear_velocity[lower]
            + alpha * self._linear_velocity[upper]
        )
        return position + self._offset, velocity

    def _publish_trigger(self, stamp) -> None:
        trigger = PoseStamped()
        trigger.header.stamp = stamp
        trigger.header.frame_id = self._frame_id
        self._trigger_pub.publish(trigger)
        self._triggered = True
        self.get_logger().info("Planner heartbeat received; sent tracking trigger")

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self._start_time_ns is None:
            self._start_time_ns = now.nanoseconds
        elapsed_s = max(0.0, (now.nanoseconds - self._start_time_ns) * 1.0e-9)
        position, velocity = self._sample(elapsed_s)

        message = Odometry()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self._frame_id
        message.child_frame_id = "target_task"
        message.pose.pose.position.x = float(position[0])
        message.pose.pose.position.y = float(position[1])
        message.pose.pose.position.z = float(position[2])
        # Elastic-Tracker reads only target position and linear velocity. Use a
        # valid identity quaternion rather than coupling replay to a car body.
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = float(velocity[0])
        message.twist.twist.linear.y = float(velocity[1])
        message.twist.twist.linear.z = float(velocity[2])
        self._odometry_pub.publish(message)

        if (
            self._auto_trigger
            and self._planner_ready
            and not self._triggered
            and elapsed_s >= self._trigger_delay_s
        ):
            self._publish_trigger(message.header.stamp)

        duration_s = (self._time_s[-1] - self._time_s[0]) / self._playback_rate
        if elapsed_s >= duration_s and not self._finished_logged:
            self._finished_logged = True
            self.get_logger().info("Target trajectory reached its final sample and is held")


def main() -> None:
    rclpy.init()
    node = TargetTrajectoryReplay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
