"""Run Elastic-Tracker as the policy for Tracking-Direct-v2.

The TrackingEnv simulator supplies exact UAV and target state. ROS 2 performs
target prediction and trajectory optimization, while this process converts the
sampled p/v/a/yaw reference to the environment's native CTBR action.
"""

from __future__ import annotations

import argparse
import math
import os
import time
from dataclasses import dataclass

os.environ.setdefault("ROS_DISTRO", "jazzy")

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Elastic-Tracker TrackingEnv baseline")
parser.add_argument("--task", type=str, default="Tracking-Direct-v2")
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--max-steps", type=int, default=0)
parser.add_argument("--episodes", type=int, default=0)
parser.add_argument("--reference-timeout", type=float, default=0.5)
parser.add_argument("--trigger-delay", type=float, default=0.5)
parser.add_argument("--real-time-factor", type=float, default=1.0)
parser.add_argument("--uav-odometry-topic", default="/tracking/uav/odometry")
parser.add_argument("--target-odometry-topic", default="/tracking/target/odometry")
parser.add_argument("--trigger-topic", default="/trigger")
parser.add_argument("--desired-pose-topic", default="/desired_pose")
parser.add_argument("--desired-twist-topic", default="/desired_twist")
parser.add_argument(
    "--desired-acceleration-topic", default="/desired_acceleration"
)
parser.add_argument("--disable-fabric", action="store_true", default=False)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if args_cli.real_time_factor < 0.0:
    parser.error("--real-time-factor must be non-negative")

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401
from isaaclab.utils.math import quat_apply
from isaaclab_tasks.utils import parse_env_cfg
from isaacsim.core.utils.extensions import enable_extension

import tracking.tasks  # noqa: F401
from tracking.tasks.direct.tracking.tracking_env import TrackingEnv

from elastic_tracker_controller import ctbr_controller


def seconds_to_stamp(seconds: float):
    from builtin_interfaces.msg import Time

    whole_seconds = math.floor(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1.0e9))
    if nanoseconds == 1_000_000_000:
        whole_seconds += 1
        nanoseconds = 0
    return Time(sec=whole_seconds, nanosec=nanoseconds)


def stamp_key(stamp) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


def yaw_from_wxyz(quaternion: torch.Tensor) -> torch.Tensor:
    real, i, j, k = quaternion.unbind(dim=-1)
    return torch.atan2(
        2.0 * (real * k + i * j), 1.0 - 2.0 * (j.square() + k.square())
    )


@dataclass(frozen=True)
class Reference:
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    acceleration: tuple[float, float, float]
    yaw: float
    yaw_rate: float
    received_at: float


class RosInterface:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self.node = rclpy.create_node("elastic_tracker_tracking_env")
        self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        self.odometry_pub = self.node.create_publisher(
            Odometry, args_cli.uav_odometry_topic, qos_profile_sensor_data
        )
        self.target_pub = self.node.create_publisher(
            Odometry, args_cli.target_odometry_topic, qos_profile_sensor_data
        )
        self.trigger_pub = self.node.create_publisher(
            PoseStamped, args_cli.trigger_topic, 1
        )
        self._subscriptions = [
            self.node.create_subscription(
                PoseStamped, args_cli.desired_pose_topic, self._pose_callback, 10
            ),
            self.node.create_subscription(
                TwistStamped,
                args_cli.desired_twist_topic,
                self._twist_callback,
                10,
            ),
            self.node.create_subscription(
                AccelStamped,
                args_cli.desired_acceleration_topic,
                self._acceleration_callback,
                10,
            ),
        ]
        self._pending: dict[tuple[int, int], dict[str, object]] = {}
        self.reference: Reference | None = None

    def _entry(self, stamp) -> dict[str, object]:
        key = stamp_key(stamp)
        entry = self._pending.setdefault(key, {})
        if len(self._pending) > 20:
            for old_key in sorted(self._pending)[:-10]:
                self._pending.pop(old_key, None)
        return entry

    def _pose_callback(self, message):
        position = message.pose.position
        orientation = message.pose.orientation
        self._entry(message.header.stamp)["pose"] = (
            (position.x, position.y, position.z),
            math.atan2(
                2.0
                * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
            ),
        )
        self._complete(stamp_key(message.header.stamp))

    def _twist_callback(self, message):
        linear = message.twist.linear
        self._entry(message.header.stamp)["twist"] = (
            (linear.x, linear.y, linear.z),
            message.twist.angular.z,
        )
        self._complete(stamp_key(message.header.stamp))

    def _acceleration_callback(self, message):
        linear = message.accel.linear
        self._entry(message.header.stamp)["acceleration"] = (
            linear.x,
            linear.y,
            linear.z,
        )
        self._complete(stamp_key(message.header.stamp))

    def _complete(self, key: tuple[int, int]):
        entry = self._pending.get(key)
        if entry is None or not {"pose", "twist", "acceleration"} <= entry.keys():
            return
        position, yaw = entry["pose"]
        velocity, yaw_rate = entry["twist"]
        self.reference = Reference(
            position=position,
            velocity=velocity,
            acceleration=entry["acceleration"],
            yaw=yaw,
            yaw_rate=yaw_rate,
            received_at=time.monotonic(),
        )
        self._pending.clear()

    def reset_reference(self):
        self.reference = None
        self._pending.clear()

    def spin(self):
        for _ in range(12):
            self._rclpy.spin_once(self.node, timeout_sec=0.0)

    def publish_state(self, sim_time: float, tracking_env: TrackingEnv):
        from rosgraph_msgs.msg import Clock

        stamp = seconds_to_stamp(sim_time)
        self.clock_pub.publish(Clock(clock=stamp))

        robot = tracking_env._robot.data
        robot_message = self._odometry_message(
            stamp,
            robot.root_pos_w[0],
            robot.root_quat_w[0],
            robot.root_lin_vel_w[0],
            robot.root_ang_vel_w[0],
        )
        self.odometry_pub.publish(robot_message)

        target_position, target_quaternion = tracking_env._get_car_task_pose_w()
        target_linear_velocity, target_angular_velocity = (
            tracking_env._get_car_root_velocity_w()
        )
        offset_world = quat_apply(
            target_quaternion,
            tracking_env._car_visual_root_offset.unsqueeze(0),
        )
        target_task_velocity = target_linear_velocity + torch.cross(
            target_angular_velocity, offset_world, dim=-1
        )
        target_message = self._odometry_message(
            stamp,
            target_position[0],
            target_quaternion[0],
            target_task_velocity[0],
            target_angular_velocity[0],
        )
        self.target_pub.publish(target_message)

    @staticmethod
    def _odometry_message(
        stamp, position, quaternion_wxyz, linear_velocity, angular_velocity
    ):
        from nav_msgs.msg import Odometry

        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.child_frame_id = "world"
        message.pose.pose.position.x = float(position[0])
        message.pose.pose.position.y = float(position[1])
        message.pose.pose.position.z = float(position[2])
        message.pose.pose.orientation.w = float(quaternion_wxyz[0])
        message.pose.pose.orientation.x = float(quaternion_wxyz[1])
        message.pose.pose.orientation.y = float(quaternion_wxyz[2])
        message.pose.pose.orientation.z = float(quaternion_wxyz[3])
        message.twist.twist.linear.x = float(linear_velocity[0])
        message.twist.twist.linear.y = float(linear_velocity[1])
        message.twist.twist.linear.z = float(linear_velocity[2])
        message.twist.twist.angular.x = float(angular_velocity[0])
        message.twist.twist.angular.y = float(angular_velocity[1])
        message.twist.twist.angular.z = float(angular_velocity[2])
        return message

    def publish_trigger(self, sim_time: float):
        from geometry_msgs.msg import PoseStamped

        message = PoseStamped()
        message.header.stamp = seconds_to_stamp(sim_time)
        message.header.frame_id = "world"
        message.pose.orientation.w = 1.0
        self.trigger_pub.publish(message)

    def close(self):
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


def configure_env() -> object:
    TrackingEnv.USE_RGB_CAMERA = False
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=1,
        use_fabric=not args_cli.disable_fabric,
    )
    env_cfg.seed = args_cli.seed
    env_cfg.scene.num_envs = 1
    env_cfg.debug_vis = False
    env_cfg.enable_drone_logging = False
    env_cfg.command_mode = "manual_none"
    env_cfg.disable_dynamics_randomization = True
    motor = env_cfg.motor_controller
    motor.enable_dynamics_randomization = False
    motor.thrust_k_noise_range = (1.0, 1.0)
    motor.torque_k_noise_range = (1.0, 1.0)
    motor.force_noise_weight_ratio = 0.0
    motor.torque_noise_range = (0.0, 0.0)
    motor.angvel_noise_std = 0.0
    motor.angacc_noise_std = 0.0
    motor.rpm2_noise_range = (1.0, 1.0)
    return env_cfg


def tensor3(values, tracking_env: TrackingEnv) -> torch.Tensor:
    return torch.tensor(
        [values], device=tracking_env.device, dtype=torch.float32
    )


def main() -> None:
    if not enable_extension("isaacsim.ros2.bridge"):
        raise RuntimeError("Failed to enable Isaac Sim ROS 2 Bridge")

    env = gym.make(args_cli.task, cfg=configure_env())
    ros = RosInterface()
    try:
        env.reset(seed=args_cli.seed)
        tracking_env: TrackingEnv = env.unwrapped
        tracking_env.episode_length_buf.zero_()
        hold_position = tracking_env._robot.data.root_pos_w.clone()
        hold_yaw = yaw_from_wxyz(tracking_env._robot.data.root_quat_w).clone()
        trigger_due = float(tracking_env.sim.current_time) + args_cli.trigger_delay
        trigger_sent = False
        episode_count = 0
        step_count = 0
        control_dt = float(tracking_env.step_dt)
        next_deadline = time.monotonic()
        print(
            "[baseline] Tracking-Direct-v2 true-state bridge ready: "
            f"control={1.0 / control_dt:.1f} Hz, cameras=off, dynamics DR=off"
        )

        while simulation_app.is_running():
            sim_time = float(tracking_env.sim.current_time)
            ros.publish_state(sim_time, tracking_env)
            ros.spin()
            if (
                not trigger_sent
                and sim_time >= trigger_due
                and ros.trigger_pub.get_subscription_count() > 0
            ):
                ros.reset_reference()
                ros.publish_trigger(sim_time)
                trigger_sent = True
                print(f"[baseline] trigger published at t={sim_time:.2f}s")

            reference = ros.reference
            reference_fresh = (
                reference is not None
                and time.monotonic() - reference.received_at
                <= args_cli.reference_timeout
            )
            if reference_fresh:
                desired_position = tensor3(reference.position, tracking_env)
                desired_velocity = tensor3(reference.velocity, tracking_env)
                desired_acceleration = tensor3(reference.acceleration, tracking_env)
                desired_yaw = torch.tensor(
                    [reference.yaw], device=tracking_env.device
                )
                desired_yaw_rate = torch.tensor(
                    [reference.yaw_rate], device=tracking_env.device
                )
            else:
                desired_position = hold_position
                desired_velocity = torch.zeros_like(hold_position)
                desired_acceleration = torch.zeros_like(hold_position)
                desired_yaw = hold_yaw
                desired_yaw_rate = torch.zeros_like(hold_yaw)

            with torch.inference_mode():
                actions = ctbr_controller(
                    position_w=tracking_env._robot.data.root_pos_w,
                    quaternion_wxyz=tracking_env._robot.data.root_quat_w,
                    linear_velocity_w=tracking_env._robot.data.root_lin_vel_w,
                    desired_position_w=desired_position,
                    desired_velocity_w=desired_velocity,
                    desired_acceleration_w=desired_acceleration,
                    desired_yaw=desired_yaw,
                    desired_yaw_rate=desired_yaw_rate,
                    gravity=float(tracking_env._gravity_magnitude.item()),
                    max_collective_thrust=3.0,
                    max_body_rate=float(tracking_env.cfg.motor_controller.max_rate),
                )
                _, _, terminated, truncated, _ = env.step(actions)

            done = bool((terminated | truncated)[0].item())
            if done:
                episode_count += 1
                tracking_env.episode_length_buf.zero_()
                hold_position = tracking_env._robot.data.root_pos_w.clone()
                hold_yaw = yaw_from_wxyz(
                    tracking_env._robot.data.root_quat_w
                ).clone()
                ros.reset_reference()
                trigger_due = (
                    float(tracking_env.sim.current_time) + args_cli.trigger_delay
                )
                trigger_sent = False
                print(f"[baseline] episode {episode_count} reset")

            step_count += 1
            if args_cli.max_steps > 0 and step_count >= args_cli.max_steps:
                break
            if args_cli.episodes > 0 and episode_count >= args_cli.episodes:
                break

            if args_cli.real_time_factor > 0.0:
                next_deadline += control_dt / args_cli.real_time_factor
                remaining = next_deadline - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)
                elif remaining < -control_dt:
                    next_deadline = time.monotonic()
    finally:
        ros.close()
        env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
