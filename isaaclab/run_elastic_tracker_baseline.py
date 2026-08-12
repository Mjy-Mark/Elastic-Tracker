"""Run a single-vehicle Isaac Lab scene against the ROS 2 Elastic-Tracker port.

Launch the ROS 2 baseline in a separate terminal, then start this script with
Isaac Lab's Python environment. The script uses Isaac Sim's bundled Jazzy
``rclpy`` and exchanges only standard ROS 2 messages.
"""

from __future__ import annotations

import argparse
import math
import os

os.environ.setdefault("ROS_DISTRO", "jazzy")

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Elastic-Tracker Isaac Lab baseline")
parser.add_argument("--state-rate", type=float, default=100.0)
parser.add_argument("--pointcloud-rate", type=float, default=10.0)
parser.add_argument("--pointcloud-stride", type=int, default=2)
parser.add_argument("--sensor-range", type=float, default=10.0)
parser.add_argument("--target-radius", type=float, default=2.0)
parser.add_argument("--target-speed", type=float, default=0.6)
parser.add_argument("--target-height", type=float, default=0.5)
parser.add_argument("--max-thrust-to-weight", type=float, default=2.0)
parser.add_argument("--max-torque", type=float, default=0.02)
parser.add_argument(
    "--auto-trigger-delay",
    type=float,
    default=1.0,
    help="Simulation seconds before sending /trigger; use a negative value to disable",
)
parser.add_argument(
    "--max-steps", type=int, default=0, help="Stop after N physics steps; zero runs forever"
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg
from isaaclab.sensors.camera import Camera, CameraCfg
from isaaclab.sensors.camera.utils import create_pointcloud_from_depth
from isaaclab.sim import SimulationContext
from isaacsim.core.utils.extensions import enable_extension

from elastic_tracker_controller import geometric_controller
from isaaclab_assets import CRAZYFLIE_CFG


SENSOR_OFFSET = (0.08, 0.0, 0.0)


def _seconds_to_stamp(seconds: float):
    from builtin_interfaces.msg import Time

    whole_seconds = math.floor(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1.0e9))
    if nanoseconds == 1_000_000_000:
        whole_seconds += 1
        nanoseconds = 0
    return Time(sec=whole_seconds, nanosec=nanoseconds)


class RosInterface:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import PointCloud2

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self.node = rclpy.create_node("elastic_tracker_isaaclab")
        self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        self.odometry_pub = self.node.create_publisher(
            Odometry, "/isaac/uav/odometry", qos_profile_sensor_data
        )
        self.target_pub = self.node.create_publisher(
            Odometry, "/isaac/target/odometry", qos_profile_sensor_data
        )
        self.pointcloud_pub = self.node.create_publisher(
            PointCloud2, "/isaac/depth/points", qos_profile_sensor_data
        )
        self.trigger_pub = self.node.create_publisher(PoseStamped, "/trigger", 1)
        self.node.create_subscription(
            PoseStamped, "/desired_pose", self._pose_callback, 10
        )
        self.node.create_subscription(
            TwistStamped, "/desired_twist", self._twist_callback, 10
        )
        self.node.create_subscription(
            AccelStamped,
            "/desired_acceleration",
            self._acceleration_callback,
            10,
        )
        self.desired_position = None
        self.desired_velocity = (0.0, 0.0, 0.0)
        self.desired_acceleration = (0.0, 0.0, 0.0)
        self.desired_yaw = 0.0

    def _pose_callback(self, message):
        position = message.pose.position
        quaternion = message.pose.orientation
        self.desired_position = (position.x, position.y, position.z)
        self.desired_yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
        )

    def _twist_callback(self, message):
        linear = message.twist.linear
        self.desired_velocity = (linear.x, linear.y, linear.z)

    def _acceleration_callback(self, message):
        linear = message.accel.linear
        self.desired_acceleration = (linear.x, linear.y, linear.z)

    def spin_once(self):
        self._rclpy.spin_once(self.node, timeout_sec=0.0)

    def publish_clock(self, sim_time: float):
        from rosgraph_msgs.msg import Clock

        self.clock_pub.publish(Clock(clock=_seconds_to_stamp(sim_time)))

    def publish_trigger(self, sim_time: float):
        from geometry_msgs.msg import PoseStamped

        message = PoseStamped()
        message.header.stamp = _seconds_to_stamp(sim_time)
        message.header.frame_id = "world"
        message.pose.orientation.w = 1.0
        self.trigger_pub.publish(message)

    @staticmethod
    def _odometry_message(sim_time, position, quaternion_wxyz, linear_velocity, angular_velocity):
        from nav_msgs.msg import Odometry

        message = Odometry()
        message.header.stamp = _seconds_to_stamp(sim_time)
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

    def publish_odometry(self, sim_time, robot):
        message = self._odometry_message(
            sim_time,
            robot.data.root_pos_w[0],
            robot.data.root_quat_w[0],
            robot.data.root_lin_vel_w[0],
            robot.data.root_ang_vel_w[0],
        )
        self.odometry_pub.publish(message)

    def publish_target(self, sim_time, position, velocity):
        message = self._odometry_message(
            sim_time,
            position,
            (1.0, 0.0, 0.0, 0.0),
            velocity,
            (0.0, 0.0, 0.0),
        )
        self.target_pub.publish(message)

    def publish_pointcloud(self, sim_time: float, points_w: np.ndarray):
        from sensor_msgs.msg import PointCloud2, PointField

        points_w = np.ascontiguousarray(points_w, dtype=np.float32)
        message = PointCloud2()
        message.header.stamp = _seconds_to_stamp(sim_time)
        message.header.frame_id = "world"
        message.height = 1
        message.width = points_w.shape[0]
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = points_w.tobytes()
        message.is_dense = True
        self.pointcloud_pub.publish(message)

    def close(self):
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


def design_scene():
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=2500.0)
    light_cfg.func("/World/Light", light_cfg)

    obstacle_positions = (
        (1.5, 1.5, 1.0),
        (-1.5, 1.5, 1.0),
        (1.5, -1.5, 1.0),
        (-1.5, -1.5, 1.0),
        (3.5, 0.0, 1.0),
        (-3.5, 0.0, 1.0),
    )
    for index, position in enumerate(obstacle_positions):
        obstacle_cfg = sim_utils.CuboidCfg(
            size=(0.5, 0.5, 2.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.25, 0.3, 0.35)
            ),
        )
        obstacle_cfg.func(f"/World/Obstacle_{index}", obstacle_cfg, translation=position)

    robot_cfg = CRAZYFLIE_CFG.replace(prim_path="/World/Crazyflie")
    robot = Articulation(robot_cfg)
    camera_cfg = CameraCfg(
        prim_path="/World/Crazyflie/body/elastic_tracker_camera",
        update_period=1.0 / args_cli.pointcloud_rate,
        height=240,
        width=320,
        data_types=["distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=16.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, args_cli.sensor_range),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=SENSOR_OFFSET,
            rot=(0.5, -0.5, 0.5, -0.5),
            convention="ros",
        ),
        depth_clipping_behavior="none",
        update_latest_camera_pose=True,
    )
    camera = Camera(camera_cfg)

    target_cfg = RigidObjectCfg(
        prim_path="/World/Target",
        spawn=sim_utils.SphereCfg(
            radius=0.25,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=True, disable_gravity=True
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.1, 0.1)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(args_cli.target_radius, 0.0, args_cli.target_height)),
    )
    target = RigidObject(target_cfg)
    return robot, camera, target


def target_state(sim_time: float, device: str):
    angular_speed = args_cli.target_speed / args_cli.target_radius
    phase = angular_speed * sim_time
    position = torch.tensor(
        [[
            args_cli.target_radius * math.cos(phase),
            args_cli.target_radius * math.sin(phase),
            args_cli.target_height,
        ]],
        device=device,
        dtype=torch.float32,
    )
    velocity = torch.tensor(
        [[
            -args_cli.target_speed * math.sin(phase),
            args_cli.target_speed * math.cos(phase),
            0.0,
        ]],
        device=device,
        dtype=torch.float32,
    )
    return position, velocity


def main():
    if not enable_extension("isaacsim.ros2.bridge"):
        raise RuntimeError("Failed to enable Isaac Sim ROS 2 Bridge")

    sim = SimulationContext(sim_utils.SimulationCfg(dt=0.005, device=args_cli.device))
    sim.set_camera_view(eye=[7.0, 7.0, 5.0], target=[0.0, 0.0, 1.0])
    robot, camera, target = design_scene()
    sim.reset()
    robot.reset()
    target.reset()

    ros = RosInterface()
    body_ids = robot.find_bodies("body")[0]
    mass = robot.root_physx_view.get_masses()[0].sum()
    gravity = torch.tensor(sim.cfg.gravity, device=sim.device).norm().item()
    max_thrust = args_cli.max_thrust_to_weight * mass.item() * gravity
    sim_dt = sim.get_physics_dt()
    state_period = 1.0 / args_cli.state_rate
    pointcloud_period = 1.0 / args_cli.pointcloud_rate
    next_state_time = 0.0
    next_pointcloud_time = 0.0
    sim_time = 0.0
    step_count = 0
    trigger_sent = False
    hover_position = robot.data.root_pos_w.clone()

    print("[INFO] Isaac Lab bridge ready; publish /trigger after ROS inputs appear")
    try:
        while simulation_app.is_running():
            ros.spin_once()
            desired_position = (
                hover_position
                if ros.desired_position is None
                else torch.tensor([ros.desired_position], device=sim.device)
            )
            desired_velocity = torch.tensor([ros.desired_velocity], device=sim.device)
            desired_acceleration = torch.tensor(
                [ros.desired_acceleration], device=sim.device
            )
            desired_yaw = torch.tensor([ros.desired_yaw], device=sim.device)
            force_b, torque_b = geometric_controller(
                robot.data.root_pos_w,
                robot.data.root_quat_w,
                robot.data.root_lin_vel_w,
                robot.data.root_ang_vel_b,
                desired_position,
                desired_velocity,
                desired_acceleration,
                desired_yaw,
                mass,
                gravity=gravity,
                max_thrust=max_thrust,
                max_torque=args_cli.max_torque,
            )
            robot.permanent_wrench_composer.set_forces_and_torques(
                forces=force_b[:, None, :],
                torques=torque_b[:, None, :],
                body_ids=body_ids,
            )

            target_position, target_velocity = target_state(sim_time, sim.device)
            target_pose = torch.cat(
                (
                    target_position,
                    torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=sim.device),
                ),
                dim=-1,
            )
            target.write_root_pose_to_sim(target_pose)
            target.write_root_velocity_to_sim(
                torch.cat((target_velocity, torch.zeros_like(target_velocity)), dim=-1)
            )

            robot.write_data_to_sim()
            target.write_data_to_sim()
            sim.step()
            sim_time += sim_dt
            robot.update(sim_dt)
            target.update(sim_dt)
            camera.update(sim_dt)

            ros.publish_clock(sim_time)
            if sim_time + 1.0e-9 >= next_state_time:
                ros.publish_odometry(sim_time, robot)
                ros.publish_target(sim_time, target_position[0], target_velocity[0])
                next_state_time += state_period

            if sim_time + 1.0e-9 >= next_pointcloud_time:
                depth = camera.data.output.get("distance_to_image_plane")
                if depth is not None and depth.numel() > 0:
                    points_w = create_pointcloud_from_depth(
                        intrinsic_matrix=camera.data.intrinsic_matrices[0],
                        depth=depth[0],
                        position=camera.data.pos_w[0],
                        orientation=camera.data.quat_w_ros[0],
                        device=sim.device,
                    )
                    points_w = points_w[:: max(1, args_cli.pointcloud_stride)]
                    distances = torch.linalg.vector_norm(
                        points_w - camera.data.pos_w[0], dim=-1
                    )
                    points_w = points_w[distances <= args_cli.sensor_range]
                    ros.publish_pointcloud(
                        sim_time, points_w.detach().cpu().numpy()
                    )
                next_pointcloud_time += pointcloud_period
            if (
                not trigger_sent
                and args_cli.auto_trigger_delay >= 0.0
                and sim_time >= args_cli.auto_trigger_delay
                and ros.trigger_pub.get_subscription_count() > 0
            ):
                ros.publish_trigger(sim_time)
                trigger_sent = True
                print("[INFO] Published /trigger")
            step_count += 1
            if args_cli.max_steps > 0 and step_count >= args_cli.max_steps:
                break
    finally:
        ros.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
