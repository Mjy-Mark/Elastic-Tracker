from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory("elastic_tracker_bringup"))
        / "config"
        / "planner.yaml"
    )
    topics = {
        "odometry": LaunchConfiguration("odometry_topic"),
        "target_odometry": LaunchConfiguration("target_odometry_topic"),
        "pointcloud": LaunchConfiguration("pointcloud_topic"),
        "occupancy_map": LaunchConfiguration("occupancy_map_topic"),
        "trigger": LaunchConfiguration("trigger_topic"),
        "trajectory": LaunchConfiguration("trajectory_topic"),
        "position_command": LaunchConfiguration("position_command_topic"),
        "desired_pose": LaunchConfiguration("desired_pose_topic"),
        "desired_twist": LaunchConfiguration("desired_twist_topic"),
        "desired_acceleration": LaunchConfiguration("desired_acceleration_topic"),
        "heartbeat": LaunchConfiguration("heartbeat_topic"),
    }
    arguments = [
        DeclareLaunchArgument("odometry_topic", default_value="odometry"),
        DeclareLaunchArgument(
            "target_odometry_topic", default_value="target_odometry"
        ),
        DeclareLaunchArgument("pointcloud_topic", default_value="pointcloud"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_empty_map", default_value="true"),
        DeclareLaunchArgument(
            "occupancy_map_topic", default_value="occupancy_map"
        ),
        DeclareLaunchArgument("trigger_topic", default_value="trigger"),
        DeclareLaunchArgument("trajectory_topic", default_value="trajectory"),
        DeclareLaunchArgument(
            "position_command_topic", default_value="position_command"
        ),
        DeclareLaunchArgument("desired_pose_topic", default_value="desired_pose"),
        DeclareLaunchArgument(
            "desired_twist_topic", default_value="desired_twist"
        ),
        DeclareLaunchArgument(
            "desired_acceleration_topic", default_value="desired_acceleration"
        ),
        DeclareLaunchArgument("heartbeat_topic", default_value="heartbeat"),
    ]
    nodes = [
        Node(
            package="elastic_tracker_nodes",
            executable="pointcloud_mapper_node",
            name="elastic_tracker_mapper",
            output="screen",
            condition=UnlessCondition(LaunchConfiguration("use_empty_map")),
            parameters=[str(config), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            remappings=[
                ("odometry", topics["odometry"]),
                ("target_odometry", topics["target_odometry"]),
                ("pointcloud", topics["pointcloud"]),
                ("occupancy_map", topics["occupancy_map"]),
            ],
        ),
        Node(
            package="elastic_tracker_nodes",
            executable="empty_map_node",
            name="elastic_tracker_empty_map",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_empty_map")),
            parameters=[str(config), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            remappings=[
                ("odometry", topics["odometry"]),
                ("occupancy_map", topics["occupancy_map"]),
            ],
        ),
        Node(
            package="elastic_tracker_nodes",
            executable="planner_interface_node",
            name="elastic_tracker_planner",
            output="screen",
            parameters=[str(config), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            remappings=[
                ("odometry", topics["odometry"]),
                ("target_odometry", topics["target_odometry"]),
                ("occupancy_map", topics["occupancy_map"]),
                ("trigger", topics["trigger"]),
                ("trajectory", topics["trajectory"]),
                ("heartbeat", topics["heartbeat"]),
            ],
        ),
        Node(
            package="elastic_tracker_nodes",
            executable="trajectory_sampler_node",
            name="trajectory_sampler",
            output="screen",
            parameters=[str(config), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            remappings=[
                ("trajectory", topics["trajectory"]),
                ("position_command", topics["position_command"]),
                ("desired_pose", topics["desired_pose"]),
                ("desired_twist", topics["desired_twist"]),
                ("desired_acceleration", topics["desired_acceleration"]),
                ("trigger", topics["trigger"]),
            ],
        ),
    ]
    return LaunchDescription(arguments + nodes)
