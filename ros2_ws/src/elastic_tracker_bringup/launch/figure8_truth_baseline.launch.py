"""Run Elastic-Tracker against the recorded figure-8 target truth trajectory."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_TRAJECTORY = (
    "/home/mark/mydata/ws/tracking/logs/trajectory_conversion/"
    "polo_mocap_20260806_213950_figure8_50hz/trajectory_50hz.npz"
)


def generate_launch_description():
    share = Path(get_package_share_directory("elastic_tracker_bringup"))
    baseline_launch = share / "launch" / "baseline.launch.py"
    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "trajectory_file", default_value=DEFAULT_TRAJECTORY
        ),
        DeclareLaunchArgument(
            "odometry_topic", default_value="/tracking/uav/odometry"
        ),
        DeclareLaunchArgument(
            "target_odometry_topic", default_value="/tracking/target/odometry"
        ),
        DeclareLaunchArgument(
            "trigger_topic", default_value="/tracking/elastic_tracker/trigger"
        ),
        DeclareLaunchArgument(
            "heartbeat_topic", default_value="/tracking/elastic_tracker/heartbeat"
        ),
        DeclareLaunchArgument(
            "occupancy_map_topic",
            default_value="/tracking/elastic_tracker/occupancy_map",
        ),
        DeclareLaunchArgument(
            "trajectory_topic", default_value="/tracking/elastic_tracker/trajectory"
        ),
        DeclareLaunchArgument(
            "position_command_topic",
            default_value="/tracking/elastic_tracker/position_command",
        ),
        DeclareLaunchArgument(
            "desired_pose_topic", default_value="/tracking/elastic_tracker/desired_pose"
        ),
        DeclareLaunchArgument(
            "desired_twist_topic", default_value="/tracking/elastic_tracker/desired_twist"
        ),
        DeclareLaunchArgument(
            "desired_acceleration_topic",
            default_value="/tracking/elastic_tracker/desired_acceleration",
        ),
    ]
    forwarded = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "use_empty_map": "true",
        "odometry_topic": LaunchConfiguration("odometry_topic"),
        "target_odometry_topic": LaunchConfiguration("target_odometry_topic"),
        "trigger_topic": LaunchConfiguration("trigger_topic"),
        "heartbeat_topic": LaunchConfiguration("heartbeat_topic"),
        "occupancy_map_topic": LaunchConfiguration("occupancy_map_topic"),
        "trajectory_topic": LaunchConfiguration("trajectory_topic"),
        "position_command_topic": LaunchConfiguration("position_command_topic"),
        "desired_pose_topic": LaunchConfiguration("desired_pose_topic"),
        "desired_twist_topic": LaunchConfiguration("desired_twist_topic"),
        "desired_acceleration_topic": LaunchConfiguration("desired_acceleration_topic"),
    }
    return LaunchDescription(
        arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(baseline_launch)),
                launch_arguments=forwarded.items(),
            ),
            Node(
                package="elastic_tracker_nodes",
                executable="target_trajectory_replay.py",
                name="figure8_target_truth",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "trajectory_file": LaunchConfiguration("trajectory_file"),
                        "frame_id": "world",
                        "playback_rate": 1.0,
                        "publish_rate": 50.0,
                        "auto_trigger": True,
                        "trigger_delay_s": 0.5,
                    }
                ],
                remappings=[
                    ("target_odometry", LaunchConfiguration("target_odometry_topic")),
                    ("trigger", LaunchConfiguration("trigger_topic")),
                    ("heartbeat", LaunchConfiguration("heartbeat_topic")),
                ],
            ),
        ]
    )
