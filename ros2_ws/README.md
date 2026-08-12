# Elastic-Tracker ROS 2 / Isaac Lab baseline

This workspace is a ROS 2 Jazzy port of the planning baseline from
Elastic-Tracker. The original ROS 1 source remains under `../src`; only the
parts required by the baseline planner are reused.

You do **not** need to port the original simulator to ROS 2 first. Keep the
planner as an external ROS 2 C++ process and let Isaac Lab provide simulation,
sensors, vehicle state, target state, and control through Isaac Sim's ROS 2
Bridge.

## Architecture

```text
Isaac Lab / Isaac Sim
  ROS 2 Bridge: /clock, odometry, target odometry, world XYZ point cloud
       |
       v
pointcloud_mapper_node -> OccMap3d
       |
       v
planner_interface_node
  prediction -> visible path -> visible regions -> SFC -> MINCO/L-BFGS
       |
       v
PolyTraj -> trajectory_sampler_node (100 Hz)
       |
       +-> PositionCommand (full p/v/a/jerk/yaw)
       +-> PoseStamped + TwistStamped + AccelStamped
       |
       v
Isaac Lab controller / vehicle action
```

The repository also contains a ready-to-run single-environment adapter:

```text
../isaaclab/
  elastic_tracker_controller.py       batched SE(3) position controller
  run_elastic_tracker_baseline.py     Isaac scene, ROS bridge and sensors
  run_baseline.zsh                    isolated Isaac Sim/Jazzy launcher
  test_elastic_tracker_controller.py  simulator-free controller tests
```

## Workspace structure

```text
ros2_ws/src/
  elastic_tracker_msgs/       ROS 2 OccMap3d, PolyTraj and command messages
  elastic_tracker_core/       ROS-independent map, prediction, search,
                              corridor and MINCO/L-BFGS code
  elastic_tracker_nodes/      mapper, planner pipeline and trajectory sampler
  elastic_tracker_bringup/    launch file and baseline parameters
```

The legacy ROS 1 simulator, CUDA depth renderer, mock map, nodelets, and RViz
plugins are intentionally not part of this port. Isaac Lab replaces them.

## Build and test

```bash
source /opt/ros/jazzy/setup.zsh
cd /home/mark/mydata/ws/Elastic-Tracker-main/ros2_ws
CCACHE_DISABLE=1 colcon build --symlink-install
source install/setup.zsh
ROS_LOG_DIR=/tmp/elastic_tracker_ros_logs colcon test
colcon test-result --verbose
```

`CCACHE_DISABLE=1` is only required when the configured ccache directory is
not writable.

## Run the complete baseline

Build the ROS 2 workspace once as described above. Then use two terminals.

Terminal 1 runs the external planner:

```bash
source /opt/ros/jazzy/setup.zsh
source /home/mark/mydata/ws/Elastic-Tracker-main/ros2_ws/install/setup.zsh
ros2 launch elastic_tracker_bringup baseline.launch.py \
  odometry_topic:=/isaac/uav/odometry \
  target_odometry_topic:=/isaac/target/odometry \
  pointcloud_topic:=/isaac/depth/points
```

Terminal 2 runs Isaac Lab without sourcing system ROS 2 into its Python 3.11
environment:

```bash
cd /home/mark/mydata/ws/Elastic-Tracker-main
zsh isaaclab/run_baseline.zsh
```

Use `--headless` for headless simulation. The adapter automatically publishes
`/trigger` one simulation second after the planner subscribes. To trigger again
manually:

```bash
ros2 topic pub --once /trigger geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}}"
```

Use `ros2 launch elastic_tracker_bringup baseline.launch.py --show-args` to
list every configurable topic.

The supplied Isaac scene uses one Crazyflie, a circular kinematic target, six
static obstacles and a forward depth camera. It publishes:

- `/clock`
- `/isaac/uav/odometry`
- `/isaac/target/odometry`
- `/isaac/depth/points`

The depth image is unprojected and transformed using the camera's current pose,
so the `PointCloud2` already contains world-frame XYZ points. The camera is
`0.08 m` ahead of the vehicle body; the same offset is set in `planner.yaml`.
The adapter subscribes to `/desired_pose`, `/desired_twist` and
`/desired_acceleration`, then applies body-frame thrust and torque through an
SE(3) controller.

For a short integration smoke test:

```bash
zsh isaaclab/run_baseline.zsh --headless --max-steps 200
```

The simulator-free controller test is faster:

```bash
cd /home/mark/mydata/ws/Elastic-Tracker-main/isaaclab
/home/mark/micromamba/envs/env_isaaclab/bin/python \
  -m unittest -v test_elastic_tracker_controller.py
```

## Isaac Lab contract

Configure Isaac Sim's ROS 2 Bridge to publish:

- `/clock` (`rosgraph_msgs/msg/Clock`), because `use_sim_time` is enabled.
- UAV state (`nav_msgs/msg/Odometry`) in the `world` ENU frame.
- Target state (`nav_msgs/msg/Odometry`) in the same frame.
- Depth/LiDAR points (`sensor_msgs/msg/PointCloud2`) with float32 `x`, `y`, and
  `z` fields already transformed into `world`.

The mapper currently rejects point clouds whose `header.frame_id` is not
`world`. Transform camera-frame or lidar-frame points inside the Isaac graph
before publishing them. Set `sensor_offset_x/y/z` to the sensor origin in the
UAV body frame so ray casting starts at the correct point. The mapper uses
`target_odometry` to clear the moving target from the occupancy grid; tune
`target_mask_xy` and `target_mask_z` to match the target asset.

Subscribe from the Isaac controller to one of these outputs:

- `position_command` (`elastic_tracker_msgs/msg/PositionCommand`) for the full
  position, velocity, acceleration, jerk, and yaw reference.
- `desired_pose`, `desired_twist`, and `desired_acceleration` for standard ROS 2
  messages that Isaac Sim's bridge can consume without loading custom message
  definitions.

Use metres, seconds, radians, ENU axes, and ROS quaternion ordering. The
controller should track position/velocity/acceleration/yaw references and
apply thrust/body-rate or rotor-force actions to the Isaac Lab vehicle.

Do not import the system ROS Jazzy `rclpy` (Python 3.12) into Isaac Lab's
Python 3.11 environment. The provided launcher clears the system ROS Python
paths and selects the Jazzy `rclpy` bundled with Isaac Sim's ROS 2 Bridge. The
ROS 2 C++ nodes remain in the system ROS environment.

## Controller tuning

The default controller gains are in `../isaaclab/elastic_tracker_controller.py`.
The most important runtime limits are:

- `--max-thrust-to-weight 2.0`
- `--max-torque 0.02`
- `--state-rate 100`
- `--pointcloud-rate 10`
- `--pointcloud-stride 2`

Tune the gains and limits against the final vehicle asset. The bundled
Crazyflie uses a direct body wrench for baseline reproduction. If the final
experiment requires motor dynamics, keep the same controller output and add a
control-allocation step using the multirotor asset's allocation matrix.

## Important parameters

Planner and mapper defaults live in
`elastic_tracker_bringup/config/planner.yaml`. The most important values are:

- `tracking_duration`, `tracking_dt`, `tracking_distance`
- `max_velocity`, `max_acceleration`
- `corridor_width`, `corridor_clearance`, `visibility_clearance`
- `resolution`, `local_x`, `local_y`, `local_z`, `sensor_range`

The ring-buffer map dimensions are powers of two internally. The supplied
defaults (`19.2 x 19.2 x 9.6 m` at `0.15 m`) produce `128 x 128 x 64` cells.
