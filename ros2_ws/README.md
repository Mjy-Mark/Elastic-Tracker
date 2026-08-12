# Elastic-Tracker ROS 2 baseline

This workspace ports the planning portion of Elastic-Tracker to ROS 2 Jazzy.
It is intended to run beside `Tracking-Direct-v2`: Isaac Lab supplies UAV
truth odometry and consumes the planner references, while this workspace
supplies target prediction, visible-region planning, safe-flight corridors and
polynomial trajectory optimization.  The legacy ROS 1 implementation under
`../src` is retained only as the upstream algorithm source.

## Runtime contract

```text
converted figure-8 NPZ (50 Hz)
        |
target_trajectory_replay.py -> target Odometry (truth)
        |
Tracking-Direct / Isaac Lab -> UAV Odometry + /clock
        |
empty_map_node -> all-free OccMap3d
        |
planner_interface_node -> PolyTraj
        |
trajectory_sampler_node (100 Hz) -> p/v/a/yaw reference
        |
Tracking-Direct baseline controller -> its existing motor model
```

There is no camera, point cloud, mapper or EKF in this benchmark path.  The
target odometry is the task-reference truth position used by TrackingEnv, not
the car rigid-body root.  The free map keeps the planning pipeline active
without inventing obstacles.

## Workspace structure

```text
ros2_ws/src/
  elastic_tracker_msgs/       ROS 2 OccMap3d, PolyTraj and command messages
  elastic_tracker_core/       map, prediction, visibility and MINCO/L-BFGS core
  elastic_tracker_nodes/      map, planner, sampler and NPZ truth replay node
  elastic_tracker_bringup/    launch files and figure-8 parameters
```

## Build

```bash
source /opt/ros/jazzy/setup.zsh
cd /home/mark/mydata/ws/Elastic-Tracker-main/ros2_ws
CCACHE_DISABLE=1 colcon build --symlink-install
source install/setup.zsh
```

The prior source-level core tests can be run separately after a build:

```bash
ROS_LOG_DIR=/tmp/elastic_tracker_ros_logs colcon test
colcon test-result --verbose
```

## Figure-8 launch

`figure8_truth_baseline.launch.py` loads the recorded trajectory directly from
the converted NPZ, publishes target truth at 50 Hz, waits for the planner
heartbeat, then sends one trigger.  It expects the external Tracking-Direct
bridge to publish `/clock` and `/tracking/uav/odometry` and to subscribe to the
three reference outputs.

```bash
source /opt/ros/jazzy/setup.zsh
source /home/mark/mydata/ws/Elastic-Tracker-main/ros2_ws/install/setup.zsh
ros2 launch elastic_tracker_bringup figure8_truth_baseline.launch.py
```

Default topic mapping:

| Purpose | Topic |
| --- | --- |
| UAV truth input | `/tracking/uav/odometry` |
| Target truth output | `/tracking/target/odometry` |
| planner p/v/a outputs | `/tracking/elastic_tracker/desired_pose`, `/desired_twist`, `/desired_acceleration` |
| trigger / heartbeat | `/tracking/elastic_tracker/trigger`, `/tracking/elastic_tracker/heartbeat` |

All names are launch arguments.  For example, use a different NPZ or bridge
topic without editing code:

```bash
ros2 launch elastic_tracker_bringup figure8_truth_baseline.launch.py \
  trajectory_file:=/absolute/path/to/trajectory_50hz.npz \
  odometry_topic:=/your/uav/odometry
```

## Figure-8-derived planner limits

The default trajectory is
`polo_mocap_20260806_213950_figure8_50hz/trajectory_50hz.npz`.  Its motion
interval is 3.94--69.44 s, its task-point envelope is 10.674 x 6.583 m, and
its horizontal speed is 4.538 m/s maximum (4.292 m/s P99).  The original
shared 3.0 m/s limit was therefore infeasible.

`planner.yaml` separates target-prediction and UAV-trajectory limits:

| Parameter | Value | Reason |
| --- | ---: | --- |
| `target_prediction_max_velocity` | 5.0 m/s | exceeds the measured 4.538 m/s peak |
| `target_prediction_max_acceleration` | 6.0 m/s² | covers the 5.81 m/s² P99 normal acceleration |
| `trajectory_max_velocity` | 5.5 m/s | gives the UAV speed margin over the target |
| `trajectory_max_acceleration` | 8.0 m/s² | gives the optimizer curvature/catch-up margin |

The data also contains one short 39.15 m/s² reversal near 39.30 s.  It is
preserved exactly in playback but is deliberately not treated as a normal
vehicle bound; report this segment separately as a stress case.

`tracking_distance` (currently 2.5 m) and `target_height_offset` (currently
1.0 m) are **evaluation-geometry choices**, not properties inferred from the
trajectory.  Keep them for an upstream-native baseline, or set them only after
declaring the same standoff/altitude protocol for the RL comparator.  If the
Isaac Lab evaluator uses the upstream 2.5 m standoff, its
`trajectory_eval_bound_y_m` must be at least 6.0 m because the target reaches
|y| = 3.291 m; the existing x bound of 10.0 m is sufficient.
