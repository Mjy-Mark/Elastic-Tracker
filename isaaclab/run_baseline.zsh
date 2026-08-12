#!/usr/bin/env zsh
set -eu

elastic_repo_root="/home/mark/mydata/ws/Elastic-Tracker-main"
elastic_isaaclab_root="/home/mark/mydata/ws/IsaacLab"
elastic_isaac_python="/home/mark/micromamba/envs/env_isaaclab/bin/python"
elastic_bridge_root="/home/mark/micromamba/envs/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge"

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${elastic_bridge_root}/jazzy/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

cd "${elastic_isaaclab_root}"
exec "${elastic_isaac_python}" \
  "${elastic_repo_root}/isaaclab/run_tracking_env_baseline.py" "$@"
