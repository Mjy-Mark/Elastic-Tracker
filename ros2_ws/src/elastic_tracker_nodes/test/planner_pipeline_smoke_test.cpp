#include <elastic_tracker_nodes/planner_pipeline.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

int main() {
  elastic_tracker_nodes::PlannerConfig config;
  config.tracking_duration = 1.0;
  elastic_tracker_nodes::PlannerPipeline pipeline(config);

  elastic_tracker_msgs::msg::OccMap3d map;
  map.resolution = 0.5F;
  map.size_x = 64;
  map.size_y = 64;
  map.size_z = 32;
  map.offset_x = -32;
  map.offset_y = -32;
  map.offset_z = -8;
  map.data.assign(
      static_cast<std::size_t>(map.size_x) * map.size_y * map.size_z,
      static_cast<int8_t>(-1));

  Eigen::Matrix3d initial_state = Eigen::Matrix3d::Zero();
  initial_state.col(0) << -2.5, 0.0, 1.0;
  elastic_tracker_nodes::PlanResult result;
  std::string error;
  if (!pipeline.plan(initial_state, Eigen::Vector3d(0.0, 0.0, 0.0),
                     Eigen::Vector3d(0.2, 0.0, 0.0), map, result, error)) {
    std::cerr << error << std::endl;
    return 1;
  }
  if (result.message.hover || result.message.order != 5 ||
      result.message.duration.empty() ||
      result.message.coef_x.size() != result.message.duration.size() * 6 ||
      result.trajectory.getPieceNum() !=
          static_cast<int>(result.message.duration.size())) {
    std::cerr << "invalid polynomial trajectory message" << std::endl;
    return 2;
  }
  if ((result.trajectory.getPos(0.0) - initial_state.col(0)).norm() > 1e-6) {
    std::cerr << "trajectory does not start at the initial state" << std::endl;
    return 3;
  }

  std::cout << "trajectory_message_pieces=" << result.message.duration.size()
            << " duration=" << result.trajectory.getTotalDuration()
            << std::endl;
  return 0;
}
