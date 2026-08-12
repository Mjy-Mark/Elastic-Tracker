#include <elastic_tracker_nodes/planner_pipeline.hpp>

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace elastic_tracker_nodes {

namespace {

env::EnvConfig makeEnvConfig(const PlannerConfig& config) {
  env::EnvConfig env_config;
  env_config.tracking_distance = config.tracking_distance;
  env_config.tracking_tolerance = config.tracking_tolerance;
  env_config.visibility_clearance = config.visibility_clearance;
  return env_config;
}

prediction::PredictConfig makePredictConfig(const PlannerConfig& config) {
  prediction::PredictConfig predict_config;
  predict_config.duration = config.tracking_duration;
  predict_config.dt = config.tracking_dt;
  predict_config.acceleration_weight = config.prediction_acceleration_weight;
  predict_config.max_velocity = config.target_prediction_max_velocity;
  predict_config.max_acceleration = config.target_prediction_max_acceleration;
  return predict_config;
}

traj_opt::TrajOptConfig makeTrajOptConfig(const PlannerConfig& config) {
  traj_opt::TrajOptConfig optimizer_config;
  optimizer_config.quadrature_resolution = config.quadrature_resolution;
  optimizer_config.max_velocity = config.trajectory_max_velocity;
  optimizer_config.max_acceleration = config.trajectory_max_acceleration;
  optimizer_config.time_weight = config.time_weight;
  optimizer_config.corridor_weight = config.corridor_weight;
  optimizer_config.velocity_weight = config.velocity_weight;
  optimizer_config.acceleration_weight = config.acceleration_weight;
  optimizer_config.tracking_weight = config.tracking_weight;
  optimizer_config.visibility_weight = config.visibility_weight;
  optimizer_config.visibility_clearance = config.visibility_clearance;
  optimizer_config.tracking_duration = config.tracking_duration;
  optimizer_config.tracking_distance = config.tracking_distance;
  optimizer_config.tracking_dt = config.tracking_dt;
  optimizer_config.corridor_clearance = config.corridor_clearance;
  optimizer_config.tracking_tolerance = config.tracking_tolerance;
  return optimizer_config;
}

bool isPowerOfTwo(const int value) {
  return value > 0 && (value & (value - 1)) == 0;
}

}  // namespace

PlannerPipeline::PlannerPipeline(const PlannerConfig& config)
    : config_(config),
      map_(std::make_shared<mapping::OccGridMap>()),
      environment_(makeEnvConfig(config), map_),
      predictor_(makePredictConfig(config)),
      optimizer_(makeTrajOptConfig(config)) {}

bool PlannerPipeline::plan(
    const Eigen::Matrix3d& initial_state,
    const Eigen::Vector3d& target_position,
    const Eigen::Vector3d& target_velocity,
    const elastic_tracker_msgs::msg::OccMap3d& map_message,
    PlanResult& result,
    std::string& error) {
  error.clear();
  if (!initial_state.allFinite() || !target_position.allFinite() ||
      !target_velocity.allFinite()) {
    error = "planner state contains non-finite values";
    return false;
  }
  if (!validMapMessage(map_message)) {
    error = "occupancy map dimensions, resolution, or data length are invalid";
    return false;
  }

  map_->from_msg(map_message);
  predictor_.setMap(*map_);

  Eigen::Vector3d elevated_target = target_position;
  elevated_target.z() += config_.target_height_offset;
  std::vector<Eigen::Vector3d> target_prediction;
  if (!predictor_.predict(
          elevated_target, target_velocity, target_prediction)) {
    error = "target prediction failed";
    return false;
  }
  if (target_prediction.size() < 2) {
    error = "target prediction contains fewer than two states";
    return false;
  }

  std::vector<Eigen::Vector3d> waypoints;
  std::vector<Eigen::Vector3d> path;
  const Eigen::Vector3d initial_position = initial_state.col(0);
  if (!environment_.findVisiblePath(
          initial_position, target_prediction, waypoints, path)) {
    error = "visible path search failed";
    return false;
  }

  target_prediction.pop_back();
  waypoints.pop_back();
  if (target_prediction.empty() || waypoints.empty()) {
    error = "visible path horizon is empty";
    return false;
  }

  std::vector<Eigen::Vector3d> visible_points;
  std::vector<double> visible_angles;
  environment_.generate_visible_regions(
      target_prediction, waypoints, visible_points, visible_angles);

  waypoints.insert(waypoints.begin(), initial_position);
  environment_.pts2path(waypoints, path);
  if (path.size() < 2) {
    error = "path contains fewer than two points";
    return false;
  }

  std::vector<Eigen::MatrixXd> corridors;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corridor_segments;
  environment_.generateSFC(
      path, config_.corridor_width, corridors, corridor_segments);
  if (corridors.empty()) {
    error = "safe flight corridor generation failed";
    return false;
  }

  Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
  final_state.col(0) = path.back();
  final_state.col(1) = target_velocity;
  Trajectory trajectory;
  if (!optimizer_.generate_traj(
          initial_state, final_state, target_prediction, visible_points,
          visible_angles, corridors, trajectory)) {
    error = "trajectory optimization failed";
    return false;
  }
  if (!collisionFree(trajectory)) {
    error = "optimized trajectory collides with the occupancy map";
    return false;
  }

  result.trajectory = trajectory;
  result.message = encode(trajectory, elevated_target, initial_position);
  return true;
}

bool PlannerPipeline::validMapMessage(
    const elastic_tracker_msgs::msg::OccMap3d& message) const {
  if (!std::isfinite(message.resolution) || message.resolution <= 0.0F ||
      !isPowerOfTwo(message.size_x) || !isPowerOfTwo(message.size_y) ||
      !isPowerOfTwo(message.size_z)) {
    return false;
  }
  const std::size_t expected_size =
      static_cast<std::size_t>(message.size_x) *
      static_cast<std::size_t>(message.size_y) *
      static_cast<std::size_t>(message.size_z);
  return message.data.size() == expected_size;
}

bool PlannerPipeline::collisionFree(const Trajectory& trajectory) const {
  const double duration = trajectory.getTotalDuration();
  for (double time = 0.0; time <= duration; time += 0.01) {
    if (map_->isOccupied(trajectory.getPos(time))) {
      return false;
    }
  }
  return !map_->isOccupied(trajectory.getPos(duration));
}

elastic_tracker_msgs::msg::PolyTraj PlannerPipeline::encode(
    const Trajectory& trajectory,
    const Eigen::Vector3d& target_position,
    const Eigen::Vector3d& initial_position) const {
  elastic_tracker_msgs::msg::PolyTraj message;
  message.hover = false;
  message.order = 5;
  const Eigen::Vector3d target_delta = target_position - initial_position;
  message.yaw = static_cast<float>(
      std::atan2(target_delta.y(), target_delta.x()));

  const Eigen::VectorXd durations = trajectory.getDurations();
  const int piece_count = trajectory.getPieceNum();
  message.duration.resize(piece_count);
  message.coef_x.resize(6 * piece_count);
  message.coef_y.resize(6 * piece_count);
  message.coef_z.resize(6 * piece_count);
  for (int piece = 0; piece < piece_count; ++piece) {
    message.duration[piece] = static_cast<float>(durations(piece));
    const CoefficientMat& coefficients = trajectory[piece].getCoeffMat();
    for (int coefficient = 0; coefficient < 6; ++coefficient) {
      const int index = piece * 6 + coefficient;
      message.coef_x[index] =
          static_cast<float>(coefficients(0, coefficient));
      message.coef_y[index] =
          static_cast<float>(coefficients(1, coefficient));
      message.coef_z[index] =
          static_cast<float>(coefficients(2, coefficient));
    }
  }
  return message;
}

}  // namespace elastic_tracker_nodes
