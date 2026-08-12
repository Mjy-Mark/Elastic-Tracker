#pragma once

#include <elastic_tracker_msgs/msg/occ_map3d.hpp>
#include <elastic_tracker_msgs/msg/poly_traj.hpp>
#include <env/env.hpp>
#include <prediction/prediction.hpp>
#include <traj_opt/traj_opt.h>

#include <Eigen/Core>

#include <memory>
#include <string>

namespace elastic_tracker_nodes {

struct PlannerConfig {
  double tracking_duration = 3.0;
  double tracking_dt = 0.2;
  double tracking_distance = 2.5;
  double tracking_tolerance = 0.3;
  double target_height_offset = 1.0;
  double prediction_acceleration_weight = 1.0;
  double max_velocity = 3.0;
  double max_acceleration = 6.0;
  double corridor_width = 2.0;
  double visibility_clearance = 0.8;
  int quadrature_resolution = 8;
  double time_weight = 100.0;
  double corridor_weight = 10000.0;
  double velocity_weight = 1000.0;
  double acceleration_weight = 1000.0;
  double tracking_weight = 1000.0;
  double visibility_weight = 10000.0;
  double corridor_clearance = 0.2;
};

struct PlanResult {
  Trajectory trajectory;
  elastic_tracker_msgs::msg::PolyTraj message;
};

class PlannerPipeline {
 public:
  explicit PlannerPipeline(const PlannerConfig& config);

  bool plan(const Eigen::Matrix3d& initial_state,
            const Eigen::Vector3d& target_position,
            const Eigen::Vector3d& target_velocity,
            const elastic_tracker_msgs::msg::OccMap3d& map_message,
            PlanResult& result,
            std::string& error);

 private:
  bool validMapMessage(
      const elastic_tracker_msgs::msg::OccMap3d& message) const;
  bool collisionFree(const Trajectory& trajectory) const;
  elastic_tracker_msgs::msg::PolyTraj encode(
      const Trajectory& trajectory,
      const Eigen::Vector3d& target_position,
      const Eigen::Vector3d& initial_position) const;

  PlannerConfig config_;
  std::shared_ptr<mapping::OccGridMap> map_;
  env::Env environment_;
  prediction::Predict predictor_;
  traj_opt::TrajOpt optimizer_;
};

}  // namespace elastic_tracker_nodes
