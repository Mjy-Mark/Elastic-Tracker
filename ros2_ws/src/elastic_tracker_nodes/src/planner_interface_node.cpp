#include <elastic_tracker_msgs/msg/occ_map3d.hpp>
#include <elastic_tracker_msgs/msg/poly_traj.hpp>
#include <elastic_tracker_nodes/planner_pipeline.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace elastic_tracker_nodes {

class PlannerInterfaceNode : public rclcpp::Node {
 public:
  PlannerInterfaceNode()
      : Node("elastic_tracker_planner"),
        trajectory_start_delay_(
            declare_parameter("trajectory_start_delay", 0.03)),
        trajectory_handoff_tolerance_(
            declare_parameter("trajectory_handoff_tolerance", 1.0)),
        trigger_resets_trajectory_(
            declare_parameter("trigger_resets_trajectory", true)) {
    PlannerConfig config;
    const int plan_hz = static_cast<int>(
        std::max<int64_t>(1, declare_parameter("plan_hz", 20)));
    config.tracking_duration =
        declare_parameter("tracking_duration", config.tracking_duration);
    config.tracking_dt = declare_parameter("tracking_dt", config.tracking_dt);
    config.tracking_distance =
        declare_parameter("tracking_distance", config.tracking_distance);
    config.tracking_tolerance =
        declare_parameter("tracking_tolerance", config.tracking_tolerance);
    config.target_height_offset =
        declare_parameter("target_height_offset", config.target_height_offset);
    config.prediction_acceleration_weight = declare_parameter(
        "prediction_acceleration_weight",
        config.prediction_acceleration_weight);
    config.target_prediction_max_velocity = declare_parameter(
        "target_prediction_max_velocity",
        config.target_prediction_max_velocity);
    config.target_prediction_acceleration_step = declare_parameter(
        "target_prediction_acceleration_step",
        config.target_prediction_acceleration_step);
    config.trajectory_max_velocity = declare_parameter(
        "trajectory_max_velocity", config.trajectory_max_velocity);
    config.trajectory_max_acceleration = declare_parameter(
        "trajectory_max_acceleration", config.trajectory_max_acceleration);
    config.corridor_width =
        declare_parameter("corridor_width", config.corridor_width);
    config.visibility_clearance =
        declare_parameter("visibility_clearance", config.visibility_clearance);
    config.quadrature_resolution = declare_parameter(
        "quadrature_resolution", config.quadrature_resolution);
    config.time_weight =
        declare_parameter("time_weight", config.time_weight);
    config.corridor_weight =
        declare_parameter("corridor_weight", config.corridor_weight);
    config.velocity_weight =
        declare_parameter("velocity_weight", config.velocity_weight);
    config.acceleration_weight =
        declare_parameter("acceleration_weight", config.acceleration_weight);
    config.tracking_weight =
        declare_parameter("tracking_weight", config.tracking_weight);
    config.visibility_weight =
        declare_parameter("visibility_weight", config.visibility_weight);
    config.corridor_clearance =
        declare_parameter("corridor_clearance", config.corridor_clearance);
    pipeline_ = std::make_unique<PlannerPipeline>(config);

    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr message) {
          odometry_ = std::move(message);
        });
    target_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "target_odometry", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr message) {
          target_ = std::move(message);
        });
    map_sub_ = create_subscription<elastic_tracker_msgs::msg::OccMap3d>(
        "occupancy_map", rclcpp::QoS(1).reliable(),
        [this](elastic_tracker_msgs::msg::OccMap3d::SharedPtr message) {
          occupancy_map_ = std::move(message);
        });
    trigger_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "trigger", rclcpp::QoS(1).reliable(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr) {
          triggered_ = true;
          if (trigger_resets_trajectory_) {
            has_previous_trajectory_ = false;
          }
          RCLCPP_INFO(get_logger(), "Tracking trigger received");
        });
    heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>("heartbeat", 10);
    trajectory_pub_ =
        create_publisher<elastic_tracker_msgs::msg::PolyTraj>(
            "trajectory", rclcpp::QoS(1).reliable());
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / plan_hz),
        std::bind(&PlannerInterfaceNode::tick, this));

    RCLCPP_INFO(get_logger(), "Elastic-Tracker ROS 2 planning pipeline is ready");
  }

 private:
  Eigen::Matrix3d initialState(const rclcpp::Time& plan_start) const {
    Eigen::Matrix3d state = Eigen::Matrix3d::Zero();
    const double trajectory_time =
        (plan_start - previous_start_time_).seconds();
    if (has_previous_trajectory_ && trajectory_time >= 0.0 &&
        trajectory_time <= previous_trajectory_.getTotalDuration()) {
      const Eigen::Vector3d trajectory_position =
          previous_trajectory_.getPos(trajectory_time);
      const Eigen::Vector3d odometry_position(
          odometry_->pose.pose.position.x, odometry_->pose.pose.position.y,
          odometry_->pose.pose.position.z);
      if ((trajectory_position - odometry_position).norm() <=
          trajectory_handoff_tolerance_) {
        state.col(0) = trajectory_position;
        state.col(1) = previous_trajectory_.getVel(trajectory_time);
        state.col(2) = previous_trajectory_.getAcc(trajectory_time);
        return state;
      }
    }

    state.col(0) << odometry_->pose.pose.position.x,
        odometry_->pose.pose.position.y, odometry_->pose.pose.position.z;
    state.col(1) << odometry_->twist.twist.linear.x,
        odometry_->twist.twist.linear.y, odometry_->twist.twist.linear.z;
    return state;
  }

  void tick() {
    heartbeat_pub_->publish(std_msgs::msg::Empty());
    if (!triggered_) {
      return;
    }
    if (!odometry_ || !target_ || !occupancy_map_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for odometry, target_odometry, and occupancy_map");
      return;
    }

    const rclcpp::Time plan_start =
        now() + rclcpp::Duration::from_seconds(trajectory_start_delay_);
    Eigen::Vector3d target_position(
        target_->pose.pose.position.x, target_->pose.pose.position.y,
        target_->pose.pose.position.z);
    Eigen::Vector3d target_velocity(
        target_->twist.twist.linear.x, target_->twist.twist.linear.y,
        target_->twist.twist.linear.z);

    PlanResult result;
    std::string error;
    if (!pipeline_->plan(initialState(plan_start), target_position,
                         target_velocity, *occupancy_map_, result, error)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Planning failed: %s",
          error.c_str());
      return;
    }

    result.message.start_time = plan_start;
    result.message.traj_id = trajectory_id_++;
    trajectory_pub_->publish(result.message);
    previous_trajectory_ = result.trajectory;
    previous_start_time_ = plan_start;
    has_previous_trajectory_ = true;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Published trajectory with %zu pieces",
        result.message.duration.size());
  }

  bool triggered_ = false;
  bool has_previous_trajectory_ = false;
  int trajectory_id_ = 0;
  double trajectory_start_delay_;
  double trajectory_handoff_tolerance_;
  bool trigger_resets_trajectory_;
  rclcpp::Time previous_start_time_{0, 0, RCL_ROS_TIME};
  Trajectory previous_trajectory_;
  std::unique_ptr<PlannerPipeline> pipeline_;
  nav_msgs::msg::Odometry::SharedPtr odometry_;
  nav_msgs::msg::Odometry::SharedPtr target_;
  elastic_tracker_msgs::msg::OccMap3d::SharedPtr occupancy_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
  rclcpp::Subscription<elastic_tracker_msgs::msg::OccMap3d>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<elastic_tracker_msgs::msg::PolyTraj>::SharedPtr
      trajectory_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace elastic_tracker_nodes

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<elastic_tracker_nodes::PlannerInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
