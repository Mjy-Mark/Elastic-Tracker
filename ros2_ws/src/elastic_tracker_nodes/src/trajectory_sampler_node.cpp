#include <elastic_tracker_msgs/msg/poly_traj.hpp>
#include <elastic_tracker_msgs/msg/position_command.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_opt/poly_traj_utils.hpp>

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace elastic_tracker_nodes {

class TrajectorySamplerNode : public rclcpp::Node {
 public:
  TrajectorySamplerNode() : Node("trajectory_sampler") {
    const double command_rate = declare_parameter("command_rate", 100.0);
    trajectory_sub_ = create_subscription<elastic_tracker_msgs::msg::PolyTraj>(
        "trajectory", rclcpp::QoS(1).reliable(),
        [this](elastic_tracker_msgs::msg::PolyTraj::SharedPtr message) {
          std::scoped_lock lock(mutex_);
          current_message_ = std::move(message);
        });
    command_pub_ = create_publisher<elastic_tracker_msgs::msg::PositionCommand>(
        "position_command", rclcpp::QoS(10).reliable());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "desired_pose", rclcpp::QoS(10).reliable());
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "desired_twist", rclcpp::QoS(10).reliable());
    acceleration_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>(
        "desired_acceleration", rclcpp::QoS(10).reliable());
    trigger_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "trigger", rclcpp::QoS(1).reliable(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr) {
          std::scoped_lock lock(mutex_);
          current_message_.reset();
        });
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / command_rate),
        std::bind(&TrajectorySamplerNode::sample, this));
  }

 private:
  static std::optional<Trajectory> decode(
      const elastic_tracker_msgs::msg::PolyTraj& message) {
    if (message.hover || message.order != 5 || message.duration.empty()) {
      return std::nullopt;
    }
    const std::size_t piece_count = message.duration.size();
    const std::size_t coefficient_count = piece_count * 6;
    if (message.coef_x.size() != coefficient_count ||
        message.coef_y.size() != coefficient_count ||
        message.coef_z.size() != coefficient_count) {
      return std::nullopt;
    }

    std::vector<double> durations(piece_count);
    std::vector<CoefficientMat> coefficients(piece_count);
    for (std::size_t piece = 0; piece < piece_count; ++piece) {
      durations[piece] = message.duration[piece];
      for (std::size_t coefficient = 0; coefficient < 6; ++coefficient) {
        const std::size_t index = piece * 6 + coefficient;
        coefficients[piece](0, coefficient) = message.coef_x[index];
        coefficients[piece](1, coefficient) = message.coef_y[index];
        coefficients[piece](2, coefficient) = message.coef_z[index];
      }
    }
    return Trajectory(durations, coefficients);
  }

  void sample() {
    elastic_tracker_msgs::msg::PolyTraj::SharedPtr message;
    {
      std::scoped_lock lock(mutex_);
      message = current_message_;
    }
    if (!message) {
      return;
    }

    elastic_tracker_msgs::msg::PositionCommand command;
    command.header.stamp = now();
    command.header.frame_id = "world";
    command.trajectory_id = message->traj_id;
    command.yaw = message->yaw;

    if (message->hover) {
      if (message->hover_p.size() != 3) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                              "Invalid hover trajectory");
        return;
      }
      command.position.x = message->hover_p[0];
      command.position.y = message->hover_p[1];
      command.position.z = message->hover_p[2];
      publish(command);
      return;
    }

    const auto trajectory = decode(*message);
    if (!trajectory) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Invalid polynomial trajectory");
      return;
    }
    const rclcpp::Time start_time(message->start_time);
    const double elapsed = (now() - start_time).seconds();
    if (elapsed < 0.0 || elapsed > trajectory->getTotalDuration()) {
      return;
    }

    const Eigen::Vector3d position = trajectory->getPos(elapsed);
    const Eigen::Vector3d velocity = trajectory->getVel(elapsed);
    const Eigen::Vector3d acceleration = trajectory->getAcc(elapsed);
    const Eigen::Vector3d jerk = trajectory->getJer(elapsed);
    command.position.x = position.x();
    command.position.y = position.y();
    command.position.z = position.z();
    command.velocity.x = velocity.x();
    command.velocity.y = velocity.y();
    command.velocity.z = velocity.z();
    command.acceleration.x = acceleration.x();
    command.acceleration.y = acceleration.y();
    command.acceleration.z = acceleration.z();
    command.jerk.x = jerk.x();
    command.jerk.y = jerk.y();
    command.jerk.z = jerk.z();
    publish(command);
  }

  void publish(const elastic_tracker_msgs::msg::PositionCommand& command) {
    command_pub_->publish(command);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = command.header;
    pose.pose.position = command.position;
    pose.pose.orientation.z = std::sin(command.yaw * 0.5);
    pose.pose.orientation.w = std::cos(command.yaw * 0.5);
    pose_pub_->publish(pose);

    geometry_msgs::msg::TwistStamped twist;
    twist.header = command.header;
    twist.twist.linear = command.velocity;
    twist.twist.angular.z = command.yaw_dot;
    twist_pub_->publish(twist);

    geometry_msgs::msg::AccelStamped acceleration;
    acceleration.header = command.header;
    acceleration.accel.linear = command.acceleration;
    acceleration_pub_->publish(acceleration);
  }

  std::mutex mutex_;
  elastic_tracker_msgs::msg::PolyTraj::SharedPtr current_message_;
  rclcpp::Subscription<elastic_tracker_msgs::msg::PolyTraj>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<elastic_tracker_msgs::msg::PositionCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr acceleration_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace elastic_tracker_nodes

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<elastic_tracker_nodes::TrajectorySamplerNode>());
  rclcpp::shutdown();
  return 0;
}
