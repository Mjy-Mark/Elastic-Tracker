#include <elastic_tracker_msgs/msg/occ_map3d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace elastic_tracker_nodes {

class EmptyMapNode : public rclcpp::Node {
 public:
  EmptyMapNode() : Node("elastic_tracker_empty_map") {
    const double resolution = declare_parameter("resolution", 0.15);
    const double local_x = declare_parameter("local_x", 19.2);
    const double local_y = declare_parameter("local_y", 19.2);
    const double local_z = declare_parameter("local_z", 9.6);
    const double publish_rate =
        std::max(0.1, declare_parameter("empty_map_rate", 2.0));

    map_.header.frame_id = "world";
    map_.resolution = static_cast<float>(resolution);
    map_.size_x = cells(local_x, resolution);
    map_.size_y = cells(local_y, resolution);
    map_.size_z = cells(local_z, resolution);
    map_.offset_x = static_cast<int16_t>(-map_.size_x / 2);
    map_.offset_y = static_cast<int16_t>(-map_.size_y / 2);
    map_.offset_z = static_cast<int16_t>(-map_.size_z / 2);
    const std::size_t cell_count =
        static_cast<std::size_t>(map_.size_x) * map_.size_y * map_.size_z;
    map_.data.assign(cell_count, static_cast<int8_t>(-1));

    publisher_ = create_publisher<elastic_tracker_msgs::msg::OccMap3d>(
        "occupancy_map", rclcpp::QoS(1).reliable().transient_local());
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          center_on(message->pose.pose.position.x,
                    message->pose.pose.position.y,
                    message->pose.pose.position.z);
        });
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate),
        std::bind(&EmptyMapNode::publish, this));
    publish();
    RCLCPP_INFO(get_logger(),
                "Publishing an obstacle-free %d x %d x %d occupancy map",
                map_.size_x, map_.size_y, map_.size_z);
  }

 private:
  static int16_t cells(const double length, const double resolution) {
    const double requested = std::max(2.0, length / resolution);
    const int exponent = static_cast<int>(std::floor(std::log2(requested)));
    return static_cast<int16_t>(1 << std::clamp(exponent, 1, 14));
  }

  static int16_t centered_offset(const double position,
                                 const double resolution,
                                 const int16_t size) {
    const long center = static_cast<long>(std::floor(position / resolution));
    return static_cast<int16_t>(std::clamp<long>(
        center - size / 2, INT16_MIN, INT16_MAX - size + 1));
  }

  void center_on(const double x, const double y, const double z) {
    map_.offset_x = centered_offset(x, map_.resolution, map_.size_x);
    map_.offset_y = centered_offset(y, map_.resolution, map_.size_y);
    map_.offset_z = centered_offset(z, map_.resolution, map_.size_z);
  }

  void publish() {
    map_.header.stamp = now();
    publisher_->publish(map_);
  }

  elastic_tracker_msgs::msg::OccMap3d map_;
  rclcpp::Publisher<elastic_tracker_msgs::msg::OccMap3d>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace elastic_tracker_nodes

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<elastic_tracker_nodes::EmptyMapNode>());
  rclcpp::shutdown();
  return 0;
}
