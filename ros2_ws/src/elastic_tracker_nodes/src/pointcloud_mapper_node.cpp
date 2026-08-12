#include <elastic_tracker_msgs/msg/occ_map3d.hpp>
#include <mapping/mapping.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace elastic_tracker_nodes {

class PointcloudMapperNode : public rclcpp::Node {
 public:
  PointcloudMapperNode() : Node("elastic_tracker_mapper") {
    const double resolution = declare_parameter("resolution", 0.15);
    const double local_x = declare_parameter("local_x", 19.2);
    const double local_y = declare_parameter("local_y", 19.2);
    const double local_z = declare_parameter("local_z", 9.6);
    const double sensor_range = declare_parameter("sensor_range", 10.0);
    sensor_offset_ << declare_parameter("sensor_offset_x", 0.0),
        declare_parameter("sensor_offset_y", 0.0),
        declare_parameter("sensor_offset_z", 0.0);
    mask_target_ = declare_parameter("mask_target", true);
    target_mask_xy_ = declare_parameter("target_mask_xy", 0.5);
    target_mask_z_ = declare_parameter("target_mask_z", 1.0);
    map_.inflate_size = static_cast<int>(std::max<int64_t>(
        0, declare_parameter("inflate_size", 1)));
    map_.setup(resolution, Eigen::Vector3d(local_x, local_y, local_z),
               sensor_range);
    map_.setupP(-199, 220, 62, 62, 139, -199);

    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr message) {
          odometry_ = std::move(message);
        });
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud", rclcpp::SensorDataQoS(),
        std::bind(&PointcloudMapperNode::pointcloudCallback, this,
                  std::placeholders::_1));
    target_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "target_odometry", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr message) {
          target_ = std::move(message);
        });
    map_pub_ = create_publisher<elastic_tracker_msgs::msg::OccMap3d>(
        "occupancy_map", rclcpp::QoS(1).reliable());
    RCLCPP_INFO(
        get_logger(),
        "Point-cloud mapper expects XYZ points in the world ENU frame");
  }

 private:
  void pointcloudCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    if (!odometry_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for odometry before mapping");
      return;
    }
    if (message->header.frame_id != "world") {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring point cloud in frame '%s'; publish world-frame points",
          message->header.frame_id.c_str());
      return;
    }

    std::vector<Eigen::Vector3d> points;
    points.reserve(static_cast<std::size_t>(message->width) * message->height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          points.emplace_back(*x, *y, *z);
        }
      }
    } catch (const std::runtime_error& exception) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Invalid XYZ point cloud: %s", exception.what());
      return;
    }

    const Eigen::Vector3d body_position(
        odometry_->pose.pose.position.x, odometry_->pose.pose.position.y,
        odometry_->pose.pose.position.z);
    const Eigen::Quaterniond body_orientation(
        odometry_->pose.pose.orientation.w,
        odometry_->pose.pose.orientation.x,
        odometry_->pose.pose.orientation.y,
        odometry_->pose.pose.orientation.z);
    const Eigen::Vector3d sensor_position =
        body_position + body_orientation * sensor_offset_;
    map_.updateMap(sensor_position, points);
    if (mask_target_ && target_) {
      Eigen::Vector3d lower(
          target_->pose.pose.position.x - target_mask_xy_,
          target_->pose.pose.position.y - target_mask_xy_,
          target_->pose.pose.position.z - target_mask_z_);
      Eigen::Vector3d upper(
          target_->pose.pose.position.x + target_mask_xy_,
          target_->pose.pose.position.y + target_mask_xy_,
          target_->pose.pose.position.z + target_mask_z_);
      map_.setFree(lower, upper);
    }

    elastic_tracker_msgs::msg::OccMap3d map_message;
    map_message.header.stamp = message->header.stamp;
    map_message.header.frame_id = "world";
    map_.to_msg(map_message);
    map_pub_->publish(map_message);
  }

  mapping::OccGridMap map_;
  bool mask_target_ = true;
  double target_mask_xy_ = 0.5;
  double target_mask_z_ = 1.0;
  Eigen::Vector3d sensor_offset_ = Eigen::Vector3d::Zero();
  nav_msgs::msg::Odometry::SharedPtr odometry_;
  nav_msgs::msg::Odometry::SharedPtr target_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
  rclcpp::Publisher<elastic_tracker_msgs::msg::OccMap3d>::SharedPtr map_pub_;
};

}  // namespace elastic_tracker_nodes

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<elastic_tracker_nodes::PointcloudMapperNode>());
  rclcpp::shutdown();
  return 0;
}
