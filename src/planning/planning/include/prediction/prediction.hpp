#pragma once

#include <mapping/mapping.h>
#ifndef ELASTIC_TRACKER_ROS2
#include <ros/ros.h>
#endif

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <deque>
#include <iostream>
#include <queue>
#include <vector>

namespace prediction {

struct Node {
  Eigen::Vector3d p, v, a;
  double t;
  double score;
  double h;
  Node* parent = nullptr;
};
typedef Node* NodePtr;
class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) const {
    return lhs->score + lhs->h > rhs->score + rhs->h;
  }
};

struct PredictConfig {
  double duration = 3.0;
  double dt = 0.2;
  double acceleration_weight = 1.0;
  double max_velocity = 4.0;
  double max_acceleration = 3.0;
};

struct Predict {
 private:
  static constexpr int MAX_MEMORY = 1 << 22;
  // searching

  double dt;
  double pre_dur;
  double rho_a;
  double car_z, vmax, max_acceleration;
  mapping::OccGridMap map;
#ifdef ELASTIC_TRACKER_ROS2
  std::deque<Node> data;
#else
  NodePtr data[MAX_MEMORY];
#endif
  int stack_top;

  inline bool isValid(const Eigen::Vector3d& p, const Eigen::Vector3d& v) const {
    return (v.norm() < vmax) && (!map.isOccupied(p));
  }

 public:
#ifdef ELASTIC_TRACKER_ROS2
  inline explicit Predict(const PredictConfig& config)
      : dt(config.dt),
        pre_dur(config.duration),
        rho_a(config.acceleration_weight),
        vmax(config.max_velocity),
        max_acceleration(config.max_acceleration) {}
#else
  inline Predict(ros::NodeHandle& nh) {
    nh.getParam("tracking_dur", pre_dur);
    nh.getParam("tracking_dt", dt);
    nh.getParam("prediction/rho_a", rho_a);
    nh.getParam("prediction/vmax", vmax);
    nh.param("prediction/amax", max_acceleration, 3.0);
    for (int i = 0; i < MAX_MEMORY; ++i) {
      data[i] = new Node;
    }
  }
#endif
  inline void setMap(const mapping::OccGridMap& _map) {
    map = _map;
    // map.inflate_last();
  }

  inline bool predict(const Eigen::Vector3d& target_p,
                      const Eigen::Vector3d& target_v,
                      std::vector<Eigen::Vector3d>& target_predcit,
                      const double& max_time = 0.1) {
    auto score = [&](const NodePtr& ptr) -> double {
      return rho_a * ptr->a.norm();
    };
    Eigen::Vector3d end_p = target_p + target_v * pre_dur;
    auto calH = [&](const NodePtr& ptr) -> double {
      return 0.001 * (ptr->p - end_p).norm();
    };
    const auto t_start = std::chrono::steady_clock::now();
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;

    Eigen::Vector3d input(0, 0, 0);
    const double acceleration_step = std::max(max_acceleration, 1.0e-3);

    stack_top = 0;
#ifdef ELASTIC_TRACKER_ROS2
    data.clear();
    data.emplace_back();
    NodePtr curPtr = &data.back();
    ++stack_top;
#else
    NodePtr curPtr = data[stack_top++];
#endif
    curPtr->p = target_p;
    curPtr->v = target_v;
    curPtr->a.setZero();
    curPtr->parent = nullptr;
    curPtr->score = 0;
    curPtr->h = 0;
    curPtr->t = 0;
    double dt2_2 = dt * dt / 2;
    while (curPtr->t < pre_dur) {
      for (input.x() = -acceleration_step; input.x() <= acceleration_step;
           input.x() += acceleration_step)
        for (input.y() = -acceleration_step; input.y() <= acceleration_step;
             input.y() += acceleration_step) {
          Eigen::Vector3d p = curPtr->p + curPtr->v * dt + input * dt2_2;
          Eigen::Vector3d v = curPtr->v + input * dt;
          if (!isValid(p, v)) {
            continue;
          }
          if (stack_top == MAX_MEMORY) {
            std::cout << "[prediction] out of memory!" << std::endl;
            return false;
          }
          double t_cost = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - t_start)
                              .count();
          if (t_cost > max_time) {
            std::cout << "[prediction] too slow!" << std::endl;
            return false;
          }
#ifdef ELASTIC_TRACKER_ROS2
          data.emplace_back();
          NodePtr ptr = &data.back();
          ++stack_top;
#else
          NodePtr ptr = data[stack_top++];
#endif
          ptr->p = p;
          ptr->v = v;
          ptr->a = input;
          ptr->parent = curPtr;
          ptr->t = curPtr->t + dt;
          ptr->score = curPtr->score + score(ptr);
          ptr->h = calH(ptr);
          open_set.push(ptr);
          // std::cout << "open set push: " << state.transpose() << std::endl;
        }
      if (open_set.empty()) {
        std::cout << "[prediction] no way!" << std::endl;
        return false;
      }
      curPtr = open_set.top();
      open_set.pop();
    }
    target_predcit.clear();
    while (curPtr != nullptr) {
      target_predcit.push_back(curPtr->p);
      curPtr = curPtr->parent;
    }
    std::reverse(target_predcit.begin(), target_predcit.end());
    return true;
  }
};

}  // namespace prediction
