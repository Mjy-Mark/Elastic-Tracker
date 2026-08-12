#include <env/env.hpp>
#include <mapping/mapping.h>
#include <prediction/prediction.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

namespace {

struct MapMessage {
  float resolution;
  int16_t size_x;
  int16_t size_y;
  int16_t size_z;
  int16_t offset_x;
  int16_t offset_y;
  int16_t offset_z;
  std::vector<int8_t> data;
};

MapMessage makeEmptyMap() {
  MapMessage message;
  message.resolution = 0.5F;
  message.size_x = 64;
  message.size_y = 64;
  message.size_z = 32;
  message.offset_x = -32;
  message.offset_y = -32;
  message.offset_z = -8;
  message.data.assign(
      static_cast<std::size_t>(message.size_x) * message.size_y * message.size_z,
      -1);
  return message;
}

}  // namespace

int main() {
  auto map = std::make_shared<mapping::OccGridMap>();
  const MapMessage message = makeEmptyMap();
  map->from_msg(message);
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  if (!map->isInMap(origin) || map->isOccupied(origin) ||
      map->isUnKnown(origin)) {
    std::cerr << "empty map conversion failed" << std::endl;
    return 1;
  }

  prediction::PredictConfig predict_config;
  predict_config.duration = 1.0;
  predict_config.dt = 0.2;
  prediction::Predict predictor(predict_config);
  predictor.setMap(*map);

  std::vector<Eigen::Vector3d> target_prediction;
  if (!predictor.predict(Eigen::Vector3d(0.0, 0.0, 1.0),
                         Eigen::Vector3d(0.2, 0.0, 0.0),
                         target_prediction)) {
    std::cerr << "target prediction failed" << std::endl;
    return 2;
  }
  if (target_prediction.size() < 2) {
    std::cerr << "target prediction is too short" << std::endl;
    return 3;
  }

  env::EnvConfig env_config;
  env_config.tracking_distance = 2.5;
  env_config.tracking_tolerance = 0.3;
  env::Env environment(env_config, map);

  std::vector<Eigen::Vector3d> waypoints;
  std::vector<Eigen::Vector3d> visible_path;
  const Eigen::Vector3d start(-2.5, 0.0, 1.0);
  if (!environment.findVisiblePath(
          start, target_prediction, waypoints, visible_path)) {
    std::cerr << "visible path search failed" << std::endl;
    return 4;
  }
  if (waypoints.size() != target_prediction.size() || visible_path.size() < 2) {
    std::cerr << "visible path output is inconsistent" << std::endl;
    return 5;
  }

  target_prediction.pop_back();
  waypoints.pop_back();
  std::vector<Eigen::Vector3d> visible_points;
  std::vector<double> visible_angles;
  environment.generate_visible_regions(
      target_prediction, waypoints, visible_points, visible_angles);
  if (visible_points.size() != target_prediction.size() ||
      visible_angles.size() != target_prediction.size()) {
    std::cerr << "visible region generation failed" << std::endl;
    return 6;
  }

  waypoints.insert(waypoints.begin(), start);
  std::vector<Eigen::Vector3d> path;
  environment.pts2path(waypoints, path);
  std::vector<Eigen::MatrixXd> corridors;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corridor_segments;
  environment.generateSFC(path, 2.0, corridors, corridor_segments);
  if (corridors.empty() || corridor_segments.empty()) {
    std::cerr << "safe flight corridor generation failed" << std::endl;
    return 7;
  }

  std::cout << "prediction_points=" << target_prediction.size()
            << " path_points=" << path.size()
            << " corridors=" << corridors.size() << std::endl;
  return 0;
}
