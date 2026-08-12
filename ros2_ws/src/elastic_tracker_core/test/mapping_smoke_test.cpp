#include <mapping/mapping.h>

#include <Eigen/Core>

#include <cstdint>
#include <iostream>
#include <vector>

int main() {
  mapping::OccGridMap map;
  map.inflate_size = 1;
  map.setup(0.5, Eigen::Vector3d(16.0, 16.0, 8.0), 10.0);

  const Eigen::Vector3d sensor(0.0, 0.0, 1.0);
  const Eigen::Vector3d obstacle(2.0, 0.0, 1.0);
  const Eigen::Vector3d same_voxel(0.1, 0.1, 1.1);
  const std::vector<Eigen::Vector3d> points{obstacle, same_voxel};
  for (int update = 0; update < 6; ++update) {
    map.updateMap(sensor, points);
  }
  if (!map.isOccupied(obstacle)) {
    std::cerr << "repeated point-cloud hits did not mark the obstacle"
              << std::endl;
    return 1;
  }
  if (map.isUnKnown(Eigen::Vector3d(1.0, 0.0, 1.0))) {
    std::cerr << "ray-cast free space remains unknown" << std::endl;
    return 2;
  }

  std::cout << "mapping obstacle and free-space updates passed" << std::endl;
  return 0;
}
