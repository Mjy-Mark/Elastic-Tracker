cmake_minimum_required(VERSION 3.16)
project(elastic_tracker_nodes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(elastic_tracker_core REQUIRED)
find_package(elastic_tracker_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(elastic_tracker_planner SHARED src/planner_pipeline.cpp)
target_include_directories(elastic_tracker_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(elastic_tracker_planner elastic_tracker_core::elastic_tracker_core)
ament_target_dependencies(elastic_tracker_planner elastic_tracker_msgs nav_msgs)

add_executable(trajectory_sampler_node src/trajectory_sampler_node.cpp)
target_link_libraries(trajectory_sampler_node elastic_tracker_core::elastic_tracker_core)
ament_target_dependencies(
  trajectory_sampler_node elastic_tracker_msgs geometry_msgs rclcpp
)

add_executable(planner_interface_node src/planner_interface_node.cpp)
target_link_libraries(planner_interface_node elastic_tracker_planner)
ament_target_dependencies(
  planner_interface_node
  elastic_tracker_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)

add_executable(planner_pipeline_smoke_test test/planner_pipeline_smoke_test.cpp)
target_link_libraries(planner_pipeline_smoke_test elastic_tracker_planner)
ament_target_dependencies(planner_pipeline_smoke_test elastic_tracker_msgs nav_msgs)

add_executable(pointcloud_mapper_node src/pointcloud_mapper_node.cpp)
target_link_libraries(pointcloud_mapper_node elastic_tracker_core::elastic_tracker_core)
ament_target_dependencies(
  pointcloud_mapper_node elastic_tracker_msgs nav_msgs rclcpp sensor_msgs
)

add_executable(empty_map_node src/empty_map_node.cpp)
ament_target_dependencies(empty_map_node elastic_tracker_msgs nav_msgs rclcpp)

if(BUILD_TESTING)
  enable_testing()
  add_test(NAME planner_pipeline_smoke_test COMMAND planner_pipeline_smoke_test)
endif()

install(
  TARGETS elastic_tracker_planner trajectory_sampler_node
          planner_interface_node planner_pipeline_smoke_test
          pointcloud_mapper_node empty_map_node
  EXPORT export_elastic_tracker_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/target_trajectory_replay.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_elastic_tracker_planner HAS_LIBRARY_TARGET)

ament_package()
