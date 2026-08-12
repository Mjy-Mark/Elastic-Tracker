cmake_minimum_required(VERSION 3.16)
project(elastic_tracker_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

set(LEGACY_TRAJ_OPT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/planning/traj_opt")
set(LEGACY_PLANNING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/planning/planning")
set(LEGACY_MAPPING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/mapping")
set(DECOMP_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/planning/DecompROS/decomp_ros_utils/include")

add_library(${PROJECT_NAME} SHARED
  "${LEGACY_MAPPING_DIR}/src/mapping.cc"
  "${LEGACY_TRAJ_OPT_DIR}/src/traj_opt.cc"
  "${LEGACY_TRAJ_OPT_DIR}/src/traj_opt_fake.cc"
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_compile_definitions(${PROJECT_NAME} PUBLIC ELASTIC_TRACKER_ROS2)
target_compile_options(${PROJECT_NAME} PRIVATE -O3 -Wall -Wextra)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${LEGACY_TRAJ_OPT_DIR}/include>
    $<BUILD_INTERFACE:${LEGACY_PLANNING_DIR}/include>
    $<BUILD_INTERFACE:${LEGACY_MAPPING_DIR}/include>
    $<BUILD_INTERFACE:${DECOMP_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

add_executable(traj_opt_smoke_test test/traj_opt_smoke_test.cpp)
target_link_libraries(traj_opt_smoke_test PRIVATE ${PROJECT_NAME})

add_executable(planning_core_smoke_test test/planning_core_smoke_test.cpp)
target_link_libraries(planning_core_smoke_test PRIVATE ${PROJECT_NAME})

add_executable(mapping_smoke_test test/mapping_smoke_test.cpp)
target_link_libraries(mapping_smoke_test PRIVATE ${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_test(NAME traj_opt_smoke_test COMMAND traj_opt_smoke_test)
  add_test(NAME planning_core_smoke_test COMMAND planning_core_smoke_test)
  add_test(NAME mapping_smoke_test COMMAND mapping_smoke_test)
endif()

install(
  TARGETS ${PROJECT_NAME} traj_opt_smoke_test planning_core_smoke_test
          mapping_smoke_test
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY "${LEGACY_TRAJ_OPT_DIR}/include/traj_opt" DESTINATION include)
install(DIRECTORY "${LEGACY_PLANNING_DIR}/include/env" DESTINATION include)
install(DIRECTORY "${LEGACY_PLANNING_DIR}/include/prediction" DESTINATION include)
install(DIRECTORY "${LEGACY_MAPPING_DIR}/include/mapping" DESTINATION include)
install(DIRECTORY "${DECOMP_INCLUDE_DIR}/decomp_basis" DESTINATION include)
install(DIRECTORY "${DECOMP_INCLUDE_DIR}/decomp_geometry" DESTINATION include)
install(DIRECTORY "${DECOMP_INCLUDE_DIR}/decomp_util" DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
