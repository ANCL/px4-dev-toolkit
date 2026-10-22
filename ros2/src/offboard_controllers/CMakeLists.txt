cmake_minimum_required(VERSION 3.8)
project(offboard_controllers)

# Build reusable control laws and ROS nodes separately.
# Trajectory mathematics is a toolkit-level library independent of ROS.

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

get_filename_component(
  PX4_ENV_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  ABSOLUTE
)

set(TRAJECTORY_GENERATOR_WITH_YAML ON)
add_subdirectory("${PX4_ENV_ROOT}/trajectory_generator" "${CMAKE_CURRENT_BINARY_DIR}/trajectory_generator")

add_executable(offboard_position
  src/offboard_position/node.cpp
  src/offboard_position/state.cpp
  src/offboard_position/publication.cpp
  src/offboard_position/lifecycle.cpp
)

# Stateless geometric control laws; called directly by the SE(3) runtime.
add_library(se3_control
  src/se3/control/translation.cpp
  src/se3/control/force_dynamics.cpp
  src/se3/control/attitude_reference.cpp
  src/se3/control/attitude_geometry.cpp
  src/se3/control/attitude_control.cpp
  src/se3/control/normalized_torque.cpp
  src/se3/control/physical_moment.cpp
)

add_library(se3_vehicle_state
  src/se3/runtime/vehicle_state.cpp
)

add_library(px4_wrench
  src/px4/wrench.cpp
)

add_library(px4_rate
  src/px4/rate_controller.cpp
)

add_library(physical_wrench_adapter
  src/se3/physical_wrench/adapter.cpp
  src/se3/physical_wrench/matrices.cpp
  src/se3/physical_wrench/actuators.cpp
)

add_library(vehicle_config
  src/se3/physical_wrench/config/loader.cpp
  src/se3/physical_wrench/config/parsing.cpp
)

add_executable(se3
  src/se3/node.cpp
  src/se3/runtime/config.cpp
  src/se3/runtime/lifecycle.cpp
  src/se3/runtime/control.cpp
  src/se3/runtime/publication.cpp
)

target_include_directories(offboard_position PRIVATE
  "${PX4_ENV_ROOT}/config"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(se3_control PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(se3_vehicle_state PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(se3_vehicle_state px4_msgs)

target_include_directories(px4_wrench PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(se3_vehicle_state px4_wrench)

target_include_directories(px4_rate PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(physical_wrench_adapter PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(vehicle_config PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(vehicle_config PUBLIC
  physical_wrench_adapter
  yaml-cpp
)

target_link_libraries(offboard_position
  trajectory_generator_yaml
)

target_include_directories(se3 PRIVATE
  "${PX4_ENV_ROOT}/config"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(offboard_position
  rclcpp
  px4_msgs
)

ament_target_dependencies(se3
  rclcpp
  geometry_msgs
  std_msgs
  px4_msgs
)

target_link_libraries(se3
  se3_control
  se3_vehicle_state
  trajectory_generator_yaml
  px4_rate
  px4_wrench
  physical_wrench_adapter
  vehicle_config
  yaml-cpp
)

install(
  TARGETS
    offboard_position
    se3
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    se3_control
    se3_vehicle_state
    trajectory_generator
    trajectory_generator_yaml
    px4_wrench
    px4_rate
    physical_wrench_adapter
    vehicle_config
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Launch and recording configuration are runtime package resources. Installing
# them keeps `ros2 launch` independent of the source-tree working directory.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Keep packaged, non-checkout launches compatible with existing paths.
install(
  FILES "${PX4_ENV_ROOT}/config/trajectories.yaml"
  DESTINATION share/${PROJECT_NAME}/config/trajectory
)

install(
  DIRECTORY "${PX4_ENV_ROOT}/config/vehicles/"
  DESTINATION share/${PROJECT_NAME}/config/vehicles
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY "${PX4_ENV_ROOT}/trajectory_generator/include/"
  DESTINATION include
)

ament_package()
