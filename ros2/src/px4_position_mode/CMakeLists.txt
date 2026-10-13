cmake_minimum_required(VERSION 3.8)
project(px4_position_mode)

# These executables are separate PX4 Position-mode workflows: one runs a
# native takeoff/hover/land reference flight, and one stages for independent
# Offboard takeover. Neither implements an external Offboard controller.

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_control_common REQUIRED)

add_executable(position_takeoff_hover
  src/takeoff_hover/node.cpp
  src/takeoff_hover/state.cpp
  src/takeoff_hover/joystick.cpp
  src/takeoff_hover/publication.cpp
  src/takeoff_hover/lifecycle.cpp
)

get_filename_component(
  PX4_ENV_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  ABSOLUTE
)

target_include_directories(position_takeoff_hover PRIVATE
  "${PX4_ENV_ROOT}/config"
)

ament_target_dependencies(position_takeoff_hover
  rclcpp
  px4_msgs
  px4_control_common
)

add_executable(offboard_takeoff_handoff
  src/offboard_staging/node.cpp
  src/offboard_staging/state.cpp
  src/offboard_staging/joystick.cpp
  src/offboard_staging/publication.cpp
  src/offboard_staging/lifecycle.cpp
)

target_include_directories(offboard_takeoff_handoff PRIVATE
  "${PX4_ENV_ROOT}/config"
)

ament_target_dependencies(offboard_takeoff_handoff
  rclcpp
  px4_msgs
  px4_control_common
)

install(
  TARGETS position_takeoff_hover offboard_takeoff_handoff
  DESTINATION lib/${PROJECT_NAME}
)

# Launch and recording configuration are runtime package resources. Installing
# them keeps `ros2 launch` independent of the source-tree working directory.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
