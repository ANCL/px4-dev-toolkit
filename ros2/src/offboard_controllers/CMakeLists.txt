cmake_minimum_required(VERSION 3.8)
project(offboard_controllers)

# Build reusable trajectory, physical-model, and control-law libraries apart
# from ROS node executables. The SE(3) node selects handoffs; the library
# targets implement model/math operations without owning ROS process lifecycle.

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

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

add_library(trajectory
  src/trajectory/trajectory.cpp
  src/trajectory/sampling.cpp
  src/trajectory/config.cpp
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

get_filename_component(
  PX4_ENV_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  ABSOLUTE
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

target_include_directories(trajectory PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

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

target_link_libraries(trajectory PUBLIC
  yaml-cpp
)

target_link_libraries(offboard_position
  trajectory
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
  trajectory
  px4_rate
  px4_wrench
  physical_wrench_adapter
  vehicle_config
  yaml-cpp
)

if(BUILD_TESTING)
  add_executable(test_se3_control_laws
    test/se3/test_control_laws.cpp
  )

  target_link_libraries(test_se3_control_laws
    se3_control
  )

  add_test(
    NAME se3_control_laws
    COMMAND test_se3_control_laws
  )

  add_executable(test_se3_modules
    test/se3/test_modules.cpp
  )

  target_link_libraries(test_se3_modules
    se3_control
  )

  add_test(
    NAME se3_modules
    COMMAND test_se3_modules
  )

  add_executable(test_se3_vehicle_state
    test/se3/test_vehicle_state.cpp
  )

  target_link_libraries(test_se3_vehicle_state
    se3_vehicle_state
  )

  ament_target_dependencies(test_se3_vehicle_state px4_msgs)

  add_test(
    NAME se3_vehicle_state
    COMMAND test_se3_vehicle_state
  )

  add_executable(test_trajectory
    test/trajectory/test_trajectory.cpp
  )

  target_include_directories(test_trajectory PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  target_link_libraries(test_trajectory
    trajectory
  )

  target_compile_definitions(test_trajectory PRIVATE
    TRAJECTORY_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/trajectory/trajectories.yaml"
  )

  add_test(
    NAME trajectory_math
    COMMAND test_trajectory
  )

  add_executable(test_runtime_timing
    test/test_runtime_timing.cpp
  )

  target_include_directories(test_runtime_timing PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  add_test(
    NAME runtime_timing
    COMMAND test_runtime_timing
  )

  add_executable(test_px4_wrench
    test/px4/test_wrench.cpp
  )

  target_include_directories(test_px4_wrench PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  target_link_libraries(test_px4_wrench
    px4_wrench
  )

  add_test(
    NAME px4_wrench
    COMMAND test_px4_wrench
  )

  add_executable(test_px4_rate
    test/px4/test_rate_controller.cpp
  )

  target_link_libraries(test_px4_rate
    px4_rate
  )

  add_test(
    NAME px4_rate
    COMMAND test_px4_rate
  )

  add_executable(test_physical_wrench_adapter
    test/se3/test_physical_wrench_adapter.cpp
  )

  target_link_libraries(test_physical_wrench_adapter
    physical_wrench_adapter
  )

  add_test(
    NAME physical_wrench_adapter
    COMMAND test_physical_wrench_adapter
  )

  add_executable(test_vehicle_config
    test/se3/test_vehicle_config.cpp
  )

  target_link_libraries(test_vehicle_config
    vehicle_config
  )

  target_compile_definitions(test_vehicle_config PRIVATE
    VEHICLE_CONFIG_DIR="${PX4_ENV_ROOT}/config/vehicles"
  )

  add_test(
    NAME vehicle_config
    COMMAND test_vehicle_config
  )

  add_executable(test_offboard_lifecycle
    test/test_offboard_lifecycle.cpp
  )

  target_include_directories(test_offboard_lifecycle PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  add_test(
    NAME offboard_lifecycle
    COMMAND test_offboard_lifecycle
  )
endif()

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
    trajectory
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


install(
  DIRECTORY "${PX4_ENV_ROOT}/config/vehicles/"
  DESTINATION share/${PROJECT_NAME}/config/vehicles
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
