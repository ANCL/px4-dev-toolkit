cmake_minimum_required(VERSION 3.8)
project(px4_mocap_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

get_filename_component(
  PX4_ENV_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  ABSOLUTE
)

add_library(mocap_conversion STATIC
  src/conversion.cpp
)

target_include_directories(mocap_conversion PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(mocap_conversion
  geometry_msgs
  px4_msgs
)

target_link_libraries(mocap_conversion
  Eigen3::Eigen
)

add_executable(px4_mocap_bridge
  src/px4_mocap_bridge.cpp
)

target_include_directories(px4_mocap_bridge PRIVATE
  "${PX4_ENV_ROOT}/config"
)

ament_target_dependencies(px4_mocap_bridge
  geometry_msgs
  px4_msgs
  rclcpp
)

target_link_libraries(px4_mocap_bridge
  mocap_conversion
)

install(
  TARGETS px4_mocap_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(test_conversion
    test/test_conversion.cpp
  )

  target_link_libraries(test_conversion
    mocap_conversion
  )

  add_test(
    NAME mocap_conversion
    COMMAND test_conversion
  )
endif()

ament_package()
