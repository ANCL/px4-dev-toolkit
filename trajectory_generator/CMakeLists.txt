cmake_minimum_required(VERSION 3.16)
project(trajectory_generator LANGUAGES CXX)

# ROS- and PX4-independent analytic sampler. No YAML dependencies.
add_library(trajectory_generator STATIC
  src/analytic/trajectory.cpp
  src/analytic/sampling.cpp
)
target_compile_features(trajectory_generator PUBLIC cxx_std_17)
target_include_directories(trajectory_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Host-side configuration reader. Embedded consumers use only the core library.
option(TRAJECTORY_GENERATOR_WITH_YAML "Build host-side YAML trajectory loader" OFF)
if(TRAJECTORY_GENERATOR_WITH_YAML)
  find_package(yaml-cpp REQUIRED)
  add_library(trajectory_generator_yaml STATIC src/config/yaml.cpp)
  target_link_libraries(trajectory_generator_yaml PUBLIC trajectory_generator yaml-cpp)
endif()
