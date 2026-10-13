cmake_minimum_required(VERSION 3.8)
project(px4_control_common)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_python_install_package(${PROJECT_NAME})

include(CTest)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  add_test(
    NAME recording_helpers
    COMMAND "${Python3_EXECUTABLE}" -m unittest discover
      -s "${CMAKE_CURRENT_SOURCE_DIR}/test" -p "test_*.py"
  )
endif()

ament_package()
