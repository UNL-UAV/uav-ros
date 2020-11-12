cmake_minimum_required(VERSION 3.16)
project(uav_ros)

find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  std_msgs
  mavros_msgs
)

include_directories(include include/Preflight ${catkin_INCLUDE_DIRS})

add_library(core src/core/Application.cpp src/core/PX4.cpp)
target_precompile_headers(core PUBLIC include/pch.hpp)

add_executable(safety_test src/Preflight/safety_test.cpp src/Preflight/safety_check.cpp)
target_link_libraries(safety_test core ${catkin_LIBRARIES})
add_dependencies(safety_test core ${catkin_EXPORTED_TARGETS})
