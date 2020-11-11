cmake_minimum_required(VERSION 3.0.2)
project(uav_ros)

find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  std_msgs
  mavros_msgs
)

include_directories(include include/Preflight ${catkin_INCLUDE_DIRS})

add_executable(safety_test src/Preflight/safety_test.cpp src/Preflight/safety_check.cpp)
target_link_libraries(safety_test ${catkin_LIBRARIES})
add_dependencies(safety_test ${catkin_EXPORTED_TARGETS})
