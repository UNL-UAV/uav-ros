cmake_minimum_required(VERSION 3.0.2)
project(Preflight)

find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(Preflight_node src/preflight_check.cpp)
target_link_libraries(Preflight_node ${catkin_LIBRARIES})
add_dependencies(Preflight_node ${catkin_EXPORTED_TARGETS})