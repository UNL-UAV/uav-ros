cmake_minimum_required(VERSION 3.16)
project(uav_ros)

find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  std_msgs
  mavros_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(Core 
	src/core/Application.cpp 
	src/core/PX4.cpp
)
target_precompile_headers(Core PUBLIC include/pch.hpp)
target_include_directories(Core PUBLIC include/core)

add_executable(safety_test 
	src/Preflight/safety_test.cpp 
	src/Preflight/safety_check.cpp
)
target_include_directories(safety_test PUBLIC include/Preflight)
target_link_libraries(safety_test Core ${catkin_LIBRARIES})
add_dependencies(safety_test Core ${catkin_EXPORTED_TARGETS})
target_precompile_headers(safety_test PUBLIC include/pch.hpp)

add_executable(newtakeoff
	src/newtakeoff/NewTakeoffApp.cpp
)

add_executable(waypoint_provider
	src/waypoint_provider/waypoint_provider.cpp src/waypoint_provider/file_utility.cpp
)
target_link_libraries(waypoint_provider ${catkin_LIBRARIES})
target_include_directories(waypoint_provider PUBLIC include/waypoint_provider)
target_precompile_headers(waypoint_provider PUBLIC include/pch.hpp)

target_include_directories(newtakeoff PUBLIC include/newtakeoff)
target_link_libraries(newtakeoff Core ${catkin_LIBRARIES})
add_dependencies(newtakeoff Core ${catkin_EXPORTED_TARGETS})
target_precompile_headers(newtakeoff PUBLIC include/pch.hpp)
