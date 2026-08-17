cmake_minimum_required(VERSION 3.16)
project(ros2_dynamic_path_planning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# The static core library is linked into the shared plugin library.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ROS-free planning core (also buildable standalone via core/Makefile).
add_library(${PROJECT_NAME}_core
  core/src/astar.cpp
  core/src/dstar_lite.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
  $<INSTALL_INTERFACE:include>
)

# Nav2 global planner plugins.
add_library(${PROJECT_NAME}_planners SHARED
  src/astar_global_planner.cpp
  src/dstar_global_planner.cpp
)
target_include_directories(${PROJECT_NAME}_planners PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_planners ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_planners
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  nav2_core
  nav2_util
  nav_msgs
  geometry_msgs
  pluginlib
  tf2
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core planner_plugins.xml)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_planners
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ core/include/
  DESTINATION include
)
install(DIRECTORY config launch maps rviz worlds models
  DESTINATION share/${PROJECT_NAME}
)
install(FILES planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Integration test for the plugin boundary: launches planner_server with one of
# these planners, drives it to active and asserts on the path that comes back
# out of ComputePathToPose. The algorithms themselves are covered by the unit
# suites under core/tests, which do not need ROS at all.
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Generous timeout: the server has to configure, activate and build a costmap
  # before it advertises the action.
  add_launch_test(test/test_planner_server_launch.py TIMEOUT 180)
endif()

ament_package()
