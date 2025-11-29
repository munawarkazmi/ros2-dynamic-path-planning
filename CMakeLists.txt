cmake_minimum_required(VERSION 3.16)
project(ros2_astar_dstar_lite LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# === Planner library ===
add_library(${PROJECT_NAME}_planners SHARED
  src/astar_planner.cpp
  src/dstar_lite_planner.cpp
  src/global_planner_wrapper.cpp
)

# Modern include (replaces deprecated include_directories)
target_include_directories(${PROJECT_NAME}_planners PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_planners
  rclcpp
  nav2_costmap_2d
  nav2_core
  nav2_util
  tf2
  tf2_ros
  pluginlib
)

target_compile_definitions(${PROJECT_NAME}_planners PRIVATE "ROS2_ASTAR_DSTAR_LITE_BUILDING_DLL")

# === Benchmark node ===
add_executable(benchmark_node src/benchmark_node.cpp)
target_link_libraries(benchmark_node ${PROJECT_NAME}_planners)
ament_target_dependencies(benchmark_node rclcpp nav2_costmap_2d)

# === Plugin registration ===
pluginlib_export_plugin_description_file(nav2_core planner_plugins.xml)

# === Install ===
install(TARGETS ${PROJECT_NAME}_planners benchmark_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch maps rviz worlds models reports media
  DESTINATION share/${PROJECT_NAME}
)

install(FILES planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# === Testing ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
