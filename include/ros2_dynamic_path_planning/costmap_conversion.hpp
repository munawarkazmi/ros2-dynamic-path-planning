// include/ros2_dynamic_path_planning/costmap_conversion.hpp
//
// Conversions between nav2 costmaps/paths and the ROS-free core types.
// Costmap cost values (0..255) carry the same semantics as
// planning::Grid, so cell costs copy through unchanged.
#pragma once

#include <algorithm>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "std_msgs/msg/header.hpp"

#include "planning/grid.hpp"

namespace ros2_dynamic_path_planning {

inline planning::Grid gridFromCostmap(const nav2_costmap_2d::Costmap2D& costmap) {
  const std::size_t w = costmap.getSizeInCellsX();
  const std::size_t h = costmap.getSizeInCellsY();
  planning::Grid grid(w, h, 0);
  const unsigned char* data = costmap.getCharMap();
  std::copy(data, data + w * h, grid.data().begin());
  return grid;
}

inline std::vector<geometry_msgs::msg::PoseStamped> cellsToPoses(
    const nav2_costmap_2d::Costmap2D& costmap, const planning::Path& path,
    const std_msgs::msg::Header& header) {
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.reserve(path.size());
  for (const auto& [x, y] : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    costmap.mapToWorld(static_cast<unsigned int>(x), static_cast<unsigned int>(y),
                       pose.pose.position.x, pose.pose.position.y);
    pose.pose.orientation.w = 1.0;
    poses.push_back(pose);
  }
  return poses;
}

}  // namespace ros2_dynamic_path_planning
