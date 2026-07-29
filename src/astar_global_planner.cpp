// src/astar_global_planner.cpp
#include "ros2_dynamic_path_planning/astar_global_planner.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "ros2_dynamic_path_planning/costmap_conversion.hpp"

namespace ros2_dynamic_path_planning {

void AStarGlobalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = std::move(name);
  costmap_ros_ = std::move(costmap_ros);
}

void AStarGlobalPlanner::cleanup() {
  costmap_ros_.reset();
  node_.reset();
}

nav_msgs::msg::Path AStarGlobalPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
  auto* costmap = costmap_ros_->getCostmap();

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
  path_msg.header.stamp = node_ ? node_->now() : rclcpp::Time();

  unsigned int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
      !costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
    return path_msg;  // empty path = no plan, per nav2_core contract
  }

  const planning::Grid grid = gridFromCostmap(*costmap);
  const auto path = planner_.plan(grid, sx, sy, gx, gy);
  if (!path) return path_msg;

  path_msg.poses = cellsToPoses(*costmap, *path, path_msg.header);
  return path_msg;
}

}  // namespace ros2_dynamic_path_planning

PLUGINLIB_EXPORT_CLASS(ros2_dynamic_path_planning::AStarGlobalPlanner,
                       nav2_core::GlobalPlanner)
