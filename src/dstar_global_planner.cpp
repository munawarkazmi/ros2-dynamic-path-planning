// src/dstar_global_planner.cpp
#include "ros2_dynamic_path_planning/dstar_global_planner.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "ros2_dynamic_path_planning/costmap_conversion.hpp"

namespace ros2_dynamic_path_planning {

void DStarGlobalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = std::move(name);
  costmap_ros_ = std::move(costmap_ros);
}

void DStarGlobalPlanner::cleanup() {
  planner_.reset();
  has_goal_ = false;
  costmap_ros_.reset();
  node_.reset();
}

nav_msgs::msg::Path DStarGlobalPlanner::createPlan(
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

  const std::size_t w = costmap->getSizeInCellsX();
  const std::size_t h = costmap->getSizeInCellsY();
  const bool same_goal = has_goal_ && gx == goal_x_ && gy == goal_y_;
  const bool same_shape = planner_ && planner_->grid().width() == w &&
                          planner_->grid().height() == h;

  if (!same_goal || !same_shape) {
    // New goal (or resized costmap): start a fresh incremental search.
    planner_.emplace(gridFromCostmap(*costmap));
    planner_->setGoal(gx, gy);
    goal_x_ = gx;
    goal_y_ = gy;
    has_goal_ = true;
  } else {
    // Same goal: feed only the changed cells to the planner so it can
    // repair its previous search instead of starting over.
    const unsigned char* data = costmap->getCharMap();
    const auto& known = planner_->grid().data();
    for (std::size_t idx = 0; idx < w * h; ++idx) {
      if (known[idx] != data[idx]) {
        planner_->updateCell(idx % w, idx / w, data[idx]);
      }
    }
  }

  const auto path = planner_->plan(sx, sy);
  if (!path) return path_msg;

  path_msg.poses = cellsToPoses(*costmap, *path, path_msg.header);
  return path_msg;
}

}  // namespace ros2_dynamic_path_planning

PLUGINLIB_EXPORT_CLASS(ros2_dynamic_path_planning::DStarGlobalPlanner,
                       nav2_core::GlobalPlanner)
