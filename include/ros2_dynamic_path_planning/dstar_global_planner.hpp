// include/ros2_dynamic_path_planning/dstar_global_planner.hpp
//
// Nav2 GlobalPlanner plugin wrapping the ROS-free D* Lite from core/.
// The planner instance persists between createPlan() calls: as long as
// the goal stays the same, each call diffs the current costmap against
// the planner's copy, feeds the changed cells through updateCell(), and
// repairs the previous search instead of replanning from scratch.
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "planning/dstar_lite.hpp"

namespace ros2_dynamic_path_planning {

class DStarGlobalPlanner : public nav2_core::GlobalPlanner {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override {}
  void deactivate() override {}
  void cleanup() override;

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                 const geometry_msgs::msg::PoseStamped& goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::optional<planning::DStarLitePlanner> planner_;
  std::size_t goal_x_{0}, goal_y_{0};
  bool has_goal_{false};
};

}  // namespace ros2_dynamic_path_planning
