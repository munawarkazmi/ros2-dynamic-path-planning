// include/ros2_dynamic_path_planning/astar_global_planner.hpp
//
// Nav2 GlobalPlanner plugin wrapping the ROS-free A* from core/.
#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "planning/astar.hpp"

namespace ros2_dynamic_path_planning {

class AStarGlobalPlanner : public nav2_core::GlobalPlanner {
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
  planning::AStarPlanner planner_;
};

}  // namespace ros2_dynamic_path_planning
