// src/global_planner_wrapper.cpp
#include "global_planner_wrapper.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace global_planner_wrapper
{

GlobalPlannerWrapper::GlobalPlannerWrapper()
: logger_(rclcpp::get_logger("GlobalPlannerWrapper"))
{
}

void GlobalPlannerWrapper::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node_->get_logger();

  // Get which planner to instantiate
  node_->declare_parameter(name_ + ".planner_type", rclcpp::ParameterValue("dstar_lite_planner"));
  std::string planner_type = node_->get_parameter(name_ + ".planner_type").as_string();

  RCLCPP_INFO(logger_, "Configuring GlobalPlannerWrapper with planner: %s", planner_type.c_str());

  if (planner_type == "astar_planner") {
    planner_ = std::make_unique<astar_planner::AStarPlanner>();
  } else if (planner_type == "dstar_lite_planner") {
    planner_ = std::make_unique<dstar_lite::DStarLitePlanner>();
  } else {
    RCLCPP_ERROR(logger_, "Unknown planner_type: %s. Defaulting to D* Lite.", planner_type.c_str());
    planner_ = std::make_unique<dstar_lite::DStarLitePlanner>();
  }

  // Forward all parameters under this namespace to the actual planner
  auto node_params = node_->get_parameters_by_prefix(name_, {});
  rclcpp::ParameterMap param_map;
  for (const auto & p : node_params) {
    param_map[p.first] = p.second;
  }

  // Call the real planner's configure
  planner_->configure(parent, name_, tf, costmap_ros_);
}

void GlobalPlannerWrapper::activate()
{
  planner_->activate();
}

void GlobalPlannerWrapper::deactivate()
{
  planner_->deactivate();
}

void GlobalPlannerWrapper::cleanup()
{
  planner_->cleanup();
  planner_.reset();
}

nav_msgs::msg::Path GlobalPlannerWrapper::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  return planner_->createPlan(start, goal);
}

}  // namespace global_planner_wrapper

PLUGINLIB_EXPORT_CLASS(global_planner_wrapper::GlobalPlannerWrapper, nav2_core::GlobalPlanner)
