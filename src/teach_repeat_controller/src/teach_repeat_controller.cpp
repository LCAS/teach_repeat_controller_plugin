/*
 *  Author(s):Ibrahim Hroob <ibrahim.hroub7@gmail.com>
 */

#include <algorithm>
#include <string>
#include <memory>

#include "nav2_util/node_utils.hpp"
#include "nav2_teach_repeat_controller/teach_repeat_controller.hpp"
#include "nav2_util/geometry_utils.hpp"


namespace nav2_teach_repeat_controller
{

void TeachRepeatController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(
      1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(
      1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".vel_cmd_topic", rclcpp::ParameterValue(
      "/teach_repeat/vel_cmd"));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".vel_cmd_topic", vel_cmd_topic_);

  cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(vel_cmd_topic_, 100, 
    std::bind(&TeachRepeatController::velCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "Teach & Repeat controller has been started.");

}

void TeachRepeatController::cleanup() {
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type teach_repeat_controller::TeachRepeatController",
    plugin_name_.c_str());
  clearQueue();
}

void TeachRepeatController::activate() {
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type teach_repeat_controller::TeachRepeatController\"  %s",
    plugin_name_.c_str(),plugin_name_.c_str());
}

void TeachRepeatController::deactivate() {
  RCLCPP_INFO(
    logger_,
    "Dectivating controller: %s of type teach_repeat_controller::TeachRepeatController\"  %s",
    plugin_name_.c_str(),plugin_name_.c_str());
}

void TeachRepeatController::setSpeedLimit(const double& speed_limit, const bool& percentage) {
  (void) speed_limit;
  (void) percentage;
}

geometry_msgs::msg::TwistStamped TeachRepeatController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker) {
  (void)velocity;
  (void)goal_checker;

  auto cmd_vel = popFromQueue();
  cmd_vel.header.frame_id = pose.header.frame_id;

  return cmd_vel;
}

void TeachRepeatController::velCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
  std::lock_guard<std::mutex> lock(queue_mutex_);
  vel_queue_.push(*msg);
}


geometry_msgs::msg::TwistStamped TeachRepeatController::popFromQueue(){
  std::lock_guard<std::mutex> lock(queue_mutex_);

  if (!vel_queue_.empty()){
    geometry_msgs::msg::TwistStamped msg = vel_queue_.front();
    vel_queue_.pop();
    return msg;
  }

  // Create TwistStamped message with the 0 linear and angular vel 
  geometry_msgs::msg::TwistStamped default_cmd_vel;
  default_cmd_vel.header.stamp = clock_->now(); //TODO: this may cause an issue 
  default_cmd_vel.twist.linear.x = 0.0;
  default_cmd_vel.twist.angular.z = 0.0;

  return default_cmd_vel; // Return default_cmd_vel
}

void TeachRepeatController::clearQueue() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!vel_queue_.empty())     {
        vel_queue_.pop();
    }
    RCLCPP_INFO(logger_, "Queue cleared.");
}

}  // namespace nav2_teach_repeat_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_teach_repeat_controller::TeachRepeatController, nav2_core::Controller)