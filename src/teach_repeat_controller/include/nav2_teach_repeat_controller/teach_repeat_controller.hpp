/*
 *  Author(s): Ibrahim Hroob <ibrahim.hroub7@gmail.com>
 *
 */

#ifndef NAV2_TEACH_REPEAT_CONTROLLER_HPP_
#define NAV2_TEACH_REPEAT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <queue>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_teach_repeat_controller
{

class TeachRepeatController : public nav2_core::Controller
{
public:
  TeachRepeatController() = default;
  ~TeachRepeatController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;


  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override {(void) path;};

protected:

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("TeachRepeatController")};
  rclcpp::Clock::SharedPtr clock_;

  double desired_linear_vel_;
  double max_angular_vel_;
  std::string vel_cmd_topic_;

  std::queue<geometry_msgs::msg::TwistStamped> vel_queue_; 
  std::mutex queue_mutex_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;

  void velCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void clearQueue();
  geometry_msgs::msg::TwistStamped popFromQueue();

};

}  // namespace nav2_teach_repeat_controller

#endif  // NAV2_TEACH_REPEAT_CONTROLLER_HPP_