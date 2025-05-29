#ifndef ORCHESTRATOR_H
#define ORCHESTRATOR_H

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <urc_msgs/action/follow_path.hpp>
#include <urc_msgs/msg/status_light_command.hpp>
#include <urc_msgs/srv/generate_plan.hpp>

namespace orchestrator {

class Orchestrator : public rclcpp::Node {
public:
  explicit Orchestrator(const rclcpp::NodeOptions &options);
  using FollowPath = urc_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

private:
  bool ongoing;
  void Loop(const std_msgs::msg::Bool &msg);

  void goal_response_callback(const GoalHandleFollowPath::SharedPtr & goal_handle);

  void feedback_callback(
      rclcpp_action::ClientGoalHandle<urc_msgs::action::FollowPath>::SharedPtr,
      const std::shared_ptr<const urc_msgs::action::FollowPath::Feedback>
          feedback);

  void result_callback(const rclcpp_action::ClientGoalHandle<
                       urc_msgs::action::FollowPath>::WrappedResult &result);

  geometry_msgs::msg::PoseStamped current_pose_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<urc_msgs::srv::GeneratePlan>::SharedPtr plan_client_;
  rclcpp_action::Client<urc_msgs::action::FollowPath>::SharedPtr
      follow_path_client_;
  rclcpp::Publisher<urc_msgs::msg::StatusLightCommand>::SharedPtr
      status_light_pub_;
};
} // namespace orchestrator

#endif
