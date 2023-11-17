#include "joystick_driver.hpp"
#include "joystick_mapping.hpp"
#include "preprocess.hpp"

#include <rclcpp/qos.hpp>
#include <string>
#include <utility>

namespace joystick_driver
{

JoystickDriver::JoystickDriver(const rclcpp::NodeOptions& options) : rclcpp::Node("joystick_driver", options)
{
  joy_subscriber = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SystemDefaultsQoS(), [this](const sensor_msgs::msg::Joy msg) { JoyCallback(msg); });

  drivetrain_cmd_publisher =
      create_publisher<geometry_msgs::msg::TwistStamped>("rover_drivetrain/cmd_vel", rclcpp::SystemDefaultsQoS());
  max_velocity = 4.0;
  velocity_axis = std::make_pair(1, 4);
  invert_pair = std::make_pair(true, true);
}

void JoystickDriver::JoyCallback(const sensor_msgs::msg::Joy& msg)
{
  geometry_msgs::msg::TwistStamped drive_velocity;
  drive_velocity.twist.linear.x =
      PreProcessing::preprocess(msg.axes[velocity_axis.first], max_velocity, invert_pair.first);
  drive_velocity.twist.angular.z =
      PreProcessing::preprocess(msg.axes[velocity_axis.second], max_velocity, invert_pair.second);
  drivetrain_cmd_publisher->publish(drive_velocity);
}

}  // namespace joystick_driver

RCLCPP_COMPONENTS_REGISTER_NODE(joystick_driver::JoystickDriver)
