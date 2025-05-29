#include "orchestrator.hpp"

namespace orchestrator {
using FollowPath = urc_msgs::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

Orchestrator::Orchestrator(const rclcpp::NodeOptions &options)
    : rclcpp::Node("orchestrator", options) {
  this->declare_parameter("latitudes", std::vector<double>({0.0}));
  this->declare_parameter("longitudes", std::vector<double>({0.0}));
  this->declare_parameter("odom_topic",
                          std::string("/odometry/filtered_global"));

  // TODO: convert goals to UTM

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("odom_topic").as_string(),
      rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->current_pose_.pose = msg->pose.pose;
      });

  // Create the client for the GeneratePlan service and the FollowPath action
  plan_client_ = create_client<urc_msgs::srv::GeneratePlan>("plan");
  follow_path_client_ =
      rclcpp_action::create_client<urc_msgs::action::FollowPath>(this,
                                                                 "follow_path");
  RCLCPP_INFO(get_logger(), "Initialized action client");
  navigation_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "start_autonomous", 10,
      std::bind(&Orchestrator::Loop, this, std::placeholders::_1));
}

void Orchestrator::goal_response_callback(
    const GoalHandleFollowPath::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
    ongoing = false;
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server");
  }
}

void Orchestrator::feedback_callback(
    rclcpp_action::ClientGoalHandle<urc_msgs::action::FollowPath>::SharedPtr,
    const std::shared_ptr<const urc_msgs::action::FollowPath::Feedback>
        feedback) {}

// Handle the result of the action
void Orchestrator::result_callback(
    const rclcpp_action::ClientGoalHandle<
        urc_msgs::action::FollowPath>::WrappedResult &result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Action completed successfully!");
    ongoing = false;
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_ERROR(this->get_logger(), "Action was aborted!");
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(this->get_logger(), "Action was canceled!");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown result code!");
  }
}

void Orchestrator::Loop(const std_msgs::msg::Bool &msg) {
  for (int goal_num = 0; goal_num < 2; goal_num++) {
    double lat = this->get_parameter("latitudes").as_double_array()[goal_num];
    double lon = this->get_parameter("longitudes").as_double_array()[goal_num];

    ongoing = true;
    while (ongoing) {
      // Create a request for the GeneratePlan service
      auto request = std::make_shared<urc_msgs::srv::GeneratePlan::Request>();
      request->start = current_pose_;
      request->goal.pose.position.x = lat;
      request->goal.pose.position.y = lon;

      // Wait for the service to be available
      while (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
      }

      // Call the service
      auto result_future = plan_client_->async_send_request(
          request,
          [this](rclcpp::Client<urc_msgs::srv::GeneratePlan>::SharedFuture
                     result) {
            if (result.valid() &&
                result.get()->error_code == result.get()->SUCCESS) {
              RCLCPP_INFO(this->get_logger(), "Service call succeeded!");
            } else {
              RCLCPP_ERROR(this->get_logger(), "Service call failed!");
            }
          });

      nav_msgs::msg::Path path = result_future.get()->path;
      if (path.poses.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No path received from service.");
        continue; // Retry if no path is received
      }

      // Wait for the action client to be available
      if (!this->follow_path_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Action server not available after waiting");
        rclcpp::shutdown();
      }

      // Create a goal for the FollowPath action
      auto goal_msg = urc_msgs::action::FollowPath::Goal();
      goal_msg.path = path;
      RCLCPP_INFO(this->get_logger(), "Sending goal");

      auto send_goal_options = rclcpp_action::Client<
          urc_msgs::action::FollowPath>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(
          &Orchestrator::goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback =
          std::bind(&Orchestrator::feedback_callback, this,
                    std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback = std::bind(
          &Orchestrator::result_callback, this, std::placeholders::_1);
      this->follow_path_client_->async_send_goal(goal_msg, send_goal_options);
    }
  }
}

} // namespace orchestrator

RCLCPP_COMPONENTS_REGISTER_NODE(orchestrator::Orchestrator)
