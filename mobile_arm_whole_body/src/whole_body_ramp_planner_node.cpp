#include <memory>
#include <string>
#include <thread>

#include "mobile_arm_whole_body/action/execute_whole_body.hpp"
#include "mobile_arm_whole_body/command_utils.hpp"
#include "mobile_arm_whole_body/msg/whole_body_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/header.hpp"

using ExecuteWholeBody = mobile_arm_whole_body::action::ExecuteWholeBody;

class WholeBodyRampPlannerNode : public rclcpp::Node
{
public:
  WholeBodyRampPlannerNode()
  : Node("whole_body_ramp_planner")
  {
    action_server_ = rclcpp_action::create_server<ExecuteWholeBody>(
      this,
      "whole_body/execute",
      std::bind(&WholeBodyRampPlannerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WholeBodyRampPlannerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&WholeBodyRampPlannerNode::handle_accepted, this, std::placeholders::_1));

    command_pub_ = create_publisher<mobile_arm_whole_body::msg::WholeBodyCommand>(
      "/whole_body/ramp_command", rclcpp::QoS(1).reliable());

    RCLCPP_INFO(get_logger(), "whole_body_ramp_planner action server ready");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteWholeBody::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal %s with %zu EE waypoints", goal->request_id.c_str(), goal->ee_waypoints.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteWholeBody>> &)
  {
    RCLCPP_INFO(get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteWholeBody>> goal_handle)
  {
    std::thread{[this, goal_handle]() {
        execute_goal(goal_handle);
      }}.detach();
  }

  void execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteWholeBody>> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExecuteWholeBody::Feedback>();
    auto result = std::make_shared<ExecuteWholeBody::Result>();

    feedback->status.state = mobile_arm_whole_body::msg::CoordinatorStatus::STATE_CHASSIS_RAMP;
    feedback->status.active_request_id = goal->request_id;
    goal_handle->publish_feedback(feedback);

    mobile_arm_whole_body::msg::EndEffectorReference reference;
    if (!goal->ee_waypoints.empty()) {
      const auto base_time = goal->ee_waypoints.front().header.stamp;
      for (size_t i = 0; i < goal->ee_waypoints.size(); ++i) {
        builtin_interfaces::msg::Duration dt;
        dt.sec = goal->ee_waypoints[i].header.stamp.sec - base_time.sec;
        dt.nanosec = goal->ee_waypoints[i].header.stamp.nanosec - base_time.nanosec;
        reference.time_from_start.push_back(dt);
        reference.desired.push_back(goal->ee_waypoints[i].pose);
        reference.actual.push_back(goal->ee_waypoints[i].pose);
        reference.position_error_norm.push_back(0.0F);
        reference.orientation_error_norm.push_back(0.0F);
      }
      reference.max_error_position = 0.0F;
      reference.mean_error_position = 0.0F;
      reference.max_error_position_tracking = 0.0F;
      reference.mean_error_position_tracking = 0.0F;
    }

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = goal->ee_waypoints.empty() ? "world" : goal->ee_waypoints.front().header.frame_id;

    auto cmd = mobile_arm_whole_body::command_utils::build_command(header, goal->request_id, reference);
    command_pub_->publish(cmd);

    result->success = true;
    result->message = "Ramp planner placeholder completed";
    result->executed_command = cmd;
    goal_handle->succeed(result);
  }

  rclcpp_action::Server<ExecuteWholeBody>::SharedPtr action_server_;
  rclcpp::Publisher<mobile_arm_whole_body::msg::WholeBodyCommand>::SharedPtr command_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WholeBodyRampPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
