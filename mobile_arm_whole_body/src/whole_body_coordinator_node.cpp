#include <algorithm>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mobile_arm_whole_body/msg/coordinator_status.hpp"
#include "mobile_arm_whole_body/command_utils.hpp"
#include "mobile_arm_whole_body/msg/end_effector_reference.hpp"
#include "mobile_arm_whole_body/msg/stage_info.hpp"
#include "mobile_arm_whole_body/msg/sync_status.hpp"
#include "mobile_arm_whole_body/msg/whole_body_command.hpp"
#include "mobile_arm_whole_body/msg/whole_body_metrics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class WholeBodyCoordinatorNode : public rclcpp::Node
{
public:
  WholeBodyCoordinatorNode()
  : Node("whole_body_coordinator"),
    state_(mobile_arm_whole_body::msg::CoordinatorStatus::STATE_IDLE)
  {
    auto diag_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    reference_sub_ = create_subscription<mobile_arm_whole_body::msg::EndEffectorReference>(
      declare_parameter<std::string>("reference_topic", "/whole_body/reference"),
      rclcpp::QoS(5).reliable(),
      std::bind(&WholeBodyCoordinatorNode::reference_callback, this, std::placeholders::_1));

    status_pub_ = create_publisher<mobile_arm_whole_body::msg::CoordinatorStatus>(
      "/whole_body/state", diag_qos);
    command_pub_ = create_publisher<mobile_arm_whole_body::msg::WholeBodyCommand>(
      "/whole_body/trajectory_cmd", diag_qos);
    arm_cmd_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/whole_body/arm_cmd", command_qos);
    base_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/whole_body/base_cmd", command_qos);

    servo_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&WholeBodyCoordinatorNode::servo_step, this));
    base_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&WholeBodyCoordinatorNode::publish_base_command, this));
    arm_timer_ = create_wall_timer(
      std::chrono::milliseconds(4),
      std::bind(&WholeBodyCoordinatorNode::publish_arm_command, this));

    state_start_time_ = now();
    RCLCPP_INFO(get_logger(), "whole_body_coordinator node ready");
  }

private:
  void reference_callback(const mobile_arm_whole_body::msg::EndEffectorReference::SharedPtr msg)
  {
    latest_reference_ = msg;
    if (state_ == mobile_arm_whole_body::msg::CoordinatorStatus::STATE_IDLE && !msg->time_from_start.empty()) {
      state_ = mobile_arm_whole_body::msg::CoordinatorStatus::STATE_TRACKING;
      state_start_time_ = now();
    }
  }

  void servo_step()
  {
    auto status_msg = mobile_arm_whole_body::msg::CoordinatorStatus();
    status_msg.header.stamp = now();
    status_msg.state = state_;
    status_msg.state_elapsed = to_duration(now() - state_start_time_);
    status_msg.state_remaining = builtin_interfaces::msg::Duration();
    status_msg.state_progress = 0.0F;
    status_msg.active_request_id = active_request_id_;
    status_msg.last_event = last_event_;
    status_msg.tracking_warning = false;
    status_msg.ik_warning = false;
    status_msg.joint_limit_warning = false;
    status_msg.base_limit_warning = false;
    status_msg.collision_warning = false;

    mobile_arm_whole_body::msg::WholeBodyCommand command_msg;
    command_msg.header.stamp = status_msg.header.stamp;
    command_msg.source_id = active_request_id_;

    if (latest_reference_) {
      command_msg = mobile_arm_whole_body::command_utils::build_command(
        status_msg.header, active_request_id_, *latest_reference_);
      status_msg.current_stage = command_msg.stages.empty() ? mobile_arm_whole_body::msg::StageInfo() : command_msg.stages.front();
      status_msg.metrics = command_msg.metrics;
      status_msg.sync = command_msg.sync;
      status_msg.state_progress = latest_reference_->time_from_start.empty() ? 0.0F : 1.0F;
    } else {
      status_msg.current_stage = mobile_arm_whole_body::msg::StageInfo();
    }

    status_pub_->publish(status_msg);
    command_pub_->publish(command_msg);
  }

  void publish_arm_command()
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = now();
    traj.joint_names = declared_joint_names_;

    if (!latest_reference_) {
      arm_cmd_pub_->publish(traj);
      return;
    }

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
    pt.positions.resize(declared_joint_names_.size(), 0.0);
    pt.velocities.resize(declared_joint_names_.size(), 0.0);
    traj.points.push_back(pt);
    arm_cmd_pub_->publish(traj);
  }

  void publish_base_command()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";

    if (latest_reference_ && latest_reference_->desired.size() >= 2 && base_sample_idx_ + 1 < latest_reference_->desired.size()) {
      const auto & pose_prev = latest_reference_->desired[base_sample_idx_];
      const auto & pose_next = latest_reference_->desired[base_sample_idx_ + 1];
      const auto & dt_msg = latest_reference_->time_from_start[base_sample_idx_ + 1];
      const double dt = dt_msg.sec + dt_msg.nanosec * 1e-9;
      if (dt > 1e-6) {
        cmd.twist.linear.x = static_cast<float>((pose_next.position.x - pose_prev.position.x) / dt);
        cmd.twist.linear.y = static_cast<float>((pose_next.position.y - pose_prev.position.y) / dt);
      }
      base_sample_idx_ = (base_sample_idx_ + 1) % latest_reference_->desired.size();
    } else {
      base_sample_idx_ = 0;
    }
    base_cmd_pub_->publish(cmd);
  }

  static builtin_interfaces::msg::Duration to_duration(const rclcpp::Duration & dur)
  {
    builtin_interfaces::msg::Duration out;
    const auto total_ns = dur.nanoseconds();
    out.sec = static_cast<int32_t>(total_ns / 1000000000LL);
    out.nanosec = static_cast<uint32_t>(total_ns % 1000000000LL);
    return out;
  }

  rclcpp::Subscription<mobile_arm_whole_body::msg::EndEffectorReference>::SharedPtr reference_sub_;
  rclcpp::Publisher<mobile_arm_whole_body::msg::CoordinatorStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<mobile_arm_whole_body::msg::WholeBodyCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr base_cmd_pub_;

  rclcpp::TimerBase::SharedPtr servo_timer_;
  rclcpp::TimerBase::SharedPtr base_timer_;
  rclcpp::TimerBase::SharedPtr arm_timer_;

  mobile_arm_whole_body::msg::EndEffectorReference::SharedPtr latest_reference_;
  uint8_t state_;
  rclcpp::Time state_start_time_;
  std::string active_request_id_ {"stream"};
  std::string last_event_ {"startup"};
  std::vector<std::string> declared_joint_names_ {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  size_t base_sample_idx_ {0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WholeBodyCoordinatorNode>());
  rclcpp::shutdown();
  return 0;
}
