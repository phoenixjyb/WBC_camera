#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "mobile_arm_whole_body_control/trajectory_utils.hpp"
#include "mobile_arm_whole_body_interfaces/msg/camera_pose_target.hpp"
#include "mobile_arm_whole_body_interfaces/msg/tracking_phase_state.hpp"
#include "mobile_arm_whole_body_interfaces/srv/plan_arm_ramp.hpp"
#include "mobile_arm_whole_body_interfaces/srv/plan_base_ramp.hpp"
#include "mobile_arm_whole_body_interfaces/srv/plan_tracking_segment.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

using geometry_msgs::msg::PoseStamped;
using mobile_arm_whole_body_interfaces::msg::CameraPoseTarget;
using mobile_arm_whole_body_interfaces::msg::TrackingPhaseState;
using mobile_arm_whole_body_interfaces::srv::PlanArmRamp;
using mobile_arm_whole_body_interfaces::srv::PlanBaseRamp;
using mobile_arm_whole_body_interfaces::srv::PlanTrackingSegment;

namespace traj_utils = mobile_arm_whole_body_control::trajectory_utils;

class WholeBodySupervisorNode : public rclcpp::Node
{
public:
  WholeBodySupervisorNode()
  : rclcpp::Node("whole_body_supervisor")
  {
    declare_parameter<double>("arm_joint_skip_tolerance", 1e-4);
    declare_parameter<double>("base_path_skip_tolerance", 0.05);
    declare_parameter<std::vector<double>>("initial_base_pose", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("initial_ee_pose", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter<std::string>("reference_frame", "odom");
    declare_parameter<double>("base_goal_offset", 0.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    active_target_sub_ = create_subscription<CameraPoseTarget>(
      "camera_path/active", qos,
      std::bind(&WholeBodySupervisorNode::on_active_target, this, std::placeholders::_1));

    state_pub_ = create_publisher<TrackingPhaseState>("whole_body/state", qos);
    arm_plan_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "whole_body/arm_plan", qos);
    base_plan_pub_ = create_publisher<nav_msgs::msg::Path>("whole_body/base_plan", qos);

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&WholeBodySupervisorNode::publish_state, this));

    update_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&WholeBodySupervisorNode::update_state_machine, this));

    request_next_client_ = create_client<std_srvs::srv::Trigger>("camera_path/request_next");
    arm_plan_client_ = create_client<PlanArmRamp>("/whole_body/plan_arm_ramp");
    base_plan_client_ = create_client<PlanBaseRamp>("/whole_body/plan_base_ramp");
    tracking_plan_client_ = create_client<PlanTrackingSegment>("/whole_body/plan_tracking_segment");

    arm_skip_joint_tol_ = get_parameter("arm_joint_skip_tolerance").as_double();
    base_skip_distance_ = get_parameter("base_path_skip_tolerance").as_double();
    reference_frame_ = get_parameter("reference_frame").as_string();
    base_goal_offset_ = get_parameter("base_goal_offset").as_double();

    initialize_base_state();
    initialize_ee_state();

    RCLCPP_INFO(get_logger(), "whole_body_supervisor node ready (skeleton)");
  }

private:
  enum class Mode : uint8_t
  {
    Idle = 0,
    ArmRampPlanning,
    BaseRampPlanning,
    TrackingPlanning,
  };

  void on_active_target(const CameraPoseTarget::SharedPtr target)
  {
    if (target == nullptr) {
      return;
    }

    active_target_ = *target;
    if (active_target_.id == 0U) {
      active_target_.id = 1U;
    }

    current_target_id_ = active_target_.id;
    mode_ = Mode::ArmRampPlanning;
    mode_start_time_ = now();
    reset_planning_state();

    RCLCPP_INFO(get_logger(), "Supervisor accepted target id=%u", current_target_id_);
  }

  void publish_state()
  {
    TrackingPhaseState state_msg;
    state_msg.active_target_id = current_target_id_;
    state_msg.stamp = now();

    switch (mode_) {
      case Mode::Idle:
        state_msg.mode = TrackingPhaseState::MODE_IDLE;
        state_msg.mode_label = "IDLE";
        break;
      case Mode::ArmRampPlanning:
        state_msg.mode = TrackingPhaseState::MODE_ARM_RAMP;
        state_msg.mode_label = "ARM_RAMP";
        break;
      case Mode::BaseRampPlanning:
        state_msg.mode = TrackingPhaseState::MODE_BASE_RAMP;
        state_msg.mode_label = "BASE_RAMP";
        break;
      case Mode::TrackingPlanning:
        state_msg.mode = TrackingPhaseState::MODE_TRACKING;
        state_msg.mode_label = "TRACKING";
        break;
      default:
        state_msg.mode = TrackingPhaseState::MODE_IDLE;
        state_msg.mode_label = "UNKNOWN";
        break;
    }

    state_pub_->publish(state_msg);
  }

  void update_state_machine()
  {
    switch (mode_) {
      case Mode::ArmRampPlanning:
        process_arm_planning();
        break;
      case Mode::BaseRampPlanning:
        process_base_planning();
        break;
      case Mode::TrackingPlanning:
        process_tracking_planning();
        break;
      case Mode::Idle:
      default:
        break;
    }
  }

  void complete_current_target(const std::string & reason)
  {
    if (current_target_id_ == 0U) {
      return;
    }

    RCLCPP_INFO(get_logger(), "Completing target id=%u (%s)", current_target_id_, reason.c_str());

    mode_ = Mode::Idle;
    reset_planning_state();
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    if (!request_next_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Request-next service not ready; target id=%u completion deferred", current_target_id_);
      current_target_id_ = 0U;
      return;
    }

    auto future = request_next_client_->async_send_request(
      request,
      std::bind(&WholeBodySupervisorNode::on_request_next_response, this, std::placeholders::_1));

    (void)future;
    current_target_id_ = 0U;
  }

  void process_arm_planning()
  {
    if (current_target_id_ == 0U) {
      return;
    }

    if (!arm_request_sent_) {
      if (!arm_plan_client_->service_is_ready()) {
        return;
      }

      auto request = std::make_shared<PlanArmRamp::Request>();
      request->target_pose = active_target_.pose;
      request->xy_tolerance = active_target_.position_tolerance > 0.0F ?
        active_target_.position_tolerance : 0.05F;
      request->sample_dt = 0.1F;
      request->planning_timeout = 1.0F;
      request->enforce_orientation = true;

      auto future_and_id = arm_plan_client_->async_send_request(request);
      arm_plan_future_ = future_and_id.future.share();
      arm_request_sent_ = true;
      return;
    }

    if (arm_plan_future_.valid() &&
      arm_plan_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto response = arm_plan_future_.get();
      arm_plan_future_ = std::shared_future<PlanArmRamp::Response::SharedPtr>();
      arm_request_sent_ = false;

      if (!response->success) {
        RCLCPP_WARN(get_logger(), "Arm ramp planning failed: %s", response->message.c_str());
        complete_current_target("Arm ramp planning failed");
        return;
      }

      arm_plan_requires_motion_ = traj_utils::trajectory_requires_motion(
        response->trajectory, arm_skip_joint_tol_);
      last_arm_trajectory_ = response->trajectory;
      last_arm_trajectory_.header.stamp = now();
      if (!arm_plan_requires_motion_) {
        RCLCPP_INFO(get_logger(), "Arm ramp skipped: trajectory within %.2e joint tolerance",
          arm_skip_joint_tol_);
      }

      if (!last_arm_trajectory_.points.empty()) {
        publish_arm_plan(last_arm_trajectory_, "arm_ramp");
      }

      current_ee_pose_.header.frame_id = reference_frame_;
      current_ee_pose_.header.stamp = now();
      current_ee_pose_.pose = active_target_.pose;
      current_ee_pose_.pose.orientation = traj_utils::normalize_quaternion(
        current_ee_pose_.pose.orientation);

      mode_ = Mode::BaseRampPlanning;
      mode_start_time_ = now();
    }
  }

  void process_base_planning()
  {
    if (!base_request_sent_) {
      if (!base_plan_client_->service_is_ready()) {
        return;
      }

      auto request = std::make_shared<PlanBaseRamp::Request>();
      request->planning_timeout = 1.0F;
      request->allow_reverse = true;
      PoseStamped start_pose = current_base_pose_;
      start_pose.header.stamp = now();
      PoseStamped goal_pose;
      goal_pose.header.frame_id = reference_frame_;
      goal_pose.header.stamp = now();
      const geometry_msgs::msg::Pose base_pose_snapshot = current_base_pose_.pose;
      goal_pose.pose = traj_utils::compute_base_goal_pose(
        active_target_.pose, base_pose_snapshot, base_goal_offset_);

      request->start = start_pose;
      request->goal = goal_pose;
      request->static_obstacles = nav_msgs::msg::OccupancyGrid();

      auto future_and_id = base_plan_client_->async_send_request(request);
      base_plan_future_ = future_and_id.future.share();
      base_request_sent_ = true;
      return;
    }

    if (base_plan_future_.valid() &&
      base_plan_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto response = base_plan_future_.get();
      base_plan_future_ = std::shared_future<PlanBaseRamp::Response::SharedPtr>();
      base_request_sent_ = false;

      if (!response->success) {
        RCLCPP_WARN(get_logger(), "Base ramp planning failed: %s", response->message.c_str());
        complete_current_target("Base ramp planning failed");
        return;
      }

      last_base_path_ = response->path;
      last_base_path_.header.stamp = now();
      double path_len = compute_path_length(last_base_path_);
      base_plan_requires_motion_ = path_len > base_skip_distance_;
      if (!base_plan_requires_motion_) {
        RCLCPP_INFO(get_logger(), "Base ramp skipped: path length %.3f m <= %.3f m",
          path_len, base_skip_distance_);
      }

      if (!last_base_path_.poses.empty()) {
        publish_base_plan(last_base_path_, "base_ramp");
        const auto & last_pose = last_base_path_.poses.back();
        current_base_pose_ = last_pose;
        current_base_pose_.pose.orientation = traj_utils::normalize_quaternion(
          current_base_pose_.pose.orientation);
      } else {
        const geometry_msgs::msg::Pose prior_base_pose = current_base_pose_.pose;
        current_base_pose_.header.frame_id = reference_frame_;
        current_base_pose_.header.stamp = now();
        current_base_pose_.pose = traj_utils::compute_base_goal_pose(
          active_target_.pose, prior_base_pose, base_goal_offset_);
      }

      mode_ = Mode::TrackingPlanning;
      mode_start_time_ = now();
    }
  }

  void process_tracking_planning()
  {
    if (!tracking_request_sent_) {
      if (!tracking_plan_client_->service_is_ready()) {
        return;
      }

      auto request = std::make_shared<PlanTrackingSegment::Request>();
      request->sample_dt = 0.1F;
      request->planning_timeout = 1.0F;
      request->enforce_orientation = true;
      PoseStamped current_base = current_base_pose_;
      current_base.header.stamp = now();
      request->current_base = current_base;
      PoseStamped current_ee = current_ee_pose_;
      current_ee.header.stamp = now();
      request->current_ee = current_ee;
      request->target_poses.push_back(active_target_.pose);

      auto future_and_id = tracking_plan_client_->async_send_request(request);
      tracking_plan_future_ = future_and_id.future.share();
      tracking_request_sent_ = true;
      return;
    }

    if (tracking_plan_future_.valid() &&
      tracking_plan_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto response = tracking_plan_future_.get();
      tracking_plan_future_ = std::shared_future<PlanTrackingSegment::Response::SharedPtr>();
      tracking_request_sent_ = false;

      if (!response->success) {
        RCLCPP_WARN(get_logger(), "Tracking planning failed: %s", response->message.c_str());
        complete_current_target("Tracking planning failed");
        return;
      }

      if (!response->arm_trajectory.points.empty()) {
        publish_arm_plan(response->arm_trajectory, "tracking");
        last_arm_trajectory_ = response->arm_trajectory;
        last_arm_trajectory_.header.stamp = now();
      }

      if (!response->base_path.poses.empty()) {
        publish_base_plan(response->base_path, "tracking");
        last_base_path_ = response->base_path;
        last_base_path_.header.stamp = now();
        const auto & last_pose = last_base_path_.poses.back();
        current_base_pose_ = last_pose;
        current_base_pose_.pose.orientation = traj_utils::normalize_quaternion(
          current_base_pose_.pose.orientation);
      } else if (!last_base_path_.poses.empty()) {
        current_base_pose_ = last_base_path_.poses.back();
      }

      current_ee_pose_.header.frame_id = reference_frame_;
      current_ee_pose_.header.stamp = now();
      current_ee_pose_.pose = active_target_.pose;
      current_ee_pose_.pose.orientation = traj_utils::normalize_quaternion(
        current_ee_pose_.pose.orientation);

      complete_current_target("Tracking plan complete");
    }
  }

  void reset_planning_state()
  {
    arm_request_sent_ = false;
    base_request_sent_ = false;
    tracking_request_sent_ = false;
    arm_plan_future_ = std::shared_future<PlanArmRamp::Response::SharedPtr>{};
    base_plan_future_ = std::shared_future<PlanBaseRamp::Response::SharedPtr>{};
    tracking_plan_future_ = std::shared_future<PlanTrackingSegment::Response::SharedPtr>{};
    arm_plan_requires_motion_ = true;
    base_plan_requires_motion_ = true;
    last_base_path_.poses.clear();
  }

  void on_request_next_response(const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
  {
    try {
      const auto response = future.get();
      if (!response->success) {
        RCLCPP_WARN(get_logger(), "Request-next response indicated failure: %s", response->message.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Request-next response: %s", response->message.c_str());
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to process request-next response: %s", ex.what());
    }
  }

  rclcpp::Subscription<CameraPoseTarget>::SharedPtr active_target_sub_;
  rclcpp::Publisher<TrackingPhaseState>::SharedPtr state_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr base_plan_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr request_next_client_;
  rclcpp::Client<PlanArmRamp>::SharedPtr arm_plan_client_;
  rclcpp::Client<PlanBaseRamp>::SharedPtr base_plan_client_;
  rclcpp::Client<PlanTrackingSegment>::SharedPtr tracking_plan_client_;

  Mode mode_ {Mode::Idle};
  uint32_t current_target_id_ {0};
  rclcpp::Time mode_start_time_ {0, 0, RCL_ROS_TIME};
  CameraPoseTarget active_target_;

  bool arm_request_sent_ {false};
  bool base_request_sent_ {false};
  bool tracking_request_sent_ {false};
  bool arm_plan_requires_motion_ {true};
  bool base_plan_requires_motion_ {true};

  std::shared_future<PlanArmRamp::Response::SharedPtr> arm_plan_future_;
  std::shared_future<PlanBaseRamp::Response::SharedPtr> base_plan_future_;
  std::shared_future<PlanTrackingSegment::Response::SharedPtr> tracking_plan_future_;

  double arm_skip_joint_tol_ {1e-4};
  double base_skip_distance_ {0.05};
  nav_msgs::msg::Path last_base_path_;
  trajectory_msgs::msg::JointTrajectory last_arm_trajectory_;
  std::string reference_frame_ {"odom"};
  PoseStamped current_base_pose_;
  PoseStamped current_ee_pose_;
  double base_goal_offset_ {0.0};

  double compute_path_length(const nav_msgs::msg::Path & path) const
  {
    if (path.poses.size() < 2) {
      return 0.0;
    }
    double length = 0.0;
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position;
      const auto & b = path.poses[i].pose.position;
      length += std::hypot(b.x - a.x, b.y - a.y);
    }
    return length;
  }

  void publish_arm_plan(
    const trajectory_msgs::msg::JointTrajectory & traj,
    const std::string & stage)
  {
    auto msg = traj;
    msg.header.stamp = now();
    arm_plan_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published %s arm plan with %zu points", stage.c_str(), msg.points.size());
  }

  void publish_base_plan(
    const nav_msgs::msg::Path & path,
    const std::string & stage)
  {
    auto msg = path;
    msg.header.stamp = now();
    base_plan_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published %s base plan with %zu poses", stage.c_str(), msg.poses.size());
  }

  void initialize_base_state()
  {
    auto init = get_parameter("initial_base_pose").as_double_array();
    current_base_pose_.header.frame_id = reference_frame_;
    current_base_pose_.header.stamp = now();
    current_base_pose_.pose.position.x = init.size() > 0 ? init[0] : 0.0;
    current_base_pose_.pose.position.y = init.size() > 1 ? init[1] : 0.0;
    current_base_pose_.pose.position.z = 0.0;
    double yaw = init.size() > 2 ? init[2] : 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    current_base_pose_.pose.orientation = traj_utils::normalize_quaternion(tf2::toMsg(q));
  }

  void initialize_ee_state()
  {
    auto init = get_parameter("initial_ee_pose").as_double_array();
    current_ee_pose_.header.frame_id = reference_frame_;
    current_ee_pose_.header.stamp = now();
    current_ee_pose_.pose.position.x = init.size() > 0 ? init[0] : 0.0;
    current_ee_pose_.pose.position.y = init.size() > 1 ? init[1] : 0.0;
    current_ee_pose_.pose.position.z = init.size() > 2 ? init[2] : 0.0;
    double roll = init.size() > 3 ? init[3] : 0.0;
    double pitch = init.size() > 4 ? init[4] : 0.0;
    double yaw = init.size() > 5 ? init[5] : 0.0;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    current_ee_pose_.pose.orientation = traj_utils::normalize_quaternion(tf2::toMsg(q));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WholeBodySupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
