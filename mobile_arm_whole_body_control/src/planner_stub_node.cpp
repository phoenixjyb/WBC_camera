#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mobile_arm_whole_body_interfaces/msg/tracking_phase_state.hpp"
#include "mobile_arm_whole_body_interfaces/srv/plan_arm_ramp.hpp"
#include "mobile_arm_whole_body_interfaces/srv/plan_base_ramp.hpp"
#include "mobile_arm_whole_body_interfaces/srv/plan_tracking_segment.hpp"
#include "nav_msgs/msg/path.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using mobile_arm_whole_body_interfaces::msg::TrackingPhaseState;
using mobile_arm_whole_body_interfaces::srv::PlanArmRamp;
using mobile_arm_whole_body_interfaces::srv::PlanBaseRamp;
using mobile_arm_whole_body_interfaces::srv::PlanTrackingSegment;

class PlannerStubNode : public rclcpp::Node
{
public:
  PlannerStubNode()
  : rclcpp::Node("planner_stub")
  {
    arm_service_ = create_service<PlanArmRamp>(
      "/whole_body/plan_arm_ramp",
      std::bind(&PlannerStubNode::handle_arm_plan, this, std::placeholders::_1, std::placeholders::_2));

    base_service_ = create_service<PlanBaseRamp>(
      "/whole_body/plan_base_ramp",
      std::bind(&PlannerStubNode::handle_base_plan, this, std::placeholders::_1, std::placeholders::_2));

    tracking_service_ = create_service<PlanTrackingSegment>(
      "/whole_body/plan_tracking_segment",
      std::bind(&PlannerStubNode::handle_tracking_plan, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "planner_stub services ready");
  }

private:
  void handle_arm_plan(
    const PlanArmRamp::Request::SharedPtr request,
    PlanArmRamp::Response::SharedPtr response)
  {
    (void)request;

    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = get_clock()->now();
    response->trajectory = traj;
    response->success = true;
    response->message = "Stub arm ramp trajectory";
    response->estimated_duration = 0.0F;
  }

  void handle_base_plan(
    const PlanBaseRamp::Request::SharedPtr request,
    PlanBaseRamp::Response::SharedPtr response)
  {
    (void)request;

    nav_msgs::msg::Path path;
    path.header.stamp = get_clock()->now();
    response->path = path;
    response->success = true;
    response->message = "Stub base ramp path";
    response->nominal_linear_speed = 0.0F;
    response->nominal_yaw_rate = 0.0F;
  }

  void handle_tracking_plan(
    const PlanTrackingSegment::Request::SharedPtr request,
    PlanTrackingSegment::Response::SharedPtr response)
  {
    (void)request;

    response->success = true;
    response->message = "Stub tracking plan";
    response->arm_trajectory = trajectory_msgs::msg::JointTrajectory{};
    response->base_path = nav_msgs::msg::Path{};

    TrackingPhaseState state_msg;
    state_msg.mode = TrackingPhaseState::MODE_TRACKING;
    state_msg.mode_label = "TRACKING";
    response->phase_state = state_msg;
  }

  rclcpp::Service<PlanArmRamp>::SharedPtr arm_service_;
  rclcpp::Service<PlanBaseRamp>::SharedPtr base_service_;
  rclcpp::Service<PlanTrackingSegment>::SharedPtr tracking_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerStubNode>());
  rclcpp::shutdown();
  return 0;
}
