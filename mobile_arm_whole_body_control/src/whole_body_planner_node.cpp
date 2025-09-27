#include <cmath>
#include <iomanip>
#include <ios>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "mobile_arm_whole_body_control/trajectory_utils.hpp"
#include "mobile_arm_whole_body_interfaces/srv/plan_arm_ramp.hpp"
#include "mobile_arm_whole_body_interfaces/srv/plan_base_ramp.hpp"
#include "mobile_arm_whole_body_interfaces/srv/plan_tracking_segment.hpp"
#include "mobile_arm_whole_body_interfaces/msg/tracking_phase_state.hpp"

#include "nav_msgs/msg/path.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/collision_detection/collision_common.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace mobile_arm_whole_body_control
{

class WholeBodyPlannerNode : public rclcpp::Node,
                              public std::enable_shared_from_this<WholeBodyPlannerNode>
{
public:
  explicit WholeBodyPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("whole_body_moveit_planner", options)
  {
    declare_parameter<std::string>("manipulator_group", "manipulator");
    declare_parameter<std::string>("whole_body_group", "whole_body");
    declare_parameter<std::string>("ee_link", "left_gripper_link");
    declare_parameter<double>("default_xy_tolerance", 0.02);
    declare_parameter<double>("default_orientation_tolerance", 0.05);
    declare_parameter<double>("planning_timeout", 2.0);
  }

  void initialize()
  {
    manipulator_group_name_ = get_parameter("manipulator_group").as_string();
    whole_body_group_name_ = get_parameter("whole_body_group").as_string();
    ee_link_ = get_parameter("ee_link").as_string();
    default_xy_tol_ = get_parameter("default_xy_tolerance").as_double();
    default_orientation_tol_ = get_parameter("default_orientation_tolerance").as_double();
    planning_timeout_ = get_parameter("planning_timeout").as_double();

    auto node_handle = this->rclcpp::Node::shared_from_this();

    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      node_handle, "robot_description");
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_loader_->getModel());

    move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_handle, manipulator_group_name_);
    move_group_whole_body_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_handle, whole_body_group_name_);

    move_group_arm_->setPlanningTime(planning_timeout_);
    move_group_whole_body_->setPlanningTime(planning_timeout_);

    arm_plan_service_ = create_service<mobile_arm_whole_body_interfaces::srv::PlanArmRamp>(
      "/whole_body/plan_arm_ramp",
      std::bind(&WholeBodyPlannerNode::handle_arm_plan, this, std::placeholders::_1,
        std::placeholders::_2));

    tracking_plan_service_ =
      create_service<mobile_arm_whole_body_interfaces::srv::PlanTrackingSegment>(
      "/whole_body/plan_tracking_segment",
      std::bind(&WholeBodyPlannerNode::handle_tracking_plan, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "whole_body_moveit_planner ready (MoveGroup interface)");
  }

private:
  using PlanArmRamp = mobile_arm_whole_body_interfaces::srv::PlanArmRamp;
  using PlanTracking = mobile_arm_whole_body_interfaces::srv::PlanTrackingSegment;
  using TrackingPhaseState = mobile_arm_whole_body_interfaces::msg::TrackingPhaseState;

  void handle_arm_plan(
    const PlanArmRamp::Request::SharedPtr request,
    PlanArmRamp::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!move_group_arm_) {
      response->success = false;
      response->message = "Manipulator MoveGroup not initialized";
      return;
    }

    move_group_arm_->setStartStateToCurrentState();

    Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
    tf2::fromMsg(request->target_pose, target_pose);

    geometry_msgs::msg::Pose target_pose_msg = request->target_pose;
    move_group_arm_->setPoseTarget(target_pose_msg, ee_link_);

    const double position_tolerance = (request->xy_tolerance > 0.0) ?
      request->xy_tolerance : default_xy_tol_;
    move_group_arm_->setGoalPositionTolerance(position_tolerance);
    move_group_arm_->setGoalOrientationTolerance(
      request->enforce_orientation ? default_orientation_tol_ : M_PI);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_arm_->plan(plan);

    move_group_arm_->clearPoseTargets();

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = false;
      response->message = "MoveIt failed to plan arm ramp";
      return;
    }

    response->trajectory = plan.trajectory_.joint_trajectory;
    if (!response->trajectory.points.empty()) {
      const auto & duration = response->trajectory.points.back().time_from_start;
      response->estimated_duration = static_cast<float>(
        static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9);
    } else {
      response->estimated_duration = 0.0F;
    }

    response->success = true;
    response->message = "Arm ramp plan computed";
  }

  void handle_tracking_plan(
    const PlanTracking::Request::SharedPtr request,
    PlanTracking::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!move_group_whole_body_) {
      response->success = false;
      response->message = "Whole-body MoveGroup not initialized";
      return;
    }

    if (request->target_poses.empty()) {
      response->success = false;
      response->message = "No target poses provided";
      return;
    }

    move_group_whole_body_->setStartStateToCurrentState();

    geometry_msgs::msg::Pose target_pose_msg = request->target_poses.front();
    move_group_whole_body_->setPoseTarget(target_pose_msg, ee_link_);

    move_group_whole_body_->setGoalPositionTolerance(default_xy_tol_);
    move_group_whole_body_->setGoalOrientationTolerance(default_orientation_tol_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_whole_body_->plan(plan);
    move_group_whole_body_->clearPoseTargets();

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = false;
      response->message = "MoveIt failed to plan tracking segment";
      return;
    }

    trajectory_msgs::msg::JointTrajectory whole = plan.trajectory_.joint_trajectory;
    if (!trajectory_utils::split_whole_body_plan(
        whole, response->arm_trajectory, response->base_path, "odom"))
    {
      response->success = false;
      response->message = "Whole-body trajectory missing base joints";
      return;
    }

    if (!response->base_path.poses.empty() && !response->arm_trajectory.points.empty()) {
      std::string collision_report;
      if (!check_collision_free(response->arm_trajectory, response->base_path, collision_report)) {
        response->success = false;
        response->message = collision_report.empty() ?
          "Collision detected along tracking trajectory" : collision_report;
        return;
      }
    }

    TrackingPhaseState phase_state;
    phase_state.mode = TrackingPhaseState::MODE_TRACKING;
    phase_state.mode_label = "TRACKING";
    phase_state.stamp = now();
    response->phase_state = phase_state;

    response->success = true;
    response->message = "Whole-body tracking plan computed";
  }

  std::mutex mutex_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_whole_body_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  planning_scene::PlanningScenePtr planning_scene_;

  std::string manipulator_group_name_;
  std::string whole_body_group_name_;
  std::string ee_link_;
  double default_xy_tol_{0.02};
  double default_orientation_tol_{0.05};
  double planning_timeout_{2.0};

  rclcpp::Service<PlanArmRamp>::SharedPtr arm_plan_service_;
  rclcpp::Service<PlanTracking>::SharedPtr tracking_plan_service_;

  bool check_collision_free(
    const trajectory_msgs::msg::JointTrajectory & arm_traj,
    const nav_msgs::msg::Path & base_path,
    std::string & diagnostic)
  {
    diagnostic.clear();
    if (!planning_scene_) {
      return true;
    }

    const auto robot_model = robot_model_loader_->getModel();
    if (!robot_model) {
      return true;
    }

    const auto * joint_model_group = robot_model->getJointModelGroup(whole_body_group_name_);
    if (!joint_model_group) {
      RCLCPP_WARN(get_logger(), "Joint model group %s not found", whole_body_group_name_.c_str());
      return true;
    }

    if (base_path.poses.size() != arm_traj.points.size()) {
      RCLCPP_WARN(get_logger(), "Base path and arm trajectory size mismatch: %zu vs %zu",
        base_path.poses.size(), arm_traj.points.size());
      return true;
    }

    std::vector<double> joint_positions(joint_model_group->getVariableNames().size(), 0.0);
    moveit::core::RobotState robot_state(robot_model);
    robot_state.setToDefaultValues();

    for (std::size_t i = 0; i < arm_traj.points.size(); ++i) {
      const auto & arm_point = arm_traj.points[i];
      const auto & base_pose = base_path.poses[i].pose;

      for (std::size_t j = 0; j < joint_model_group->getVariableNames().size(); ++j) {
        const auto & name = joint_model_group->getVariableNames()[j];
        if (name == "world_joint/x") {
          joint_positions[j] = base_pose.position.x;
        } else if (name == "world_joint/y") {
          joint_positions[j] = base_pose.position.y;
        } else if (name == "world_joint/theta") {
          tf2::Quaternion q;
          tf2::fromMsg(base_pose.orientation, q);
          double roll, pitch, yaw;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
          joint_positions[j] = yaw;
        } else {
          auto it = std::find(arm_traj.joint_names.begin(), arm_traj.joint_names.end(), name);
          if (it != arm_traj.joint_names.end()) {
            std::size_t idx = std::distance(arm_traj.joint_names.begin(), it);
            if (idx < arm_point.positions.size()) {
              joint_positions[j] = arm_point.positions[idx];
            }
          }
        }
      }

      robot_state.setJointGroupPositions(joint_model_group, joint_positions);
      robot_state.update();

      collision_detection::CollisionRequest request;
      request.contacts = true;
      request.max_contacts = 10;
      request.max_contacts_per_pair = 2;
      request.verbose = false;

      collision_detection::CollisionResult result;
      planning_scene_->checkCollision(request, result, robot_state);

      if (result.collision) {
        std::ostringstream oss;
        oss << "Collision at sample " << i;

        std::size_t reported_pairs = 0;
        for (const auto & contact_map : result.contacts) {
          if (reported_pairs >= 3) {
            break;
          }
          const auto & link_pair = contact_map.first;
          oss << (reported_pairs == 0 ? ": " : ", ")
              << link_pair.first << " vs " << link_pair.second;
          if (!contact_map.second.empty()) {
            const auto & contact = contact_map.second.front();
            oss << " (penetration " << std::fixed << std::setprecision(3)
                << contact.depth << " m)" << std::defaultfloat;
          }
          ++reported_pairs;
        }

        double x = base_pose.position.x;
        double y = base_pose.position.y;
        tf2::Quaternion q_base;
        tf2::fromMsg(base_pose.orientation, q_base);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(q_base).getRPY(roll, pitch, yaw);
        oss << std::fixed << std::setprecision(3);
        oss << "; base pose (x=" << x
            << ", y=" << y << ", yaw=" << yaw << ")";
        oss << std::defaultfloat;

        diagnostic = oss.str();
        RCLCPP_WARN(get_logger(), "%s", diagnostic.c_str());
        return false;
      }
    }

    return true;
  }
};

}  // namespace mobile_arm_whole_body_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mobile_arm_whole_body_control::WholeBodyPlannerNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
