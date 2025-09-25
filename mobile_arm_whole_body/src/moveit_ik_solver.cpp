#include "mobile_arm_whole_body/moveit_ik_solver.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/kinematic_constraints/utils.h"
#include "moveit/kinematics_base/kinematics_base.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mobile_arm_whole_body
{

namespace
{
constexpr double kMinTimeout = 1e-4;
constexpr double kDefaultTimeout = 0.05;
}  // namespace

MoveItIKSolver::MoveItIKSolver(const rclcpp::Node::SharedPtr & node,
                               const std::string & planning_group,
                               const std::string & tip_link)
: node_(node), group_name_(planning_group), tip_link_(tip_link)
{
  auto node_shared = node_.lock();
  if (!node_shared) {
    throw std::runtime_error("MoveItIKSolver requires a live rclcpp node");
  }

  auto robot_description_param = node_shared->declare_parameter<std::string>("robot_description", "");
  if (robot_description_param.empty()) {
    throw std::runtime_error("robot_description parameter is empty; ensure the URDF is loaded");
  }

  moveit::robot_model_loader::RobotModelLoader loader(node_shared, "robot_description");
  robot_model_ = loader.getModel();
  if (!robot_model_) {
    throw std::runtime_error("Failed to load robot model for MoveIt IK solver");
  }

  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  scratch_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
  scratch_state_->setToDefaultValues();

  joint_model_group_ = robot_model_->getJointModelGroup(group_name_);
  if (!joint_model_group_) {
    throw std::runtime_error("Joint model group " + group_name_ + " not found in robot model");
  }

  if (tip_link_.empty()) {
    tip_link_ = joint_model_group_->getLinkModelNames().back();
  }
}

void MoveItIKSolver::update_planning_scene(const moveit_msgs::msg::PlanningScene & scene_msg)
{
  if (!planning_scene_) {
    return;
  }
  planning_scene_->setPlanningSceneDiffMsg(scene_msg);
}

bool MoveItIKSolver::solve(const std::vector<geometry_msgs::msg::Pose> & poses,
                           std::vector<IKSolution> & solutions,
                           const IKOptions & options,
                           const std::vector<double> & seed)
{
  solutions.clear();
  solutions.reserve(poses.size());

  std::vector<double> warm_start = seed;
  if (warm_start.empty()) {
    warm_start = scratch_state_->getJointGroupPositions(joint_model_group_);
  }

  for (size_t i = 0; i < poses.size(); ++i) {
    IKSolution result;
    if (compute_pose(poses[i], options, result.joint_positions, result.error_code, warm_start,
                     i == 0 ? nullptr : &solutions.back().joint_positions)) {
      result.success = true;
      warm_start = result.joint_positions;
    } else {
      result.success = false;
    }
    solutions.push_back(result);
  }

  return std::all_of(solutions.begin(), solutions.end(), [](const IKSolution & r) {return r.success;});
}

bool MoveItIKSolver::compute_pose(const geometry_msgs::msg::Pose & target,
                                  const IKOptions & options,
                                  std::vector<double> & solution,
                                  moveit_msgs::msg::MoveItErrorCodes & error_code,
                                  const std::vector<double> & seed,
                                  const std::vector<double> * warm_start)
{
  auto timeout = std::max(options.timeout, kMinTimeout);
  auto attempts = std::max(options.max_attempts, 1);

  moveit::core::RobotState state(*scratch_state_);
  state.setToDefaultValues();

  if (warm_start && warm_start->size() == joint_model_group_->getVariableCount()) {
    state.setJointGroupPositions(joint_model_group_, *warm_start);
  } else if (seed.size() == joint_model_group_->getVariableCount()) {
    state.setJointGroupPositions(joint_model_group_, seed);
  } else {
    state.setToDefaultValues(joint_model_group_);
  }

 std::vector<double> consistency_limits(joint_model_group_->getVariableCount(), options.orientation_tolerance);

  auto validity_callback = [&](moveit::core::RobotState * rs, const moveit::core::JointModelGroup * jmg, const double * joint_positions) {
      rs->setJointGroupPositions(jmg, joint_positions);
      return is_state_valid(*rs, options);
    };

  kinematics::KinematicsQueryOptions ik_options;
  ik_options.return_approximate_solution = options.return_approximate;

  bool success = false;
  for (int attempt = 0; attempt < attempts && !success; ++attempt) {
    success = state.setFromIK(joint_model_group_, target, tip_link_, timeout,
                              validity_callback, ik_options);
    if (!success && options.return_approximate) {
      moveit::core::GroupStateValidityCallbackFn empty_cb;
      success = state.setFromIK(joint_model_group_, target, tip_link_, timeout,
                                empty_cb, ik_options);
      if (success && options.check_collisions && !is_state_valid(state, options)) {
        success = false;
      }
    }
    timeout *= 1.5;  // grow timeout slightly for subsequent attempts
  }

  if (success) {
    solution.resize(joint_model_group_->getVariableCount());
    state.copyJointGroupPositions(joint_model_group_, solution);
    scratch_state_->setJointGroupPositions(joint_model_group_, solution);
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  }

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  solution.clear();
  return false;
}

bool MoveItIKSolver::is_state_valid(moveit::core::RobotState & state,
                                    const IKOptions & options) const
{
  if (!options.check_collisions || !planning_scene_) {
    return true;
  }
  state.update();
  return !planning_scene_->isStateColliding(state, group_name_);
}

}  // namespace mobile_arm_whole_body
