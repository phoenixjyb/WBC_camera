#ifndef MOBILE_ARM_WHOLE_BODY_CONTROL_TRAJECTORY_UTILS_HPP_
#define MOBILE_ARM_WHOLE_BODY_CONTROL_TRAJECTORY_UTILS_HPP_

#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace mobile_arm_whole_body_control::trajectory_utils
{

/// Split a whole-body joint trajectory (world/base + arm joints) into arm-only trajectory
/// and planar base path. Returns true when the input contains the required world joints.
bool split_whole_body_plan(
  const trajectory_msgs::msg::JointTrajectory & input,
  trajectory_msgs::msg::JointTrajectory & arm_traj,
  nav_msgs::msg::Path & base_path,
  const std::string & default_frame = "odom");

/// Determine whether a trajectory requires motion beyond the supplied tolerance.
bool trajectory_requires_motion(
  const trajectory_msgs::msg::JointTrajectory & traj,
  double joint_tol);

geometry_msgs::msg::Pose compute_base_goal_pose(
  const geometry_msgs::msg::Pose & target_pose,
  const geometry_msgs::msg::Pose & current_base_pose,
  double base_offset);

geometry_msgs::msg::Quaternion normalize_quaternion(const geometry_msgs::msg::Quaternion & q);

double quaternion_yaw(const geometry_msgs::msg::Quaternion & q);

}  // namespace mobile_arm_whole_body_control::trajectory_utils

#endif  // MOBILE_ARM_WHOLE_BODY_CONTROL_TRAJECTORY_UTILS_HPP_
