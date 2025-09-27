#include "mobile_arm_whole_body_control/trajectory_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

namespace mobile_arm_whole_body_control::trajectory_utils
{

namespace
{
constexpr const char * WORLD_X = "world_joint/x";
constexpr const char * WORLD_Y = "world_joint/y";
constexpr const char * WORLD_THETA = "world_joint/theta";
}

bool split_whole_body_plan(
  const trajectory_msgs::msg::JointTrajectory & input,
  trajectory_msgs::msg::JointTrajectory & arm_traj,
  nav_msgs::msg::Path & base_path,
  const std::string & default_frame)
{
  arm_traj = input;
  base_path.poses.clear();
  base_path.header = input.header;
  if (base_path.header.frame_id.empty()) {
    base_path.header.frame_id = default_frame;
  }

  if (input.points.empty() || input.joint_names.empty()) {
    arm_traj.joint_names.clear();
    arm_traj.points.clear();
    base_path.poses.clear();
    return false;
  }

  int idx_x = -1;
  int idx_y = -1;
  int idx_theta = -1;
  std::vector<std::size_t> arm_indices;
  arm_indices.reserve(input.joint_names.size());

  for (std::size_t i = 0; i < input.joint_names.size(); ++i) {
    const auto & name = input.joint_names[i];
    if (name == WORLD_X) {
      idx_x = static_cast<int>(i);
    } else if (name == WORLD_Y) {
      idx_y = static_cast<int>(i);
    } else if (name == WORLD_THETA) {
      idx_theta = static_cast<int>(i);
    } else {
      arm_indices.push_back(i);
    }
  }

  if (idx_x == -1 || idx_y == -1 || idx_theta == -1) {
    arm_traj.joint_names.clear();
    arm_traj.points.clear();
    base_path.poses.clear();
    return false;
  }

  arm_traj.joint_names.clear();
  arm_traj.points.clear();
  arm_traj.points.reserve(input.points.size());
  for (auto idx : arm_indices) {
    arm_traj.joint_names.push_back(input.joint_names[idx]);
  }

  rclcpp::Time base_time(base_path.header.stamp);

  auto copy_subset = [&](const std::vector<double> & source, std::vector<double> & target) {
    target.clear();
    target.reserve(arm_indices.size());
    if (source.empty()) {
      target.assign(arm_indices.size(), 0.0);
      return;
    }
    for (auto idx : arm_indices) {
      if (idx < source.size()) {
        target.push_back(source[idx]);
      } else {
        target.push_back(0.0);
      }
    }
  };

  for (const auto & point : input.points) {
    if (static_cast<int>(point.positions.size()) <= std::max({idx_x, idx_y, idx_theta})) {
      continue;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = base_path.header;
    double delta_sec = static_cast<double>(point.time_from_start.sec) +
      static_cast<double>(point.time_from_start.nanosec) * 1e-9;
    pose.header.stamp = base_time + rclcpp::Duration::from_seconds(delta_sec);
    pose.pose.position.x = point.positions[idx_x];
    pose.pose.position.y = point.positions[idx_y];
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, point.positions[idx_theta]);
    pose.pose.orientation = tf2::toMsg(q);
    base_path.poses.push_back(pose);

    trajectory_msgs::msg::JointTrajectoryPoint arm_point;
    arm_point.time_from_start = point.time_from_start;
    copy_subset(point.positions, arm_point.positions);
    copy_subset(point.velocities, arm_point.velocities);
    copy_subset(point.accelerations, arm_point.accelerations);
    copy_subset(point.effort, arm_point.effort);

    arm_traj.points.push_back(std::move(arm_point));
  }

  return !arm_traj.points.empty() && arm_traj.points.size() == base_path.poses.size();
}

bool trajectory_requires_motion(
  const trajectory_msgs::msg::JointTrajectory & traj,
  double joint_tol)
{
  if (traj.points.size() <= 1) {
    return false;
  }

  const auto & first = traj.points.front();
  for (const auto & point : traj.points) {
    if (point.positions.size() != first.positions.size()) {
      return true;
    }
    for (std::size_t i = 0; i < point.positions.size(); ++i) {
      if (std::fabs(point.positions[i] - first.positions[i]) > joint_tol) {
        return true;
      }
    }
  }

  return false;
}

geometry_msgs::msg::Quaternion normalize_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion q_tf;
  tf2::fromMsg(q, q_tf);
  if (q_tf.length2() <= std::numeric_limits<double>::epsilon()) {
    q_tf.setRPY(0.0, 0.0, 0.0);
  } else {
    q_tf.normalize();
  }
  return tf2::toMsg(q_tf);
}

double quaternion_yaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion q_tf;
  tf2::fromMsg(q, q_tf);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
  if (yaw > M_PI) {
    yaw -= 2.0 * M_PI;
  } else if (yaw < -M_PI) {
    yaw += 2.0 * M_PI;
  }
  return yaw;
}

geometry_msgs::msg::Pose compute_base_goal_pose(
  const geometry_msgs::msg::Pose & target_pose,
  const geometry_msgs::msg::Pose & current_base_pose,
  double base_offset)
{
  geometry_msgs::msg::Pose goal;
  goal.position.z = current_base_pose.position.z;

  const auto normalized_target = normalize_quaternion(target_pose.orientation);
  const double yaw = quaternion_yaw(normalized_target);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  goal.position.x = target_pose.position.x - base_offset * cos_yaw;
  goal.position.y = target_pose.position.y - base_offset * sin_yaw;
  goal.orientation = normalized_target;

  return goal;
}

}  // namespace mobile_arm_whole_body_control::trajectory_utils
