#include <gtest/gtest.h>

#include <cmath>

#include "mobile_arm_whole_body_control/trajectory_utils.hpp"

#include "nav_msgs/msg/path.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace traj_utils = mobile_arm_whole_body_control::trajectory_utils;

TEST(TrajectoryUtils, SplitWholeBodyPlanExtractsArmAndBase)
{
  trajectory_msgs::msg::JointTrajectory input;
  input.header.frame_id = "map";
  input.joint_names = {
    "world_joint/x",
    "world_joint/y",
    "world_joint/theta",
    "shoulder_joint",
    "elbow_joint"
  };

  trajectory_msgs::msg::JointTrajectoryPoint p0;
  p0.time_from_start.sec = 0;
  p0.positions = {0.0, 0.0, 0.0, 0.1, -0.1};
  p0.velocities = {0.0, 0.0, 0.0, 0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.time_from_start.sec = 1;
  p1.positions = {0.5, 0.2, 0.1, 0.15, -0.05};
  p1.velocities = {0.5, 0.2, 0.1, 0.05, 0.10};

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.time_from_start.sec = 2;
  p2.positions = {0.8, 0.4, 0.3, 0.2, 0.0};
  p2.velocities = {0.3, 0.2, 0.2, 0.04, 0.05};

  input.points = {p0, p1, p2};

  trajectory_msgs::msg::JointTrajectory arm_traj;
  nav_msgs::msg::Path base_path;

  const bool success = traj_utils::split_whole_body_plan(input, arm_traj, base_path, "odom");
  ASSERT_TRUE(success);

  ASSERT_EQ(arm_traj.joint_names.size(), 2u);
  EXPECT_EQ(arm_traj.joint_names[0], "shoulder_joint");
  EXPECT_EQ(arm_traj.joint_names[1], "elbow_joint");
  ASSERT_EQ(arm_traj.points.size(), 3u);
  EXPECT_DOUBLE_EQ(arm_traj.points[1].positions[0], 0.15);
  EXPECT_DOUBLE_EQ(arm_traj.points[2].positions[1], 0.0);

  ASSERT_EQ(base_path.poses.size(), 3u);
  EXPECT_EQ(base_path.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(base_path.poses[1].pose.position.x, 0.5);
  EXPECT_DOUBLE_EQ(base_path.poses[1].pose.position.y, 0.2);

  const double yaw_last = tf2::getYaw(base_path.poses.back().pose.orientation);
  EXPECT_NEAR(yaw_last, 0.3, 1e-6);
}

TEST(TrajectoryUtils, SplitWholeBodyPlanFailsWithoutWorldJoints)
{
  trajectory_msgs::msg::JointTrajectory input;
  input.joint_names = {"joint_a", "joint_b"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.1, 0.2};
  input.points.push_back(point);

  trajectory_msgs::msg::JointTrajectory arm_traj;
  nav_msgs::msg::Path base_path;

  const bool success = traj_utils::split_whole_body_plan(input, arm_traj, base_path, "odom");
  EXPECT_FALSE(success);
  EXPECT_TRUE(arm_traj.points.empty());
  EXPECT_TRUE(base_path.poses.empty());
}

TEST(TrajectoryUtils, TrajectoryRequiresMotion)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"joint1", "joint2"};

  trajectory_msgs::msg::JointTrajectoryPoint a;
  a.positions = {0.1, -0.2};
  trajectory_msgs::msg::JointTrajectoryPoint b = a;
  b.time_from_start.sec = 1;

  traj.points = {a, b};

  EXPECT_FALSE(traj_utils::trajectory_requires_motion(traj, 1e-3));

  trajectory_msgs::msg::JointTrajectoryPoint c = a;
  c.time_from_start.sec = 2;
  c.positions[1] = -0.25;
  traj.points.push_back(c);

  EXPECT_TRUE(traj_utils::trajectory_requires_motion(traj, 1e-3));
}

TEST(TrajectoryUtils, BaseGoalOffsetAndYaw)
{
  geometry_msgs::msg::Pose target;
  target.position.x = 1.0;
  target.position.y = 2.0;
  target.position.z = 0.5;
  tf2::Quaternion q_target;
  q_target.setRPY(0.0, 0.0, M_PI_2);
  target.orientation = tf2::toMsg(q_target);

  geometry_msgs::msg::Pose current_base;
  current_base.position.x = -0.5;
  current_base.position.y = 0.3;
  current_base.position.z = 0.2;
  tf2::Quaternion q_base;
  q_base.setRPY(0.0, 0.0, 0.0);
  current_base.orientation = tf2::toMsg(q_base);

  const double offset = 0.4;
  const auto goal = traj_utils::compute_base_goal_pose(target, current_base, offset);

  EXPECT_NEAR(goal.position.x, 1.0, 1e-6);
  EXPECT_NEAR(goal.position.y, 2.0 - offset, 1e-6);
  EXPECT_NEAR(goal.position.z, 0.2, 1e-6);

  const double yaw = traj_utils::quaternion_yaw(goal.orientation);
  EXPECT_NEAR(yaw, M_PI_2, 1e-6);

  const auto norm = std::sqrt(
    goal.orientation.x * goal.orientation.x +
    goal.orientation.y * goal.orientation.y +
    goal.orientation.z * goal.orientation.z +
    goal.orientation.w * goal.orientation.w);
  EXPECT_NEAR(norm, 1.0, 1e-6);
}

TEST(TrajectoryUtils, NormalizeQuaternionHandlesZero)
{
  geometry_msgs::msg::Quaternion q;
  q.x = q.y = q.z = q.w = 0.0;
  auto normalized = traj_utils::normalize_quaternion(q);
  EXPECT_DOUBLE_EQ(normalized.x, 0.0);
  EXPECT_DOUBLE_EQ(normalized.y, 0.0);
  EXPECT_DOUBLE_EQ(normalized.z, 0.0);
  EXPECT_DOUBLE_EQ(normalized.w, 1.0);
}
