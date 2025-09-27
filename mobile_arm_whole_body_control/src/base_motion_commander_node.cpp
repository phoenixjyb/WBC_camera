#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "angles/angles.h"

class BaseMotionCommanderNode : public rclcpp::Node
{
public:
  BaseMotionCommanderNode()
  : rclcpp::Node("base_motion_commander")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "whole_body/base_plan", qos,
      std::bind(&BaseMotionCommanderNode::on_plan, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("whole_body/cmd_vel", qos);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "whole_body/odom", qos,
      std::bind(&BaseMotionCommanderNode::on_odometry, this, std::placeholders::_1));

    step_period_ = declare_parameter<double>("control_period", 0.1);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.4);
    slowdown_distance_ = declare_parameter<double>("slowdown_distance", 0.6);
    max_speed_ = declare_parameter<double>("max_speed", 0.6);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.5);
    kp_linear_ = declare_parameter<double>("kp_linear", 0.8);
    kp_angular_ = declare_parameter<double>("kp_angular", 1.2);
    goal_tolerance_linear_ = declare_parameter<double>("goal_tolerance_linear", 0.05);
    goal_tolerance_angular_ = declare_parameter<double>("goal_tolerance_angular", 0.1);

    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(step_period_),
      std::bind(&BaseMotionCommanderNode::publish_command, this));

    RCLCPP_INFO(get_logger(), "base_motion_commander node ready (diff-drive tracker)");
  }

private:
  struct PlanSample
  {
    geometry_msgs::msg::Pose pose;
    double yaw{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  void on_plan(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    raw_plan_ = *msg;
    samples_.clear();
    current_index_ = 0;
    plan_active_ = false;

    for (const auto & pose_stamped : raw_plan_.poses) {
      PlanSample sample;
      sample.pose = pose_stamped.pose;
      sample.stamp = rclcpp::Time(pose_stamped.header.stamp);

      tf2::Quaternion q;
      tf2::fromMsg(sample.pose.orientation, q);
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      if (!std::isfinite(yaw)) {
        yaw = 0.0;
      }
      sample.yaw = yaw;
      samples_.push_back(sample);
    }

    if (samples_.size() < 2) {
      RCLCPP_WARN(get_logger(), "Received base plan with insufficient samples (%zu); holding position",
        samples_.size());
      return;
    }

    plan_active_ = true;
    plan_completed_logged_ = false;
    RCLCPP_INFO(get_logger(), "Received base plan with %zu poses", samples_.size());
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_pose_ = *msg;
    have_odom_ = true;
  }

  void publish_command()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    geometry_msgs::msg::Twist cmd;

    if (!have_odom_ || !plan_active_ || samples_.empty()) {
      cmd_pub_->publish(cmd);
      return;
    }

    const auto & robot_pose = latest_pose_.pose.pose;
    tf2::Quaternion q_robot;
    tf2::fromMsg(robot_pose.orientation, q_robot);
    double roll, pitch, yaw_robot;
    tf2::Matrix3x3(q_robot).getRPY(roll, pitch, yaw_robot);
    yaw_robot = normalize_angle(yaw_robot);

    const auto & goal_sample = samples_.back();
    double goal_dx = goal_sample.pose.position.x - robot_pose.position.x;
    double goal_dy = goal_sample.pose.position.y - robot_pose.position.y;
    double goal_distance = std::hypot(goal_dx, goal_dy);
    double goal_heading_error = angles::shortest_angular_distance(yaw_robot, goal_sample.yaw);

    if (goal_distance <= goal_tolerance_linear_) {
      cmd.angular.z = std::clamp(kp_angular_ * goal_heading_error, -max_yaw_rate_, max_yaw_rate_);
      if (std::fabs(goal_heading_error) <= goal_tolerance_angular_) {
        plan_active_ = false;
        if (!plan_completed_logged_) {
          RCLCPP_INFO(get_logger(), "Base plan completed: within %.3f m and %.3f rad tolerances",
            goal_tolerance_linear_, goal_tolerance_angular_);
          plan_completed_logged_ = true;
        }
        cmd.angular.z = 0.0;
      }
      cmd_pub_->publish(cmd);
      return;
    }

    std::size_t nearest_index = find_nearest_index(robot_pose.position);
    std::size_t target_index = find_lookahead_index(nearest_index);

    const auto & target_sample = samples_[target_index];
    double dx = target_sample.pose.position.x - robot_pose.position.x;
    double dy = target_sample.pose.position.y - robot_pose.position.y;

    double cos_yaw = std::cos(yaw_robot);
    double sin_yaw = std::sin(yaw_robot);
    double x_body = cos_yaw * dx + sin_yaw * dy;
    double y_body = -sin_yaw * dx + cos_yaw * dy;
    double distance = std::hypot(x_body, y_body);

    if (distance < 1e-6) {
      cmd_pub_->publish(cmd);
      return;
    }

    double linear_speed = std::clamp(kp_linear_ * x_body, -max_speed_, max_speed_);

    if (slowdown_distance_ > goal_tolerance_linear_ && goal_distance < slowdown_distance_) {
      double scale = goal_distance / slowdown_distance_;
      scale = std::clamp(scale, 0.1, 1.0);
      linear_speed *= scale;
    }

    double curvature = (distance > 1e-6) ? (2.0 * y_body / (distance * distance)) : 0.0;
    double angular_cmd = linear_speed * curvature;

    double heading_error = angles::shortest_angular_distance(yaw_robot, target_sample.yaw);
    angular_cmd += kp_angular_ * heading_error;

    cmd.linear.x = saturate(linear_speed, max_speed_);
    cmd.angular.z = saturate(angular_cmd, max_yaw_rate_);

    cmd_pub_->publish(cmd);
  }

  std::size_t find_nearest_index(const geometry_msgs::msg::Point & position)
  {
    if (samples_.empty()) {
      return 0;
    }

    std::size_t nearest = current_index_;
    double min_distance = std::numeric_limits<double>::max();

    const std::size_t start_index = current_index_;
    for (std::size_t i = start_index; i < samples_.size(); ++i) {
      const auto & pose = samples_[i].pose.position;
      double distance = std::hypot(pose.x - position.x, pose.y - position.y);
      if (distance < min_distance) {
        min_distance = distance;
        nearest = i;
      }

      if (min_distance < lookahead_distance_ * 0.5 && distance > lookahead_distance_ * 2.0) {
        break;
      }
    }

    current_index_ = nearest;
    return nearest;
  }

  std::size_t find_lookahead_index(std::size_t start_index) const
  {
    if (samples_.empty()) {
      return 0;
    }

    double accumulated = 0.0;
    std::size_t index = start_index;
    for (std::size_t i = start_index; i + 1 < samples_.size(); ++i) {
      const auto & a = samples_[i].pose.position;
      const auto & b = samples_[i + 1].pose.position;
      double segment = std::hypot(b.x - a.x, b.y - a.y);
      accumulated += segment;
      if (accumulated >= lookahead_distance_) {
        index = i + 1;
        break;
      }
      index = i + 1;
    }

    return index;
  }

  static double normalize_angle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double saturate(double value, double limit)
  {
    if (limit <= 0.0) {
      return value;
    }
    return std::max(-limit, std::min(limit, value));
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  nav_msgs::msg::Path raw_plan_;
  std::vector<PlanSample> samples_;
  nav_msgs::msg::Odometry latest_pose_;
  std::size_t current_index_ {0};
  bool have_odom_ {false};
  bool plan_active_ {false};
  bool plan_completed_logged_ {false};
  double step_period_ {0.1};
  double lookahead_distance_ {0.4};
  double slowdown_distance_ {0.6};
  double max_speed_ {0.6};
  double max_yaw_rate_ {0.5};
  double kp_linear_ {0.8};
  double kp_angular_ {1.2};
  double goal_tolerance_linear_ {0.05};
  double goal_tolerance_angular_ {0.1};
  std::mutex mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseMotionCommanderNode>());
  rclcpp::shutdown();
  return 0;
}
