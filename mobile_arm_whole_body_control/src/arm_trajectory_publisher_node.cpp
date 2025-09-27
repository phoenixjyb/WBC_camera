#include <algorithm>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class ArmTrajectoryPublisherNode : public rclcpp::Node
{
public:
  ArmTrajectoryPublisherNode()
  : rclcpp::Node("arm_trajectory_publisher")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    plan_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "whole_body/arm_plan", qos,
      std::bind(&ArmTrajectoryPublisherNode::on_plan, this, std::placeholders::_1));

    command_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "whole_body/arm_command", qos);

    control_period_ = declare_parameter<double>("control_period", 0.05);

    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(control_period_),
      std::bind(&ArmTrajectoryPublisherNode::publish_command, this));

    RCLCPP_INFO(get_logger(), "arm_trajectory_publisher node ready (sampled output)");
  }

private:
  void on_plan(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->points.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty arm trajectory");
      active_plan_.reset();
      return;
    }
    active_plan_ = *msg;
    start_time_ = now();
    RCLCPP_INFO(get_logger(), "Loaded arm trajectory with %zu points", msg->points.size());
  }

  void publish_command()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_plan_) {
      return;
    }

    const auto & traj = *active_plan_;
    const auto & points = traj.points;
    if (points.empty()) {
      active_plan_.reset();
      return;
    }

    double elapsed = (now() - start_time_).seconds();
    double final_time = to_seconds(points.back().time_from_start);
    if (elapsed > final_time) {
      publish_sample(points.back(), traj.joint_names);
      active_plan_.reset();
      return;
    }

    auto upper = std::lower_bound(points.begin(), points.end(), elapsed,
        [](const trajectory_msgs::msg::JointTrajectoryPoint & pt, double value) {
          return to_seconds(pt.time_from_start) < value;
        });

    if (upper == points.begin()) {
      publish_sample(points.front(), traj.joint_names);
      return;
    }

    if (upper == points.end()) {
      publish_sample(points.back(), traj.joint_names);
      return;
    }

    const auto & p1 = *(upper - 1);
    const auto & p2 = *upper;
    double t1 = to_seconds(p1.time_from_start);
    double t2 = to_seconds(p2.time_from_start);
    double alpha = (elapsed - t1) / std::max(1e-6, t2 - t1);

    trajectory_msgs::msg::JointTrajectoryPoint sample;
    interpolate_vector(p1.positions, p2.positions, alpha, sample.positions);
    interpolate_vector(p1.velocities, p2.velocities, alpha, sample.velocities);
    interpolate_vector(p1.accelerations, p2.accelerations, alpha, sample.accelerations);
    interpolate_vector(p1.effort, p2.effort, alpha, sample.effort);
    sample.time_from_start = rclcpp::Duration::from_seconds(control_period_);

    publish_sample(sample, traj.joint_names);
  }

  void publish_sample(
    const trajectory_msgs::msg::JointTrajectoryPoint & point,
    const std::vector<std::string> & joint_names)
  {
    trajectory_msgs::msg::JointTrajectory cmd;
    cmd.header.stamp = now();
    cmd.joint_names = joint_names;
    cmd.points.push_back(point);
    command_pub_->publish(cmd);
  }

  static double to_seconds(const rclcpp::Duration & dur)
  {
    return static_cast<double>(dur.nanoseconds()) * 1e-9;
  }

  static void interpolate_vector(
    const std::vector<double> & a,
    const std::vector<double> & b,
    double alpha,
    std::vector<double> & out)
  {
    if (a.empty() && b.empty()) {
      out.clear();
      return;
    }
    const auto & ref = !a.empty() ? a : b;
    out.resize(ref.size(), 0.0);
    for (std::size_t i = 0; i < ref.size(); ++i) {
      double av = (i < a.size()) ? a[i] : 0.0;
      double bv = (i < b.size()) ? b[i] : 0.0;
      out[i] = (1.0 - alpha) * av + alpha * bv;
    }
  }

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr plan_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::optional<trajectory_msgs::msg::JointTrajectory> active_plan_;
  rclcpp::Time start_time_;
  double control_period_ {0.05};
  std::mutex mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmTrajectoryPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
