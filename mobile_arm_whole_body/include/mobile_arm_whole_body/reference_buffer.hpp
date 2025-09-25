#pragma once

#include <deque>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mobile_arm_whole_body
{

class ReferenceBuffer
{
public:
  explicit ReferenceBuffer(double horizon_seconds = 2.0)
  : horizon_seconds_(horizon_seconds)
  {
  }

  void set_horizon(double horizon_seconds)
  {
    horizon_seconds_ = horizon_seconds;
  }

  void push(const geometry_msgs::msg::PoseStamped & pose)
  {
    buffer_.push_back(pose);
    trim(rclcpp::Time(pose.header.stamp));
  }

  std::vector<geometry_msgs::msg::PoseStamped> snapshot(const rclcpp::Time & now) const
  {
    std::vector<geometry_msgs::msg::PoseStamped> out;
    out.reserve(buffer_.size());
    for (const auto & pose : buffer_) {
      if (now.seconds() - rclcpp::Time(pose.header.stamp).seconds() <= horizon_seconds_) {
        out.push_back(pose);
      }
    }
    return out;
  }

  bool empty() const {return buffer_.empty();}

private:
  void trim(const rclcpp::Time & now)
  {
    while (!buffer_.empty()) {
      const auto & pose = buffer_.front();
      if (now.seconds() - rclcpp::Time(pose.header.stamp).seconds() <= horizon_seconds_) {
        break;
      }
      buffer_.pop_front();
    }
  }

  double horizon_seconds_;
  std::deque<geometry_msgs::msg::PoseStamped> buffer_;
};

}  // namespace mobile_arm_whole_body
