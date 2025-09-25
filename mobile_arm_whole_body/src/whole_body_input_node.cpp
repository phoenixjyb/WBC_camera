#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mobile_arm_whole_body/reference_buffer.hpp"
#include "mobile_arm_whole_body/msg/end_effector_reference.hpp"
#include "rclcpp/rclcpp.hpp"

namespace maw = mobile_arm_whole_body;

class WholeBodyInputNode : public rclcpp::Node
{
public:
  WholeBodyInputNode()
  : Node("whole_body_input"),
    reference_buffer_(declare_parameter<double>("buffer_horizon", 2.0))
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera/ee_reference");
    output_topic_ = declare_parameter<std::string>("output_topic", "/whole_body/reference");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    reference_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, qos,
      std::bind(&WholeBodyInputNode::reference_callback, this, std::placeholders::_1));

    reference_pub_ = create_publisher<mobile_arm_whole_body::msg::EndEffectorReference>(
      output_topic_, qos);

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&WholeBodyInputNode::publish_reference, this));

    RCLCPP_INFO(get_logger(), "whole_body_input node started. listening on %s", input_topic_.c_str());
  }

private:
  void reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    reference_buffer_.push(*msg);
  }

  void publish_reference()
  {
    if (reference_buffer_.empty()) {
      return;
    }

    const auto now = get_clock()->now();
    auto samples = reference_buffer_.snapshot(now);
    if (samples.empty()) {
      return;
    }

    mobile_arm_whole_body::msg::EndEffectorReference ref_msg;
    ref_msg.time_from_start.reserve(samples.size());
    ref_msg.desired.reserve(samples.size());
    ref_msg.actual.reserve(samples.size());
    ref_msg.position_error_norm.resize(samples.size(), 0.0F);
    ref_msg.orientation_error_norm.resize(samples.size(), 0.0F);

    const auto base_stamp = samples.front().header.stamp;
    for (const auto & pose : samples) {
      builtin_interfaces::msg::Duration offset;
      offset.sec = pose.header.stamp.sec - base_stamp.sec;
      offset.nanosec = pose.header.stamp.nanosec - base_stamp.nanosec;
      ref_msg.time_from_start.push_back(offset);
      ref_msg.desired.push_back(pose.pose);
      ref_msg.actual.push_back(pose.pose);  // placeholder until coordinator feedback loop is wired
    }

    ref_msg.max_error_position = 0.0F;
    ref_msg.mean_error_position = 0.0F;
    ref_msg.max_error_position_tracking = 0.0F;
    ref_msg.mean_error_position_tracking = 0.0F;

    reference_pub_->publish(ref_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  maw::ReferenceBuffer reference_buffer_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_sub_;
  rclcpp::Publisher<mobile_arm_whole_body::msg::EndEffectorReference>::SharedPtr reference_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WholeBodyInputNode>());
  rclcpp::shutdown();
  return 0;
}

