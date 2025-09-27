#include <deque>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "mobile_arm_whole_body_interfaces/msg/camera_pose_target.hpp"
#include "mobile_arm_whole_body_interfaces/msg/camera_pose_status.hpp"

using mobile_arm_whole_body_interfaces::msg::CameraPoseStatus;
using mobile_arm_whole_body_interfaces::msg::CameraPoseTarget;

class TrajectoryIngestorNode : public rclcpp::Node
{
public:
  TrajectoryIngestorNode()
  : rclcpp::Node("trajectory_ingestor")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    target_sub_ = create_subscription<CameraPoseTarget>(
      "camera_path/target", qos,
      std::bind(&TrajectoryIngestorNode::on_target_received, this, std::placeholders::_1));

    active_pub_ = create_publisher<CameraPoseTarget>("camera_path/active", qos);
    status_pub_ = create_publisher<CameraPoseStatus>("camera_path/status", qos);

    request_next_srv_ = create_service<std_srvs::srv::Trigger>(
      "camera_path/request_next",
      std::bind(&TrajectoryIngestorNode::on_request_next, this,
      std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "trajectory_ingestor node ready");
  }

private:
  void on_target_received(const CameraPoseTarget::SharedPtr msg)
  {
    CameraPoseTarget target = *msg;
    if (target.id == 0) {
      target.id = next_auto_id_++;
    }

    RCLCPP_INFO(get_logger(), "Received camera target id=%u", target.id);

    queue_.push_back(target);
    publish_status(target.id, CameraPoseStatus::STATUS_NEW, "Target queued");

    try_activate_next();
  }

  void on_request_next(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!has_active_) {
      response->success = false;
      response->message = "No active target to advance";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Received next-target request, but no active target is set");
      return;
    }

    auto completed_id = active_target_.id;
    publish_status(completed_id, CameraPoseStatus::STATUS_COMPLETE, "Target completed");

    has_active_ = false;

    if (queue_.empty()) {
      response->success = true;
      response->message = "Queue empty after completion";
      RCLCPP_INFO(get_logger(), "No further targets available after completing id=%u", completed_id);
      return;
    }

    active_target_ = queue_.front();
    queue_.pop_front();
    has_active_ = true;

    active_pub_->publish(active_target_);
    publish_status(active_target_.id, CameraPoseStatus::STATUS_ACTIVE, "Activated target");

    response->success = true;
    response->message = "Advanced to next target";
    RCLCPP_INFO(get_logger(), "Advanced to target id=%u", active_target_.id);
  }

  void try_activate_next()
  {
    if (has_active_ || queue_.empty()) {
      return;
    }

    active_target_ = queue_.front();
    queue_.pop_front();
    has_active_ = true;

    active_pub_->publish(active_target_);
    publish_status(active_target_.id, CameraPoseStatus::STATUS_ACTIVE, "Activated target");
  }

  void publish_status(uint32_t id, uint8_t status, const std::string & message)
  {
    CameraPoseStatus status_msg;
    status_msg.id = id;
    status_msg.status = status;
    status_msg.message = message;
    status_msg.stamp = now();
    status_pub_->publish(status_msg);
  }

  rclcpp::Subscription<CameraPoseTarget>::SharedPtr target_sub_;
  rclcpp::Publisher<CameraPoseTarget>::SharedPtr active_pub_;
  rclcpp::Publisher<CameraPoseStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_next_srv_;
  std::deque<CameraPoseTarget> queue_;
  CameraPoseTarget active_target_;
  bool has_active_ {false};
  uint32_t next_auto_id_ {1};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryIngestorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
