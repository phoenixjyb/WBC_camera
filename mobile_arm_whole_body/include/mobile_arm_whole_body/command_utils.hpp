#pragma once

#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "mobile_arm_whole_body/msg/end_effector_reference.hpp"
#include "mobile_arm_whole_body/msg/stage_info.hpp"
#include "mobile_arm_whole_body/msg/sync_status.hpp"
#include "mobile_arm_whole_body/msg/whole_body_command.hpp"
#include "mobile_arm_whole_body/msg/whole_body_metrics.hpp"
#include "std_msgs/msg/header.hpp"

namespace mobile_arm_whole_body
{
namespace command_utils
{

inline mobile_arm_whole_body::msg::StageInfo make_tracking_stage(
  const mobile_arm_whole_body::msg::EndEffectorReference & reference)
{
  mobile_arm_whole_body::msg::StageInfo stage;
  stage.stage_id = mobile_arm_whole_body::msg::StageInfo::STAGE_TRACKING;
  stage.label = "Tracking";
  stage.start_index = 0;
  stage.end_index = static_cast<uint32_t>(reference.time_from_start.size());
  if (!reference.time_from_start.empty()) {
    stage.start_time_from_start = reference.time_from_start.front();
    stage.end_time_from_start = reference.time_from_start.back();
    stage.duration = reference.time_from_start.back();
    stage.progress = 1.0F;
  }
  return stage;
}

inline void populate_sync_defaults(mobile_arm_whole_body::msg::SyncStatus & sync)
{
  sync.scale_factor = 1.0F;
  sync.base_speed_limit = 0.6F;
  sync.base_yaw_rate_limit = 0.5F;
  sync.ramp_base_speed_limit = 0.2F;
  sync.ramp_base_yaw_rate_limit = 0.5F;
  sync.theta_ramp_end = 0.0F;
}

inline void compute_metrics(
  const mobile_arm_whole_body::msg::EndEffectorReference & reference,
  mobile_arm_whole_body::msg::WholeBodyMetrics & metrics)
{
  const auto n = reference.time_from_start.size();
  metrics.ee_error_max = reference.max_error_position;
  metrics.ee_error_mean = reference.mean_error_position;
  metrics.ee_error_max_total = reference.max_error_position;
  metrics.ee_error_mean_total = reference.mean_error_position;
  metrics.base_speed_max = 0.0F;
  metrics.base_yaw_rate_max = 0.0F;
  metrics.base_yaw_deviation_max = 0.0F;
  metrics.base_yaw_deviation_mean = 0.0F;
  metrics.ee_speed.assign(n, 0.0F);
  metrics.ee_accel.assign(n, 0.0F);
  metrics.ee_jerk.assign(n, 0.0F);
  metrics.arm_velocity.assign(n, 0.0F);
  metrics.arm_acceleration.assign(n, 0.0F);
  metrics.cmd_velocity_longitudinal.assign(n, 0.0F);
  metrics.cmd_velocity_lateral.assign(n, 0.0F);
  metrics.cmd_yaw_rate.assign(n, 0.0F);
}

inline mobile_arm_whole_body::msg::WholeBodyCommand build_command(
  const std_msgs::msg::Header & header,
  const std::string & source_id,
  const mobile_arm_whole_body::msg::EndEffectorReference & reference)
{
  mobile_arm_whole_body::msg::WholeBodyCommand cmd;
  cmd.header = header;
  cmd.source_id = source_id;
  cmd.ee_reference = reference;
  cmd.stages.push_back(make_tracking_stage(reference));
  populate_sync_defaults(cmd.sync);
  compute_metrics(reference, cmd.metrics);
  return cmd;
}

}  // namespace command_utils
}  // namespace mobile_arm_whole_body
