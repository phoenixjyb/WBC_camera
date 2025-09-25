#pragma once

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"

namespace moveit
{
namespace core
{
class RobotModel;
class JointModelGroup;
class RobotState;
}  // namespace core
}  // namespace moveit

namespace planning_scene
{
class PlanningScene;
}  // namespace planning_scene

namespace mobile_arm_whole_body
{

struct IKOptions
{
  double position_tolerance{0.005};
  double orientation_tolerance{0.0523598776};  // 3 degrees
  double timeout{0.05};
  int max_attempts{5};
  bool check_collisions{true};
  bool return_approximate{true};
};

struct IKSolution
{
  bool success{false};
  std::vector<double> joint_positions;
  moveit_msgs::msg::MoveItErrorCodes error_code;
};

class MoveItIKSolver
{
public:
  MoveItIKSolver(const rclcpp::Node::SharedPtr & node,
                 const std::string & planning_group,
                 const std::string & tip_link);

  void update_planning_scene(const moveit_msgs::msg::PlanningScene & scene_msg);

  bool solve(const std::vector<geometry_msgs::msg::Pose> & poses,
             std::vector<IKSolution> & solutions,
             const IKOptions & options,
             const std::vector<double> & seed = {});

  const std::string & group_name() const {return group_name_;}
  const std::string & tip_link() const {return tip_link_;}

private:
  bool compute_pose(const geometry_msgs::msg::Pose & target,
                    const IKOptions & options,
                    std::vector<double> & solution,
                    moveit_msgs::msg::MoveItErrorCodes & error_code,
                    const std::vector<double> & seed,
                    const std::vector<double> * warm_start);

  bool is_state_valid(moveit::core::RobotState & state,
                      const IKOptions & options) const;

  rclcpp::Node::WeakPtr node_;
  moveit::core::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  std::unique_ptr<moveit::core::RobotState> scratch_state_;
  const moveit::core::JointModelGroup * joint_model_group_{nullptr};
  std::string group_name_;
  std::string tip_link_;
};

}  // namespace mobile_arm_whole_body

