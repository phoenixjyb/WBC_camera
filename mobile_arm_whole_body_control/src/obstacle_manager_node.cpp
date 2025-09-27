#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"

namespace mobile_arm_whole_body_control
{

class ObstacleManagerNode : public rclcpp::Node
{
public:
  ObstacleManagerNode()
  : rclcpp::Node("obstacle_manager")
  {
    declare_parameter<double>("map_resolution", 0.05);
    declare_parameter<int>("map_width", 200);
    declare_parameter<int>("map_height", 200);
    declare_parameter<std::vector<double>>("map_origin", {-5.0, -5.0});
    declare_parameter<std::vector<double>>("disc_obstacles", {
      0.053595, 0.113448, 0.1, 0.1,
      -1.0, -1.0, 0.1, 0.1
    });
    declare_parameter<std::string>("map_frame", "odom");
    declare_parameter<std::string>("world_frame", "world");
    declare_parameter<std::string>("base_frame", "chassis_center_link");

    map_resolution_ = get_parameter("map_resolution").as_double();
    map_width_ = get_parameter("map_width").as_int();
    map_height_ = get_parameter("map_height").as_int();
    origin_ = get_parameter("map_origin").as_double_array();
    discs_ = get_parameter("disc_obstacles").as_double_array();
    map_frame_ = get_parameter("map_frame").as_string();
    world_frame_ = get_parameter("world_frame").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/whole_body/obstacle_grid", qos);

    publish_grid();
    apply_collision_objects();

    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      publish_grid();
    });
  }

private:
  void publish_grid()
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = map_frame_;
    grid.header.stamp = now();
    grid.info.resolution = map_resolution_;
    grid.info.width = static_cast<uint32_t>(map_width_);
    grid.info.height = static_cast<uint32_t>(map_height_);
    grid.info.origin.position.x = origin_.size() > 0 ? origin_[0] : 0.0;
    grid.info.origin.position.y = origin_.size() > 1 ? origin_[1] : 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<std::size_t>(map_width_ * map_height_), 0);

    for (std::size_t i = 0; i + 3 < discs_.size(); i += 4) {
      double cx = discs_[i];
      double cy = discs_[i + 1];
      double radius = discs_[i + 2];
      double radius_sq = radius * radius;

      for (int y = 0; y < map_height_; ++y) {
        for (int x = 0; x < map_width_; ++x) {
          double wx = grid.info.origin.position.x + (x + 0.5) * map_resolution_;
          double wy = grid.info.origin.position.y + (y + 0.5) * map_resolution_;
          double dx = wx - cx;
          double dy = wy - cy;
          if (dx * dx + dy * dy <= radius_sq) {
            grid.data[static_cast<std::size_t>(y * map_width_ + x)] = 100;
          }
        }
      }
    }

    grid_pub_->publish(grid);
  }

  void apply_collision_objects()
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    std::vector<moveit_msgs::msg::CollisionObject> objects;

    for (std::size_t i = 0; i + 3 < discs_.size(); i += 4) {
      double cx = discs_[i];
      double cy = discs_[i + 1];
      double radius = discs_[i + 2];
      double height = discs_[i + 3];

      moveit_msgs::msg::CollisionObject obj;
      obj.header.frame_id = map_frame_;
      obj.id = "disc_obstacle_" + std::to_string(i / 4);

      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      primitive.dimensions = {height, radius};

      geometry_msgs::msg::Pose pose;
      pose.position.x = cx;
      pose.position.y = cy;
      pose.position.z = height / 2.0;
      pose.orientation.w = 1.0;

      obj.primitives.push_back(primitive);
      obj.primitive_poses.push_back(pose);
      obj.operation = moveit_msgs::msg::CollisionObject::ADD;
      objects.push_back(obj);
    }

    if (!objects.empty()) {
      psi.applyCollisionObjects(objects);
      RCLCPP_INFO(get_logger(), "Applied %zu collision objects to planning scene", objects.size());
    }
  }

  double map_resolution_ {0.05};
  int map_width_ {200};
  int map_height_ {200};
  std::string map_frame_ {"odom"};
  std::string world_frame_ {"world"};
  std::vector<double> origin_;
  std::vector<double> discs_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mobile_arm_whole_body_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_arm_whole_body_control::ObstacleManagerNode>());
  rclcpp::shutdown();
  return 0;
}
