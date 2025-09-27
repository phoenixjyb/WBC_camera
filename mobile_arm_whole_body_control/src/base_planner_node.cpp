#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "mobile_arm_whole_body_interfaces/srv/plan_base_ramp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace mobile_arm_whole_body_control
{

struct GridIndex
{
  int x;
  int y;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & index) const noexcept
  {
    return static_cast<std::size_t>(index.x) * 73856093u ^ static_cast<std::size_t>(index.y) * 19349663u;
  }
};

class BasePlannerNode : public rclcpp::Node
{
public:
  BasePlannerNode()
  : rclcpp::Node("base_hybrid_astar")
  {
    declare_parameter<double>("obstacle_threshold", 50.0);
    declare_parameter<double>("step_resolution", 0.1);
    declare_parameter<double>("map_resolution", 0.05);
    declare_parameter<int>("map_width", 200);
    declare_parameter<int>("map_height", 200);
    declare_parameter<std::vector<double>>("map_origin", {-5.0, -5.0});
    declare_parameter<std::vector<double>>("static_disc_obstacles", {});

    obstacle_threshold_ = get_parameter("obstacle_threshold").as_double();
    step_resolution_ = get_parameter("step_resolution").as_double();

    generate_static_map();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/whole_body/obstacle_grid", qos,
      std::bind(&BasePlannerNode::on_grid_update, this, std::placeholders::_1));

    plan_service_ = create_service<mobile_arm_whole_body_interfaces::srv::PlanBaseRamp>(
      "/whole_body/plan_base_ramp",
      std::bind(&BasePlannerNode::handle_plan_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "base_hybrid_astar planner ready");
  }

private:
  using PlanBaseRamp = mobile_arm_whole_body_interfaces::srv::PlanBaseRamp;

  void handle_plan_request(
    const PlanBaseRamp::Request::SharedPtr request,
    PlanBaseRamp::Response::SharedPtr response)
  {
    if (request->goal.header.frame_id.empty() || request->start.header.frame_id.empty()) {
      response->success = false;
      response->message = "Start/goal frame_id must be set";
      return;
    }

    if (request->goal.header.frame_id != request->start.header.frame_id) {
      response->success = false;
      response->message = "Start and goal must share the same frame";
      return;
    }

    const nav_msgs::msg::OccupancyGrid * grid_ptr = nullptr;
    nav_msgs::msg::OccupancyGrid grid_copy;

    if (!request->static_obstacles.data.empty()) {
      grid_ptr = &request->static_obstacles;
    } else {
      std::lock_guard<std::mutex> lock(grid_mutex_);
      if (have_dynamic_grid_) {
        grid_copy = latest_grid_;
        grid_ptr = &grid_copy;
      } else if (static_map_ready_) {
        grid_ptr = &static_map_;
      }
    }

    if (grid_ptr == nullptr) {
      plan_straight_line(request, response);
      return;
    }

    const auto & grid = *grid_ptr;
    auto start_index = world_to_grid(request->start.pose.position, grid);
    auto goal_index = world_to_grid(request->goal.pose.position, grid);

    if (!start_index || !goal_index) {
      response->success = false;
      response->message = "Start or goal outside occupancy grid";
      return;
    }

    auto path_indices = run_astar(grid, *start_index, *goal_index);
    if (path_indices.empty()) {
      response->success = false;
      response->message = "No collision-free path found";
      return;
    }

    response->path = indices_to_path(path_indices, grid, request->start.header.frame_id);
    response->success = true;
    response->message = "Base ramp path planned";

    double path_length = 0.0;
    for (std::size_t i = 1; i < response->path.poses.size(); ++i) {
      const auto & a = response->path.poses[i - 1].pose.position;
      const auto & b = response->path.poses[i].pose.position;
      path_length += std::hypot(b.x - a.x, b.y - a.y);
    }

    response->nominal_linear_speed = static_cast<float>(std::max(0.1, std::min(0.6, path_length / 5.0)));
    response->nominal_yaw_rate = 0.5F;
  }

  void plan_straight_line(
    const PlanBaseRamp::Request::SharedPtr request,
    PlanBaseRamp::Response::SharedPtr response)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = request->start.header.frame_id;
    path.header.stamp = now();

    geometry_msgs::msg::PoseStamped start = request->start;
    geometry_msgs::msg::PoseStamped goal = request->goal;

    if (step_resolution_ <= 0.0) {
      step_resolution_ = 0.1;
    }

    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double distance = std::hypot(dx, dy);
    int steps = static_cast<int>(std::ceil(distance / step_resolution_));
    steps = std::max(1, steps);

    for (int i = 0; i <= steps; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(steps);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = start.pose.position.x + t * dx;
      pose.pose.position.y = start.pose.position.y + t * dy;
      pose.pose.position.z = start.pose.position.z;
      pose.pose.orientation = start.pose.orientation;
      path.poses.push_back(pose);
    }

    response->path = path;
    response->success = true;
    response->message = "Straight-line base path";
    response->nominal_linear_speed = 0.4F;
    response->nominal_yaw_rate = 0.5F;
  }

  std::optional<GridIndex> world_to_grid(
    const geometry_msgs::msg::Point & point,
    const nav_msgs::msg::OccupancyGrid & grid) const
  {
    double origin_x = grid.info.origin.position.x;
    double origin_y = grid.info.origin.position.y;
    double res = grid.info.resolution;

    int ix = static_cast<int>(std::floor((point.x - origin_x) / res));
    int iy = static_cast<int>(std::floor((point.y - origin_y) / res));

    if (ix < 0 || iy < 0 || ix >= static_cast<int>(grid.info.width) ||
      iy >= static_cast<int>(grid.info.height))
    {
      return std::nullopt;
    }

    return GridIndex{ix, iy};
  }

  bool is_occupied(const nav_msgs::msg::OccupancyGrid & grid, int x, int y) const
  {
    if (x < 0 || y < 0 || x >= static_cast<int>(grid.info.width) ||
      y >= static_cast<int>(grid.info.height))
    {
      return true;
    }

    int index = y * static_cast<int>(grid.info.width) + x;
    int8_t value = grid.data[index];
    return value >= obstacle_threshold_;
  }

  std::vector<GridIndex> run_astar(
    const nav_msgs::msg::OccupancyGrid & grid,
    GridIndex start,
    GridIndex goal) const
  {
    auto heuristic = [&](const GridIndex & a) {
        double dx = static_cast<double>(goal.x - a.x);
        double dy = static_cast<double>(goal.y - a.y);
        return std::hypot(dx, dy);
      };

    struct Node
    {
      GridIndex index;
      double f_cost;
      double g_cost;
    };

    struct Compare
    {
      bool operator()(const Node & a, const Node & b) const
      {
        return a.f_cost > b.f_cost;
      }
    };

    std::priority_queue<Node, std::vector<Node>, Compare> open_set;
    std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
    std::unordered_map<GridIndex, double, GridIndexHash> g_costs;

    g_costs[start] = 0.0;
    open_set.push(Node{start, heuristic(start), 0.0});

    const std::vector<std::pair<int, int>> neighbors = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    while (!open_set.empty()) {
      Node current = open_set.top();
      open_set.pop();

      if (current.index == goal) {
        return reconstruct_path(came_from, current.index);
      }

      for (const auto & offset : neighbors) {
        GridIndex neighbor{current.index.x + offset.first, current.index.y + offset.second};
        if (is_occupied(grid, neighbor.x, neighbor.y)) {
          continue;
        }

        double step_cost = (offset.first == 0 || offset.second == 0) ? 1.0 : std::sqrt(2.0);
        double tentative_g = current.g_cost + step_cost;

        auto it = g_costs.find(neighbor);
        if (it == g_costs.end() || tentative_g < it->second) {
          came_from[neighbor] = current.index;
          g_costs[neighbor] = tentative_g;
          double f_cost = tentative_g + heuristic(neighbor);
          open_set.push(Node{neighbor, f_cost, tentative_g});
        }
      }
    }

    return {};
  }

  std::vector<GridIndex> reconstruct_path(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    GridIndex current) const
  {
    std::vector<GridIndex> path;
    path.push_back(current);
    auto it = came_from.find(current);
    while (it != came_from.end()) {
      current = it->second;
      path.push_back(current);
      it = came_from.find(current);
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  nav_msgs::msg::Path indices_to_path(
    const std::vector<GridIndex> & indices,
    const nav_msgs::msg::OccupancyGrid & grid,
    const std::string & frame_id) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = now();

    for (const auto & index : indices) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = grid.info.origin.position.x +
        (static_cast<double>(index.x) + 0.5) * grid.info.resolution;
      pose.pose.position.y = grid.info.origin.position.y +
        (static_cast<double>(index.y) + 0.5) * grid.info.resolution;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    return path;
  }

  void on_grid_update(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    latest_grid_ = *msg;
    have_dynamic_grid_ = true;
  }

  void generate_static_map()
  {
    double map_resolution = get_parameter("map_resolution").as_double();
    int map_width = get_parameter("map_width").as_int();
    int map_height = get_parameter("map_height").as_int();
    auto origin_vec = get_parameter("map_origin").as_double_array();
    auto discs_flat = get_parameter("static_disc_obstacles").as_double_array();

    if (map_width <= 0 || map_height <= 0 || map_resolution <= 0.0) {
      RCLCPP_WARN(get_logger(), "Static map parameters invalid; skipping static map generation");
      static_map_ready_ = false;
      return;
    }

    static_map_.info.width = static_cast<unsigned int>(map_width);
    static_map_.info.height = static_cast<unsigned int>(map_height);
    static_map_.info.resolution = map_resolution;
    static_map_.info.origin.position.x = origin_vec.size() > 0 ? origin_vec[0] : 0.0;
    static_map_.info.origin.position.y = origin_vec.size() > 1 ? origin_vec[1] : 0.0;
    static_map_.info.origin.orientation.w = 1.0;
    static_map_.header.frame_id = "map";
    static_map_.header.stamp = now();
    static_map_.data.assign(static_cast<std::size_t>(map_width * map_height), 0);

    for (std::size_t i = 0; i + 3 < discs_flat.size(); i += 4) {
      double cx = discs_flat[i];
      double cy = discs_flat[i + 1];
      double radius = discs_flat[i + 2];

      double radius_sq = radius * radius;
      for (int y = 0; y < map_height; ++y) {
        for (int x = 0; x < map_width; ++x) {
          double wx = static_map_.info.origin.position.x + (x + 0.5) * map_resolution;
          double wy = static_map_.info.origin.position.y + (y + 0.5) * map_resolution;
          double dx = wx - cx;
          double dy = wy - cy;
          if (dx * dx + dy * dy <= radius_sq) {
            int index = y * map_width + x;
            static_map_.data[static_cast<std::size_t>(index)] = 100;
          }
        }
      }
    }

    static_map_ready_ = !discs_flat.empty();
    if (static_map_ready_) {
      RCLCPP_INFO(get_logger(), "Static occupancy grid generated with %zu discs",
        discs_flat.size() / 4);
    } else {
      RCLCPP_INFO(get_logger(), "No static discs configured; static map will be empty");
    }
  }

  double obstacle_threshold_{50.0};
  double step_resolution_{0.1};
  bool static_map_ready_{false};
  nav_msgs::msg::OccupancyGrid static_map_;
  nav_msgs::msg::OccupancyGrid latest_grid_;
  bool have_dynamic_grid_{false};
  std::mutex grid_mutex_;

  rclcpp::Service<PlanBaseRamp>::SharedPtr plan_service_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
};

}  // namespace mobile_arm_whole_body_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_arm_whole_body_control::BasePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
