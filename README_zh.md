# 全身相机跟踪框架

这是一个基于 ROS 2 Humble（Ubuntu 22.04）的工程，用 MoveIt 2 与 Hybrid A* 规划器重现 MATLAB 的“底盘 + 机械臂”全身相机跟踪流水线。系统接收源源不断的相机目标位姿，在避障前提下协调机械臂与移动底盘，实现 10 Hz 控制频率的平滑跟踪。

## 功能概览
- **轨迹接收**：订阅 `camera_path/target`，维护待执行队列，激活目标后在 `/camera_path/active` 发布，并实时更新状态话题。
- **状态机调度**：`whole_body_supervisor` 管理“机械臂对位 → 底盘对位 → 同步跟踪”三个阶段，支持跳过逻辑、底盘偏移启发式和阶段状态发布。
- **MoveIt 规划**：提供机械臂对位轨迹和全身跟踪轨迹，并针对同步轨迹逐采样碰撞检测，输出详细告警。
- **底盘 Hybrid A**：在占据栅格上做 SE(2) 搜索，生成 `nav_msgs/Path`，附带建议速度/转动率。
- **执行节点**：
  - `arm_trajectory_publisher` 以固定周期重采样 `/whole_body/arm_plan`，输出 `/whole_body/arm_command`。
  - `base_motion_commander` 使用纯追踪算法，将 `/whole_body/base_plan` + `/whole_body/odom` 转为 `/whole_body/cmd_vel`。
- **障碍管理**：`obstacle_manager` 将圆柱体障碍同时注入规划栅格与 MoveIt 规划场景。
- **文档工具**：架构文档（含 Mermaid 流程图与 ROS 图）、绘图脚本、工作日志等完整资料。

## 快速上手
```bash
# 安装 ROS 2 Humble + MoveIt 2.5.9 后，安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建工程
tools/run_whole_body_stack.sh --build-args "--packages-select mobile_arm_whole_body_control"

# 若已构建，可直接跳过 build
tools/run_whole_body_stack.sh --skip-build
```
脚本会自动 source Humble 环境、执行（或跳过） `colcon build --symlink-install`，再 source `install/setup.bash` 并启动 `mobile_arm_whole_body_bringup/whole_body_streaming.launch.py`。可通过 `--launch-args` 追加 launch 参数（例如 `rviz:=false`）。

## 核心节点与话题
| 节点 | 作用 | 关键话题 |
|------|------|----------|
| `trajectory_ingestor_node` | 维护相机位姿队列 | `camera_path/target`、`camera_path/active`、`camera_path/status` |
| `whole_body_supervisor_node` | 阶段状态机 + 发布规划结果 | `/whole_body/state`、`/whole_body/arm_plan`、`/whole_body/base_plan`、`/camera_path/request_next` |
| `whole_body_planner_node` | MoveIt 机械臂 / 全身规划 | `/whole_body/plan_arm_ramp`、`/whole_body/plan_tracking_segment` |
| `base_planner_node` | Hybrid A* 底盘规划 | `/whole_body/plan_base_ramp` |
| `arm_trajectory_publisher_node` | 机械臂轨迹重采样 | `/whole_body/arm_plan`、`/whole_body/arm_command` |
| `base_motion_commander_node` | 纯追踪底盘控制 | `/whole_body/base_plan`、`/whole_body/cmd_vel`、`/whole_body/odom` |
| `obstacle_manager_node` | 发布障碍栅格 + MoveIt 碰撞体 | `/whole_body/obstacle_grid` |

完整架构说明见 `docs/software_architecture.md`（含流程图），`docs/ros2_graph.dot` 提供 Graphviz 节点/话题图。

## 测试
```bash
colcon test --packages-select mobile_arm_whole_body_control --ctest-args -R test_trajectory_utils
```
覆盖轨迹拆分、运动判定、四元数归一化以及底盘目标启发式等单元测试。待网络/沙箱限制解除后，可补充 launch 级回归。

## 目录结构
- `mobile_arm_whole_body_control/`：各 ROS 节点、规划器、状态机、工具函数、测试
- `mobile_arm_whole_body_bringup/`：launch、RViz 配置
- `mobile_arm_whole_body_interfaces/`：自定义消息/服务
- `tools/`：辅助脚本、绘图工具
- `docs/`：架构文档、可视化流程、迁移计划、日志
- `matlab/`：原 MATLAB 工具（此处不参与编译）

## 后续计划
- 引入感知驱动的动态障碍更新与实时 replanning。
- 校准 `base_goal_offset` 与控制参数，适配真实底盘尺寸和障碍场景。
- 开放更多 launch/integration 测试场景。
- 逐步整理 AGX Orin 部署经验（实时性、容器化等）。
