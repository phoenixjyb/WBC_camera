# 全身相机跟踪软件架构

本文档总结当前 ROS 2 + MoveIt 全身跟踪系统的软体结构，涵盖节点职责、数据流、接口与配置参数。中文版内容与 `docs/software_architecture.md` 对应。

## 系统概述
- 目标：在 ROS 2 Humble 下重现 MATLAB 的相机轨迹跟踪流程，协调移动底盘与机械臂使相机末端跟随流式目标位姿。
- 关键模块：轨迹接收队列、全身调度状态机、MoveIt 机械臂/全身规划、Hybrid A* 底盘规划、执行节点（机械臂重采样 & 底盘追踪）、障碍管理、工具文档。

## 数据流（Mermaid）
```mermaid
flowchart LR
  source["`camera_path/target`\nCameraPoseTarget"] -->|排队| ingest{{Trajectory Ingestor}}
  ingest -->|`camera_path/active`| supervisor{{Whole-Body Supervisor}}
  ingest -->|`camera_path/status`| monitor[状态消费方]
  supervisor -->|`/camera_path/request_next`| ingest

  supervisor -->|Arm ramp 请求| armDecision{臂阶段是否需要运动?}
  armDecision -- 否 --> baseStage
  armDecision -- 是 --> armPlan[/`/whole_body/plan_arm_ramp`\nMoveIt 规划/]
  armPlan --> armPub[/`/whole_body/arm_plan`\n机械臂重采样节点/]
  armPub --> armExec["`whole_body/arm_command`\n控制器"]

  armPlan -. 更新规划场景 .-> planner[MoveIt Planning Scene]
  obstacles[/Obstacle Manager/] -->|`/whole_body/obstacle_grid`| basePlanner[/Hybrid A*/]
  obstacles -. Collision Object .-> planner

  supervisor -->|Base ramp 请求| baseStage{底盘阶段是否需要运动?}
  baseStage -- 否 --> trackStart
  baseStage -- 是 --> basePlanner[/`/whole_body/plan_base_ramp`/]
  basePlanner -->|`/whole_body/base_plan`| baseCmd[/Base Motion Commander/]
  baseCmd --> cmdOut["`whole_body/cmd_vel`\n底盘控制器"]
  baseCmd <-- `whole_body/odom` --> odom[里程计]

  trackStart{{Tracking 阶段}}
  baseStage --> trackStart
  armDecision --> trackStart
  supervisor -->|Tracking 请求| trackingPlan[/`/whole_body/plan_tracking_segment`/]
  trackingPlan -->|拆分臂/底盘| armPub
  trackingPlan -->|`/whole_body/base_plan`| baseCmd
  trackingPlan -. 碰撞校验 .-> planner

  basePlanner -->|推荐速度| supervisor
  armPlan -->|估计时长| supervisor
  trackingPlan -->|阶段状态| supervisor
```

## 主要节点与职责
- **Trajectory Ingestor (`src/trajectory_ingestor_node.cpp`)**：接收 `camera_path/target`，分配 ID，维护队列并在 `/camera_path/active` 发布当前目标。
- **Whole-Body Supervisor (`src/whole_body_supervisor_node.cpp`)**：100 ms 轮询的状态机，处理臂 ramp / 底盘 ramp / tracking 三阶段；
  - 参数：`initial_base_pose`、`initial_ee_pose`、`reference_frame`、`base_goal_offset`、跳过阈值等；
  - 服务调用：`/whole_body/plan_arm_ramp`、`/whole_body/plan_base_ramp`、`/whole_body/plan_tracking_segment`；
  - 输出：`/whole_body/arm_plan`、`/whole_body/base_plan`、`/whole_body/state`。
- **Whole-Body Planner (`src/whole_body_planner_node.cpp`)**：调用 MoveIt MoveGroup 接口获得机械臂与全身轨迹，拆分并校验碰撞。
- **Base Planner (`src/base_planner_node.cpp`)**：在占据栅格上做 Hybrid A* 搜索，返回底盘路径与推荐速度。
- **Arm Trajectory Publisher (`src/arm_trajectory_publisher_node.cpp`)**： 50 ms 重采样，发布 `/whole_body/arm_command`。
- **Base Motion Commander (`src/base_motion_commander_node.cpp`)**：基于纯追踪+PID 的底盘控制，订阅 `/whole_body/base_plan` 与 `/whole_body/odom`。
- **Obstacle Manager (`src/obstacle_manager_node.cpp`)**：发布 `/whole_body/obstacle_grid`，并向 MoveIt 应用 Cyliner 碰撞体。
- **Planner Stub (`src/planner_stub_node.cpp`)**：测试场景下的占位服务。

## 关键参数 & 话题
| 类型 | 名称 | 说明 |
|------|------|------|
| 参数 | `arm_joint_skip_tolerance`, `base_path_skip_tolerance` | Ramp 阶段跳过阈值 |
| 参数 | `base_goal_offset` | 底盘在 EE yaw 方向上的退让距离 |
| 话题 | `camera_path/active`, `camera_path/status` | 当前目标、状态更新 |
| 话题 | `/whole_body/arm_plan`, `/whole_body/base_plan`, `/whole_body/state` | 阶段输出 |
| 服务 | `/camera_path/request_next` | 完成当前目标后请求下一个 |
| 服务 | `/whole_body/plan_arm_ramp`, `/whole_body/plan_base_ramp`, `/whole_body/plan_tracking_segment` | 规划接口 |

## 工具
- `docs/ros2_graph.dot`：Graphviz 形式展示节点/话题连接关系，可执行 `dot -Tpng docs/ros2_graph.dot -o ros_graph.png` 导出。
- `tools/run_whole_body_stack.sh`：一键构建并启动 `whole_body_streaming.launch.py`。
- `tools/plot_whole_body_log.py`：将 rosbag 记录导出的 CSV 用于绘制 ramp/跟踪阶段指标。

## 后续工作
与英文版本一致：完善底盘重规划、动态感知数据融合、控制器调参、增加集成测试、整理 Orin 部署建议等。
