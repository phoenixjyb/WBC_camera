# Whole-Body Camera Tracking Stack

ROS 2 Humble (Ubuntu 22.04) workspace that mirrors the MATLAB whole-body camera tracking pipeline using MoveIt 2 and a Hybrid A* base planner. The framework coordinates arm and chassis motion to follow streamed camera pose targets while respecting obstacles and collision constraints.

## Features
- **Trajectory ingestion**: queue camera pose targets, publish active target, request next when tracking completes.
- **Stateful supervisor**: orchestrates arm ramp, base ramp, and tracking stages with skip logic, base offset heuristics, and phase reporting.
- **MoveIt arm & whole-body planning**: motion plans for ramp alignment and synchronized tracking with collision diagnostics.
- **Hybrid A* base planner**: grid-based SE(2) path planning using static/dynamic occupancy data.
- **Execution nodes**: arm trajectory resampler (`/whole_body/arm_plan` → `/whole_body/arm_command`) and diff-drive commander (`/whole_body/base_plan` + odometry → `/whole_body/cmd_vel`).
- **Obstacle manager**: publishes static disc obstacles to both occupancy grid and MoveIt planning scene.
- **Tooling & docs**: architecture overview with flowchart/DOT graph, plotting scripts, diary, and migration plan.

## Quick Start
```bash
# Install ROS 2 Humble + MoveIt 2.5.9, then fetch deps
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
tools/run_whole_body_stack.sh --build-args "--packages-select mobile_arm_whole_body_control"

# Or skip build if already built
tools/run_whole_body_stack.sh --skip-build
```
The helper script sources Humble, builds (unless `--skip-build`), sources the overlay, and launches `mobile_arm_whole_body_bringup/whole_body_streaming.launch.py`. Append `--launch-args` to pass extra launch parameters (e.g., `rviz:=false`).

## Key Nodes & Topics
| Node | Role | Primary Topics |
|------|------|----------------|
| `trajectory_ingestor_node` | Manage camera pose queue | `camera_path/target`, `camera_path/active`, `camera_path/status` |
| `whole_body_supervisor_node` | State machine & plan publishing | `/whole_body/state`, `/whole_body/arm_plan`, `/whole_body/base_plan`, `/camera_path/request_next` |
| `whole_body_planner_node` | MoveIt arm/whole-body planning | `/whole_body/plan_arm_ramp`, `/whole_body/plan_tracking_segment` |
| `base_planner_node` | Hybrid A* base ramp planner | `/whole_body/plan_base_ramp` |
| `arm_trajectory_publisher_node` | Resample arm trajectory | `/whole_body/arm_plan`, `/whole_body/arm_command` |
| `base_motion_commander_node` | Pure-pursuit base tracker | `/whole_body/base_plan`, `/whole_body/cmd_vel`, `/whole_body/odom` |
| `obstacle_manager_node` | Publish occupancy grid + collision objects | `/whole_body/obstacle_grid` |

Full architecture details live in `docs/software_architecture.md` (includes Mermaid flowchart) and `docs/ros2_graph.dot` (Graphviz node/topic map).

## Testing
```bash
colcon test --packages-select mobile_arm_whole_body_control --ctest-args -R test_trajectory_utils
```
This covers plan splitting, motion tolerance, quaternion normalization, and base-goal heuristic helpers. Additional launch regressions can be enabled once networking/sandbox constraints are lifted.

## Repository Structure
- `mobile_arm_whole_body_control/`: nodes, planners, supervisors, utilities, tests
- `mobile_arm_whole_body_bringup/`: launch files & RViz configs
- `mobile_arm_whole_body_interfaces/`: custom message/service definitions
- `tools/`: helper scripts, plotting utilities
- `docs/`: architecture, visualization workflow, migration plan, diary
- `matlab/`: legacy MATLAB tooling (not built here)

## Next Steps
- Integrate perception-driven obstacle updates and mid-execution replanning for the base planner.
- Tune `base_goal_offset` and controller gains against real or simulated platforms.
- Enable and expand launch/integration tests in permissive environments.
- Capture best practices for Orin deployment (real-time tuning, containerization) as deployment matures.
