# Obstacle Disc Update Notes

- Added a ground-level disc obstacle (radius 0.1 m at [0, 0.12, 0]) via `matlab/chassis_obstacles.m` so planners share the same blocking geometry.
- `rt_whole_body_controller.m` now loads those obstacle specs once, attaches auxiliary rigid bodies, feeds them into base planning, and augments GIK with distance bounds as a fallback when `constraintCollisionAvoidance` is unavailable.
- Distance constraints hook into `rt_compute_arm_gik.m`, enabling MATLAB installs without the collision-avoidance class to keep the end effector outside a configurable margin.
- `matlab/plot_pull_world_xy.m` plots the JSON `1_pull_world` trajectory on the ground plane; helpful for quick XY inspections.
- Generated `matlab/outputs/1_pull_world_obstacle_demo.mp4` demonstrating the obstacle in the animation (enable `use_gik=true` on systems with Robotics System Toolbox support for collision avoidance, e.g., MATLAB 2024b Windows).
