# Development Diary

## 2025-09-22
- Added two-phase ramp-up: arm aligns to first EE waypoint while chassis holds pose, then chassis slides to ground-plane alignment; ramp metadata (arm/base steps, duration) now stored for later analysis.
- Updated base synchronisation to choose the minimal-yaw-change heading so the chassis can reverse instead of spinning; longitudinal velocity can go negative and direction sign is logged.
- Animation helper now reflects ramp vs tracking stages with captions, darker theme, and more visible chassis path; metrics only evaluate tracking samples and results MAT exports tracking-specific error vectors.
- `run_whole_body_demo.m` provides a one-call entry point for regenerating the full pipeline (IK solve, synchronisation, plots, animation).

## In Flight / Next Steps
- Replace the chassis ramp shift with a planner (e.g., hybrid A*) that respects the chassis footprint and avoids large IK-driven corrections.
- Convert ramp and tracking IK to generalized inverse kinematics with collision avoidance against the chassis + column mesh so the arm stays within limits.
- Investigate and reduce the remaining ~0.15 m tracking error spikes by tuning the refined base path and addressing joint-limit-induced clamping.

## 2025-09-24
- Wired generalized IK across ramp, refinement, and tracking stages; ramp warm-up now uses the solved joint profiles and enforces chassis/column clearance via the shared STL collision mesh.
- Replaced the ad-hoc ramp planner with the Hybrid A* helper so the base drives from the fixed home pose (−2, −2, 0) to the first waypoint while preserving heading continuity.
- Added virtual EE ramp targets (matching first waypoint orientation/height) and logged the configuration in ramp metadata for downstream analysis.
- Next: shake out solver tolerances with `matlab -batch "run_camera_motion_tests"`, tune collision/velocity limits, and trim any residual tracking spikes before porting the setup into ROS.

## 2025-09-24 (later)
- Generated a ramp overview plot/fig that shows the robot at the fixed home pose, chassis/column mesh, EE home pose, ramp goal, and first desired waypoint using `plot_ramp_overview`.
- Scaled the chassis STL to meters and saved both PNG and FIG artefacts (`ramp_overview_file.{png,fig}`) for inspection.
- Highlighted the need to adjust ramp alignment: with the chassis fixed at (-2,-2,0), the EE home pose and ramp goal expose the gap the base ramp must close before tracking.
