# Development Diary

## 2025-09-24
- Added two-phase ramp-up: arm aligns to first EE waypoint while the chassis holds pose, then the chassis slides to ground-plane alignment; ramp metadata (arm/base steps, duration) now stored for later analysis.
- Updated base synchronisation to choose the minimal-yaw-change heading so the chassis can reverse instead of spinning; longitudinal velocity can go negative and direction sign is logged.
- Animation helper now reflects ramp vs tracking stages with captions, darker theme, and more visible chassis path; metrics only evaluate tracking samples and results MAT exports tracking-specific error vectors.
- `run_whole_body_demo.m` provides a one-call entry point for regenerating the full pipeline (IK solve, synchronisation, plots, animation).
- Wired generalized IK across ramp, refinement, and tracking stages; ramp warm-up now uses the solved joint profiles and enforces chassis/column clearance via the shared STL collision mesh.
- Replaced the ad-hoc ramp planner with the Hybrid A* helper so the base drives from the fixed home pose (−2, −2, 0) to the first waypoint while preserving heading continuity.
- Added virtual EE ramp targets (matching first waypoint orientation/height) and logged the configuration in ramp metadata for downstream analysis.
- Introduced a collision-aware warm-up phase using generalized IK, stored the IK candidates in `rampInfo`, and generated a 20-sample quintic joint trajectory (0.1 s spacing) so the arm enters tracking smoothly with known velocities/accelerations.
- Defaulted the MATLAB demo driver to `use_gik = true`, refreshed the documentation, and created `run_arm_ramp_inspection.m` plus inspection plots/MP4s for the warm-up stage.
- Hardened `helpers.animate_whole_body` by removing stale visuals and disabling `FastUpdate`, eliminating the frozen-arm artifact and producing consistent ramp/track animations.
- Seeded the tracking synchronizer with the ramp’s final heading and caged per-step yaw updates so direction reversals no longer cause single-frame spins; regenerated the full animation to verify smooth motion through the 160–270 frame region.
- Verified the ramp→tracking handoff keeps the gripper height continuous (0.86 m) and that the 5–6 cm planar offset persists as the main residual to address.
- Extended the Hybrid A* planner with obstacle checks (disc/AABB primitives) and sourced obstacle definitions via `chassis_obstacles.m`, so the chassis ramp now routes around the 20 cm disc at (−1, −1).
- Reworked the Hybrid A* wrapper to use MATLAB’s `plannerHybridAStar`, densify the polyline (≈5 cm spacing), and record planner diagnostics; regenerated ramp assets so the magenta path visibly arcs around the disc.
- Clamped chassis yaw rate to 0.5 rad/s during both ramp and tracking phases, resampling heading updates in the synchronizer so the warm-up segment no longer produces 90° frame-to-frame spins.
- Re-ran the full pipeline with the new limits, refreshed plots/MP4s, and confirmed the tracked base yaw never exceeds the cap while the arm ramp clip stays in sync with the planned chassis path.

## 2025-09-27
- Replaced the `base_motion_commander` placeholder with a lookahead pure-pursuit tracker that blends curvature steering with yaw alignment, enforces slowdown/goal tolerances, and logs graceful completion when the chassis settles inside 5 cm/0.1 rad.
- Extended `whole_body_planner_node` validation to capture colliding link pairs, penetration depth, and the offending base pose so failed tracking plans surface actionable diagnostics.
- Centralised trajectory utilities (plan splitting, ramp skip detection) in a shared library and added gtests to guard the world-joint extraction and tolerance logic; updated `colcon` build/test scripts (`colcon build --packages-select mobile_arm_whole_body_control`, `colcon test --packages-select mobile_arm_whole_body_control --ctest-args -R test_trajectory_utils`).
- Wired the supervisor to publish arm/base plans for ramp and tracking stages on `/whole_body/arm_plan` and `/whole_body/base_plan`, allowing the arm trajectory publisher and base commander nodes to consume real outputs instead of placeholders.
- Added base/EE state tracking in the supervisor with `initial_*` parameters so ramp requests start from the last planned pose, introduced a tunable `base_goal_offset` heuristic, and ensured base goals derive from the active waypoint before tracking kicks in.

## Future To-Dos
- Bias the chassis ramp end pose or allow a short arm correction so the warm-up EE pose matches the first desired waypoint, removing the ~6 cm XY jump at the tracking handoff.
- Reduce the tracking-phase IK warnings by tuning joint limits, collision constraints, or trajectory smoothing once collision avoidance primitives are available in this MATLAB install.
- Capture chassis-ramp-only diagnostics similar to the arm inspection so that future tweaks to the yaw-rate limiter and path planner can be measured quickly.
