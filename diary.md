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
