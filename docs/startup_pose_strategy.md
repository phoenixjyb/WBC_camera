# Startup Pose and Goal Planning Summary

- Set the whole-body "home" pose explicitly to `(-2, -2, 0)` so the chassis begins at a consistent world-frame location while the arm starts in its URDF home configuration.
- With that home pose, plan the ramp sequence in three stages:
  1. **Arm ramp (base fixed)** — solve GIK toward a virtual end-effector goal that copies the first trajectory waypoint’s orientation and height while leaving XY anchored at the home base. This keeps the chassis static and warms the arm into tracking posture.
  2. **Chassis ramp (arm pose held)** — run Hybrid A* from `(-2, -2, 0)` to the first reference base pose, honoring a reduced speed budget (≤ 0.2 m/s) so the platform repositions smoothly before tracking begins.
  3. **Tracking phase** — follow the remaining base/EE references with the nominal limits (base ≤ 0.6 m/s) using the synchronized retiming pipeline.
- Promote goal computations for each stage into reusable functions so the controller can swap in alternative home poses or planners without rewriting the ramp logic.
- Joint velocity limits remain the same as the existing configuration (`arm_joint_limits.m`), but tighten if future hardware specs demand lower bounds for specific joints.
- Before execution, confirm that the new home pose offset is applied consistently to both the chassis and the desired EE trajectory so the ramp and tracking goals stay aligned.
