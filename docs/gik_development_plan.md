# GIK Development Plan

## Context
- Current MATLAB pipeline (`rt_whole_body_controller.m`) generates whole-body trajectories using `inverseKinematics` via `rt_compute_arm_ik`.
- Repository goal: migrate to collision-aware `generalizedInverseKinematics` (GIK) to reduce IK residuals and prepare for planner integration.
- No source edits should occur yet to avoid branch conflicts with other active workers.

## Proposed Workflow
1. **Isolate Workspace**
   - Create a dedicated clone or `git worktree` for the new feature branch to prevent cross-branch file clashes.
   - Example: `git worktree add ../WBC_camera_gik feature/gik_migration` keeps history shared but directories isolated.
2. **Solver Abstraction**
   - Introduce a helper (e.g., `helpers.build_gik_solver`) that constructs and configures a `generalizedInverseKinematics` object with tunable tolerances and constraint hooks.
   - Keep defaults aligned with current behavior (position/orientation weights matching existing IK weights).
3. **IK Wrapper Update**
   - Extend `rt_compute_arm_ik` or add `rt_compute_arm_gik` to optionally call GIK.
   - Warm-start each solve with the previous configuration and surface diagnostics (exit flags, iteration counts, constraint activity).
4. **Collision Constraint Hooks**
   - Scaffold configuration structures for future collision pairs (e.g., arm links vs. chassis/cloud) but leave disabled until geometry validation is ready.
5. **Controller Integration**
   - Add a feature flag in `rt_whole_body_controller` to toggle GIK usage so regression tests can compare legacy IK vs. GIK.
   - Ensure ramp/tracking metrics (`rt_results`) continue to populate for downstream scripts.
6. **Validation Plan**
   - Once code changes land, rerun `matlab -batch "run_camera_motion_tests"` to benchmark velocity limits and EE error.
   - Record diffs in metrics/logs before enabling collision constraints.

## Open Questions
- Which collision geometries should be active in the first GIK rollout (arm self-collision only vs. arm-to-chassis)?
- Are there preferred solver tolerances or iteration caps from previous experiments?
- Should we bundle MoveIt export updates simultaneously or defer until solver parity is confirmed?

## Next Steps (Non-Coding)
- Confirm with collaborators whether to use a new clone or a `git worktree` for the feature branch.
- Gather answers to the open questions to finalize implementation details before touching source files.
