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

## Future To-Dos
- Bias the chassis ramp end pose or allow a short arm correction so the warm-up EE pose matches the first desired waypoint, removing the ~6 cm XY jump at the tracking handoff.
- Reduce the tracking-phase IK warnings by tuning joint limits, collision constraints, or trajectory smoothing once collision avoidance primitives are available in this MATLAB install.
- Capture chassis-ramp-only diagnostics similar to the arm inspection so that future tweaks to the yaw-rate limiter and path planner can be measured quickly.
## Streaming refactor & MoveIt integration (2024-04-06)

- Wrapped core MATLAB kernels for code generation (`wbc.sync_base_and_arm`, retiming,
  tracking metrics) and added `run_wbc_codegen_examples.m` so we can emit C++
  artifacts for retiming/sync/metrics.
- Added `mobile_arm_whole_body::MoveItIKSolver` to bridge MATLAB GIK behaviour
  into MoveIt 2.5.9 (planning scene diff, warm-starts, collision checks).
- Scaffolded ROS 2 nodes (`whole_body_input`, `whole_body_coordinator`,
  `whole_body_ramp_planner`) and wired in the rich message set plus launch
  file for integration tests.
- Introduced `wbc.stream_tracking_with_buffer` and plumbed an
  `enable_streaming` option into `rt_whole_body_controller`, enabling a
  rolling-horizon equivalent of the tracking retimer.
- Baseline vs refactor comparisons with `1_pull_world.json` show the refactor
  still differs: the tracking timeline has fewer samples (scale factor ≈3)
  vs the legacy path (scale factor 1). End-effector errors align, but sample
  counts/timing don’t match yet.

**Outstanding issues**
- Streaming retimer doesn’t reproduce the full retime timeline (346 samples vs
  441; scale factor inflated). Need to fix time-offset accumulation and window
  trimming so streamed output equals the full-horizon result.
- ramp planner still placeholder in ROS 2 scaffold; Hybrid A*/collision
  parity pending.
- Need an automated regression harness (MATLAB diff or ROS test) to check EE/
  base errors every change.
- MoveIt solver not yet invoked by ROS coordinator; need to swap out MATLAB IK
  calls and integrate constraint mappings.

**Next steps**
1. Align streamed retiming with legacy output (match sample count, timeline,
   scale factor). Instrument and debug window offsets.
2. Once parity achieved, refresh baseline/refactor diff to confirm numerical
   equivalence.
3. Integrate MoveIt IK solver into coordinator/ramp planner so ROS stack uses
   the new helper.
4. Port ramp planner (Hybrid A*) and collision constraints to C++.
5. Build regression tests (MATLAB + ROS) and start RealTime tuning on Orin.

## Streaming parity triage (2024-04-07)
- Replaced the experimental rolling-horizon implementation with a passthrough
  to `wbc.sync_base_and_arm` so MATLAB and ROS2 paths stay numerically aligned
  while we keep iterating on the streaming algorithm.
- Threaded the precomputed arc-length profile into the streaming call and
  removed the refTime override that was disturbing downstream metrics.
- Added a quick MATLAB harness that regenerates `debug_summary.txt` from fresh
  legacy vs streaming runs; parity now shows identical sample counts and scale
  factors when the passthrough is used.

**Open items**
- The real rolling-horizon retimer still needs a proper fix (window offsets,
  unique sample stitching). The passthrough is a temporary guard-rail.
- Clean up the debug harness to avoid sprinkling MAT files when not needed.

## Streaming instrumentation (2024-04-08)
- Extended `wbc.stream_tracking_with_buffer` to record per-window summaries
  (start/end indices, duration, overlap flag) and expose the configured
  streaming mode so we can cross-check future rolling-horizon attempts.
- Threaded the new diagnostics through `rt_whole_body_controller` and the
  MATLAB harness; `outputs/comparison/debug_summary.txt` now collects
  baseline vs streaming metrics alongside the window breakdown captured in
  `diagStream.sync.windowSummary`.
- Re-running `debug_compare_streaming` (passthrough mode) still shows the
  regression: legacy tracking remains 441 samples with scale factor 3.02,
  while the streaming path delivers 493 samples at scale factor 1.26. The
  window log (157 chunks, avg 0.56 s span) confirms the fallback retime is
  behaving differently when invoked via the streaming path. Need to diff the
  sync inputs across modes before reactivating the rolling-horizon logic.

### IK warm-start alignment (later 2024-04-08)
- Added `debug_diff_sync_inputs.m` plus extra diagnostics to capture the
  inputs passed into `sync_base_and_arm`, along with the refined IK seeds.
  The diff exposed that only `armTrajectoryRef` differed between runs
  (pi-flip on joints 4–6) even though the base/EE references matched.
- Seeded the refined IK call with the first nominal configuration so
  successive solves warm-start consistently. With that change the streaming
  and legacy paths now produce matching arm trajectories (max diff ≈1e-6),
  equal sample counts (492) and identical scale factors (1.2606).
- Updated `outputs/comparison/debug_summary.txt` and the new diff script to
  confirm parity; next step is to re-enable the real streaming retimer using
  the window diagnostics without breaking the newly aligned IK seeds.

### Rolling replay diagnostics (2024-04-08)
- Rebuilt `wbc.stream_tracking_with_buffer` to keep the offline synchronizer
  as the numerical source but emit a `streamReplay` log that mimics the
  rolling-horizon release (per-iteration append ranges, window coverage).
- `debug_compare_streaming` now runs the streaming path in `rolling` mode,
  producing ~202 replay iterations for `1_pull_world.json` with the same
  492-sample timeline and scale factor 1.2606; `diagStream.sync.streamReplay`
  captures the append/window indices needed for future C++ buffering logic.
- The diff helper still shows sub-micro differences in `armTrajectoryRef`
  (floating-point noise) and confirms all other sync inputs match exactly,
  so we can iterate on a true online retimer while watching the replay log
  for regressions.

## Status snapshot (2024-04-08)
**Progress**
- MATLAB refactor now splits IK warm-start, synchronization, and metrics so the controller is codegen-friendly and ready for the C++ port.
- Streaming entry point exposes per-window diagnostics (`windowSummary`, `streamReplay`) without breaking parity; debug harness persists legacy vs streaming artifacts for `1_pull_world.json`.
- ROS 2 MoveIt scaffold, message contracts, and diary documentation align with the MATLAB helpers, outlining the deployment path for Jetson AGX Orin.

**Issues / open items**
- True rolling-horizon retimer still missing; current implementation replays offline results and only simulates streaming behaviour.
- IK warnings remain (joint-limit hits, 0.195 m tracking spike) until collision constraints and trajectory smoothing are retuned.
- ROS 2 ramp planner and MoveIt IK integration are placeholders; C++ parity with MATLAB yet to be implemented.
- Automated regression harness (MATLAB + ROS) outstanding to guard against future numerical drift.

**Next steps**
1. Replace the replay shim with a genuine streaming retimer that uses the new diagnostics to validate sample stitching and scaling.
2. Port synchronization/IK/ramp planners to C++ (MoveIt-based) and hook them into the ROS 2 coordinator, leveraging URDF/SRDF assets for collision checks.
3. Triage IK limit warnings by refining joint limits, collision constraints, or reference trajectories prior to moving onto hardware.
4. Build automated regression tests (MATLAB diff + ROS launch_testing) so baseline parity is checked each iteration.

## Streaming retimer attempt (2024-04-09)
- Implemented an initial rolling-horizon retimer in `wbc.stream_tracking_with_buffer`
  that slices the reference data per window/step, retimes each chunk with the
  existing synchronizer, and stitches the outputs while recording the release
  log (`streamReplay`). Added a post-pass velocity check that stretches the
  stitched timeline until base speed/yaw-rate limits are respected.
- The current prototype keeps velocities capped (max base speed now 0.20 m/s)
  but only yields 50 tracking samples (287 total incl. ramp) compared to the
  legacy 492, compressing the tracking timeline to 70.5 s (vs 111.5 s) and
  inflating EE error slightly (0.197 m vs 0.195 m). The shortfall comes from
  releasing only a subset of the densified retimed samples per window; the
  replay log now exposes this for follow-up.
- `debug_compare_streaming` and `debug_diff_sync_inputs` capture the gap for
  triage; next step is to map release thresholds to all retimed samples below
  the horizon cut so the streaming variant reproduces the full offline
  timeline before we revisit ROS integration.

## Handoff notes (2024-04-09)
- The MATLAB refactor (including rolling replay instrumentation and the first
  streaming-retimer prototype) lives on branch `cpp-codegen-refactor`. Latest
  comparison artifacts are under `outputs/comparison/`; `debug_summary.txt`
  shows the streaming regression (287 samples vs 492) for tracking.
- Outstanding MATLAB tasks:
  * Fix streaming retimer stitching so all retimed samples under each release
    horizon are appended, restoring parity with the offline timeline.
  * Address IK limit warnings (0.197 m spike) by tightening collision/limit
    tuning once collision avoidance primitives are available.
- Next phase will happen on Linux (Ubuntu 22.04, ROS 2 Humble), matching the
  Jetson AGX Orin target. Set up a ROS workspace (`colcon build`) with MoveIt
  2.5.9, clone this repo, and branch from `cpp-codegen-refactor` to begin the
  MoveIt/ROS integration work.
- Initial Linux tasks:
  1. Port the streaming synchronizer and IK helpers to a C++ MoveIt node,
     consuming the URDF/SRDF in `mobile_arm_whole_body/` (use the new
     diagnostics as reference).
  2. Implement the ramp planner in C++ (Hybrid A* + warm-up choreography) and
     ensure it mirrors MATLAB outputs for `1_pull_world.json`.
  3. Build a regression harness: MATLAB diff scripts for the legacy/refactor
     parity plus ROS `launch_testing` cases that validate EE error, scale
     factor, and command velocities.
- Git checklist before switching machines:
  * Commit the streaming retimer work-in-progress along with `diary.md` notes.
  * Push branch `cpp-codegen-refactor` to origin so the Linux environment can
    continue from the same state.
