# Whole-Body Controller C++ Conversion Plan

## Status Summary
- **Target platform:** NVIDIA Jetson AGX Orin running Ubuntu 22.04, ROS 2 Humble, MoveIt 2.5.9.
- **Current state:** Planning and requirements gathering.
- **Primary goal:** Port MATLAB whole-body controller to a deployable C++/ROS 2 stack with matching performance and feature parity.
- **References:** `matlab/rt_whole_body_controller.m`, `matlab/+helpers/*.m`, `mobile_arm_whole_body/` URDF/SRDF assets.

## Phase 0 – Platform Baseline & Requirements
- Lock toolchain (colcon, compiler flags, CUDA/Nv libraries) and document build instructions.
- Capture actuator limits, chassis footprint, collision meshes, and compare with MATLAB defaults.
- Define ROS 2 interface contracts (topics/services/actions) for EE goals, chassis commands, arm trajectories, and state transitions.
- Identify third-party dependencies (MoveIt, Nav2 Hybrid A*, planners) and performance expectations vs MATLAB codegen.

## Phase 1 – MATLAB Refactor for Code Generation
- Wrap the main script as a function (e.g., `whole_body_controller_run(opts, overrides)`) with typed structs for inputs and a fixed-schema `results` return, leaving shim scripts for backwards compatibility.
- Guard visualization, logging, and filesystem side-effects behind a `enableVisualization` flag so codegen paths stay clean while MATLAB runs keep full plotting/animation behavior.
- Replace dynamic struct growth with predefined coder-friendly types (`coder.cstructname`), preallocate arrays to max sizes, and convert optional data to flag-controlled fixed fields.
- Extract computational kernels (reference prep, ramp planner, IK, synchronization, metrics) into pure functions with explicit signatures to ease MATLAB Coder usage and future C++ ports.
- Provide MATLAB Coder entry points under `+wbc_codegen/` and the helper script `matlab/codegen/run_wbc_codegen_examples.m` to generate C++ for retiming, synchronization, and metric evaluation kernels.
- Inventory Robotics System Toolbox dependencies; decide where to retain MATLAB implementations vs. expect MoveIt equivalents in C++, documenting any codegen limitations.
- Author MATLAB unit tests that compare new function outputs against legacy baselines (e.g., `1_pull_world.json`) within defined tolerances, ensuring behavior parity during refactors.

### Kernel Extraction Targets (MATLAB → Codegen)
- **Reference preparation** (`rt_whole_body_controller.m:115-210`): loading trajectories, smoothing base waypoints, heading computation, ramp seed assembly → new helper `wbc.prepare_reference_data` returning deterministic structs.
- **Ramp synthesis** (`prepend_ramp_segment`, `compute_arm_ramp_goal`, `compute_chassis_ramp_goal`, seed builders): promote to `wbc.plan_ramp_segment` module that encapsulates arm warm-up IK, Hybrid A* base path, and metadata logging.
- **IK solving stages** (nominal, refined, final IK blocks around lines 223-356): now wrapped by `wbc.solve_arm_ik`, keeping RT IK/GIK behaviour consistent while preparing for codegen/C++ reuse.
- **Synchronization & retiming** (`helpers.retime_joint_trajectory`, `synchronize_base_trajectory`, curvature/limit checks): expose as `wbc.sync_base_and_arm` operating on struct inputs (arm/base limits).
- **Metrics & diagnostics** (`compute_ee_metrics`, error stats, stage boundaries, logging) → consolidate under `wbc.compute_metrics` for reuse in tests and ROS diagnostics.
- Each helper should accept coder-friendly structs, avoid dynamic allocation, and emit outputs that feed directly back into the orchestrator without post-processing tweaks.

## Phase 2 – Algorithm Specification & Validation Assets
- Document inputs/outputs, assumptions, and numerical tolerances for each kernel (ramp planner, retimer, IK helpers, synchronizer, tracker, metrics).
- Capture edge-case regression cases (no ramp, reverse driving, high curvature, collision obstacles) and expected outcomes.
- Produce serialized reference data (MAT/JSON/CSV) to drive cross-language regression for later phases.

## Phase 3 – C++ Core Library (`wbc_core`)
- Implement ramp planner (MoveIt/Nav2 Hybrid A* or custom) honoring velocity/yaw limits and collision footprints derived from MATLAB logic.
- Port trajectory retimer (trapezoidal/quintic) to match MATLAB timing and joint-limit handling.
- Integrate MoveIt 2.5.9 IK pipelines mirroring `rt_compute_arm_ik`/GIK behavior; ensure collision constraints replicate MATLAB configuration.
- A new helper (`mobile_arm_whole_body::MoveItIKSolver`) provides `robot_state::setFromIK` driven solves with collision checking, warm-starts, and MoveIt planning-scene integration; wire this into the C++ coordinator in place of MATLAB GIK calls.
- See `docs/moveit_ik_solver.md` for usage patterns and configuration tips when adopting the MoveIt-based solver.
- Implement base/arm synchronization, EE pose reconstruction, and metric calculations with deterministic outputs.
- Maintain parity with MATLAB staging metadata (ramp steps, tracking start index, diagnostics).

## Phase 4 – ROS 2 Integration & State Machine
- Develop `WholeBodyCoordinator` rclcpp node managing modes (`Idle`, `ArmRamp`, `ChassisRamp`, `Tracking`) and publishing command topics.
- Implement services/actions for mode control, trajectory submission, and diagnostic retrieval.
- Bridge MoveIt planning scene updates, chassis planner feedback, and dynamic obstacle inputs (from `chassis_obstacles`).
- Provide parameter YAML files reflecting MATLAB limits (`arm_joint_limits`, base velocity caps) under `mobile_arm_whole_body/config/`.
- Follow the real-time streaming design outlined in `ros2_realtime_execution.md` to meet 10 Hz EE updates while issuing high-rate arm/base commands on AGX Orin.

## Phase 5 – Verification & Continuous Integration
- Build regression harness comparing C++ outputs against MATLAB reference datasets (EE path, joint profiles, base yaw, timing).
- Create simulation launch (MoveIt FakeController + RViz) for manual validation; capture automated test cases with ros2 launch testing.
- Retain MATLAB animation/plotting by ingesting C++ outputs (CSV/JSON) for visual QA.
- Integrate tests into CI (colcon test) with tolerance assertions.

## Phase 6 – Deployment on AGX Orin
- Establish native and cross-compilation workflows; profile runtime with representative trajectories.
- Validate real-time performance (latency, jitter) and optimize threading/affinity settings.
- Package launch files, ROS 2 parameters, and systemd/container scripts for production deployment.
- Add monitoring/logging hooks (diagnostics, tracing) for on-robot observability.

## Phase 7 – Python Wrapper (Nice-to-have)
- Expose C++ core via pybind11 for scripting/regression automation.
- Provide CLI/ROS 2 tools to trigger ramps and record outputs for MATLAB comparison.
- Document developer workflow for mixed MATLAB/Python validation loops.

## Next Immediate Actions
- Confirm collision mesh fidelity and IK constraint mapping between MATLAB and MoveIt configurations.
- Begin MATLAB code cleanup branch focused on codegen compatibility for `rt_whole_body_controller` and helper kernels.
- Draft ROS 2 interface definitions (msgs/srvs/actions) to unblock C++ API design.
- Prototype the streaming coordinator skeleton based on `docs/ros2_realtime_execution.md`, using replayed MATLAB baselines for timing validation.
