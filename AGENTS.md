# Repository Guidelines

## Project Structure & Module Organization
`mobile_arm_whole_body/` is the ROS 2 package that ships URDF/SRDF models (`urdf/`, `srdf/`), tuning files in `config/`, launch entry points in `launch/`, and placeholder integration code under `scripts/`. MATLAB tooling lives in `matlab/`, with shared helpers under `+helpers/`, batch runners (for example `run_camera_motion_tests.m`), and sample MoveIt exports such as `1_pull_world.json`. Documentation notes sit in `docs/`, while generated artifacts should stay in `matlab/outputs/` and out of version control.

## Build, Test, and Development Commands
From a ROS 2 workspace, build the package with `colcon build --symlink-install` and source the overlay via `source install/setup.bash`. Launch the demo stack with `ros2 launch mobile_arm_whole_body whole_body_demo.launch.py` to spin up `robot_state_publisher`, `move_group`, and RViz. In MATLAB, prefer non-interactive runs: `matlab -batch "run_camera_motion_tests"` exercises the canned trajectories; `matlab -batch "rt_whole_body_controller"` drives a single scenario and drops plots/data into `matlab/outputs/`.

## Coding Style & Naming Conventions
ROS assets mirror upstream MoveIt expectations—use lowercase snake_case for filenames, keep YAML keys aligned, and group related parameters in the existing config files. Python nodes should follow PEP 8 (4-space indents, descriptive log messages) and stay executable (`#!/usr/bin/env python3`). MATLAB scripts in this repo follow snake_case filenames, top-of-file comments, and 4-space indents inside control blocks; favor vectorized math and place reusable logic in `+helpers/` packages.

## Testing Guidelines
Before proposing controller or config changes, run `matlab -batch "run_camera_motion_tests"` and confirm the printed velocity summary stays within documented joint/base limits. When adjusting the ROS package, validate planners by launching the demo, sampling Start/Goal pairs in RViz, and ensuring the FakeController executes without warnings. Attach any new regression assets to `matlab/outputs/` locally but avoid committing raw MAT/JSON outputs; instead, document reproducible steps in your PR.

## Commit & Pull Request Guidelines
Keep commits focused and message them in the style of the history (component first, present-tense summary, e.g., `mobile-base: tune diff-drive limits`). PRs should link tracking issues, outline the motivation, note which commands were run (`colcon build`, MATLAB batches, RViz checks), and include screenshots or logs when behavior changes. Highlight impacts on robot limits or controller gains so reviewers can mirror the setup.
