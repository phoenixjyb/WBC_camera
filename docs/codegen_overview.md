# MATLAB Coder Support for Whole-Body Kernels

To unblock the C++ port and embedded workflows, the essential timing and
synchronisation routines now expose MATLAB Coder friendly entry-points under
`matlab/+wbc_codegen/`.

## Available Entry Points

| Function | Purpose | Notes |
|----------|---------|-------|
| `wbc_codegen.sync_base_and_arm` | Wraps `wbc.sync_base_and_arm`, returning fixed-structure outputs (including `4x4xN` transforms) for code generation. | Use when you need the retimed arm trajectory, synchronized base profile, and diagnostics without Robotics System Toolbox dependencies. |
| `wbc_codegen.retime_joint_trajectory` | Wrapper for `helpers.retime_joint_trajectory` with plain-array arguments. | Accepts max velocity/acceleration vectors and emits retimed samples plus metadata. |
| `wbc_codegen.compute_tracking_metrics` | Lightweight tracking error calculator operating on precomputed EE positions and base poses. | Designed for verification loops that already have end-effector states from C++/ROS execution. |

## Generating C++

The helper script `matlab/codegen/run_wbc_codegen_examples.m` configures
representative argument sizes and invokes MATLAB Coder with a static library
target:

```matlab
% From matlab/
codegen_setup_path; % (if you have a path helper)
run_wbc_codegen_examples
```

Each invocation writes generated C++ into `wbc_codegen_*` folders (MATLAB
Coder defaults). Adjust the `N_MAX`/`M_MAX` constants in the script to match
your planning horizon or number of joints.

### Custom Builds

Alternatively, call `codegen` manually:

```matlab
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
baseLimits = struct('v_max', 0, 'omega_max', 0, 'lat_acc_max', 0);
codegen('-config', cfg, 'wbc_codegen.sync_base_and_arm', ...
    '-args', {... % provide coder.typeof inputs as needed
    });
```

Remember that Robotics System Toolbox objects such as `rigidBodyTree` are not
supported for code generation. The codegen entry points operate purely on
numeric inputs, so compute any kinematic data (e.g., end-effector positions)
using MoveIt or pre-generated datasets before invoking the MATLAB Coder
kernels.

## Next Steps

- Extend the codegen surface with additional helpers (e.g., ramp planning)
  once Robotics System Toolbox dependent sections are ported to pure numeric
  implementations.
- Integrate the generated C++ directly into the `wbc_core` library targeted
  by the ROS 2 coordinator.

