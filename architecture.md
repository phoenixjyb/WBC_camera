# Whole-Body Control Architecture

This document captures the planned pipeline for executing camera end-effector trajectories with the integrated arm+chassis system.

## Pipeline Summary
- **Camera EE trajectory goals** supply the desired path for the manipulator tip.
- **Trajectory builder / MoveIt client** packages those goals into whole-body planning requests.
- **move_group (MoveIt 2)** plans in joint space for the base planar joint and the six arm joints.
- **Trajectory splitter** separates the resulting trajectory into base vs arm segments while keeping timing aligned.
- **Arm controller** receives the six-joint trajectory over a `FollowJointTrajectory` interface.
- **Planar execution path** routes the base segment through a diff-drive tracker (or your existing chassis interface) to generate wheel commands.
- **State feedback loop** provides joint states and odometry back to MoveIt for consistent planning and visualization.

```mermaid
flowchart LR
    A[Camera EE Trajectory Goals] --> B[Trajectory Builder / MoveIt Client]
    B --> C(move_group)
    C --> D[Joint Trajectory (world + arm)]
    D --> E[Trajectory Splitter]
    E --> F[Arm Joint Trajectory]\n--> G[Arm Controller]
    E --> H[Planar Trajectory]
    H --> I[(Diff-Drive Tracker\nor Chassis Interface)]
    I --> J[Wheel-Speed Controller]
    G --> K[Arm Joint Feedback]
    J --> L[Base Odometry]
    K --> M[Joint State Publisher]
    L --> M
    M --> C
```

## Notes
- Replace the diff-drive tracker block with your existing chassis interface if it already converts planar trajectories to wheel speeds.
- The splitter is the key new node to implement next so whole-body plans drive both the arm controller and the base interface automatically.
