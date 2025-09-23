# Current GIK and Ramp Constraints

- **Arm ramp virtual goal**: matches the first waypoint's orientation and height; X/Y currently set to the home EE position, but can be loosened to allow smooth IK without yaw flips.
- **Chassis ramp yaw**: chassis ramps from home `[-2,-2,0]` to the first waypoint yaw, keeping the camera facing forward.
- **Stage boundaries**: `arm ramp`, `chassis ramp`, `tracking`, used by the animator and diagnostic tools.
- **Tracking orientation**: desired EE yaw is enforced on the chassis; geometry heading still used for curvature checks.
- **Chassis limits**: ramp speed capped at 0.2 m/s, tracking at 0.6 m/s; yaw rate/lat accel limits unchanged.

## Notes
- The arm ramp currently hits the orientation goal on the last sample, causing a large joint jump.
- Plan: relax or gradually apply the orientation requirement so joints move smoothly across all ramp samples.
