# Unique identifier for the target
uint32 id
# Desired camera pose in world frame
geometry_msgs/Pose pose
# Optional timestamp associated with this target
builtin_interfaces/Time stamp
# Position tolerance (meters)
float32 position_tolerance
# Orientation tolerance (radians)
float32 orientation_tolerance
