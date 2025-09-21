function limits = arm_joint_limits()
%ARM_JOINT_LIMITS Central place to adjust arm joint limits for retiming.
%   Returns a struct with fields:
%     velocity     - 1x6 vector [rad/s]
%     acceleration - 1x6 vector [rad/s^2]
%
%   Update these values as your hardware specifications evolve.

limits.velocity = [1.6, 1.6, 4.0, 4.0, 4.0, 4.0];
% Conservative accelerations derived from legacy specs (120–240 deg/s^2).
limits.acceleration = [2.0944, 2.0944, 2.6179, 3.1416, 3.1416, 4.1888];
end
