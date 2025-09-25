function [tSamples, qSamples, qdSamples, info] = retime_joint_trajectory(waypoints, maxVel, maxAcc, timeStep, minSegmentTime) %#codegen
%RETIME_JOINT_TRAJECTORY Codegen entry point for helpers.retime_joint_trajectory.
%   Simplifies the interface to plain numeric arrays compatible with C++
%   generation. Limits are provided as row vectors.

arguments
    waypoints double
    maxVel double
    maxAcc double
    timeStep double = 0.05
    minSegmentTime double = 0.1
end

opts = struct('MaxVelocity', maxVel, ...
              'MaxAcceleration', maxAcc, ...
              'TimeStep', timeStep, ...
              'MinSegmentTime', minSegmentTime);
[tSamples, qSamples, qdSamples, info] = helpers.retime_joint_trajectory(waypoints, opts);
end

