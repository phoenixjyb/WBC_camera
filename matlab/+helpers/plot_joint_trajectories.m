function plot_joint_trajectories(timeVec, jointTraj, jointNames)
%PLOT_JOINT_TRAJECTORIES Plot joint positions vs time.
%   plot_joint_trajectories(timeVec, jointTraj, jointNames) creates a figure
%   with joint angles (radians) over the provided time vector.
%
%   Inputs:
%     timeVec   - 1xN or Nx1 vector of timestamps (seconds).
%     jointTraj - NxM matrix where each column corresponds to a joint.
%     jointNames- 1xM cell array of joint name strings.

arguments
    timeVec double
    jointTraj double
    jointNames (1,:) cell
end

timeVec = timeVec(:);
if size(jointTraj,1) ~= numel(timeVec)
    error('Rows of jointTraj (%d) must match numel(timeVec) (%d).', size(jointTraj,1), numel(timeVec));
end

fig = figure('Name', 'Arm Joint Trajectories', 'Color', 'w');
ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on');
plot(ax, timeVec, jointTraj, 'LineWidth', 1.5);
xlabel(ax, 'Time [s]'); ylabel(ax, 'Joint position [rad]');
if nargin >= 3 && numel(jointNames) == size(jointTraj,2)
    legend(ax, jointNames, 'Location', 'bestoutside');
end
title(ax, 'Arm Joint Angles vs Time');
end
