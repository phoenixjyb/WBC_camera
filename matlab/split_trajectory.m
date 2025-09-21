function [planar_traj, arm_traj] = split_trajectory(joint_traj)
% split_trajectory Separate planar and arm joints from a MoveIt-style joint trajectory.
%   joint_traj.joint_names : cell array of joint names.
%   joint_traj.points      : struct array with fields positions, time_from_start.
%   planar_traj, arm_traj  : structs containing joint_names and points.

planar_names = {'world_joint/x','world_joint/y','world_joint/theta'};
planar_idx = zeros(size(planar_names));
for i = 1:numel(planar_names)
    idx = find(strcmp(joint_traj.joint_names, planar_names{i}), 1);
    if isempty(idx)
        error('Missing planar joint %s in trajectory.', planar_names{i});
    end
    planar_idx(i) = idx;
end
arm_idx = setdiff(1:numel(joint_traj.joint_names), planar_idx, 'stable');

planar_traj.joint_names = joint_traj.joint_names(planar_idx);
arm_traj.joint_names = joint_traj.joint_names(arm_idx);

planar_points = repmat(struct('time_from_start', 0, 'positions', zeros(1,3)), numel(joint_traj.points), 1);
arm_points = repmat(struct('time_from_start', 0, 'positions', zeros(1,numel(arm_idx))), numel(joint_traj.points), 1);

for k = 1:numel(joint_traj.points)
    planar_points(k).time_from_start = joint_traj.points(k).time_from_start;
    planar_points(k).positions = joint_traj.points(k).positions(planar_idx);
    arm_points(k).time_from_start = joint_traj.points(k).time_from_start;
    arm_points(k).positions = joint_traj.points(k).positions(arm_idx);
end

planar_traj.points = planar_points;
arm_traj.points = arm_points;
end
