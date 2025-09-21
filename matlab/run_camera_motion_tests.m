% run_camera_motion_tests.m
% Batch runner to exercise classic camera trajectories without animation.

clear; clc;

thisDir = fileparts(mfilename('fullpath'));
addpath(thisDir);

sources = ["dolly_forward", "orbit_left", "crane_up", "pan_tilt", "demo_arc", "demo_line"];
traj_duration = 8.0;
enable_animation = false;

results_table = struct('source', {}, 'maxJointVel', {}, 'maxBaseSpeed', {}, 'duration', {});

for idx = 1:numel(sources)
    traj_source = sources(idx); %#ok<NASGU>
    fprintf('\n=== Running trajectory: %s ===\n', traj_source);
    rt_whole_body_controller;
    res = rt_results;
    maxJointVel = max(abs(res.armVelocities), [], 1);
    maxBaseSpeed = max(abs(res.baseCmd(:,1)));
    results_table(idx).source = traj_source;
    results_table(idx).maxJointVel = maxJointVel;
    results_table(idx).maxBaseSpeed = maxBaseSpeed;
    results_table(idx).duration = res.armTimes(end);
    close all force;
end

fprintf('\nSummary (rad/s per joint):\n');
for idx = 1:numel(results_table)
    fprintf('%-12s  max joint vel [rad/s] = %s  | max base speed = %.3f m/s\n', ...
        results_table(idx).source, mat2str(results_table(idx).maxJointVel, 3), results_table(idx).maxBaseSpeed);
end

assignin('base', 'camera_motion_results', results_table);
