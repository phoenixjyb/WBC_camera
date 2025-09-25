% run_camera_motion_tests.m
% Batch runner to exercise classic camera trajectories without animation.

clear; clc;

thisDir = fileparts(mfilename('fullpath'));
addpath(thisDir);

sources = ["dolly_forward", "orbit_left", "crane_up", "pan_tilt", "demo_arc", "demo_line"];
traj_duration = 8.0;

results_table = struct('source', {}, 'maxJointVel', {}, 'maxBaseSpeed', {}, 'duration', {});

for idx = 1:numel(sources)
    fprintf('\n=== Running trajectory: %s ===\n', sources(idx));
    opts = struct('traj_source', sources(idx), ...
                  'traj_duration', traj_duration, ...
                  'enable_animation', false, ...
                  'enable_visualization', false);
    res = rt_whole_body_controller(opts);
    maxJointVel = max(abs(res.armVelocities), [], 1);
    maxBaseSpeed = max(abs(res.baseCmd(:,1)));
    results_table(idx).source = sources(idx);
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
