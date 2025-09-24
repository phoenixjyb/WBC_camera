% run_arm_ramp_inspection.m
% Inspect the arm warm-up ramp: solve GIK target, plot joint interpolation,
% plot end-effector path, and render a slow playback animation.

clearvars; clc;

% Trajectory overrides ---------------------------------------------------------
traj_source = 'file';
traj_file = '1_pull_world.json';
traj_duration = 12;
traj_scale = 1.0;
use_gik = true;

% Disable default animation so we can render a slow, ramp-only video later.
enable_animation = false;

% Run the full controller to populate ramp metadata and synchronized results.
rt_whole_body_controller;

% Retrieve outputs from the base workspace.
rt_results = evalin('base', 'rt_results');
rampInfo = rt_results.baseInitialization;
if rampInfo.armSteps == 0 || isempty(rampInfo.armJointTrajectory)
    error('run_arm_ramp_inspection:MissingRamp', 'Arm ramp trajectory is empty.');
end

% Load robot model for FK sampling.
thisDir = fileparts(mfilename('fullpath'));
urdfPath = fullfile(thisDir, '..', 'mobile_arm_whole_body', 'urdf', 'arm_on_car_center_rotZ_neg90.urdf');
robot = importrobot(urdfPath, 'DataFormat', 'struct');
robot.Gravity = [0 0 -9.81];
armJointNames = {'left_arm_joint1','left_arm_joint2','left_arm_joint3', ...
                 'left_arm_joint4','left_arm_joint5','left_arm_joint6'};
eeName = 'left_gripper_link';
config = homeConfiguration(robot);
configNames = {config.JointName};
armIdx = zeros(1, numel(armJointNames));
for j = 1:numel(armJointNames)
    idx = find(strcmp(configNames, armJointNames{j}), 1);
    if isempty(idx)
        error('run_arm_ramp_inspection:JointMissing', 'Joint %s not found in robot model.', armJointNames{j});
    end
    armIdx(j) = idx;
end

% Extract ramp data.
qRamp = rampInfo.armJointTrajectory;
qdRamp = rampInfo.armJointVelocity;
qddRamp = rampInfo.armJointAcceleration;
armTime = rampInfo.armTimeVector(:);

% Compute EE world positions for the arm ramp (base held at home pose).
homeBasePose = [-2, -2, 0];
Tbase = trvec2tform([homeBasePose(1:2), 0]) * axang2tform([0 0 1 homeBasePose(3)]);
eeWorld = zeros(size(qRamp,1), 3);
for k = 1:size(qRamp,1)
    for j = 1:numel(armIdx)
        config(armIdx(j)).JointPosition = qRamp(k, j);
    end
    Tee = getTransform(robot, config, eeName);
    TeeWorld = Tbase * Tee;
    eeWorld(k, :) = TeeWorld(1:3,4)';
end

% Prepare output directory.
outputDir = fullfile(thisDir, 'outputs');
if ~isfolder(outputDir)
    mkdir(outputDir);
end
prefix = 'arm_ramp_inspection';

% Plot joint positions and velocities.
figJoint = figure('Name', 'Arm Ramp Joint Positions');
hold on; grid on;
colors = lines(numel(armJointNames));
for j = 1:numel(armJointNames)
    plot(armTime, rad2deg(qRamp(:,j)), 'Color', colors(j,:), 'LineWidth', 1.5, ...
        'DisplayName', armJointNames{j});
end
xlabel('Time (s)'); ylabel('Joint angle (deg)');
title('Arm ramp joint positions (quintic interpolation)');
legend('Location','best');
exportgraphics(figJoint, fullfile(outputDir, sprintf('%s_joint_positions.png', prefix)), 'Resolution', 200);

figJointVel = figure('Name', 'Arm Ramp Joint Velocities');
hold on; grid on;
for j = 1:numel(armJointNames)
    plot(armTime, qdRamp(:,j), 'Color', colors(j,:), 'LineWidth', 1.5, ...
        'DisplayName', armJointNames{j});
end
xlabel('Time (s)'); ylabel('Joint velocity (rad/s)');
title('Arm ramp joint velocities');
legend('Location','best');
exportgraphics(figJointVel, fullfile(outputDir, sprintf('%s_joint_velocities.png', prefix)), 'Resolution', 200);

% Plot EE Cartesian coordinates across the ramp.
figEE = figure('Name', 'Arm Ramp End-Effector Path');
subplot(2,2,1);
plot3(eeWorld(:,1), eeWorld(:,2), eeWorld(:,3), 'b-o', 'LineWidth', 1.5, 'MarkerSize', 6);
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('EE path in world frame'); view(45,30);
subplot(2,2,2);
plot(armTime, eeWorld(:,1), 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('X (m)'); title('EE X vs time');
subplot(2,2,3);
plot(armTime, eeWorld(:,2), 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Y (m)'); title('EE Y vs time');
subplot(2,2,4);
plot(armTime, eeWorld(:,3), 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Z (m)'); title('EE Z vs time');
exportgraphics(figEE, fullfile(outputDir, sprintf('%s_ee_path.png', prefix)), 'Resolution', 200);

% Render a slow arm-ramp animation using the warm-up trajectory only.
videoFile = fullfile(outputDir, sprintf('%s_arm_ramp_slow.mp4', prefix));
basePoseRamp = [repmat(homeBasePose(1:2), numel(armTime), 1), repmat(homeBasePose(3), numel(armTime), 1)];
helpers.animate_whole_body(robot, armJointNames, qRamp, armTime, basePoseRamp, armTime, eeWorld, ...
    'StageBoundaries', numel(armTime), 'StageLabels', "Arm ramp", 'StageSelection', 'all', ...
    'PlaybackSpeed', 0.2, 'VideoFrameRate', 15, 'VideoFile', videoFile, 'EndEffectorName', eeName);

fprintf('Arm ramp inspection artifacts saved in %s\n', outputDir);
