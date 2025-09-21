function metrics = compute_ee_metrics(robot, armJointNames, armTrajectory, basePose, times, eeName)
%COMPUTE_EE_METRICS Forward simulate end-effector pose and kinematics.
%
% metrics = helpers.compute_ee_metrics(robot, armJointNames, armTrajectory,
%                                       basePose, times, eeName)
% returns a struct containing world-frame end-effector positions, Euler
% angles, quaternions, and derived velocity/acceleration/jerk profiles.
%
% Inputs:
%   robot          rigidBodyTree of the arm mounted at the chassis origin.
%   armJointNames  cell array of arm joint names corresponding to columns
%                  in armTrajectory.
%   armTrajectory  NxM matrix of joint positions.
%   basePose       Nx3 array [x y yaw] describing chassis pose over time.
%   times          Nx1 vector of timestamps [s].
%   eeName         name of the end-effector link.
%
% Outputs (metrics struct):
%   .positions     Nx3 world-frame XYZ position.
%   .quaternions   Nx4 quaternion [w x y z].
%   .rpy           Nx3 roll/pitch/yaw (XYZ convention).
%   .velocity      Nx3 linear velocity in world frame.
%   .speed         Nx1 linear speed magnitude.
%   .acceleration  Nx3 linear acceleration.
%   .accel_mag     Nx1 acceleration magnitude.
%   .jerk          Nx3 linear jerk.
%   .jerk_mag      Nx1 jerk magnitude.

arguments
    robot (1,1) rigidBodyTree
    armJointNames (1,:) cell
    armTrajectory double
    basePose double
    times double
    eeName (1,:) char
end

numSteps = size(armTrajectory, 1);
if size(basePose,1) ~= numSteps
    error('Base pose and arm trajectory must have the same number of samples.');
end
if numel(times) ~= numSteps
    error('Time vector must have the same length as the trajectory.');
end

configTemplate = homeConfiguration(robot);
vectorNames = {configTemplate.JointName};
armIdx = zeros(1, numel(armJointNames));
for j = 1:numel(armJointNames)
    idx = find(strcmp(vectorNames, armJointNames{j}), 1);
    if isempty(idx)
        error('Joint %s not present in robot model.', armJointNames{j});
    end
    armIdx(j) = idx;
end

positions = zeros(numSteps, 3);
quaternions = zeros(numSteps, 4);
rpy = zeros(numSteps, 3);

config = configTemplate;
for k = 1:numSteps
    for j = 1:numel(armIdx)
        config(armIdx(j)).JointPosition = armTrajectory(k, j);
    end
    Tarm = getTransform(robot, config, eeName);
    Tbase = trvec2tform([basePose(k,1), basePose(k,2), 0]) * axang2tform([0 0 1 basePose(k,3)]);
    Tworld = Tbase * Tarm;
    positions(k,:) = Tworld(1:3,4)';
    rotm = Tworld(1:3,1:3);
    quaternions(k,:) = rotm2quat(rotm);
    eul = rotm2eul(rotm, 'ZYX');
    rpy(k,:) = [eul(3), eul(2), eul(1)];
end

metrics.positions = positions;
metrics.quaternions = quaternions;
metrics.rpy = rpy;
metrics.time = times(:);

% Kinematic derivatives
metrics.velocity = compute_derivative(positions, times);
metrics.speed = sqrt(sum(metrics.velocity.^2, 2));
metrics.acceleration = compute_derivative(metrics.velocity, times);
metrics.accel_mag = sqrt(sum(metrics.acceleration.^2, 2));
metrics.jerk = compute_derivative(metrics.acceleration, times);
metrics.jerk_mag = sqrt(sum(metrics.jerk.^2, 2));

end

function deriv = compute_derivative(data, time)
if size(data,1) ~= numel(time)
    error('Data rows must match time vector length.');
end
n = size(data,1);
deriv = zeros(size(data));
if n == 1
    return;
end
for col = 1:size(data,2)
    deriv(:,col) = gradient(data(:,col), time);
end
end
