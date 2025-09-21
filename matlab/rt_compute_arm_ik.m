function [qMatrix, jointNames, infos] = rt_compute_arm_ik(robot, eeName, poseTforms, weights, initialGuess)
%RT_COMPUTE_ARM_IK Solve IK for a series of end-effector poses using Robotics System Toolbox.
%   [qMatrix, jointNames, infos] = rt_compute_arm_ik(robot, eeName, poseTforms)
%   returns a matrix of joint positions (rows = poses) corresponding to the
%   rigidBodyTree joints. poseTforms should be a cell array of 4x4 homogeneous
%   transforms or an N-by-16 array of row-major transforms.
%
%   Optional arguments:
%     weights      - 1x6 weight vector for inverseKinematics (default [0.5 0.5 0.5 1 1 1])
%     initialGuess - rigidBodyTree configuration struct array (default homeConfiguration)
%
%   Outputs:
%     qMatrix      - N-by-M matrix of joint angles (M = number of non-fixed joints)
%     jointNames   - 1-by-M cell array of joint names matching qMatrix columns
%     infos        - struct array of IK solver metadata for each pose
%
%   This helper expects MATLAB Robotics System Toolbox.

arguments
    robot (1,1) rigidBodyTree
    eeName (1,:) char
    poseTforms
    weights (1,6) double = [0.5 0.5 0.5 1 1 1]
    initialGuess = homeConfiguration(robot)
end

% Normalize pose inputs
if iscell(poseTforms)
    numPoses = numel(poseTforms);
    tforms = poseTforms;
elseif isnumeric(poseTforms)
    % Assume row-major flattened matrices
    numPoses = size(poseTforms, 1);
    tforms = cell(1, numPoses);
    for i = 1:numPoses
        tforms{i} = reshape(poseTforms(i, :), [4, 4])';
    end
else
    error('poseTforms must be a cell array of transforms or an N-by-16 numeric array.');
end

configGuess = initialGuess;
ik = inverseKinematics('RigidBodyTree', robot);
numJoints = numel(initialGuess);
qMatrix = zeros(numPoses, numJoints);
infos = cell(1, numPoses);

for i = 1:numPoses
    [configSol, info] = ik(eeName, tforms{i}, weights, configGuess);
    configGuess = configSol; % warm start next solve
    for j = 1:numJoints
        qMatrix(i, j) = configSol(j).JointPosition;
    end
    infos{i} = info;
end

jointNames = arrayfun(@(c) c.JointName, initialGuess, 'UniformOutput', false);
end
