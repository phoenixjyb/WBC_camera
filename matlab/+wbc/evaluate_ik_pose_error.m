function ikError = evaluate_ik_pose_error(robot, armJointNames, armTrajectory, poseTformsTarget, eeName)
%EVALUATE_IK_POSE_ERROR Compare realized EE poses against targets.
%   Returns translation/orientation error magnitudes for each sample.

arguments
    robot (1,1) rigidBodyTree
    armJointNames cell
    armTrajectory double
    poseTformsTarget
    eeName (1,:) char
end

numSamples = size(armTrajectory,1);
configTemplate = homeConfiguration(robot);
vectorNames = {configTemplate.JointName};
armIdx = zeros(1, numel(armJointNames));
for j = 1:numel(armJointNames)
    idx = find(strcmp(vectorNames, armJointNames{j}), 1);
    if isempty(idx)
        error('wbc:evaluateIKPoseError:JointMissing', 'Joint %s not found in robot model.', armJointNames{j});
    end
    armIdx(j) = idx;
end

translationErr = zeros(numSamples,1);
orientationErr = zeros(numSamples,1);
config = configTemplate;
for k = 1:numSamples
    for j = 1:numel(armIdx)
        config(armIdx(j)).JointPosition = armTrajectory(k, j);
    end
    Tactual = getTransform(robot, config, eeName);
    if iscell(poseTformsTarget)
        Ttarget = poseTformsTarget{k};
    else
        Ttarget = reshape(poseTformsTarget(k, :), [4,4])';
    end
    translationErr(k) = norm(Tactual(1:3,4) - Ttarget(1:3,4));
    Rrel = Ttarget(1:3,1:3)' * Tactual(1:3,1:3);
    axang = rotm2axang(Rrel);
    orientationErr(k) = abs(wrapToPi(axang(4)));
end

ikError.translation = translationErr;
ikError.orientation = orientationErr;
ikError.orientation_deg = rad2deg(orientationErr);
end
