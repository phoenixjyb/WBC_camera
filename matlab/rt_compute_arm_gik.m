function [qMatrix, jointNames, infos] = rt_compute_arm_gik(robot, eeName, poseTforms, options)
%RT_COMPUTE_ARM_GIK Solve constrained IK using generalizedInverseKinematics.
%   [qMatrix, jointNames, infos] = rt_compute_arm_gik(robot, eeName, poseTforms, options)
%   mirrors rt_compute_arm_ik but exposes constraint-based solving.
%
%   options fields (all optional):
%     InitialGuess         - rigidBodyTree configuration for warm start
%     PositionTolerance    - scalar or vector tolerance (m) for EE position
%     OrientationTolerance - scalar or vector tolerance (rad) for EE orientation
%     JointBounds          - struct with fields JointNames, Lower, Upper
%     ExtraConstraints     - cell array of additional constraint objects
%     UseWarmStart         - logical flag (default true)
%
%   If generalized IK constraints are unavailable in the current MATLAB
%   installation, this function throws an error.

if nargin < 4
    options = struct();
end

if ~isfield(options, 'InitialGuess') || isempty(options.InitialGuess)
    options.InitialGuess = homeConfiguration(robot);
end
if ~isfield(options, 'PositionTolerance') || isempty(options.PositionTolerance)
    options.PositionTolerance = 0.005;
end
if ~isfield(options, 'OrientationTolerance') || isempty(options.OrientationTolerance)
    options.OrientationTolerance = deg2rad(3);
end
if ~isfield(options, 'JointBounds')
    options.JointBounds = struct([]);
end
if ~isfield(options, 'ExtraConstraints') || isempty(options.ExtraConstraints)
    options.ExtraConstraints = {};
end
if ~isfield(options, 'UseWarmStart') || isempty(options.UseWarmStart)
    options.UseWarmStart = true;
end
if ~isfield(options, 'DistanceConstraints') || isempty(options.DistanceConstraints)
    options.DistanceConstraints = {};
end
if ~isfield(options, 'CartesianBounds') || isempty(options.CartesianBounds)
    options.CartesianBounds = {};
end
if iscell(options.DistanceConstraints)
    options.DistanceConstraints = options.DistanceConstraints(~cellfun(@isempty, options.DistanceConstraints));
end
if iscell(options.CartesianBounds)
    options.CartesianBounds = options.CartesianBounds(~cellfun(@isempty, options.CartesianBounds));
end
if iscell(options.ExtraConstraints)
    options.ExtraConstraints = options.ExtraConstraints(~cellfun(@isempty, options.ExtraConstraints));
end

[posCells, numPoses] = normalizeTransforms(poseTforms);
configGuess = options.InitialGuess;
[jointNames, lowerBounds, upperBounds] = jointBoundsFromRobot(robot, configGuess, options.JointBounds);

% Require constraint classes provided by Robotics System Toolbox
validateConstraintAvailability(options);

posTarget = constraintPositionTarget(eeName);
posTarget.ReferenceBody = robot.BaseName;
posTol = scalarTolerance(options.PositionTolerance);
posTarget.PositionTolerance = posTol;

oriTarget = constraintOrientationTarget(eeName);
oriTarget.ReferenceBody = robot.BaseName;
oriTol = scalarTolerance(options.OrientationTolerance);
oriTarget.OrientationTolerance = oriTol;

jointBounds = constraintJointBounds(robot);
jointBounds.Bounds = [lowerBounds, upperBounds];

numDistance = numel(options.DistanceConstraints);
distanceObjects = cell(1, numDistance);
for k = 1:numel(options.DistanceConstraints)
    entry = options.DistanceConstraints{k};
    if iscell(entry)
        entry = entry{1};
    end
    endEff = entry.EndEffector;
    if iscell(endEff)
        endEff = endEff{1};
    end
    distObj = constraintDistanceBounds(endEff);
    if isfield(entry, 'ReferenceBody') && ~isempty(entry.ReferenceBody)
        refBody = entry.ReferenceBody;
        if iscell(refBody)
            refBody = refBody{1};
        end
        distObj.ReferenceBody = refBody;
    else
        distObj.ReferenceBody = robot.BaseName;
    end
    if isfield(entry, 'Bounds') && ~isempty(entry.Bounds)
        bounds = entry.Bounds;
        bounds = makeFinite(bounds);
        distObj.Bounds = bounds;
    end
    if isfield(entry, 'Weights') && ~isempty(entry.Weights)
        distObj.Weights = entry.Weights;
    end
    distanceObjects{k} = distObj;
end

numCartesian = numel(options.CartesianBounds);
cartesianObjects = cell(1, numCartesian);
for k = 1:numel(options.CartesianBounds)
    entry = options.CartesianBounds{k};
    if iscell(entry)
        entry = entry{1};
    end
    endEff = entry.EndEffector;
    if iscell(endEff)
        endEff = endEff{1};
    end
    cartObj = constraintCartesianBounds(endEff);
    if isfield(entry, 'ReferenceBody') && ~isempty(entry.ReferenceBody)
        refBody = entry.ReferenceBody;
        if iscell(refBody)
            refBody = refBody{1};
        end
        cartObj.ReferenceBody = refBody;
    else
        cartObj.ReferenceBody = robot.BaseName;
    end
    if isfield(entry, 'TargetTransform') && ~isempty(entry.TargetTransform)
        cartObj.TargetTransform = entry.TargetTransform;
    end
    if isfield(entry, 'Bounds') && ~isempty(entry.Bounds)
        cartObj.Bounds = entry.Bounds;
    end
    if isfield(entry, 'Weights') && ~isempty(entry.Weights)
        cartObj.Weights = entry.Weights;
    end
    cartesianObjects{k} = cartObj;
end

constraintInputs = {'position','orientation','jointbounds'};
if numDistance > 0
    constraintInputs = [constraintInputs, repmat({'distance'}, 1, numDistance)];
end
if numCartesian > 0
    constraintInputs = [constraintInputs, repmat({'cartesian'}, 1, numCartesian)];
end
if ~isempty(options.ExtraConstraints)
    extraInputs = cellfun(@classToConstraintInput, options.ExtraConstraints, 'UniformOutput', false);
    constraintInputs = [constraintInputs, extraInputs(:)'];
end

gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', constraintInputs);

numJoints = numel(configGuess);
qMatrix = zeros(numPoses, numJoints);
infos = cell(1, numPoses);

for idx = 1:numPoses
    T = posCells{idx};
    posTarget.TargetPosition = tform2trvec(T);
    oriTarget.TargetOrientation = tform2quat(T);
    constraintArgs = {posTarget, oriTarget, jointBounds};
    if numDistance > 0
        constraintArgs = [constraintArgs, distanceObjects];
    end
    if numCartesian > 0
        constraintArgs = [constraintArgs, cartesianObjects];
    end
    if ~isempty(options.ExtraConstraints)
        constraintArgs = [constraintArgs, options.ExtraConstraints];
    end
    [configSol, solutionInfo] = gik(configGuess, constraintArgs{:});
    infos{idx} = solutionInfo;
    for j = 1:numJoints
        qMatrix(idx, j) = configSol(j).JointPosition;
    end
    if options.UseWarmStart
        configGuess = configSol;
    end
end

end

function tolScalar = scalarTolerance(val)
if isscalar(val)
    tolScalar = val;
else
    tolScalar = max(abs(val(:)));
end
end

function [tforms, numPoses] = normalizeTransforms(poseTforms)
if iscell(poseTforms)
    numPoses = numel(poseTforms);
    tforms = poseTforms;
elseif isnumeric(poseTforms)
    numPoses = size(poseTforms, 1);
    tforms = cell(1, numPoses);
    for i = 1:numPoses
        tforms{i} = reshape(poseTforms(i, :), [4, 4])';
    end
else
    error('poseTforms must be a cell array or an N-by-16 numeric array.');
end
end

function [jointNames, lowerBounds, upperBounds] = jointBoundsFromRobot(robot, config, customBounds)
numJoints = numel(config);
jointNames = cell(1, numJoints);
lowerBounds = -inf(numJoints, 1);
upperBounds = inf(numJoints, 1);

overrideLower = containers.Map('KeyType','char','ValueType','double');
overrideUpper = containers.Map('KeyType','char','ValueType','double');
if ~isempty(customBounds)
    if isfield(customBounds, 'JointNames') && isfield(customBounds, 'Lower') && isfield(customBounds, 'Upper')
        for k = 1:numel(customBounds.JointNames)
            name = char(customBounds.JointNames(k));
            overrideLower(name) = customBounds.Lower(k);
            overrideUpper(name) = customBounds.Upper(k);
        end
    else
        error('JointBounds structure must contain JointNames, Lower, and Upper fields.');
    end
end

for idx = 1:numJoints
    name = config(idx).JointName;
    jointNames{idx} = name;
    jointObj = [];
    for bodyIdx = 1:numel(robot.Bodies)
        candidate = robot.Bodies{bodyIdx};
        if strcmp(candidate.Joint.Name, name)
            jointObj = candidate.Joint;
            break;
        end
    end
    if ~isempty(jointObj) && ~isempty(jointObj.PositionLimits)
        limits = jointObj.PositionLimits;
        lowerBounds(idx) = limits(1);
        upperBounds(idx) = limits(2);
    end
    if overrideLower.isKey(name)
        lowerBounds(idx) = overrideLower(name);
    end
    if overrideUpper.isKey(name)
        upperBounds(idx) = overrideUpper(name);
    end
end
end

function name = classToConstraintInput(obj)
className = class(obj);
if contains(className, 'Position', 'IgnoreCase', true)
    name = 'position';
elseif contains(className, 'Orientation', 'IgnoreCase', true)
    name = 'orientation';
elseif contains(className, 'JointBounds', 'IgnoreCase', true)
    name = 'jointbounds';
elseif contains(className, 'CartesianBounds', 'IgnoreCase', true)
    name = 'cartesian';
elseif contains(className, 'DistanceBounds', 'IgnoreCase', true)
    name = 'distance';
elseif contains(className, 'Aiming', 'IgnoreCase', true)
    name = 'aiming';
else
    error('Unsupported constraint class %s for automatic input mapping.', className);
end
end

function boundsOut = makeFinite(boundsIn)
boundsOut = boundsIn;
idxPos = ~isfinite(boundsOut) & boundsOut > 0;
idxNeg = ~isfinite(boundsOut) & boundsOut < 0;
boundsOut(idxPos) = 1e6;
boundsOut(idxNeg) = -1e6;
boundsOut(~isfinite(boundsOut)) = 0;
end

function validateConstraintAvailability(options)
requiredClasses = {'constraintPositionTarget','constraintOrientationTarget','constraintJointBounds'};
if ~isempty(options.DistanceConstraints)
    requiredClasses{end+1} = 'constraintDistanceBounds'; %#ok<AGROW>
end
if ~isempty(options.CartesianBounds)
    requiredClasses{end+1} = 'constraintCartesianBounds'; %#ok<AGROW>
end
for i = 1:numel(requiredClasses)
    if ~exist(requiredClasses{i}, 'class') && ~exist(requiredClasses{i}, 'file')
        error('rt_compute_arm_gik:MissingConstraint', ...
            'Required constraint class %s is not available. Ensure Robotics System Toolbox is installed.', requiredClasses{i});
    end
end
end
