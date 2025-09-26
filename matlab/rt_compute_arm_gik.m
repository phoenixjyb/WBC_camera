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
%     DistanceConstraints  - cell array configuring constraintDistanceBounds
%     CartesianBounds      - cell array configuring constraintCartesianBounds
%     CollisionAvoidance   - cell array configuring constraintCollisionAvoidance
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
if ~isfield(options, 'DistanceConstraints') || isempty(options.DistanceConstraints)
    options.DistanceConstraints = {};
end
if ~isfield(options, 'CartesianBounds') || isempty(options.CartesianBounds)
    options.CartesianBounds = {};
end
if ~isfield(options, 'CollisionAvoidance') || isempty(options.CollisionAvoidance)
    options.CollisionAvoidance = {};
end
if ~isfield(options, 'ExtraConstraints') || isempty(options.ExtraConstraints)
    options.ExtraConstraints = {};
end
if ~isfield(options, 'SolverParameters') || isempty(options.SolverParameters)
    options.SolverParameters = struct();
end
if ~isfield(options, 'UseWarmStart') || isempty(options.UseWarmStart)
    options.UseWarmStart = true;
end

options.DistanceConstraints = stripEmpty(options.DistanceConstraints);
options.CartesianBounds = stripEmpty(options.CartesianBounds);
options.CollisionAvoidance = stripEmpty(options.CollisionAvoidance);
options.ExtraConstraints = stripEmpty(options.ExtraConstraints);

[posCells, numPoses] = normalizeTransforms(poseTforms);
configGuess = options.InitialGuess;
[jointNames, lowerBounds, upperBounds] = jointBoundsFromRobot(robot, configGuess, options.JointBounds);

% Require constraint classes provided by Robotics System Toolbox
validateConstraintAvailability(options);

baseFrame = robot.BaseName;

posTarget = constraintPositionTarget(eeName);
posTarget.ReferenceBody = baseFrame;
posTarget.PositionTolerance = scalarTolerance(options.PositionTolerance);

oriTarget = constraintOrientationTarget(eeName);
oriTarget.ReferenceBody = baseFrame;
oriTarget.OrientationTolerance = scalarTolerance(options.OrientationTolerance);

jointBounds = constraintJointBounds(robot);
jointBounds.Bounds = [lowerBounds, upperBounds];

[distanceObjects, distanceInputs] = buildDistanceConstraints(robot, options.DistanceConstraints);
[cartesianObjects, cartInputs] = buildCartesianConstraints(robot, options.CartesianBounds);
[collisionObjects, collisionInputs] = buildCollisionConstraints(robot, options.CollisionAvoidance);
extraInputs = cellfun(@classToConstraintInput, options.ExtraConstraints, 'UniformOutput', false);

constraintInputs = ['position','orientation','jointbounds', distanceInputs, cartInputs, collisionInputs, extraInputs];

gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', constraintInputs);

solverParamFields = fieldnames(options.SolverParameters);
for idx = 1:numel(solverParamFields)
    fieldName = solverParamFields{idx};
    if isfield(gik.SolverParameters, fieldName)
        gik.SolverParameters.(fieldName) = options.SolverParameters.(fieldName);
    end
end

numJoints = numel(configGuess);
qMatrix = zeros(numPoses, numJoints);
infos = cell(1, numPoses);

for idx = 1:numPoses
    T = posCells{idx};
    posTarget.TargetPosition = tform2trvec(T);
    oriTarget.TargetOrientation = tform2quat(T);

    constraintArgs = {posTarget, oriTarget, jointBounds};
    if ~isempty(distanceObjects)
        constraintArgs = [constraintArgs, distanceObjects]; %#ok<AGROW>
    end
    if ~isempty(cartesianObjects)
        constraintArgs = [constraintArgs, cartesianObjects]; %#ok<AGROW>
    end
    if ~isempty(collisionObjects)
        constraintArgs = [constraintArgs, collisionObjects]; %#ok<AGROW>
    end
    if ~isempty(options.ExtraConstraints)
        constraintArgs = [constraintArgs, options.ExtraConstraints]; %#ok<AGROW>
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

function [objects, inputs] = buildDistanceConstraints(robot, entries)
numEntries = numel(entries);
objects = cell(1, numEntries);
for k = 1:numEntries
    entry = entries{k};
    if isa(entry, 'constraintDistanceBounds')
        objects{k} = entry;
        continue;
    end
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
        distObj.Bounds = makeFinite(entry.Bounds);
    end
    if isfield(entry, 'Weights') && ~isempty(entry.Weights)
        distObj.Weights = entry.Weights;
    end
    objects{k} = distObj;
end
inputs = repmat({'distance'}, 1, numEntries);
end

function [objects, inputs] = buildCartesianConstraints(robot, entries)
numEntries = numel(entries);
objects = cell(1, numEntries);
for k = 1:numEntries
    entry = entries{k};
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
    objects{k} = cartObj;
end
inputs = repmat({'cartesian'}, 1, numEntries);
end

function [objects, inputs] = buildCollisionConstraints(robot, entries)
numEntries = numel(entries);
objects = cell(1, numEntries);
for k = 1:numEntries
    entry = entries{k};
    if isa(entry, 'constraintCollisionAvoidance')
        objects{k} = entry;
        continue;
    end
    if iscell(entry)
        entry = entry{1};
    end
    collObj = constraintCollisionAvoidance(robot);
    if isfield(entry, 'Environment') && ~isempty(entry.Environment)
        collObj.Environment = entry.Environment;
    end
    if isfield(entry, 'CollisionPairs') && ~isempty(entry.CollisionPairs)
        collObj.CollisionPairs = entry.CollisionPairs;
    end
    if isfield(entry, 'Weights') && ~isempty(entry.Weights)
        collObj.Weights = entry.Weights;
    end
    if isfield(entry, 'NumSamples') && ~isempty(entry.NumSamples)
        collObj.NumSamples = entry.NumSamples;
    end
    if isfield(entry, 'SelfCollisions') && ~isempty(entry.SelfCollisions)
        collObj.SelfCollisions = logical(entry.SelfCollisions);
    end
    objects{k} = collObj;
end
inputs = repmat({'collisionavoidance'}, 1, numEntries);
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

function arr = stripEmpty(arr)
if iscell(arr)
    arr = arr(~cellfun(@isempty, arr));
elseif isempty(arr)
    arr = {};
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
elseif contains(className, 'CollisionAvoidance', 'IgnoreCase', true)
    name = 'collisionavoidance';
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
if ~isempty(options.CollisionAvoidance)
    requiredClasses{end+1} = 'constraintCollisionAvoidance'; %#ok<AGROW>
end
for i = 1:numel(requiredClasses)
    if ~exist(requiredClasses{i}, 'class') && ~exist(requiredClasses{i}, 'file')
        error('rt_compute_arm_gik:MissingConstraint', ...
            'Required constraint class %s is not available. Ensure Robotics System Toolbox is installed.', requiredClasses{i});
    end
end
end
