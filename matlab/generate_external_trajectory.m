function traj = generate_external_trajectory(opts)
%GENERATE_EXTERNAL_TRAJECTORY Produce end-effector and base paths from external sources.
%   traj = GENERATE_EXTERNAL_TRAJECTORY(opts) returns a struct with fields:
%     - eePoses        : N-by-6 [x y z roll pitch yaw] end-effector poses.
%     - baseWaypoints  : M-by-2 planar waypoints for the chassis.
%     - timestamps     : 1-by-N time vector (seconds) for eePoses (optional).
%     - metadata       : struct describing the source.
%
%   opts can include:
%     opts.source = 'demo_arc' (default), 'demo_line',
%                   'dolly_forward', 'orbit_left', 'crane_up', 'pan_tilt', or 'file'.
%     opts.file   = path to .mat /.json /.csv file when source=='file'.
%     opts.scale  = scalar to scale the demo trajectories (default 1.0).
%     opts.duration = total duration for demo trajectories (seconds).
%
%   File loading rules:
%     - MAT: should contain variables `eePoses` (Nx6) and `baseWaypoints` (Mx2).
%     - JSON: expects keys `eePoses`, `baseWaypoints`, optionally `timestamps`.
%     - CSV: assumed to contain columns [t, x, y, z, roll, pitch, yaw].
%       Base waypoints are down-sampled from [x, y].
%
%   This helper lets you plug in offline planning results or designer-specified
%   paths without editing the controller script. Extend the switch cases as
%   needed to support additional trajectory generators.

if nargin < 1 || isempty(opts)
    opts = struct();
end

if ~isfield(opts, 'source') || isempty(opts.source)
    opts.source = "demo_arc";
end
opts.source = string(opts.source);

if ~isfield(opts, 'file') || isempty(opts.file)
    opts.file = "";
end
opts.file = string(opts.file);

if ~isfield(opts, 'scale') || isempty(opts.scale)
    opts.scale = 1.0;
end

if ~isfield(opts, 'duration') || isempty(opts.duration)
    opts.duration = 8.0;
end

if ~isfield(opts, 'sample_dt') || isempty(opts.sample_dt)
    opts.sample_dt = 0.1;  % seconds between samples
end

if ~isfield(opts, 'max_speed') || isempty(opts.max_speed)
    opts.max_speed = 3.0;  % m/s camera speed limit
end

switch opts.source
    case "demo_arc"
        traj = demoArc(opts.scale, opts.duration);
    case "demo_line"
        traj = demoLine(opts.scale, opts.duration);
    case "dolly_forward"
        traj = dollyForward(opts.scale, opts.duration);
    case "orbit_left"
        traj = orbitLeft(opts.scale, opts.duration);
    case "crane_up"
        traj = craneUp(opts.scale, opts.duration);
    case "pan_tilt"
        traj = panTilt(opts.scale, opts.duration);
    case "file"
        if strlength(opts.file) == 0
            error('When opts.source == "file", opts.file must be provided.');
        end
        traj = loadFromFile(opts.file, opts);
    otherwise
        error('Unknown trajectory source: %s', opts.source);
end
end

function traj = demoArc(scale, duration)
% Demo trajectory: arm sweeps a shallow arc while base performs an S curve.
numPts = 6;
t = linspace(0, duration, numPts);
baseWaypoints = scale * [
    0.0,  0.0;
    0.4,  0.0;
    0.8,  0.15;
    1.1, -0.05;
    1.4,  0.10
];
center = [0.6, 0.0, 1.1];
angles = linspace(-15, 15, numPts) * pi/180;
radial = scale * 0.2;
traj.eePoses = [
    center(1) + radial * cos(angles(:)), ...
    center(2) + radial * sin(angles(:)), ...
    center(3) - 0.05 * sin(angles(:)), ...
    zeros(numPts,2), ...
    zeros(numPts,1)
];
traj.baseWaypoints = baseWaypoints;
traj.timestamps = t(:);
traj.metadata = struct('type','demo_arc','scale',scale,'duration',duration);
end

function traj = demoLine(scale, duration)
% Demo trajectory: straight-line advance with gentle arm lift.
numPts = 5;
t = linspace(0, duration, numPts);
baseWaypoints = scale * [
    0.0, 0.0;
    0.3, 0.0;
    0.7, 0.0;
    1.0, 0.0
];
traj.eePoses = [
    linspace(0.45, 0.75, numPts)', ...
    zeros(numPts,1), ...
    linspace(1.05, 1.20, numPts)', ...
    zeros(numPts,3)
];
traj.baseWaypoints = baseWaypoints;
traj.timestamps = t(:);
traj.metadata = struct('type','demo_line','scale',scale,'duration',duration);
end

function traj = dollyForward(scale, duration)
% Classic dolly: base drives straight, camera maintains heading and slight boom.
numPts = 7;
t = linspace(0, duration, numPts);
baseWaypoints = scale * [
    0.0, 0.0;
    0.3, 0.0;
    0.6, 0.0;
    0.9, 0.0;
    1.2, 0.0;
    1.5, 0.0;
    1.8, 0.0
];
zLift = 1.05 + 0.08 * sin(linspace(0, pi/2, numPts));
traj.eePoses = [
    linspace(0.5, 1.1, numPts)', ...
    zeros(numPts,1), ...
    zLift(:), ...
    zeros(numPts,1), ...
    repmat(-deg2rad(5), numPts,1), ...
    zeros(numPts,1)
];
traj.baseWaypoints = baseWaypoints;
traj.timestamps = t(:);
traj.metadata = struct('type','dolly_forward','scale',scale,'duration',duration);
end

function traj = orbitLeft(scale, duration)
% Orbit shot: base traces an arc while camera yaws to keep focus on origin.
numPts = 8;
t = linspace(0, duration, numPts);
radius = 0.9 * scale;
theta = linspace(-pi/6, pi/6, numPts);
baseX = radius * cos(theta);
baseY = radius * sin(theta);
focus = [0.0, 0.0];
camHeight = 1.1;
yawToFocus = atan2(focus(2) - baseY, focus(1) - baseX);
traj.eePoses = [
    baseX' + 0.1, ...
    baseY', ...
    repmat(camHeight, numPts,1), ...
    zeros(numPts,1), ...
    zeros(numPts,1), ...
    yawToFocus'
];
traj.baseWaypoints = [baseX', baseY'];
traj.timestamps = t(:);
traj.metadata = struct('type','orbit_left','scale',scale,'duration',duration,'radius',radius);
end

function traj = craneUp(scale, duration)
% Crane shot: base stays put, arm lifts and pitches down.
numPts = 6;
t = linspace(0, duration, numPts);
baseWaypoints = zeros(numPts,2);
zPath = linspace(1.0, 1.4, numPts)';
pitch = linspace(-deg2rad(5), -deg2rad(25), numPts)';
traj.eePoses = [
    0.6 * scale * ones(numPts,1), ...
    -0.05 * scale * ones(numPts,1), ...
    zPath, ...
    zeros(numPts,1), ...
    pitch, ...
    zeros(numPts,1)
];
traj.baseWaypoints = [baseWaypoints(1,:); baseWaypoints(end,:)];
traj.timestamps = t;
traj.metadata = struct('type','crane_up','scale',scale,'duration',duration);
end

function traj = panTilt(scale, duration)
% Pan/tilt on tripod: base static, camera sweeps yaw/pitch.
numPts = 9;
t = linspace(0, duration, numPts);
baseWaypoints = [0 0; 0 0.0];
yawSweep = linspace(-deg2rad(30), deg2rad(30), numPts)';
pitchSweep = deg2rad(5) * sin(linspace(0, 2*pi, numPts))';
traj.eePoses = [
    0.55 * scale * ones(numPts,1), ...
    zeros(numPts,1), ...
    1.15 * ones(numPts,1), ...
    zeros(numPts,1), ...
    pitchSweep, ...
    yawSweep
];
traj.baseWaypoints = baseWaypoints;
traj.timestamps = t;
traj.metadata = struct('type','pan_tilt','scale',scale,'duration',duration);
end

function traj = loadFromFile(filePath, opts)
[~,~,ext] = fileparts(filePath);
switch lower(ext)
    case '.mat'
        data = load(filePath);
        traj.eePoses = requireField(data, 'eePoses');
        traj.baseWaypoints = requireField(data, 'baseWaypoints');
        traj.timestamps = getFieldOr(data, 'timestamps', []);
    case '.json'
        txt = fileread(filePath);
        data = jsondecode(txt);
        if isfield(data, 'eePoses')
            eePosesRaw = data.eePoses;
            if size(eePosesRaw,2) ~= 6
                error('eePoses must be Nx6 [x y z roll pitch yaw].');
            end
            traj.eePoses = eePosesRaw;
            traj.timestamps = getFieldOr(data, 'timestamps', (0:size(eePosesRaw,1)-1)' * opts.sample_dt);
            traj.timestamps = traj.timestamps(:);
            posMat = eePosesRaw(:,1:3);
        elseif isfield(data, 'poses')
            poses = data.poses;
            [eePoseMat, timestamps, posMat] = preprocess_pose_sequence(poses, opts);
            traj.eePoses = eePoseMat;
            traj.timestamps = timestamps;
        else
            error('JSON input must contain `eePoses` or `poses`.');
        end

        baseDerived = false;
        if isfield(data, 'baseWaypoints')
            traj.baseWaypoints = data.baseWaypoints;
        elseif isfield(data, 'base_waypoints')
            traj.baseWaypoints = data.base_waypoints;
        elseif exist('posMat', 'var')
            offsets = posMat(:,1:2) - posMat(1,1:2);
            traj.baseWaypoints = offsets;
            baseDerived = true;
        else
            traj.baseWaypoints = zeros(2,2);
        end

        if size(traj.baseWaypoints,1) < 2
            traj.baseWaypoints = [traj.baseWaypoints(1,:); traj.baseWaypoints(1,:)];
        end

        if size(traj.baseWaypoints,1) ~= size(traj.eePoses,1)
            tOrig = linspace(0, 1, size(traj.baseWaypoints,1));
            tNew = linspace(0, 1, size(traj.eePoses,1));
            traj.baseWaypoints = interp1(tOrig, traj.baseWaypoints, tNew, 'linear', 'extrap');
        end

        if isempty(traj.timestamps)
            traj.timestamps = (0:size(traj.eePoses,1)-1)' * opts.sample_dt;
        else
            traj.timestamps = traj.timestamps(:);
        end
    case '.csv'
        M = readmatrix(filePath);
        if size(M,2) < 7
            error('CSV input must contain columns [t, x, y, z, roll, pitch, yaw].');
        end
        traj.timestamps = M(:,1)';
        traj.eePoses = M(:,2:7);
        traj.baseWaypoints = M(:,[2,3]);
    otherwise
        error('Unsupported file extension: %s', ext);
end
traj.metadata = struct('type','file','path',filePath);
if exist('baseDerived', 'var') && baseDerived
    traj.metadata.base = 'offset_from_first_pose';
end
traj.metadata.sample_dt = opts.sample_dt;
traj.metadata.max_speed = opts.max_speed;
end

function val = requireField(s, fieldName)
if isstruct(s) && isfield(s, fieldName)
    val = s.(fieldName);
elseif isobject(s) && isprop(s, fieldName)
    val = s.(fieldName);
else
    error('Required field "%s" missing in external trajectory source.', fieldName);
end
end

function val = getFieldOr(s, fieldName, defaultVal)
if isstruct(s) && isfield(s, fieldName)
    val = s.(fieldName);
elseif isobject(s) && isprop(s, fieldName)
    val = s.(fieldName);
else
    val = defaultVal;
end
end

function [eePoseMat, timestamps, posMat] = preprocess_pose_sequence(poses, opts)
dt = opts.sample_dt;
vMax = opts.max_speed;
scale = opts.scale;

numPoses = numel(poses);
if numPoses < 2
    error('Pose sequence must contain at least two samples.');
end

pos0 = scale * poses(1).position(:)';
quat0 = normalize_quat(poses(1).orientation(:)');
rotm0 = quat2rotm(quat0);
eul0 = rotm2eul(rotm0, 'ZYX');
rpy0 = [eul0(3), eul0(2), eul0(1)];

posList = pos0;
rpyList = rpy0;
timeList = 0;
currentPos = pos0;
currentQuat = quat0;
tAccum = 0;

for i = 1:numPoses-1
    nextPos = scale * poses(i+1).position(:)';
    nextQuat = normalize_quat(poses(i+1).orientation(:)');

    dist = norm(nextPos - currentPos);
    maxStep = max(vMax * dt, 1e-6);
    steps = max(1, ceil(dist / maxStep));
    taus = linspace(0, 1, steps + 1);

    for s = 2:numel(taus)
        tau = taus(s);
        posInterp = (1 - tau) * currentPos + tau * nextPos;
        quatInterp = slerp_quat(currentQuat, nextQuat, tau);
        rotm = quat2rotm(quatInterp);
        eul = rotm2eul(rotm, 'ZYX');
        rpy = [eul(3), eul(2), eul(1)];

        posList(end+1,:) = posInterp; %#ok<AGROW>
        rpyList(end+1,:) = rpy; %#ok<AGROW>
        tAccum = tAccum + dt;
        timeList(end+1,1) = tAccum; %#ok<AGROW>
    end

    currentPos = nextPos;
    currentQuat = nextQuat;
end

eePoseMat = [posList, rpyList];
posMat = posList;
timestamps = timeList;
end

function quat = normalize_quat(q)
q = q(:)';
if numel(q) ~= 4
    error('Orientation quaternion must have four elements [w x y z].');
end
if q(1) < 0
    q = -q;
end
quat = q / norm(q);
end

function q = slerp_quat(q0, q1, tau)
q0 = normalize_quat(q0);
q1 = normalize_quat(q1);
dotProd = dot(q0, q1);
if dotProd < 0
    q1 = -q1;
    dotProd = -dotProd;
end
if dotProd > 0.9995
    q = normalize_quat((1 - tau) * q0 + tau * q1);
    return;
end
theta = acos(max(min(dotProd,1),-1));
sinTheta = sin(theta);
q = (sin((1 - tau) * theta) / sinTheta) * q0 + (sin(tau * theta) / sinTheta) * q1;
q = normalize_quat(q);
end
