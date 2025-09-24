function animate_whole_body(robot, armJointNames, armTrajectory, armTimes, basePose, baseTimes, eePoses, options)
%ANIMATE_WHOLE_BODY Visualize arm + chassis motion over time.
%   animate_whole_body(robot, armJointNames, armTrajectory, armTimes,
%                      basePose, baseTimes, eePoses) animates the robot arm
%   and chassis trajectory using Robotics System Toolbox visualization.
%
%   Inputs:
%     robot          - rigidBodyTree of the arm mounted on the chassis.
%     armJointNames  - cell array of arm joint names (same order as columns
%                      in armTrajectory).
%     armTrajectory  - N-by-M matrix of arm joint positions.
%     armTimes       - 1-by-N vector (or N-by-1) of timestamps for armTrajectory.
%     basePose       - K-by-3 matrix [x y yaw] for chassis pose history.
%     baseTimes      - 1-by-K vector timestamps matching basePose.
%     eePoses        - Optional N-by-3 or N-by-6 end-effector poses (for path).
%     options        - struct with optional fields:
%                       .ArrowLength (default 0.25 m)
%                       .PlaybackSpeed (default 1.0, 1 = real-time)
%
%   The function interpolates the chassis pose onto the arm timestamps so the
%   animation stays synchronized. End-effector waypoints (if provided) are
%   displayed as a reference path.
%
%   Requires MATLAB R2022a+ with Robotics System Toolbox.

arguments
    robot (1,1) rigidBodyTree
    armJointNames (1,:) cell
    armTrajectory double
    armTimes double
    basePose double
    baseTimes double
    eePoses double = []
    options.ArrowLength double = 0.25
    options.PlaybackSpeed double = 1.0
    options.VideoFile string = ""
    options.VideoFrameRate double = 30
    options.EndEffectorName string = "left_gripper_link"
    options.VisualAlpha double = 0.6
    options.HideEndEffectorVisual logical = false
    options.ChassisMesh string = ""
    options.ChassisColor (1,3) double = [0.4 0.4 0.4]
    options.ChassisAlpha double = 0.35
    options.ChassisScale double = 1e-3
    options.StageBreakIndex double = 1
    options.StageLabels (1,:) string = ["Ramp-up", "Tracking"]
    options.StageBoundaries double = []
    options.StageSelection string = "all"
    options.Obstacles = []
    options.TargetPath double = []
end

if options.VisualAlpha < 0 || options.VisualAlpha > 1
    error('options.VisualAlpha must be within [0, 1].');
end
if options.ChassisScale <= 0
    error('options.ChassisScale must be positive.');
end
stageLabels = string(options.StageLabels);
stageSelection = lower(string(options.StageSelection));
stageBreak = max(1, round(options.StageBreakIndex));

% Work on a copy so visual tweaks don't leak back to caller
robot = copy(robot);
eeName = char(options.EndEffectorName);
if options.HideEndEffectorVisual
    try
        eeBody = copy(getBody(robot, eeName));
        clearVisual(eeBody);
        replaceBody(robot, eeName, eeBody);
    catch
        warning('animate_whole_body:FailedToHideEE', ...
            'Unable to hide end-effector visual for body %s.', eeName);
    end
end

% Validate sizes

numSteps = size(armTrajectory, 1);
if numSteps ~= numel(armTimes)
    error('armTrajectory rows (%d) must match numel(armTimes) (%d).', numSteps, numel(armTimes));
end
armTimes = armTimes(:)';
baseTimes = baseTimes(:)';
stageBreak = min(stageBreak, numSteps + 1);

if isempty(options.StageBoundaries)
    defaultBoundaries = [stageBreak - 1, numSteps];
else
    defaultBoundaries = options.StageBoundaries(:)';
end
if isempty(defaultBoundaries)
    defaultBoundaries = numSteps;
end
stageBoundaries = round(defaultBoundaries);
stageBoundaries(stageBoundaries < 1) = 1;
stageBoundaries(stageBoundaries > numSteps) = numSteps;
stageBoundaries = unique(stageBoundaries);
if stageBoundaries(end) ~= numSteps
    stageBoundaries = [stageBoundaries, numSteps];
end
numStages = numel(stageBoundaries);
if numel(stageLabels) < numStages
    stageLabels(end+1:numStages) = stageLabels(end);
elseif numel(stageLabels) > numStages
    stageLabels = stageLabels(1:numStages);
end
stageStarts = [1, stageBoundaries(1:end-1) + 1];
stageRanges = arrayfun(@(s,e) s:e, stageStarts, stageBoundaries, 'UniformOutput', false);

switch stageSelection
    case {'all','*'}
        frameIdx = 1:numSteps;
        stageBoundariesActive = stageBoundaries;
        stageLabelsActive = stageLabels;
    case {'arm','arm_ramp','arm-ramp'}
        frameIdx = stageRanges{1};
        stageBoundariesActive = stageBoundaries(1);
        stageLabelsActive = stageLabels(1);
    case {'chassis','chassis_ramp','base_ramp'}
        if numStages < 2
            error('animate_whole_body:StageSelection', 'Chassis ramp stage not available.');
        end
        stageIdx = min(2, numStages);
        frameIdx = stageRanges{stageIdx};
        stageBoundariesActive = stageBoundaries(stageIdx) - stageBoundaries(stageIdx-1);
        stageLabelsActive = stageLabels(stageIdx);
    case {'tracking','track'}
        frameIdx = stageRanges{end};
        if numStages >= 2
            prevEnd = stageBoundaries(end-1);
        else
            prevEnd = 0;
        end
        stageBoundariesActive = stageBoundaries(end) - prevEnd;
        stageLabelsActive = stageLabels(end);
    otherwise
        error('animate_whole_body:StageSelection', 'Unknown StageSelection option: %s.', stageSelection);
end

frameIdx = frameIdx(:)';
armTrajectory = armTrajectory(frameIdx, :);
armTimes = armTimes(frameIdx);
armTimes = armTimes - armTimes(1);
numSteps = numel(frameIdx);

stageBoundariesActive = round(stageBoundariesActive);
if isempty(stageBoundariesActive)
    stageBoundariesActive = numSteps;
end
stageBoundariesActive(stageBoundariesActive < 1) = 1;
stageBoundariesActive(stageBoundariesActive > numSteps) = numSteps;
if stageBoundariesActive(end) ~= numSteps
    stageBoundariesActive(end+1) = numSteps;
end
stageLabelsActive = string(stageLabelsActive);
if numel(stageLabelsActive) < numel(stageBoundariesActive)
    stageLabelsActive(end+1:numel(stageBoundariesActive)) = stageLabelsActive(end);
elseif numel(stageLabelsActive) > numel(stageBoundariesActive)
    stageLabelsActive = stageLabelsActive(1:numel(stageBoundariesActive));
end

if size(basePose,2) ~= 3
    error('basePose must be K-by-3 with columns [x y yaw].');
end
if numel(baseTimes) ~= size(basePose,1)
    error('baseTimes length must equal number of basePose rows.');
end

% Interpolate base pose onto arm timeline
baseX = interp1(baseTimes, basePose(:,1), armTimes, 'linear', 'extrap');
baseY = interp1(baseTimes, basePose(:,2), armTimes, 'linear', 'extrap');
baseYawUnwrapped = interp1(baseTimes, unwrap(basePose(:,3)), armTimes, 'linear', 'extrap');
baseYaw = wrapToPi(baseYawUnwrapped);

% Prepare figure
fig = figure('Name', 'Whole-Body Animation', 'Color', [0.15 0.15 0.15], ...
    'Units', 'pixels', 'Position', [100 100 1120 840], 'Resize', 'off');
ax = axes('Parent', fig, 'Color', [0.05 0.05 0.05], 'XColor', [0.9 0.9 0.9], ...
    'YColor', [0.9 0.9 0.9], 'ZColor', [0.9 0.9 0.9], 'GridColor', [0.35 0.35 0.35]);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X (m)', 'Color', [0.9 0.9 0.9]);
ylabel(ax, 'Y (m)', 'Color', [0.9 0.9 0.9]);
zlabel(ax, 'Z (m)', 'Color', [0.9 0.9 0.9]);
view(ax, 45, 25);

% Plot reference paths
plot(ax, basePose(:,1), basePose(:,2), '--', 'Color', [0.85 0.7 0.1], 'LineWidth', 1.0, 'DisplayName', 'Chassis path');
if ~isempty(eePoses) && size(eePoses,2) >= 3
    plot3(ax, eePoses(:,1), eePoses(:,2), eePoses(:,3), 'r-.', 'LineWidth', 1.0, 'DisplayName', 'Desired EE path');
end
if ~isempty(options.TargetPath)
    tp = options.TargetPath;
    plot(ax, tp(:,1), tp(:,2), 'm:', 'LineWidth', 1.4, 'DisplayName', 'Planned chassis path');
end
actualEE = nan(numSteps,3);
actualLine = plot3(ax, nan, nan, nan, 'Color', [0 0.7 0.2], 'LineWidth', 1.5, 'DisplayName', 'Actual EE path');
leg = legend(ax, 'Location', 'bestoutside');
set(leg, 'TextColor', [0.95 0.95 0.95], 'Color', [0.2 0.2 0.2]);

% Markers for current pose
baseMarker = plot3(ax, baseX(1), baseY(1), 0, 'bo', 'MarkerFaceColor', 'b');
headingLen = options.ArrowLength;
headingLine = plot3(ax, [baseX(1), baseX(1)+headingLen*cos(baseYaw(1))], ...
                      [baseY(1), baseY(1)+headingLen*sin(baseYaw(1))], ...
                      [0 0], 'b-', 'LineWidth', 2);
if ~isempty(eePoses) && size(eePoses,2) >= 3
    eeMarker = plot3(ax, eePoses(1,1), eePoses(1,2), eePoses(1,3), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Desired EE waypoint');
else
    eeMarker = [];
end
actualMarker = plot3(ax, baseX(1), baseY(1), 0, 's', 'Color', [0.0 0.6 0.2], 'MarkerFaceColor', [0.0 0.6 0.2], 'DisplayName', 'Actual EE');
stageText = text(ax, 'Units', 'normalized', 'Position', [0.02 0.95 0], ...
    'String', '', 'Color', [0.95 0.95 0.95], 'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.1 0.1 0.1], 'Margin', 4, 'HorizontalAlignment', 'left');

% Map joint names
configTemplate = homeConfiguration(robot);
config = configTemplate;
vectorNames = {config.JointName};
armIdx = zeros(1, numel(armJointNames));
for j = 1:numel(armJointNames)
    idx = find(strcmp(vectorNames, armJointNames{j}), 1);
    if isempty(idx)
        error('Joint %s not present in robot model.', armJointNames{j});
    end
    armIdx(j) = idx;
end

% Animation loop
hg = hgtransform('Parent', ax);
chassisPatch = [];
robotHandlesPrev = [];

% Attempt to load chassis/support mesh once so it follows the base hgtransform
meshPath = string(options.ChassisMesh);
if strlength(meshPath) == 0
    helpersDir = fileparts(mfilename('fullpath')); % .../matlab/+helpers
    matlabDir = fileparts(helpersDir);
    defaultMesh = fullfile(matlabDir, '..', 'mobile_arm_whole_body', 'meshes', 'cr_no_V.stl');
    meshPath = string(defaultMesh);
end
if exist(meshPath, 'file') == 2
    try
        triData = stlread(meshPath);
        chassisVertices = triData.Points * options.ChassisScale;
        chassisPatch = patch('Parent', hg, 'Faces', triData.ConnectivityList, ...
            'Vertices', chassisVertices, 'FaceColor', options.ChassisColor, ...
            'FaceAlpha', options.ChassisAlpha, 'EdgeColor', 'none', ...
            'DisplayName', 'Chassis model');
    catch ME
        warning('animate_whole_body:ChassisMeshFailed', ...
            'Failed to load chassis mesh from %s (%s).', meshPath, ME.message);
        chassisPatch = [];
    end
else
    warning('animate_whole_body:ChassisMeshMissing', ...
        'Chassis mesh file %s not found. Skipping static chassis visual.', meshPath);
end

videoWriter = [];
if strlength(options.VideoFile) > 0
    try
        videoWriter = VideoWriter(char(options.VideoFile), 'MPEG-4');
        videoWriter.FrameRate = options.VideoFrameRate;
        open(videoWriter);
    catch ME
        warning('Failed to create video writer (%s). Continuing without video.', ME.message);
        videoWriter = [];
    end
end

% Draw obstacles as translucent primitives
if ~isempty(options.Obstacles)
    obsList = options.Obstacles;
    for obsIdx = 1:numel(obsList)
        obs = obsList(obsIdx);
        if ~isfield(obs, 'type'); obs.type = 'circle'; end
        switch lower(obs.type)
            case {'circle','disc','disk'}
                center = obs.center;
                radius = obs.radius;
                if isfield(obs, 'height') && ~isempty(obs.height)
                    h = max(obs.height, 0);
                else
                    h = 0.05;
                end
                thetaGrid = linspace(0, 2*pi, 40);
                xCircle = center(1) + radius * cos(thetaGrid);
                yCircle = center(2) + radius * sin(thetaGrid);
                fill3(ax, xCircle, yCircle, zeros(size(thetaGrid)), 'c', 'FaceAlpha', 0.2, ...
                    'EdgeColor', 'none', 'DisplayName', sprintf('Obstacle @ [%.1f %.1f]', center));
                if h > 0
                    fill3(ax, xCircle, yCircle, h * ones(size(thetaGrid)), 'c', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
                end
            case {'rectangle','box','aabb'}
                bounds = obs.bounds;
                xRect = [bounds(1) bounds(2) bounds(2) bounds(1) bounds(1)];
                yRect = [bounds(3) bounds(3) bounds(4) bounds(4) bounds(3)];
                fill3(ax, xRect, yRect, zeros(size(xRect)), 'c', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
            otherwise
                % ignore unknown types
        end
    end
end

for k = 1:numSteps
    if ~isempty(robotHandlesPrev)
        delete(robotHandlesPrev(ishghandle(robotHandlesPrev)));
        robotHandlesPrev = [];
    end

    for j = 1:numel(armIdx)
        config(armIdx(j)).JointPosition = armTrajectory(k, j);
    end
    % Draw robot (positions shown relative to chassis frame with base pose)
    Tbase = trvec2tform([baseX(k), baseY(k), 0]) * axang2tform([0 0 1 baseYaw(k)]);
    show(robot, config, 'Parent', ax, 'PreservePlot', false, ...
        'Frames', 'off', 'Visuals', 'on', 'FastUpdate', false);

    robotHandles = findall(ax, '-regexp', 'Tag', '^DO_NOT_EDIT');
    for h = reshape(robotHandles,1,[])
        if ~ishghandle(h)
            continue;
        end
        if get(h, 'Parent') ~= hg
            set(h, 'Parent', hg);
        end
        if isprop(h, 'FaceAlpha')
            set(h, 'FaceAlpha', options.VisualAlpha);
        end
        if isprop(h, 'EdgeAlpha')
            set(h, 'EdgeAlpha', options.VisualAlpha);
        end
    end
    set(hg, 'Matrix', Tbase);
    robotHandlesPrev = robotHandles;

    % Update markers
    set(baseMarker, 'XData', baseX(k), 'YData', baseY(k), 'ZData', 0);
    set(headingLine, 'XData', [baseX(k), baseX(k)+headingLen*cos(baseYaw(k))], ...
                     'YData', [baseY(k), baseY(k)+headingLen*sin(baseYaw(k))], ...
                     'ZData', [0, 0]);
    if ~isempty(eeMarker)
        idx = min(k, size(eePoses,1));
        set(eeMarker, 'XData', eePoses(idx,1), 'YData', eePoses(idx,2), 'ZData', eePoses(idx,3));
    end

    try
        Tee = getTransform(robot, config, eeName);
    catch
        Tee = eye(4);
    end
    TeeWorld = Tbase * Tee;
    actualEE(k, :) = TeeWorld(1:3,4)';
    set(actualMarker, 'XData', actualEE(k,1), 'YData', actualEE(k,2), 'ZData', actualEE(k,3));
    set(actualLine, 'XData', actualEE(1:k,1), 'YData', actualEE(1:k,2), 'ZData', actualEE(1:k,3));

    stageIdx = find(k <= stageBoundariesActive, 1, 'first');
    if isempty(stageIdx)
        stageIdx = numel(stageBoundariesActive);
    end
    stageText.String = sprintf('%s (frame %d/%d)', stageLabelsActive(stageIdx), k, numSteps);

    drawnow limitrate;
    if ~isempty(videoWriter)
        frame = getframe(fig);
        writeVideo(videoWriter, frame);
    end
    if k < numSteps
        dt = max(armTimes(k+1) - armTimes(k), 0);
        pause(dt / max(options.PlaybackSpeed, eps));
    end
end

if ~isempty(videoWriter)
    close(videoWriter);
end
end
