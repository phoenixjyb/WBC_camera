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
end

% Validate sizes
numSteps = size(armTrajectory, 1);
if numSteps ~= numel(armTimes)
    error('armTrajectory rows (%d) must match numel(armTimes) (%d).', numSteps, numel(armTimes));
end
if size(basePose,2) ~= 3
    error('basePose must be K-by-3 with columns [x y yaw].');
end
if numel(baseTimes) ~= size(basePose,1)
    error('baseTimes length must equal number of basePose rows.');
end
armTimes = armTimes(:)';
baseTimes = baseTimes(:)';

% Interpolate base pose onto arm timeline
baseX = interp1(baseTimes, basePose(:,1), armTimes, 'linear', 'extrap');
baseY = interp1(baseTimes, basePose(:,2), armTimes, 'linear', 'extrap');
baseYawUnwrapped = interp1(baseTimes, unwrap(basePose(:,3)), armTimes, 'linear', 'extrap');
baseYaw = wrapToPi(baseYawUnwrapped);

% Prepare figure
fig = figure('Name', 'Whole-Body Animation', 'Color', 'w');
ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
view(ax, 45, 25);

% Plot reference paths
plot(ax, basePose(:,1), basePose(:,2), 'k--', 'LineWidth', 1.0, 'DisplayName', 'Chassis path');
if ~isempty(eePoses) && size(eePoses,2) >= 3
    plot3(ax, eePoses(:,1), eePoses(:,2), eePoses(:,3), 'r-.', 'LineWidth', 1.0, 'DisplayName', 'Desired EE path');
end
actualEE = nan(numSteps,3);
actualLine = plot3(ax, nan, nan, nan, 'Color', [0 0.6 0], 'LineWidth', 1.5, 'DisplayName', 'Actual EE path');
legend(ax, 'Location', 'bestoutside');

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
robotHandles = gobjects(0);
hg = hgtransform('Parent', ax);
eeName = char(options.EndEffectorName);

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

for k = 1:numSteps
    for j = 1:numel(armIdx)
        config(armIdx(j)).JointPosition = armTrajectory(k, j);
    end
    % Draw robot (positions shown relative to chassis frame with base pose)
    Tbase = trvec2tform([baseX(k), baseY(k), 0]) * axang2tform([0 0 1 baseYaw(k)]);
    show(robot, config, 'Parent', ax, 'PreservePlot', false, ...
        'Frames', 'off', 'Visuals', 'on', 'FastUpdate', true);
    robotHandles = findobj(ax, 'Tag', 'RobotVisual');
    for h = reshape(robotHandles,1,[])
        if get(h, 'Parent') ~= hg
            set(h, 'Parent', hg);
        end
    end
    set(hg, 'Matrix', Tbase);

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
