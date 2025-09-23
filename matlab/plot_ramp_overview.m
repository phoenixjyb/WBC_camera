function plot_ramp_overview(options)
%PLOT_RAMP_OVERVIEW Visualize home pose, ramp goals, and desired trajectory.
%   plot_ramp_overview() renders a 3D scene that includes the robot at its
%   home base pose, the desired end-effector trajectory, the virtual ramp
%   target for the arm, and the chassis ramp goal. A top-down subplot shows
%   the planar geometry for additional context.
%
%   Optional fields in `options` (struct or name/value pairs):
%     traj_source   - trajectory preset (default "file")
%     traj_file     - file to load if source == "file" (default "1_pull_world.json")
%     output_dir    - directory for the saved PNG (default matlab/outputs)
%     home_base     - 1x3 [x y yaw] for the chassis home pose (default [-2 -2 0])
%
%   Example:
%     plot_ramp_overview(struct('traj_source', "demo_line"));

arguments
    options struct = struct()
end

[trajSource, trajFile, outputDir, homeBasePose] = parse_options(options);

% Ensure output directory exists
if ~isfolder(outputDir)
    mkdir(outputDir);
end

% Load robot model to obtain meshes and home EE pose
thisDir = fileparts(mfilename('fullpath'));
addpath(thisDir); % make sure helpers on path
urdfPath = fullfile(thisDir, '..', 'mobile_arm_whole_body', 'urdf', 'arm_on_car_center_rotZ_neg90.urdf');
robot = importrobot(urdfPath, 'DataFormat', 'struct');
robot.Gravity = [0 0 -9.81];
armEEName = 'left_gripper_link';

% Load chassis mesh for visualization
meshPath = fullfile(thisDir, '..', 'mobile_arm_whole_body', 'meshes', 'cr_no_V.stl');
if exist(meshPath, 'file') == 2
    chassisMeshData = stlread(meshPath);
else
    chassisMeshData = [];
end
homeConfig = homeConfiguration(robot);
TeeHome = getTransform(robot, homeConfig, armEEName);

baseTransform = trvec2tform([homeBasePose(1:2), 0]) * axang2tform([0 0 1 homeBasePose(3)]);
TeeHomeWorld = baseTransform * TeeHome;
homeEEPos = TeeHomeWorld(1:3,4)';
homeEERPY = rotm2eul(TeeHomeWorld(1:3,1:3), 'XYZ');

% Generate the desired trajectory
trajOpts = struct('source', char(trajSource), 'duration', 8.0, 'scale', 1.0, ...
                  'file', char(trajFile), 'sample_dt', 0.1, 'max_speed', 3.0);
traj = generate_external_trajectory(trajOpts);
refPos = traj.eePoses(:,1:3);
if size(traj.eePoses,2) >= 6
    refRPY = traj.eePoses(:,4:6);
else
    refRPY = zeros(size(refPos));
end
baseWaypoints = traj.baseWaypoints;
if isempty(baseWaypoints)
    error('Trajectory did not provide base waypoints.');
end

% Compute base heading (for chassis yaw reference)
[thetaRef, ~] = compute_base_heading(baseWaypoints);

% Determine ramp goals
armGoal = compute_arm_goal(homeEEPos, refPos(1,:), refRPY, homeEERPY);
desiredYaw = thetaRef(1);
if size(refRPY,2) >= 3
    desiredYaw = wrapToPi(refRPY(1,3));
end
chassisGoal = compute_chassis_goal(homeBasePose, baseWaypoints, thetaRef, desiredYaw);

% Prepare figure
fig = figure('Name', 'Ramp Overview', 'Color', [1 1 1], 'Position', [100 100 1300 600]);

%% 3D scene with robot and EE trajectory
ax3 = subplot(1,2,1); hold(ax3,'on'); grid(ax3,'on'); axis(ax3,'equal');
view(ax3, [40 20]);

% Draw robot at home pose
baseTransform = trvec2tform([homeBasePose(1:2), 0]) * axang2tform([0 0 1 homeBasePose(3)]);
hgt = hgtransform('Parent', ax3);
show(robot, homeConfig, 'Parent', ax3, 'Frames', 'off', 'PreservePlot', false, ...
    'Visuals', 'on', 'FastUpdate', true);
drawnow;
robotHandles = setdiff(allchild(ax3), hgt);
for idx = 1:numel(robotHandles)
    set(robotHandles(idx), 'Parent', hgt);
end
set(hgt, 'Matrix', baseTransform);

if ~isempty(chassisMeshData)
    meshVerts = chassisMeshData.Points * 0.001; % convert mm -> m
    patch('Parent', hgt, 'Faces', chassisMeshData.ConnectivityList, ...
        'Vertices', meshVerts, 'FaceColor', [0.75 0.75 0.8], ...
        'FaceAlpha', 0.35, 'EdgeColor', 'none', 'DisplayName', 'Chassis/column');
end

% Plot desired EE path
plot3(ax3, refPos(:,1), refPos(:,2), refPos(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Desired EE path');
rampZInterp = linspace(homeEEPos(3), armGoal.position(3), 20);
rampXYHome = repmat(homeEEPos(1:2), 20, 1);
rampXYGoal = repmat(armGoal.position(1:2), 20, 1);
rampXY = rampXYHome + (rampXYGoal - rampXYHome) .* rampZInterp(:) ./ rampZInterp(end);
plot3(ax3, rampXY(:,1), rampXY(:,2), rampZInterp, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Arm ramp interp');
scatter3(ax3, homeEEPos(1), homeEEPos(2), homeEEPos(3), 70, 'filled', 'MarkerFaceColor', [0 0.7 0], 'DisplayName', 'EE (home pose)');
scatter3(ax3, armGoal.position(1), armGoal.position(2), armGoal.position(3), 80, 'filled', 'MarkerFaceColor', [0.85 0.33 0.1], 'DisplayName', 'Arm ramp goal');
scatter3(ax3, refPos(1,1), refPos(1,2), refPos(1,3), 75, 'filled', 'MarkerFaceColor', [0 0.45 0.85], 'DisplayName', 'Desired EE (first waypoint)');

text(ax3, homeEEPos(1), homeEEPos(2), homeEEPos(3)+0.05, 'EE home pose', 'Color', [0 0.6 0]);
text(ax3, armGoal.position(1), armGoal.position(2), armGoal.position(3)+0.05, 'Arm ramp goal', 'Color', [0.7 0.2 0]);
text(ax3, refPos(1,1), refPos(1,2), refPos(1,3)+0.05, 'Desired EE (waypoint 1)', 'Color', [0 0.3 0.7]);

xlabel(ax3, 'X (m)'); ylabel(ax3, 'Y (m)'); zlabel(ax3, 'Z (m)');
title(ax3, '3D View: Robot and EE Ramp Goal');
legend(ax3, 'Location','best');

%% Top-down planar view
ax2 = subplot(1,2,2); hold(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal');
plot(ax2, baseWaypoints(:,1), baseWaypoints(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Desired base path');
plot(ax2, [homeEEPos(1) armGoal.position(1)], [homeEEPos(2) armGoal.position(2)], 'k--', 'LineWidth', 1.2, 'DisplayName', 'Arm ramp interp');
scatter(ax2, homeBasePose(1), homeBasePose(2), 70, 'filled', 'MarkerFaceColor', [0 0.7 0], 'DisplayName', 'Home base');
scatter(ax2, chassisGoal(1), chassisGoal(2), 70, 'filled', 'MarkerFaceColor', [0.7 0 0.7], 'DisplayName', 'Chassis ramp goal');
scatter(ax2, homeEEPos(1), homeEEPos(2), 60, 'filled', 'MarkerFaceColor', [0.0 0.6 0.4], 'DisplayName', 'EE home (proj)');
scatter(ax2, armGoal.position(1), armGoal.position(2), 60, 'filled', 'MarkerFaceColor', [0.85 0.33 0.1], 'DisplayName', 'Arm ramp goal (proj)');
scatter(ax2, refPos(1,1), refPos(1,2), 60, 'filled', 'MarkerFaceColor', [0 0.45 0.85], 'DisplayName', 'Desired EE (proj)');

% Draw headings as arrows
quiver(ax2, homeBasePose(1), homeBasePose(2), cos(homeBasePose(3))*0.4, sin(homeBasePose(3))*0.4, 0, ...
    'Color', [0 0.5 0], 'LineWidth', 1.5, 'DisplayName', 'Home heading');
quiver(ax2, chassisGoal(1), chassisGoal(2), cos(chassisGoal(3))*0.4, sin(chassisGoal(3))*0.4, 0, ...
    'Color', [0.5 0 0.5], 'LineWidth', 1.5, 'DisplayName', 'Ramp goal heading');

% Also project the EE ramp goal onto plane (for intuition)
scatter(ax2, armGoal.position(1), armGoal.position(2), 60, 'd', 'filled', 'MarkerFaceColor', [0.85 0.33 0.1], 'DisplayName', 'Arm ramp goal (proj)');

xlabel(ax2, 'X (m)'); ylabel(ax2, 'Y (m)');
title(ax2, 'Plan View: Base Home and Ramp Goals');
legend(ax2, 'Location','bestoutside');

sgtitle(sprintf('Ramp Overview: %s', trajSource));

% Save figure
outputBase = fullfile(outputDir, sprintf('ramp_overview_%s', trajSource));
exportgraphics(fig, strcat(outputBase, '.png'), 'Resolution', 200);
savefig(fig, strcat(outputBase, '.fig'));
fprintf('Saved ramp overview to %s.{png,fig}\n', outputBase);

end

%% Helper functions --------------------------------------------------------
function [trajSource, trajFile, outputDir, homeBasePose] = parse_options(opts)
if isfield(opts, 'traj_source') && ~isempty(opts.traj_source)
    trajSource = string(opts.traj_source);
else
    trajSource = "file";
end
if isfield(opts, 'traj_file') && ~isempty(opts.traj_file)
    trajFile = string(opts.traj_file);
else
    trajFile = "1_pull_world.json";
end
if isfield(opts, 'output_dir') && ~isempty(opts.output_dir)
    outputDir = char(opts.output_dir);
else
    outputDir = char(fullfile(fileparts(mfilename('fullpath')), 'outputs'));
end
if isfield(opts, 'home_base') && ~isempty(opts.home_base)
    homeBasePose = opts.home_base;
else
    homeBasePose = [-2, -2, 0];
end
end

function [thetaRef, arcLen] = compute_base_heading(baseWaypoints)
numPts = size(baseWaypoints,1);
if numPts < 2
    thetaRef = zeros(numPts,1);
    arcLen = zeros(numPts,1);
    return;
end
segmentDiff = diff(baseWaypoints,1,1);
segmentDist = sqrt(sum(segmentDiff.^2, 2));
arcLen = [0; cumsum(segmentDist)];
thetaRef = zeros(numPts,1);
thetaRef(1:end-1) = atan2(segmentDiff(:,2), segmentDiff(:,1));
thetaRef(end) = thetaRef(end-1);
thetaRef = unwrap(thetaRef);
end

function armGoal = compute_arm_goal(homeEEPos, firstEEPos, refRPY, homeEERPY)
armGoal = struct();
armGoal.position = [homeEEPos(1:2), firstEEPos(3)];
if isempty(refRPY)
    armGoal.rpy = homeEERPY;
else
    armGoal.rpy = refRPY(1,:);
end
armGoal.homeRPY = homeEERPY;
end

function targetPose = compute_chassis_goal(homePose, baseWaypoints, thetaRef, desiredYaw)
if isempty(baseWaypoints)
    targetPose = homePose;
else
    if nargin >= 4 && ~isempty(desiredYaw)
        yawVal = desiredYaw;
    elseif ~isempty(thetaRef)
        yawVal = thetaRef(1);
    else
        yawVal = homePose(3);
    end
    targetPose = [baseWaypoints(1,:), yawVal];
end
end
