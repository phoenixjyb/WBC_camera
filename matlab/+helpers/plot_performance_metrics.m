function figs = plot_performance_metrics(timeVec, data)
%PLOT_PERFORMANCE_METRICS Generate time-domain analysis plots.
%
% helpers.plot_performance_metrics(timeVec, data) expects:
%   data.errorNorm    : Nx1 Euclidean position error
%   data.base          struct with fields x, y, yaw, v_long, v_lat, omega
%   data.armAngles     : Nx6 joint positions (matching timeVec)
%   data.armJointNames : cell array of joint names (1x6)
%   data.eeSpeed       : Nx1 end-effector speed
%   data.eeAccel       : Nx1 end-effector acceleration magnitude
%   data.eeJerk        : Nx1 end-effector jerk magnitude

arguments
    timeVec double
    data struct
end

%% Error magnitude
figError = figure('Name', 'End-Effector Error vs Time', 'Color', 'w');
plot(timeVec, data.errorNorm, 'LineWidth', 1.6);
grid on; xlabel('Time (s)'); ylabel('||p_{des} - p_{act}|| (m)');
title('End-Effector Euclidean Error');

%% Chassis state plots
figBase = figure('Name', 'Chassis States vs Time', 'Color', 'w');
T = tiledlayout(figBase, 3, 2, 'TileSpacing', 'compact');
nexttile; plot(timeVec, data.base.x, 'LineWidth', 1.2); grid on; ylabel('x (m)');
nexttile; plot(timeVec, data.base.y, 'LineWidth', 1.2); grid on; ylabel('y (m)');
nexttile;
actYawDeg = rad2deg(wrapToPi(data.base.yaw));
plot(timeVec, actYawDeg, 'LineWidth', 1.2, 'DisplayName', '\psi_{act}');
hold on;
legendEntries = {'\psi_{act}'};
if isfield(data.base, 'yaw_ref')
    refYawDeg = rad2deg(wrapToPi(data.base.yaw_ref));
    plot(timeVec, refYawDeg, '--', 'LineWidth', 1.0, 'DisplayName', '\psi_{ref}');
    legendEntries{end+1} = '\psi_{ref}'; %#ok<AGROW>
end
if isfield(data.base, 'yaw_err')
    yyaxis right;
    plot(timeVec, rad2deg(data.base.yaw_err), ':', 'LineWidth', 1.0, 'DisplayName', '\Delta\psi');
    ylabel('\Delta\psi (deg)');
    legendEntries{end+1} = '\Delta\psi'; %#ok<AGROW>
    yyaxis left;
end
grid on; ylabel('\psi (deg)');
if numel(legendEntries) > 1
    legend(legendEntries, 'Location', 'best');
end
hold off;
nexttile; plot(timeVec, data.base.v_long, 'LineWidth', 1.2); grid on; ylabel('v_{long} (m/s)');
nexttile; plot(timeVec, data.base.v_lat, 'LineWidth', 1.2); grid on; ylabel('v_{lat} (m/s)');
nexttile; plot(timeVec, data.base.omega, 'LineWidth', 1.2); grid on; ylabel('\omega (rad/s)'); xlabel(T, 'Time (s)');

%% Arm joint angles
figArm = figure('Name', 'Arm Joint Angles', 'Color', 'w');
Tarm = tiledlayout(figArm, 3, 2, 'TileSpacing', 'compact');
for j = 1:min(6, size(data.armAngles,2))
    nexttile(Tarm); plot(timeVec, rad2deg(data.armAngles(:,j)), 'LineWidth', 1.2);
    grid on; ylabel(sprintf('%s (deg)', data.armJointNames{j}));
end
xlabel(Tarm, 'Time (s)');

%% End-effector motion metrics
figEE = figure('Name', 'End-Effector Motion Metrics', 'Color', 'w');
Tee = tiledlayout(figEE, 3, 1, 'TileSpacing', 'compact');
nexttile; plot(timeVec, data.eeSpeed, 'LineWidth', 1.3); grid on; ylabel('Speed (m/s)');
nexttile; plot(timeVec, data.eeAccel, 'LineWidth', 1.3); grid on; ylabel('|Acceleration| (m/s^2)');
nexttile; plot(timeVec, data.eeJerk, 'LineWidth', 1.3); grid on; ylabel('|Jerk| (m/s^3)'); xlabel(Tee, 'Time (s)');
title(Tee, 'End-Effector Kinematics');

figs = struct('error', figError, 'base', figBase, 'arm', figArm, 'ee', figEE);

end
