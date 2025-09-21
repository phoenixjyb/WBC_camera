function plot_chassis_profile(timeVec, basePose)
%PLOT_CHASSIS_PROFILE Plot chassis x, y, and yaw vs time along with XY path.
%
%   plot_chassis_profile(timeVec, basePose) generates a figure with three
%   subplots (x, y, yaw) and an inset showing the planar trajectory.
%
%   Inputs:
%     timeVec  - 1xN or Nx1 timestamps (seconds).
%     basePose - Nx3 matrix [x y yaw].

arguments
    timeVec double
    basePose double
end

timeVec = timeVec(:);
if size(basePose,1) ~= numel(timeVec) || size(basePose,2) ~= 3
    error('basePose must be Nx3 and match numel(timeVec).');
end

yawDeg = rad2deg(wrapToPi(basePose(:,3)));

fig = figure('Name', 'Chassis Profiles', 'Color', 'w');
tiledlayout(fig, 4, 1, 'TileSpacing', 'compact');

nexttile; plot(timeVec, basePose(:,1), 'LineWidth', 1.5);
ylabel('x [m]'); grid on;

nexttile; plot(timeVec, basePose(:,2), 'LineWidth', 1.5);
ylabel('y [m]'); grid on;

nexttile; plot(timeVec, yawDeg, 'LineWidth', 1.5);
ylabel('\psi [deg]'); grid on;

axPath = nexttile([1 1]);
plot(axPath, basePose(:,1), basePose(:,2), 'k-', 'LineWidth', 1.5);
hold(axPath, 'on'); plot(axPath, basePose(1,1), basePose(1,2), 'go', 'MarkerFaceColor', 'g');
plot(axPath, basePose(end,1), basePose(end,2), 'ro', 'MarkerFaceColor', 'r');
axis(axPath, 'equal'); grid(axPath, 'on');
xlabel(axPath, 'x [m]'); ylabel(axPath, 'y [m]');
title(axPath, 'Planar Chassis Path');
end
