function fig = plot_ee_comparison(desiredPos, actualPos, basePose)
%PLOT_EE_COMPARISON 3D comparison of desired vs. actual EE paths with chassis path.
%
%   helpers.plot_ee_comparison(desiredPos, actualPos, basePose)
%   plots the desired end-effector path, the realised path, and the chassis
%   trajectory in world coordinates. desiredPos and actualPos must be
%   N-by-3 arrays; basePose is N-by-3 [x y yaw].

arguments
    desiredPos double
    actualPos double
    basePose double
end

fig = figure('Name', 'EE Desired vs Actual (3D)', 'Color', 'w');
ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
view(ax, 42, 28);

plot3(ax, basePose(:,1), basePose(:,2), zeros(size(basePose,1),1), ...
      'k--', 'LineWidth', 1.0, 'DisplayName', 'Chassis path');
plot3(ax, desiredPos(:,1), desiredPos(:,2), desiredPos(:,3), ...
      'r-.', 'LineWidth', 1.5, 'DisplayName', 'Desired EE path');
plot3(ax, actualPos(:,1), actualPos(:,2), actualPos(:,3), ...
      'Color', [0 0.6 0], 'LineWidth', 1.8, 'DisplayName', 'Actual EE path');

scatter3(ax, desiredPos(1,1), desiredPos(1,2), desiredPos(1,3), 80, ...
         'filled', 'MarkerFaceColor', [0.8 0.2 0.2], 'DisplayName', 'Desired start');
scatter3(ax, desiredPos(end,1), desiredPos(end,2), desiredPos(end,3), 80, ...
         'filled', 'MarkerFaceColor', [0.8 0.2 0.8], 'DisplayName', 'Desired end');
scatter3(ax, actualPos(1,1), actualPos(1,2), actualPos(1,3), 70, ...
         's', 'MarkerFaceColor', [0.1 0.6 0.2], 'DisplayName', 'Actual start');
scatter3(ax, actualPos(end,1), actualPos(end,2), actualPos(end,3), 70, ...
         's', 'MarkerFaceColor', [0.1 0.4 0.8], 'DisplayName', 'Actual end');

legend(ax, 'Location', 'bestoutside');
title(ax, 'Desired vs Actual End-Effector Paths');
end
