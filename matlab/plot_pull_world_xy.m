function plot_pull_world_xy(output_path)
%PLOT_PULL_WORLD_XY Plot the 1\_pull\_world trajectory onto the ground plane.
%   plot_pull_world_xy() loads matlab/1_pull_world.json and plots the
%   end-effector path projected onto the XY plane. The figure is saved to
%   matlab/outputs/1_pull_world_xy.png by default.

    if nargin < 1
        output_path = fullfile(fileparts(mfilename('fullpath')), 'outputs', '1_pull_world_xy.png');
    end

    data_dir = fileparts(mfilename('fullpath'));
    trajectory_path = fullfile(data_dir, '1_pull_world.json');
    raw = fileread(trajectory_path);
    decoded = jsondecode(raw);

    if ~isfield(decoded, 'poses') || isempty(decoded.poses)
        error('plot_pull_world_xy:missingData', 'No poses found in %s', trajectory_path);
    end

    positions = reshape([decoded.poses.position], 3, []).';
    xy = positions(:, 1:2);

    fig = figure('Name', 'Pull World Trajectory XY', 'Color', 'w');
    plot(xy(:, 1), xy(:, 2), 'LineWidth', 1.5, 'DisplayName', 'Trajectory');
    hold on;
    plot(xy(1, 1), xy(1, 2), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot(xy(end, 1), xy(end, 2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'End');
    hold off;

    xlabel('X (m)');
    ylabel('Y (m)');
    title('1\_pull\_world trajectory projection');
    grid on;
    axis equal;
    legend('Location', 'best');

    output_dir = fileparts(output_path);
    if ~isempty(output_dir) && ~isfolder(output_dir)
        mkdir(output_dir);
    end

    exportgraphics(fig, output_path, 'Resolution', 300);
    fprintf('Saved XY trajectory plot to %s\n', output_path);
end
