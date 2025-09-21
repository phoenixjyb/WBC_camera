function plot_planar_path(planar_traj, state)
% plot_planar_path Visualize planar trajectory and tracker pose.

figure; hold on; grid on;
pts = vertcat(planar_traj.points.positions);
plot(pts(:,1), pts(:,2), 'k--o', 'DisplayName', 'Planar Waypoints');
plot(state.x, state.y, 'b-', 'DisplayName', 'Tracker Pose');
quiver(state.x, state.y, cos(state.theta)*0.1, sin(state.theta)*0.1, 0, ...
       'Color', [0 0.4 0.8], 'DisplayName', 'Heading');
xlabel('x (m)'); ylabel('y (m)'); axis equal;
legend('Location', 'best'); title('Planar Trajectory vs Tracker');
end
