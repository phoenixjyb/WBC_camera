function [v_cmd, omega_cmd, state] = diff_drive_tracker(planar_traj, limits, cfg)
% diff_drive_tracker Generate diff-drive commands via a simple lookahead tracker.
%   planar_traj : struct from split_trajectory with joints [x, y, theta].
%   limits      : struct with v_max, omega_max, a_long_max, alpha_max, lat_acc_max, kappa_max.
%   cfg         : struct with lookahead, v_nominal, k_v, stop tolerances,
%                 optional lat_error_gain to penalize lateral error coupling.

N = numel(planar_traj.points);
if N == 0
    v_cmd = [];
    omega_cmd = [];
    state = struct('x', [], 'y', [], 'theta', []);
    return;
end

if ~isfield(cfg, 'lat_error_gain')
    cfg.lat_error_gain = 1.0;
end

dt = diff([0, [planar_traj.points.time_from_start]]);
dt(dt <= 0) = mean(dt(dt > 0), 'omitnan');
if isnan(dt(1)) || dt(1) == 0
    dt(1) = 0.02;
end

v_cmd = zeros(1, N);
omega_cmd = zeros(1, N);
state.x = zeros(1, N);
state.y = zeros(1, N);
state.theta = zeros(1, N);

pose = [planar_traj.points(1).positions(1), ...
        planar_traj.points(1).positions(2), ...
        planar_traj.points(1).positions(3)];

for k = 1:N
    state.x(k) = pose(1);
    state.y(k) = pose(2);
    state.theta(k) = pose(3);

    look_idx = k;
    while look_idx < N
        dx = planar_traj.points(look_idx).positions(1) - pose(1);
        dy = planar_traj.points(look_idx).positions(2) - pose(2);
        if hypot(dx, dy) >= cfg.lookahead
            break;
        end
        look_idx = look_idx + 1;
    end

    target = planar_traj.points(look_idx).positions;
    dx = target(1) - pose(1);
    dy = target(2) - pose(2);
    dist = hypot(dx, dy);
    path_yaw = atan2(dy, dx);
    alpha = wrapToPi(path_yaw - pose(3));

    if dist < 1e-6
        kappa = 0;
    else
        lateral_term = cfg.lat_error_gain * sin(alpha);
        kappa = 2 * lateral_term / max(dist, cfg.lookahead);
    end
    kappa = min(max(kappa, -limits.kappa_max), limits.kappa_max);

    v_raw = min(cfg.v_nominal, limits.v_max);
    if abs(kappa) > 1e-6
        vmax_curv = sqrt(max(1e-6, limits.lat_acc_max / abs(kappa)));
        v_raw = min(v_raw, vmax_curv);
    end

    if k == N
        yaw_err = wrapToPi(planar_traj.points(end).positions(3) - pose(3));
        dist_to_goal = hypot(planar_traj.points(end).positions(1) - pose(1), ...
                             planar_traj.points(end).positions(2) - pose(2));
        if dist_to_goal < cfg.stop_tol_xy
            v_raw = 0;
            kappa = 0;
            omega_raw = 0;
        else
            v_raw = min(v_raw, cfg.k_v * dist_to_goal);
            omega_raw = v_raw * kappa;
        end
    else
        omega_raw = v_raw * kappa;
    end

    v_cmd(k) = min(max(v_raw, -limits.v_max), limits.v_max);
    omega_cmd(k) = min(max(omega_raw, -limits.omega_max), limits.omega_max);

    pose(3) = wrapToPi(pose(3) + omega_cmd(k) * dt(k));
    pose(1) = pose(1) + v_cmd(k) * cos(pose(3)) * dt(k);
    pose(2) = pose(2) + v_cmd(k) * sin(pose(3)) * dt(k);
end
end
