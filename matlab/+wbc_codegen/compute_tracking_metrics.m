function metrics = compute_tracking_metrics(desiredEE, actualEE, basePose, tVec, directionSign) %#codegen
%COMPUTE_TRACKING_METRICS Lightweight codegen-friendly metrics calculator.
%   METRICS = WBC_CODEGEN.COMPUTE_TRACKING_METRICS(DESIRED, ACTUAL, BASEPOSE, TVEC)
%   returns error magnitudes and base velocity projections without relying
%   on Robotics System Toolbox objects.

arguments
    desiredEE double
    actualEE double
    basePose double
    tVec double
    directionSign double = []
end

n = size(desiredEE,1);
if size(actualEE,1) ~= n
    error('Desired and actual EE arrays must match in length.');
end
if size(basePose,1) ~= n
    error('Base pose length must match EE arrays.');
end
if numel(tVec) ~= n
    error('Time vector length must match trajectories.');
end

errVec = desiredEE - actualEE;
errNorm = sqrt(sum(errVec.^2, 2));
metrics = struct();
metrics.ee_error_vec = errVec;
metrics.ee_error_norm = errNorm;

trackingIdx = 1:n;
errTrack = errNorm(trackingIdx);
if isempty(errTrack)
    metrics.max_error_tracking = 0.0;
    metrics.mean_error_tracking = 0.0;
else
    metrics.max_error_tracking = max(errTrack);
    metrics.mean_error_tracking = mean(errTrack);
end
metrics.max_error_total = max(errNorm);
metrics.mean_error_total = mean(errNorm);

vx_world = gradient(basePose(:,1), tVec);
vy_world = gradient(basePose(:,2), tVec);
omega = gradient(basePose(:,3), tVec);
metrics.v_long = vx_world .* cos(basePose(:,3)) + vy_world .* sin(basePose(:,3));
metrics.v_lat = -vx_world .* sin(basePose(:,3)) + vy_world .* cos(basePose(:,3));
metrics.omega = omega;
metrics.v_world = sqrt(vx_world.^2 + vy_world.^2);
metrics.direction_sign = directionSign;
end

