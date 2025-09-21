function [tSamples, qSamples, qdSamples, info] = retime_joint_trajectory(waypoints, opts)
arguments
    waypoints {mustBeNumeric, mustBeNonempty}
    opts.MaxVelocity (1,:) double {mustBePositive} = ones(1, size(waypoints,2))
    opts.MaxAcceleration (1,:) double {mustBePositive} = 2 * ones(1, size(waypoints,2))
    opts.TimeStep (1,1) double {mustBePositive} = 0.05
    opts.MinSegmentTime (1,1) double {mustBePositive} = 0.1
end

[M, numJoints] = size(waypoints);
if numJoints ~= numel(opts.MaxVelocity) || numJoints ~= numel(opts.MaxAcceleration)
    error('Velocity/acceleration limit vectors must match number of joints.');
end
if M < 2
    error('Need at least two waypoints to retime trajectory.');
end

segmentTimes = zeros(1, M-1);
for k = 1:M-1
    delta = abs(waypoints(k+1,:) - waypoints(k,:));
    velTime = delta ./ opts.MaxVelocity;
    accTime = sqrt(2 * delta ./ opts.MaxAcceleration);
    segmentTimes(k) = max([velTime, accTime, opts.MinSegmentTime]);
end

info = struct('segmentTimes', segmentTimes, 'arrivalTimes', [0 cumsum(segmentTimes)]);

tSamples = 0;
qSamples = waypoints(1,:);
for k = 1:M-1
    segTime = segmentTimes(k);
    nSteps = max(2, ceil(segTime / opts.TimeStep) + 1);
    localTimes = linspace(0, segTime, nSteps);
    startIdx = numel(tSamples);
    for s = 2:nSteps
        tau = localTimes(s) / segTime;
        if segTime == 0
            tau = 1;
        end
        qNew = (1 - tau) * waypoints(k,:) + tau * waypoints(k+1,:);
        tNew = tSamples(end) + localTimes(s);
        tSamples(end+1) = tNew; %#ok<AGROW>
        qSamples(end+1, :) = qNew; %#ok<AGROW>
    end
end

% compute velocities via finite difference
qdSamples = zeros(size(qSamples));
dt = diff(tSamples);
for j = 1:numJoints
    dq = diff(qSamples(:,j));
    v = dq ./ dt';
    qdSamples(1:end-1,j) = v;
    qdSamples(end,j) = v(end);
end
end
