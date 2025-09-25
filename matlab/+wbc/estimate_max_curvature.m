function maxCurvature = estimate_max_curvature(segmentDist, thetaRef)
%ESTIMATE_MAX_CURVATURE Approximate maximum path curvature for base motion.
%   Mirrors the legacy logic inside rt_whole_body_controller: finite
%   difference yaw deltas divided by segment length.

arguments
    segmentDist double
    thetaRef double
end

if numel(thetaRef) <= 2
    maxCurvature = 0;
    return;
end

curv = 0;
for k = 2:numel(thetaRef)-1
    if isempty(segmentDist)
        ds = 1e-6;
    else
        idx = min(k, numel(segmentDist));
        ds = max(segmentDist(idx), 1e-6);
    end
    dtheta = wrapToPi(thetaRef(k+1) - thetaRef(k));
    curv = max(curv, abs(dtheta) / ds);
end
maxCurvature = curv;
end
