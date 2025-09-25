function [thetaRef, arcLen, segmentDist] = compute_base_heading(baseWaypoints)
%COMPUTE_BASE_HEADING Compute chassis heading and arc-length profiles.
%   Mirrors the legacy helper embedded in rt_whole_body_controller, returning
%   the cumulative arc length and per-segment distances alongside the yaw
%   reference timeline derived from XY waypoints.

arguments
    baseWaypoints double
end

numPts = size(baseWaypoints, 1);
if numPts < 2
    thetaRef = zeros(numPts, 1);
    arcLen = zeros(numPts, 1);
    segmentDist = zeros(max(numPts-1, 1), 1);
    return;
end

segmentDiff = diff(baseWaypoints, 1, 1);
segmentDist = sqrt(sum(segmentDiff.^2, 2));
arcLen = [0; cumsum(segmentDist)];

thetaRef = zeros(numPts, 1);
thetaRef(1:end-1) = atan2(segmentDiff(:,2), segmentDiff(:,1));
thetaRef(end) = thetaRef(end-1);
thetaRef = unwrap(thetaRef);
end
