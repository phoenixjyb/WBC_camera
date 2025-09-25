function out = sync_base_and_arm(baseWaypointsRef, thetaRef, arcLen, refTimes, refPositionsWorld, refRPYWorld, armTrajectoryRef, baseLimits, baseSpeedLimit, thetaRampEnd) %#codegen
%SYNC_BASE_AND_ARM Codegen wrapper around wbc.sync_base_and_arm.
%   OUT = WBC_CODEGEN.SYNC_BASE_AND_ARM(...) exposes the retiming and
%   synchronization logic for MATLAB Coder. The wrapper strips cell arrays
%   and enforces fixed-structure outputs that are friendly to generated C++.
%
%   Inputs mirror wbc.sync_base_and_arm. Outputs:
%     out.armTimes             - column vector of timestamps
%     out.armTrajectoryInitial - retimed joint trajectory
%     out.armVelocities        - retimed joint velocities
%     out.retimeInfo           - struct with segment/arrival times
%     out.basePoseTrack        - Nx3 base pose history
%     out.baseVelocity         - struct with fields v, omega
%     out.scaleFactor          - scalar sync scale factor
%     out.syncDiag             - struct with diagnostic arrays
%     out.thetaRefTimeline     - yaw reference timeline
%     out.desiredEETrack       - Nx3 end-effector positions
%     out.desiredRPYTrack      - Nx3 end-effector orientation (RPY)
%     out.poseTformsFinal      - 4x4xN transforms (base->ee)
%
%   All numeric outputs are double for compatibility with downstream C++
%   ports.

coder.inline('never');

syncStruct = wbc.sync_base_and_arm(baseWaypointsRef, thetaRef, arcLen, refTimes, ...
    refPositionsWorld, refRPYWorld, armTrajectoryRef, baseLimits, baseSpeedLimit, thetaRampEnd);

out = struct();
out.armTimes = syncStruct.armTimes;
out.armTrajectoryInitial = syncStruct.armTrajectoryInitial;
out.armVelocities = syncStruct.armVelocities;
out.retimeInfo = syncStruct.retimeInfo;
out.basePoseTrack = syncStruct.basePoseTrack;
out.baseVelocity = syncStruct.baseVelocity;
out.scaleFactor = syncStruct.scaleFactor;
out.syncDiag = syncStruct.syncDiag;
out.thetaRefTimeline = syncStruct.thetaRefTimeline;
out.desiredEETrack = syncStruct.desiredEETrack;
out.desiredRPYTrack = syncStruct.desiredRPYTrack;

poseTforms = syncStruct.poseTformsFinal;
if ~isempty(poseTforms) && ndims(poseTforms) == 3
    out.poseTformsFinal = poseTforms;
else
    numT = numel(poseTforms);
    poseArray = zeros(4,4,numT);
    for k = 1:numT
        poseArray(:,:,k) = poseTforms{k}; %#ok<AGROW>
    end
    out.poseTformsFinal = poseArray;
end

end

