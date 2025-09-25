function out = stream_tracking_with_buffer(args)
%STREAM_TRACKING_WITH_BUFFER Offline parity fallback for streaming mode.
%   OUT = WBC.STREAM_TRACKING_WITH_BUFFER(ARGS) currently invokes the full
%   offline synchronisation helper to keep the refactored controller aligned
%   with the legacy MATLAB implementation while a true streaming retimer is
%   reworked. Rolling-horizon diagnostics are generated from the offline
%   timeline so downstream code can be instrumented without sacrificing
%   numerical parity.

arguments
    args struct
end

baseWaypointsRef = args.baseWaypointsRef;
thetaRef = args.thetaRef;
if isfield(args, 'arcLen') && ~isempty(args.arcLen)
    arcLenRef = args.arcLen;
else
    [~, arcLenRef] = local_compute_heading(baseWaypointsRef);
end
refTimes = args.refTimes;
refPositionsWorld = args.refPositionsWorld;
refRPYWorld = args.refRPYWorld;
armTrajectoryRef = args.armTrajectoryRef;
baseLimits = args.baseLimits;
baseSpeedLimit = args.baseSpeedLimit;
thetaRampEnd = args.thetaRampEnd;
debugEnabled = getfieldwithdefault(args, 'debug', false);
sampleDt = getfieldwithdefault(args, 'sample_dt', 0.0);
stepTime = getfieldwithdefault(args, 'step_time', 0.0);
horizonTime = getfieldwithdefault(args, 'horizon_time', 0.0);
streamMode = string(getfieldwithdefault(args, 'mode', "passthrough"));

fullSync = wbc.sync_base_and_arm(baseWaypointsRef, thetaRef, arcLenRef, refTimes, ...
    refPositionsWorld, refRPYWorld, armTrajectoryRef, baseLimits, baseSpeedLimit, thetaRampEnd);

if ~coder.target('MATLAB')
    % no-op during code generation
else
    try %#ok<TRYNC>
        inputs = struct();
        inputs.baseWaypointsRef = baseWaypointsRef;
        inputs.thetaRef = thetaRef;
        inputs.arcLen = arcLenRef;
        inputs.refTimes = refTimes;
        inputs.refPositionsWorld = refPositionsWorld;
        inputs.refRPYWorld = refRPYWorld;
        inputs.armTrajectoryRef = armTrajectoryRef;
        inputs.baseLimits = baseLimits;
        inputs.baseSpeedLimit = baseSpeedLimit;
        inputs.thetaRampEnd = thetaRampEnd;
        inputs.sample_dt = sampleDt;
        inputs.step_time = stepTime;
        inputs.horizon_time = horizonTime;
        inputs.mode = streamMode;
        save(fullfile(pwd, 'outputs', 'last_stream_stub.mat'), 'fullSync', 'inputs');
    catch
        % ignore logging failures
    end
end

if streamMode == "passthrough"
    out = fullSync;
    out.streamMode = streamMode;
    if isfield(fullSync, 'refTimesAligned') && ~isempty(fullSync.refTimesAligned)
        out.refTimesGlobal = fullSync.refTimesAligned(:);
    else
        out.refTimesGlobal = fullSync.armTimes(:);
    end

    windowSummary = struct();
    if debugEnabled
        windowSummary = local_window_summary(out.armTimes, stepTime, horizonTime, sampleDt);
        if ~isempty(windowSummary)
            out.windowSummary = windowSummary;
        end
    end

    if debugEnabled
        out.debug = struct('mode', streamMode, ...
                           'note', "Streaming disabled (passthrough)", ...
                           'windowSummary', {windowSummary}, ...
                           'stepTime', stepTime, ...
                           'horizonTime', horizonTime, ...
                           'sampleDt', sampleDt);
    else
        out.debug = struct([]);
    end
else
    streamingArgs = struct('baseWaypointsRef', baseWaypointsRef, ...
                           'thetaRef', thetaRef, 'arcLen', arcLenRef, 'refTimes', refTimes, ...
                           'refPositionsWorld', refPositionsWorld, 'refRPYWorld', refRPYWorld, ...
                           'armTrajectoryRef', armTrajectoryRef, 'baseLimits', baseLimits, ...
                           'baseSpeedLimit', baseSpeedLimit, 'thetaRampEnd', thetaRampEnd, ...
                           'sample_dt', sampleDt, 'step_time', stepTime, 'horizon_time', horizonTime);
    out = local_streaming_retime(streamingArgs);
    out.streamMode = streamMode;

    windowSummary = local_window_summary(out.armTimes, stepTime, horizonTime, sampleDt);
    if ~isempty(windowSummary)
        out.windowSummary = windowSummary;
    end

    if ~isfield(out, 'streamReplay') || isempty(out.streamReplay)
        out.streamReplay = local_build_stream_replay(out.armTimes, stepTime, horizonTime);
    end

    if debugEnabled
        out.debug = struct('mode', streamMode, ...
                           'note', "Rolling-horizon retime executed", ...
                           'windowSummary', {windowSummary}, ...
                           'streamReplay', {out.streamReplay}, ...
                           'stepTime', stepTime, ...
                           'horizonTime', horizonTime, ...
                           'sampleDt', sampleDt);
    else
        out.debug = struct([]);
    end
end

end

function val = getfieldwithdefault(s, fieldName, defaultVal)
if isfield(s, fieldName) && ~isempty(s.(fieldName))
    val = s.(fieldName);
else
    val = defaultVal;
end
end

function [thetaRef, arcLen] = local_compute_heading(baseWaypoints)
numPts = size(baseWaypoints, 1);
if numPts < 2
    thetaRef = zeros(numPts, 1);
    arcLen = zeros(numPts, 1);
    return;
end
segmentDiff = diff(baseWaypoints, 1, 1);
segmentDist = sqrt(sum(segmentDiff.^2, 2));
arcLen = [0; cumsum(segmentDist)];
thetaRef = zeros(numPts, 1);
thetaRef(1:end-1) = atan2(segmentDiff(:, 2), segmentDiff(:, 1));
thetaRef(end) = thetaRef(end-1);
thetaRef = unwrap(thetaRef);
end

function windows = local_window_summary(armTimes, stepTime, horizonTime, sampleDt)
windows = struct('index_start', {}, 'index_end', {}, 'time_start', {}, ...
                 'time_end', {}, 'duration', {}, 'samples', {}, ...
                 'horizon_time', {}, 'horizon_index', {}, 'overlap', {}, ...
                 'step_samples_est', {});
if isempty(armTimes)
    return;
end

if nargin < 2 || isempty(stepTime) || ~isfinite(stepTime) || stepTime <= 0
    stepTime = 0;
end
if nargin < 3 || isempty(horizonTime) || ~isfinite(horizonTime) || horizonTime <= 0
    horizonTime = 0;
end
if nargin < 4 || isempty(sampleDt) || ~isfinite(sampleDt) || sampleDt <= 0
    sampleDt = 0;
end

numSamples = numel(armTimes);
idxStart = 1;
winIdx = 0;
while idxStart <= numSamples
    timeStart = armTimes(idxStart);

    if idxStart == numSamples
        idxEnd = numSamples;
    else
        if stepTime > 0
            targetTime = timeStart + stepTime - 1e-9;
            idxEnd = find(armTimes >= targetTime, 1, 'first');
        else
            idxEnd = [];
        end
        if isempty(idxEnd)
            idxEnd = numSamples;
        end
    end

    if idxEnd <= idxStart && idxStart < numSamples
        idxEnd = idxStart + 1;
    end

    timeEnd = armTimes(idxEnd);
    duration = timeEnd - timeStart;

    horizonIndex = idxEnd;
    if horizonTime > 0 && idxStart < numSamples
        targetTime = timeStart + horizonTime - 1e-9;
        idxHorizon = find(armTimes >= targetTime, 1, 'first');
        if isempty(idxHorizon)
            idxHorizon = numSamples;
        end
        horizonIndex = idxHorizon;
    end

    overlapFlag = 0;
    if winIdx > 0 && idxStart == windows(winIdx).index_end
        overlapFlag = 1;
    end

    stepSamplesEst = 0;
    if sampleDt > 0 && stepTime > 0
        stepSamplesEst = max(1, round(stepTime / sampleDt));
    end

    winIdx = winIdx + 1;
    windows(winIdx,1) = struct('index_start', idxStart, ...
                               'index_end', idxEnd, ...
                               'time_start', timeStart, ...
                               'time_end', timeEnd, ...
                               'duration', duration, ...
                               'samples', idxEnd - idxStart + 1, ...
                               'horizon_time', horizonTime, ...
                               'horizon_index', horizonIndex, ...
                               'overlap', overlapFlag, ...
                               'step_samples_est', stepSamplesEst); %#ok<AGROW>

    if idxEnd >= numSamples
        break;
    end

    if stepTime > 0
        idxStart = idxEnd;
    else
        idxStart = idxEnd + 1;
    end
end
end

function replay = local_build_stream_replay(armTimes, stepTime, horizonTime)
replay = struct('iteration', {}, 'release_time', {}, 'append_start', {}, ...
                'append_end', {}, 'append_count', {}, 'window_end', {}, ...
                'window_count', {}, 'time_start', {}, 'time_end', {});

if isempty(armTimes)
    return;
end

if nargin < 2 || isempty(stepTime) || ~isfinite(stepTime) || stepTime <= 0
    return;
end
if nargin < 3 || isempty(horizonTime) || ~isfinite(horizonTime) || horizonTime <= 0
    horizonTime = stepTime;
end

startTime = armTimes(1);
endTime = armTimes(end);

releaseTimes = startTime:stepTime:(endTime + 1e-9);
if releaseTimes(end) < endTime - 1e-9
    releaseTimes(end+1) = endTime; %#ok<AGROW>
end

lastAppendIdx = 0;
iter = 0;

for k = 1:numel(releaseTimes)
    currentRelease = releaseTimes(k);
    idxRelease = find(armTimes <= currentRelease + 1e-9, 1, 'last');
    if isempty(idxRelease)
        idxRelease = lastAppendIdx;
    end
    if idxRelease <= lastAppendIdx
        continue;
    end

    appendStart = max(lastAppendIdx + 1, 1);
    appendEnd = idxRelease;

    horizonLimit = currentRelease + horizonTime;
    idxWindowEnd = find(armTimes <= horizonLimit + 1e-9, 1, 'last');
    if isempty(idxWindowEnd)
        idxWindowEnd = numel(armTimes);
    end

    iter = iter + 1;
    replay(iter,1) = struct('iteration', iter, ...
                            'release_time', currentRelease - startTime, ...
                            'append_start', appendStart, ...
                            'append_end', appendEnd, ...
                            'append_count', appendEnd - appendStart + 1, ...
                            'window_end', idxWindowEnd, ...
                            'window_count', idxWindowEnd - appendStart + 1, ...
                            'time_start', armTimes(appendStart) - startTime, ...
                            'time_end', armTimes(appendEnd) - startTime); %#ok<AGROW>

    lastAppendIdx = appendEnd;
    if lastAppendIdx >= numel(armTimes)
        break;
    end
end

if isempty(replay)
    replay = struct();
end
end

function out = local_streaming_retime(args)
%LOCAL_STREAMING_RETIME Execute a rolling-horizon retime with buffer.

baseWaypointsRef = args.baseWaypointsRef;
thetaRef = args.thetaRef;
arcLen = args.arcLen;
refTimes = args.refTimes;
refPositionsWorld = args.refPositionsWorld;
refRPYWorld = args.refRPYWorld;
armTrajectoryRef = args.armTrajectoryRef;
baseLimits = args.baseLimits;
baseSpeedLimit = args.baseSpeedLimit;
thetaRampEnd = args.thetaRampEnd;
sampleDt = args.sample_dt;
stepTime = args.step_time;
horizonTime = args.horizon_time;

numSamples = size(armTrajectoryRef, 1);
numJoints = size(armTrajectoryRef, 2);

if stepTime <= 0
    stepTime = max(sampleDt, 0.1);
end
if horizonTime <= stepTime
    if sampleDt > 0
        horizonTime = stepTime + 2 * sampleDt;
    else
        horizonTime = stepTime * 1.5;
    end
end

% Build release/horizon schedule on reference timeline
schedule = local_build_release_schedule(refTimes, stepTime, horizonTime);

% Preallocate global buffers
globalArmTimes = zeros(numSamples, 1);
globalArmTrajectory = zeros(numSamples, numJoints);
globalArmVel = zeros(numSamples, numJoints);
globalBasePose = zeros(numSamples, 3);
globalDesiredEE = zeros(numSamples, size(refPositionsWorld, 2));
globalDesiredRPY = zeros(numSamples, size(refRPYWorld, 2));
globalThetaRef = zeros(numSamples, 1);
globalVBase = zeros(numSamples, 1);
globalOmegaBase = zeros(numSamples, 1);
globalDirSign = zeros(numSamples, 1);
globalRefTimesAligned = zeros(numSamples, 1);
globalPoseTforms = zeros(4, 4, numSamples);
scaleHistoryAccum = [];

appendPtr = 0;
timeOffset = 0.0;
prevTheta = thetaRampEnd;
replayLog = struct('iteration', {}, 'release_time', {}, 'append_start', {}, ...
                   'append_end', {}, 'append_count', {}, 'window_end', {}, ...
                   'window_count', {}, 'time_start', {}, 'time_end', {});

for iter = 1:numel(schedule)
    windowStart = schedule(iter).window_start_idx;
    windowEnd = schedule(iter).window_end_idx;
    releaseIdx = schedule(iter).release_idx;
    if windowStart > windowEnd || releaseIdx <= appendPtr
        continue;
    end

    idxRange = windowStart:windowEnd;
    refTimesWindow = refTimes(idxRange);
    refTimeOffset = refTimesWindow(1);
    refTimesShift = refTimesWindow - refTimeOffset;

    baseWindow = baseWaypointsRef(idxRange, :);
    thetaWindow = thetaRef(idxRange);
    arcWindow = arcLen(idxRange);
    arcWindow = arcWindow - arcWindow(1);
    posWindow = refPositionsWorld(idxRange, :);
    rpyWindow = refRPYWorld(idxRange, :);
    armWindow = armTrajectoryRef(idxRange, :);

    thetaInit = prevTheta;
    if isempty(thetaInit)
        thetaInit = thetaWindow(1);
    end

    if numel(idxRange) < 2
        localCount = releaseIdx - appendPtr;
        if localCount <= 0
            continue;
        end
        appendCount = min(localCount, numel(idxRange));
        appendIdx = 1:appendCount;
        globalIdx = (appendPtr + 1):(appendPtr + appendCount);
        dtLocal = sampleDt;
        if dtLocal <= 0
            dtLocal = stepTime;
        end
        timesLocal = (appendIdx - 1)' * dtLocal;
        if appendPtr > 0 && appendIdx(1) == 1
            timesLocal = timesLocal + dtLocal;
        end
        timesGlobal = timesLocal + timeOffset;

        globalArmTimes(globalIdx) = timesGlobal;
        globalArmTrajectory(globalIdx, :) = armWindow(appendIdx, :);
        globalArmVel(globalIdx, :) = 0;
        globalBasePose(globalIdx, :) = [baseWindow(appendIdx, :), thetaWindow(appendIdx)];
        globalDesiredEE(globalIdx, :) = posWindow(appendIdx, :);
        globalDesiredRPY(globalIdx, :) = rpyWindow(appendIdx, :);
        globalThetaRef(globalIdx) = thetaWindow(appendIdx);
        globalVBase(globalIdx) = 0;
        globalOmegaBase(globalIdx) = 0;
        globalRefTimesAligned(globalIdx) = refTimes(idxRange(appendIdx));
        for kk = 1:numel(globalIdx)
            basePose = baseWindow(appendIdx(kk), :);
            thetaVal = thetaWindow(appendIdx(kk));
            Tbase = trvec2tform([basePose, 0]) * axang2tform([0 0 1 thetaVal]);
            Tworld = trvec2tform(posWindow(appendIdx(kk), :)) * eul2tform(rpyWindow(appendIdx(kk), :), 'XYZ');
            globalPoseTforms(:, :, globalIdx(kk)) = Tbase \ Tworld;
        end
        prevTheta = thetaWindow(appendIdx(end));

        replayLog(end+1,1) = struct('iteration', iter, ...
                                    'release_time', refTimes(releaseIdx) - refTimes(1), ...
                                    'append_start', globalIdx(1), ...
                                    'append_end', globalIdx(end), ...
                                    'append_count', numel(globalIdx), ...
                                    'window_end', windowEnd, ...
                                    'window_count', windowEnd - windowStart + 1, ...
                                    'time_start', timesGlobal(1), ...
                                    'time_end', timesGlobal(end)); %#ok<AGROW>

        appendPtr = appendPtr + appendCount;
        timeOffset = timesGlobal(end);

        if appendPtr >= numSamples
            break;
        end

        continue;
    end

    chunkOut = wbc.sync_base_and_arm(baseWindow, thetaWindow, arcWindow, refTimesShift, ...
        posWindow, rpyWindow, armWindow, baseLimits, baseSpeedLimit, thetaInit);

    localCount = releaseIdx - appendPtr;
    if localCount <= 0
        continue;
    end

    % Determine indices to append from chunk
    appendIdx = 1:min(localCount, size(chunkOut.armTimes, 1));
    if isempty(appendIdx)
        continue;
    end

    globalIdx = (appendPtr + 1):(appendPtr + numel(appendIdx));
    timesLocal = chunkOut.armTimes(appendIdx);
    if appendPtr > 0 && appendIdx(1) == 1
        if numel(timesLocal) >= 2
            dtLead = timesLocal(2) - timesLocal(1);
        else
            dtLead = sampleDt;
        end
        if dtLead <= 0
            dtLead = max(sampleDt, stepTime);
        end
        timesLocal = timesLocal + dtLead;
    end
    timesGlobal = timesLocal + timeOffset;

    globalArmTimes(globalIdx) = timesGlobal;
    globalArmTrajectory(globalIdx, :) = chunkOut.armTrajectoryInitial(appendIdx, :);
    if size(chunkOut.armVelocities, 1) >= appendIdx(end)
        globalArmVel(globalIdx, :) = chunkOut.armVelocities(appendIdx, :);
    end
    if size(chunkOut.basePoseTrack, 1) >= appendIdx(end)
        globalBasePose(globalIdx, :) = chunkOut.basePoseTrack(appendIdx, :);
        prevTheta = chunkOut.basePoseTrack(appendIdx(end), 3);
    end
    if size(chunkOut.desiredEETrack, 1) >= appendIdx(end)
        globalDesiredEE(globalIdx, :) = chunkOut.desiredEETrack(appendIdx, :);
    end
    if size(chunkOut.desiredRPYTrack, 1) >= appendIdx(end)
        globalDesiredRPY(globalIdx, :) = chunkOut.desiredRPYTrack(appendIdx, :);
    end
    if isfield(chunkOut, 'thetaRefTimeline') && numel(chunkOut.thetaRefTimeline) >= appendIdx(end)
        globalThetaRef(globalIdx) = chunkOut.thetaRefTimeline(appendIdx);
    else
        globalThetaRef(globalIdx) = thetaWindow(appendIdx);
    end
    if isfield(chunkOut, 'baseVelocity')
        vLocal = chunkOut.baseVelocity.v;
        omegaLocal = chunkOut.baseVelocity.omega;
        if numel(vLocal) >= appendIdx(end)
            globalVBase(globalIdx) = vLocal(appendIdx);
        end
        if numel(omegaLocal) >= appendIdx(end)
            globalOmegaBase(globalIdx) = omegaLocal(appendIdx);
        end
    end
    if isfield(chunkOut, 'poseTformsFinal') && size(chunkOut.poseTformsFinal, 3) >= appendIdx(end)
        globalPoseTforms(:, :, globalIdx) = chunkOut.poseTformsFinal(:, :, appendIdx);
    end
    if isfield(chunkOut, 'syncDiag') && isfield(chunkOut.syncDiag, 'directionSign')
        dirLocal = chunkOut.syncDiag.directionSign;
        if numel(dirLocal) >= appendIdx(end)
            globalDirSign(globalIdx) = dirLocal(appendIdx);
        end
        if isfield(chunkOut.syncDiag, 'scaleHistory') && ~isempty(chunkOut.syncDiag.scaleHistory)
            scaleHistoryAccum = [scaleHistoryAccum; chunkOut.syncDiag.scaleHistory(:)]; %#ok<AGROW>
        end
    end

    if isfield(chunkOut, 'refTimesAligned') && numel(chunkOut.refTimesAligned) >= appendIdx(end)
        globalRefTimesAligned(globalIdx) = chunkOut.refTimesAligned(appendIdx) + timeOffset;
    else
        globalRefTimesAligned(globalIdx) = refTimes(idxRange(appendIdx));
    end

    replayLog(end+1,1) = struct('iteration', iter, ...
                                'release_time', refTimes(releaseIdx) - refTimes(1), ...
                                'append_start', globalIdx(1), ...
                                'append_end', globalIdx(end), ...
                                'append_count', numel(globalIdx), ...
                                'window_end', windowEnd, ...
                                'window_count', windowEnd - windowStart + 1, ...
                                'time_start', timesGlobal(1), ...
                                'time_end', timesGlobal(end)); %#ok<AGROW>

    appendPtr = appendPtr + numel(appendIdx);
    timeOffset = timesGlobal(end);

    if appendPtr >= numSamples
        break;
    end
end

if appendPtr < numSamples
    % Append any leftover samples directly from remaining reference (no additional retime)
    remainingIdx = appendPtr+1:numSamples;
    globalArmTimes(remainingIdx) = timeOffset + sampleDt * (1:numel(remainingIdx))';
    globalArmTrajectory(remainingIdx, :) = armTrajectoryRef(remainingIdx, :);
    globalArmVel(remainingIdx, :) = 0;
    globalBasePose(remainingIdx, :) = [baseWaypointsRef(remainingIdx, :), thetaRef(remainingIdx)];
    globalDesiredEE(remainingIdx, :) = refPositionsWorld(remainingIdx, :);
    globalDesiredRPY(remainingIdx, :) = refRPYWorld(remainingIdx, :);
    globalThetaRef(remainingIdx) = thetaRef(remainingIdx);
    globalVBase(remainingIdx) = 0;
    globalOmegaBase(remainingIdx) = 0;
    globalRefTimesAligned(remainingIdx) = refTimes(remainingIdx);
    for idx = remainingIdx
        globalPoseTforms(:,:,idx) = eye(4);
    end
end

% Recompute base velocities on the stitched timeline and enforce limits
[globalVBase, globalOmegaBase] = local_compute_base_velocities(globalBasePose, globalArmTimes);

scaleFactor = max([1, max(abs(globalVBase))/max(baseSpeedLimit,1e-6), ...
                   max(abs(globalOmegaBase))/max(baseLimits.omega_max,1e-6)]);

refineIter = 0;
while scaleFactor > 1 + 1e-6 && refineIter < 4
    globalArmTimes = globalArmTimes * scaleFactor;
    globalRefTimesAligned = globalRefTimesAligned * scaleFactor;
    [globalVBase, globalOmegaBase] = local_compute_base_velocities(globalBasePose, globalArmTimes);
    scaleFactor = max([1, max(abs(globalVBase))/max(baseSpeedLimit,1e-6), ...
                       max(abs(globalOmegaBase))/max(baseLimits.omega_max,1e-6)]);
    refineIter = refineIter + 1;
end

out = struct();
out.armTimes = globalArmTimes(:);
out.armTrajectoryInitial = globalArmTrajectory;
out.armVelocities = globalArmVel;
out.retimeInfo = struct('arrivalTimes', globalArmTimes(:), ...
                        'segmentTimes', [globalArmTimes(1); diff(globalArmTimes(:))]);
out.basePoseTrack = globalBasePose;
out.baseVelocity = struct('v', globalVBase(:), 'omega', globalOmegaBase(:));
out.scaleFactor = scaleFactor;
out.syncDiag = struct('thetaRefTimeline', globalThetaRef(:), ...
                      'scaleHistory', scaleHistoryAccum, ...
                      'directionSign', globalDirSign(:));
out.thetaRefTimeline = globalThetaRef(:);
out.desiredEETrack = globalDesiredEE;
out.desiredRPYTrack = globalDesiredRPY;
out.poseTformsFinal = globalPoseTforms;
out.refTimesAligned = globalRefTimesAligned(:);
out.streamReplay = replayLog;

end

function schedule = local_build_release_schedule(refTimes, stepTime, horizonTime)
numSamples = numel(refTimes);
schedule = struct('window_start_idx', {}, 'window_end_idx', {}, 'release_idx', {});

if numSamples == 0
    return;
end

startTime = refTimes(1);
endTime = refTimes(end);

if stepTime <= 0
    schedule = struct('window_start_idx', 1, 'window_end_idx', numSamples, 'release_idx', numSamples);
    return;
end

releaseTimes = startTime:stepTime:(endTime + 1e-9);
if releaseTimes(end) < endTime - 1e-9
    releaseTimes(end+1) = endTime; %#ok<AGROW>
end

lastReleaseIdx = 0;

for k = 1:numel(releaseTimes)
    releaseTime = releaseTimes(k);
    releaseIdx = find(refTimes <= releaseTime + 1e-9, 1, 'last');
    if isempty(releaseIdx) || releaseIdx <= lastReleaseIdx
        continue;
    end

    windowEndTime = releaseTime + horizonTime;
    windowEndIdx = find(refTimes <= windowEndTime + 1e-9, 1, 'last');
    if isempty(windowEndIdx)
        windowEndIdx = numSamples;
    end
    if windowEndIdx < releaseIdx
        windowEndIdx = releaseIdx;
    end

    windowStartIdx = lastReleaseIdx + 1;
    schedule(end+1,1) = struct('window_start_idx', windowStartIdx, ...
                               'window_end_idx', windowEndIdx, ...
                               'release_idx', releaseIdx); %#ok<AGROW>

    lastReleaseIdx = releaseIdx;
    if lastReleaseIdx >= numSamples
        break;
    end
end

if isempty(schedule) || schedule(end).release_idx < numSamples
    if isempty(schedule)
        windowStartIdx = 1;
    else
        windowStartIdx = schedule(end).release_idx + 1;
    end
    schedule(end+1,1) = struct('window_start_idx', windowStartIdx, ...
                               'window_end_idx', numSamples, ...
                               'release_idx', numSamples);
end

end

function [vLong, omega] = local_compute_base_velocities(basePose, armTimes)
if isempty(basePose) || numel(armTimes) < 2
    vLong = zeros(size(armTimes));
    omega = zeros(size(armTimes));
    return;
end

x = basePose(:, 1);
y = basePose(:, 2);
theta = basePose(:, 3);

vx = gradient(x, armTimes);
vy = gradient(y, armTimes);
thetaUnwrapped = unwrap(theta);
omegaRaw = gradient(thetaUnwrapped, armTimes);

if numel(vx) >= 2
    vx([1 end]) = vx([2 end-1]);
    vy([1 end]) = vy([2 end-1]);
    omegaRaw([1 end]) = omegaRaw([2 end-1]);
end

vLong = vx .* cos(theta) + vy .* sin(theta);
omega = omegaRaw;

vLong = vLong(:);
omega = omega(:);
end
