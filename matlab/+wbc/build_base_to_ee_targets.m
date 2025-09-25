function poseTforms = build_base_to_ee_targets(baseWaypoints, thetaRef, refPositionsWorld, refRPYWorld)
%BUILD_BASE_TO_EE_TARGETS Construct base-to-EE target transforms.
%   Returns a cell array of homogeneous transforms representing the desired
%   end-effector pose expressed in the chassis frame for each sample.

arguments
    baseWaypoints double
    thetaRef double
    refPositionsWorld double
    refRPYWorld double
end

numSamples = size(refPositionsWorld, 1);
poseTforms = cell(1, numSamples);

for i = 1:numSamples
    idxBase = min(i, size(baseWaypoints, 1));
    yaw = thetaRef(min(i, numel(thetaRef)));

    Tbase = trvec2tform([baseWaypoints(idxBase, :), 0]) * axang2tform([0 0 1 yaw]);
    Tworld = trvec2tform(refPositionsWorld(i, :)) * eul2tform(refRPYWorld(i, :), 'XYZ');

    poseTforms{i} = Tbase \ Tworld;
end
end
