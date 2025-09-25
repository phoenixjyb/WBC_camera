function collisionEntries = configure_arm_collision_avoidance(robot, baseDir)
%CONFIGURE_ARM_COLLISION_AVOIDANCE Build default collision avoidance setup.
%   Mirrors the legacy inlined helper: if constraintCollisionAvoidance is
%   available, load the chassis mesh from mobile_arm_whole_body/meshes and
%   enable self-collision checking.

arguments
    robot (1,1) rigidBodyTree %#ok<INUSA>
    baseDir (1,:) char
end

collisionEntries = {};

if exist('constraintCollisionAvoidance', 'class') ~= 8 && exist('constraintCollisionAvoidance', 'file') ~= 2
    warning('rt_whole_body_controller:CollisionConstraintUnavailable', ...
        'constraintCollisionAvoidance is unavailable; skipping collision avoidance.');
    return;
end

envObjects = {};
chassisMeshPath = fullfile(baseDir, '..', 'mobile_arm_whole_body', 'meshes', 'cr_no_V.stl');
if exist(chassisMeshPath, 'file') == 2
    try
        chassisMesh = helpers.load_collision_mesh(chassisMeshPath, struct('Scale', 1e-3));
        envObjects{end+1} = chassisMesh; %#ok<AGROW>
    catch ME
        warning('rt_whole_body_controller:CollisionMeshLoad', ...
            'Failed to load collision mesh %s (%s). Continuing with self-collision only.', chassisMeshPath, ME.message);
    end
else
    warning('rt_whole_body_controller:CollisionMeshMissing', ...
        'Collision mesh %s not found. Continuing with self-collision only.', chassisMeshPath);
end

collisionEntry = struct();
collisionEntry.Environment = envObjects;
collisionEntry.SelfCollisions = true;
collisionEntry.NumSamples = 8;
collisionEntry.Weights = 1.0;

collisionEntries = {collisionEntry};
end
