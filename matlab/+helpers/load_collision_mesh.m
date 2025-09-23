function collisionObj = load_collision_mesh(meshPath, options)
%LOAD_COLLISION_MESH Load an STL mesh and return a collisionMesh object.
%   collisionObj = helpers.load_collision_mesh(meshPath, options) reads the
%   STL located at meshPath and applies optional scaling and pose offsets.
%
%   Name-value fields in options (all optional):
%     Scale - Uniform scale factor (default 1.0)
%     Pose  - 4x4 homogeneous transform placing the mesh in the base frame
%
arguments
    meshPath {mustBeTextScalar}
    options.Scale (1,1) double {mustBePositive} = 1.0
    options.Pose (4,4) double = eye(4)
end

if exist(meshPath, 'file') ~= 2
    error('helpers:load_collision_mesh:FileNotFound', 'Mesh file %s not found.', meshPath);
end

triData = stlread(meshPath);
vertices = triData.Points * options.Scale;
faces = triData.ConnectivityList;

collisionObj = collisionMesh(vertices, faces);
collisionObj.Pose = options.Pose;
end
