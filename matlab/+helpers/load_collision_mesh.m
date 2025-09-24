function collisionObj = load_collision_mesh(meshPath, varargin)
%LOAD_COLLISION_MESH Load an STL mesh and return a collisionMesh object.
%   collisionObj = helpers.load_collision_mesh(meshPath) reads the STL located
%   at meshPath. Optional name-value arguments:
%     'Scale' - Uniform scale factor (default 1.0)
%     'Pose'  - 4x4 homogeneous transform (default eye(4))
%
if isstring(meshPath)
    if numel(meshPath) ~= 1
        error('helpers:load_collision_mesh:InvalidPath', 'meshPath must be a scalar string or char vector.');
    end
    meshPath = char(meshPath);
elseif ~ischar(meshPath) || size(meshPath,1) ~= 1
    error('helpers:load_collision_mesh:InvalidPath', 'meshPath must be a scalar string or char vector.');
end

parser = inputParser;
parser.FunctionName = 'helpers.load_collision_mesh';
addParameter(parser, 'Scale', 1.0, @(x) validateattributes(x, {'numeric'}, {'scalar','positive','finite'}));
addParameter(parser, 'Pose', eye(4), @(x) validateattributes(x, {'numeric'}, {'size',[4 4]}));

if numel(varargin) == 1 && isstruct(varargin{1})
    optsStruct = varargin{1};
    fields = fieldnames(optsStruct);
    values = struct2cell(optsStruct);
    nvPairs = reshape([fields.'; values.'], 1, []);
else
    nvPairs = varargin;
end

parser.parse(nvPairs{:});
opts = parser.Results;

if exist(meshPath, 'file') ~= 2
    error('helpers:load_collision_mesh:FileNotFound', 'Mesh file %s not found.', meshPath);
end

triData = stlread(meshPath);
vertices = triData.Points * opts.Scale;
faces = triData.ConnectivityList;

collisionObj = collisionMesh(vertices, faces);
collisionObj.Pose = opts.Pose;
end
