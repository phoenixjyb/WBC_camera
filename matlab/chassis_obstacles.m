function obstacles = chassis_obstacles()
%CHASSIS_OBSTACLES Define world-frame obstacles for chassis planning.
%   Returns an array of structs with fields:
%     type    - 'circle' (disc), 'rectangle', etc.
%     center  - [x y] center for planar obstacles [m]
%     radius  - radius for circular obstacles [m]
%     height  - optional metadata for visualization [m]
%     origin  - optional 3-D [x y z] location for collision geometry [m]
%
%   Configuration attaches a 0.1 m radius safety disc centered in front of
%   the robot at [0, 0.12, 0] (ground level) to represent an immovable
%   object that both the chassis and manipulator must avoid.

obstacles = struct('type', 'circle', ...
                   'center', [0.0, 0.12], ...
                   'radius', 0.10, ...
                   'height', 0.20, ...
                   'origin', [0.0, 0.12, 0.0]);
end
