function obstacles = chassis_obstacles()
%CHASSIS_OBSTACLES Define world-frame obstacles for chassis planning.
%   Returns an array of structs with fields:
%     type   - 'circle' (disc), 'rectangle', etc.
%     center - [x y] center for circular obstacles [m]
%     radius - radius for circular obstacles [m]
%     height - optional metadata for visualization [m]
%
%   Current configuration adds a 20 cm diameter, 5 cm tall disc at (-1, -1, 0).

obstacles = struct('type', 'circle', ...
                   'center', [-1.0, -1.0], ...
                   'radius', 0.10, ...
                   'height', 0.05);
end

