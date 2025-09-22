%RUN_WHOLE_BODY_DEMO Convenience wrapper for rt_whole_body_controller.
%   RUN_WHOLE_BODY_DEMO() executes the whole-body controller using the
%   `1_pull_world.json` trajectory, generates plots, metrics, and the
%   animation MP4 in `matlab/outputs/`.
%
%   The script simply populates the override variables that
%   `rt_whole_body_controller` already knows how to consume. Tweak the
%   assignments below if you want to point at a different trajectory or
%   adjust animation settings, then re-run.

% Trajectory/source overrides --------------------------------------------------
traj_source = 'file';
traj_file = '1_pull_world.json';
traj_duration = 12;              % seconds to retime over
traj_scale = 1.0;

% Animation / visualization controls ------------------------------------------
enable_animation = true;
animation_video_file = '1_pull_world_animation_new.mp4';
animation_playback_speed = 2;     % faster-than-real rendering

% Kick off the controller ------------------------------------------------------
rt_whole_body_controller;

% Optional: retrieve results from base workspace
if evalin('base', 'exist(''rt_results'', ''var'')')
    rt_results = evalin('base', 'rt_results'); %#ok<NASGU>
end
if evalin('base', 'exist(''rt_ikInfos'', ''var'')')
    rt_ikInfos = evalin('base', 'rt_ikInfos'); %#ok<NASGU>
end
