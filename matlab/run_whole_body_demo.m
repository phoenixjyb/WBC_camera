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

% Prefer constraint-based IK so collision avoidance is active
use_gik = true;

% Animation / visualization controls ------------------------------------------
enable_animation = true;
animation_video_file = '1_pull_world_animation_new.mp4';
animation_playback_speed = 2;     % faster-than-real rendering

%% Stage-specific animation controls -------------------------------------------
stage_selection = "arm_ramp";
stage_animation_file = '1_pull_world_arm_ramp.mp4';

% Kick off the controller ------------------------------------------------------
rt_whole_body_controller;

% Optional: retrieve results from base workspace
haveResults = evalin('base', 'exist(''rt_results'', ''var'')');
if haveResults
    rt_results = evalin('base', 'rt_results'); %#ok<NASGU>
end
if evalin('base', 'exist(''rt_ikInfos'', ''var'')')
    rt_ikInfos = evalin('base', 'rt_ikInfos'); %#ok<NASGU>
end

if haveResults && strlength(stage_animation_file) > 0
    thisDir = fileparts(mfilename('fullpath'));
    urdfPath = fullfile(thisDir, '..', 'mobile_arm_whole_body', 'urdf', 'arm_on_car_center_rotZ_neg90.urdf');
    robot = importrobot(urdfPath, 'DataFormat', 'struct');
    robot.Gravity = [0 0 -9.81];
    helpers.animate_whole_body(robot, rt_results.armJointNames, rt_results.armTrajectory, rt_results.armTimes, ...
        rt_results.basePose, rt_results.baseTimes, rt_results.eeDesired, ...
        'VideoFile', stage_animation_file, 'VideoFrameRate', 30, ...
        'PlaybackSpeed', animation_playback_speed, 'StageBoundaries', rt_results.stageBoundaries, ...
        'StageLabels', rt_results.stageLabels, 'StageSelection', stage_selection, ...
        'EndEffectorName', 'left_gripper_link');
end
