%RUN_WHOLE_BODY_DEMO Convenience wrapper for rt_whole_body_controller.
%   RUN_WHOLE_BODY_DEMO() executes the whole-body controller using the
%   `1_pull_world.json` trajectory, generates plots, metrics, and the
%   animation MP4 in `matlab/outputs/`.
%
%   The script constructs an options struct consumed by
%   `rt_whole_body_controller`. Tweak the assignments below if you want to
%   point at a different trajectory or adjust animation settings, then
%   re-run.

% Trajectory/source overrides --------------------------------------------------
opts = struct();
opts.traj_source = 'file';
opts.traj_file = '1_pull_world.json';
opts.traj_duration = 12;              % seconds to retime over
opts.traj_scale = 1.0;

% Prefer constraint-based IK so collision avoidance is active
opts.use_gik = true;

% Animation / visualization controls ------------------------------------------
opts.enable_animation = true;
opts.enable_visualization = true;
opts.animation_video_file = '1_pull_world_animation_new.mp4';
opts.animation_playback_speed = 2;     % faster-than-real rendering

%% Stage-specific animation controls -------------------------------------------
stage_selection = "arm_ramp";
stage_animation_file = '1_pull_world_arm_ramp.mp4';

% Kick off the controller ------------------------------------------------------
[rt_results, rt_ikInfos] = rt_whole_body_controller(opts); %#ok<NASGU>

if strlength(stage_animation_file) > 0
    thisDir = fileparts(mfilename('fullpath'));
    urdfPath = fullfile(thisDir, '..', 'mobile_arm_whole_body', 'urdf', 'arm_on_car_center_rotZ_neg90.urdf');
    robot = importrobot(urdfPath, 'DataFormat', 'struct');
    robot.Gravity = [0 0 -9.81];
    animateArgs = {'VideoFile', stage_animation_file, 'VideoFrameRate', 30, ...
        'PlaybackSpeed', opts.animation_playback_speed, 'StageBoundaries', rt_results.stageBoundaries, ...
        'StageLabels', rt_results.stageLabels, 'StageSelection', stage_selection, ...
        'EndEffectorName', 'left_gripper_link'};
    baseInit = rt_results.baseInitialization;
    if isfield(baseInit, 'obstacles') && ~isempty(baseInit.obstacles)
        animateArgs = [animateArgs, {'Obstacles', baseInit.obstacles}]; %#ok<AGROW>
    end
    if isfield(baseInit, 'plannedPath') && ~isempty(baseInit.plannedPath)
        animateArgs = [animateArgs, {'TargetPath', baseInit.plannedPath}]; %#ok<AGROW>
    end
    helpers.animate_whole_body(robot, rt_results.armJointNames, rt_results.armTrajectory, rt_results.armTimes, ...
        rt_results.basePose, rt_results.baseTimes, rt_results.eeDesired, animateArgs{:});
end
