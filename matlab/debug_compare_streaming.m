function debug_compare_streaming()
%DEBUG_COMPARE_STREAMING Compare legacy vs streaming retiming timelines.
optsLegacy = struct('traj_source','file','traj_file','1_pull_world.json', ...
    'traj_duration',12,'traj_scale',1.0,'use_gik',false,'enable_visualization',false, ...
    'enable_animation',false,'enable_streaming',false);
[resLegacy,diagLegacy] = rt_whole_body_controller(optsLegacy);

optsStream = optsLegacy;
optsStream.enable_streaming = true;
optsStream.streaming_horizon_time = 2.0;
optsStream.streaming_step_time = 0.2;
optsStream.streaming_debug = true;
optsStream.streaming_mode = "rolling";
[resStream,diagStream] = rt_whole_body_controller(optsStream);

outDir = fullfile('outputs','comparison');
if ~isfolder(outDir), mkdir(outDir); end
save(fullfile(outDir,'debug_legacy.mat'),'resLegacy','diagLegacy');
save(fullfile(outDir,'debug_stream.mat'),'resStream','diagStream');

leg = resLegacy;
str = resStream;
summary = struct();
summary.arm_size_legacy = size(leg.armTrajectory);
summary.arm_size_stream = size(str.armTrajectory);
summary.base_size_legacy = size(leg.basePose);
summary.base_size_stream = size(str.basePose);
summary.scale_factor_legacy = leg.baseSyncScale;
summary.scale_factor_stream = str.baseSyncScale;
summary.base_speed_max = [max(leg.baseCmd(:,1)), max(str.baseCmd(:,1))];
summary.base_yaw_rate_max = [max(leg.baseCmd(:,2)), max(str.baseCmd(:,2))];
summary.ee_error_max = [leg.maxTrackingError, str.maxTrackingError];
summary.ee_error_mean = [leg.meanTrackingError, str.meanTrackingError];

save(fullfile(outDir,'debug_summary.mat'),'summary');
fid = fopen(fullfile(outDir,'debug_summary.txt'),'w');
fprintf(fid,'Arm size legacy: %d x %d\n',summary.arm_size_legacy);
fprintf(fid,'Arm size stream: %d x %d\n',summary.arm_size_stream);
fprintf(fid,'Base size legacy: %d x %d\n',summary.base_size_legacy);
fprintf(fid,'Base size stream: %d x %d\n',summary.base_size_stream);
fprintf(fid,'Scale factor legacy: %f\n',summary.scale_factor_legacy);
fprintf(fid,'Scale factor stream: %f\n',summary.scale_factor_stream);
fprintf(fid,'Base v max (legacy, stream): %f %f\n',summary.base_speed_max);
fprintf(fid,'Base omega max (legacy, stream): %f %f\n',summary.base_yaw_rate_max);
fprintf(fid,'EE max error (legacy, stream): %f %f\n',summary.ee_error_max);
fprintf(fid,'EE mean error (legacy, stream): %f %f\n',summary.ee_error_mean);
fclose(fid);

end
