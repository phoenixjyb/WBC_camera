function baseline = run_capture_baseline(varargin)
%RUN_CAPTURE_BASELINE Generate controller baseline outputs for regression use.
%   BASELINE = RUN_CAPTURE_BASELINE(Name,Value,...) invokes
%   rt_whole_body_controller with visualization disabled, saves the produced
%   results/diagnostics into a dedicated baseline directory, and returns the
%   in-memory structures for immediate inspection.
%
%   Name-value pairs:
%     'OutputDir'   : directory for artifacts (default matlab/outputs/baseline_demo)
%     'TrajSource'  : trajectory source string (default "demo_arc")
%     'TrajFile'    : optional JSON file overriding TrajSource
%
%   The underlying controller already exports `<traj>_results.mat` and the
%   associated figures when running inside MATLAB. This helper adds a
%   timestamped manifest (baseline_manifest.json) describing the inputs used
%   so cross-language implementations can replay identical conditions.

p = inputParser;
p.addParameter('OutputDir', fullfile(pwd, 'outputs', 'baseline_demo'), @(v) ischar(v) || isstring(v));
p.addParameter('TrajSource', "demo_arc", @(v) ischar(v) || isstring(v));
p.addParameter('TrajFile', "", @(v) ischar(v) || isstring(v));
p.parse(varargin{:});

outputDir = string(p.Results.OutputDir);
trajSource = string(p.Results.TrajSource);
trajFile = string(p.Results.TrajFile);

if ~isfolder(outputDir)
    mkdir(outputDir);
end

controllerOpts = struct('traj_source', trajSource, ...
                        'traj_file', trajFile, ...
                        'enable_animation', false, ...
                        'enable_visualization', false, ...
                        'output_dir', outputDir);

[results, diagnostics] = rt_whole_body_controller(controllerOpts);

baseline = struct('results', results, 'diagnostics', diagnostics);

if ~coder.target('MATLAB')
    return;
end

manifest = struct();
manifest.generated_at = datestr(datetime('now', 'TimeZone', 'UTC'), 'yyyy-mm-ddTHH:MM:SSZ');
manifest.traj_source = trajSource;
manifest.traj_file = trajFile;
manifest.output_dir = outputDir;
rtControllerPath = which('rt_whole_body_controller');
manifest.controller_version = struct('plan_helpers', "matlab/+wbc", ...
    'rt_controller_checksum', local_file_checksum(rtControllerPath));

manifestPath = fullfile(outputDir, 'baseline_manifest.json');
try
    jsonText = jsonencode(manifest);
    fid = fopen(manifestPath, 'w');
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '%s', jsonText);
catch ME
    warning('run_capture_baseline:ManifestWrite', 'Failed to write manifest: %s', ME.message);
end

end

function checksum = local_file_checksum(filePath)
if nargin == 0 || isempty(filePath) || exist(filePath, 'file') ~= 2
    checksum = "missing";
    return;
end

if exist('hash', 'file') == 2
    checksum = string(hash(getFileText(filePath)));
    return;
end

fid = fopen(filePath, 'r');
cleanup = onCleanup(@() fclose(fid));
fileText = fread(fid, '*char')';
checksum = string(dec2hex(sum(double(fileText))));
end

function txt = getFileText(filePath)
fid = fopen(filePath, 'r');
cleanup = onCleanup(@() fclose(fid));
txt = fread(fid, '*char')';
end
