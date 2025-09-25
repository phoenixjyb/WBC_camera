function debug_diff_sync_inputs()
%DEBUG_DIFF_SYNC_INPUTS Compare synchronization inputs between legacy and streaming runs.
legacyPath = fullfile('outputs','comparison','debug_legacy.mat');
streamPath = fullfile('outputs','comparison','debug_stream.mat');
if ~isfile(legacyPath) || ~isfile(streamPath)
    error('Comparison artifacts not found. Run debug_compare_streaming first.');
end

load(legacyPath, 'diagLegacy');
load(streamPath, 'diagStream');

legacyInputs = diagLegacy.syncInputs;
streamInputs = diagStream.syncInputs;

fields = fieldnames(legacyInputs);
fprintf('Comparing %d input fields\n', numel(fields));
diffReport = struct();
for k = 1:numel(fields)
    name = fields{k};
    valLegacy = legacyInputs.(name);
    if ~isfield(streamInputs, name)
        fprintf(' - %s missing in streaming inputs\n', name);
        continue;
    end
    valStream = streamInputs.(name);
    if isnumeric(valLegacy) && isnumeric(valStream)
        sizeMatch = isequal(size(valLegacy), size(valStream));
        if ~sizeMatch
            fprintf(' - %s size mismatch legacy %s vs stream %s\n', name, mat2str(size(valLegacy)), mat2str(size(valStream)));
        end
        maxDiff = max(abs(valLegacy(:) - valStream(:)));
        diffReport.(name) = maxDiff;
        fprintf(' - %s max abs diff %.6g\n', name, maxDiff);
    else
        isEqual = isequaln(valLegacy, valStream);
        fprintf(' - %s equality %d\n', name, isEqual);
    end
end

if isfield(diagStream, 'streamArgs') && ~isempty(diagStream.streamArgs)
    fprintf('\nStreaming args contain additional fields: %s\n', strjoin(fieldnames(diagStream.streamArgs), ', '));
end

fprintf('\nScale factors: legacy %.6f, stream %.6f\n', diagLegacy.sync.scaleFactor, diagStream.sync.scaleFactor);
fprintf('Arm samples: legacy %d, stream %d\n', numel(diagLegacy.sync.armTimes), numel(diagStream.sync.armTimes));

end
