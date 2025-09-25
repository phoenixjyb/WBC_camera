function entriesOut = normalize_collision_avoidance_entries(entriesIn)
%NORMALIZE_COLLISION_AVOIDANCE_ENTRIES Ensure collision config has required fields.
if exist('constraintCollisionAvoidance', 'class') ~= 8 && exist('constraintCollisionAvoidance', 'file') ~= 2
    if ~isempty(entriesIn)
        warning('wbc:CollisionConstraintUnavailable', ...
            'constraintCollisionAvoidance is unavailable; ignoring CollisionAvoidance options.');
    end
    entriesOut = {};
    return;
end

if isempty(entriesIn)
    entriesOut = {};
    return;
end

if ~iscell(entriesIn)
    entriesIn = {entriesIn};
end

entriesOut = entriesIn;
for idx = 1:numel(entriesIn)
    entry = entriesIn{idx};
    if isa(entry, 'constraintCollisionAvoidance')
        entry.SelfCollisions = true;
        if isempty(entry.Environment)
            entry.Environment = {};
        end
        entriesOut{idx} = entry;
    elseif isstruct(entry)
        if ~isfield(entry, 'Environment') || isempty(entry.Environment)
            entry.Environment = {};
        end
        if ~isfield(entry, 'SelfCollisions') || isempty(entry.SelfCollisions)
            entry.SelfCollisions = true;
        else
            entry.SelfCollisions = logical(entry.SelfCollisions);
        end
        if ~isfield(entry, 'NumSamples') || isempty(entry.NumSamples)
            entry.NumSamples = 8;
        end
        if ~isfield(entry, 'Weights') || isempty(entry.Weights)
            entry.Weights = 1.0;
        end
        if ~isfield(entry, 'CollisionPairs')
            entry.CollisionPairs = {};
        end
        entriesOut{idx} = entry;
    else
        error('wbc:InvalidCollisionEntry', ...
            'Unsupported CollisionAvoidance entry of type %s.', class(entry));
    end
end
end
