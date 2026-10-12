function R = run(db, id, topics)
%F1DB.RUN  Load one run as a struct of timetables.
%   R = f1db.run(db, "P004-R016")                       all topics
%   R = f1db.run(db, "P004-R016", {'kin', 'cmd', 'scan'})  only these
%
%   id is a full or short run ID. Only the requested variables are read from
%   runs/<run_id>.mat (load(file, topics{:})). Each topic becomes a timetable
%   with RowTimes = seconds(t_rel), t_rel measured from the drive start
%   (mission_started); per-topic constants are in tt.Properties.UserData.
%   R.meta is always loaded; R.map (if requested and present) stays a struct.
if nargin < 3
    topics = {};
end
run_id = resolve_ids(db, id);
if numel(run_id) ~= 1
    error('f1db:oneRun', 'f1db.run loads one run; use f1db.runs for several.');
end
file = fullfile(db.root, 'runs', char(run_id + ".mat"));
if ~isfile(file)
    error('f1db:noRunFile', 'No %s. Run export_matlab for this run.', file);
end
available = who('-file', file);
topics = cellstr(string(topics));
if isempty(topics)
    vars = available;
else
    missing = setdiff(topics(:), available(:));
    if ~isempty(missing)
        warning('f1db:missingTopic', '%s has no %s.', run_id, strjoin(missing, ', '));
    end
    vars = unique([intersect(topics(:), available(:), 'stable'); {'meta'}], 'stable');
end
S = load(file, vars{:});
R = struct('run_id', run_id, 'meta', S.meta);
names = setdiff(fieldnames(S), {'meta'}, 'stable');
for k = 1:numel(names)
    R.(names{k}) = to_timetable(S.(names{k}));
end
end
