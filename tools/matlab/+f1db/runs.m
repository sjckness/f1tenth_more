function T = runs(db, ids, topics)
%F1DB.RUNS  Load many runs: one table row per run, one column per topic.
%   T = f1db.runs(db, ["P004-R014" "P004-R015"], {'kin'})
%   T = f1db.runs(db, db.runs(db.runs.mission == "M04_person", :), {'kin', 'cmd'})
%
%   ids: anything f1db.run accepts, a filtered db.runs table, or a logical
%   mask over db.runs. T.run_id is the full ID, T.meta a cell of meta
%   structs, and T.<topic>{i} the timetable of run i ([] when that run has
%   no such topic, e.g. a bag topic on a run without a bag).
if nargin < 3
    topics = {};
end
run_ids = resolve_ids(db, ids);
loaded = cell(numel(run_ids), 1);
names = {};
for i = 1:numel(run_ids)
    loaded{i} = f1db.run(db, run_ids(i), topics);
    names = union(names, setdiff(fieldnames(loaded{i}), {'run_id', 'meta'}, 'stable'), 'stable');
end
T = table(run_ids, 'VariableNames', {'run_id'});
T.meta = cellfun(@(r) r.meta, loaded, 'UniformOutput', false);
for k = 1:numel(names)
    T.(names{k}) = cellfun(@(r) field_or_empty(r, names{k}), loaded, 'UniformOutput', false);
end
end

function v = field_or_empty(s, name)
if isfield(s, name)
    v = s.(name);
else
    v = [];
end
end
