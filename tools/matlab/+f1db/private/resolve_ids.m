function ids = resolve_ids(db, ids)
%RESOLVE_IDS  Run selectors -> full run IDs (string column), in the order given.
%   Accepts full IDs ("P003-R009-20260928T170138"), short IDs ("P003-R009"),
%   a string/char/cellstr of either, a table with a run_id column (e.g. a
%   filtered db.runs), or a logical row mask over db.runs.
if istable(ids)
    ids = ids.run_id;
elseif islogical(ids)
    ids = db.runs.run_id(ids);
end
ids = string(ids);
ids = ids(:);
full = string(db.runs.run_id);
short = string(db.runs.run_short);
out = strings(numel(ids), 1);
for i = 1:numel(ids)
    k = find(full == ids(i) | short == ids(i));
    if isempty(k)
        error('f1db:unknownRun', 'Unknown run %s (not in %s/index/runs.csv).', ids(i), db.root);
    elseif numel(k) > 1
        error('f1db:ambiguousRun', '%s matches %d runs; use the full ID.', ids(i), numel(k));
    end
    out(i) = full(k);
end
ids = out;
end
