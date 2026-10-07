function [runs, cmds] = read_index(db_root)
%READ_INDEX  index/runs.csv (+ index/cmds.csv) -> the db.runs and db.cmds tables.
runs_csv = fullfile(db_root, 'index', 'runs.csv');
if ~isfile(runs_csv)
    error('f1db:noIndex', 'No %s. Run export_matlab first.', runs_csv);
end

% Identifiers and labels are text; every other column is a number, and an
% empty cell is NaN (the exporter never writes 0 for "unknown").
text_cols = {'mission', 'test_id', 'date', 'time', 'plan_id', 'plan_hash', ...
    'auto_outcome', 'corridor_schema', 'object_corridor_mode', 'object_shape', ...
    'backfilled', 'notes', 'archive_run_id', 'bag_status', 'mat_file'};
opts = detectImportOptions(runs_csv, 'Delimiter', ',', 'TextType', 'string', ...
    'VariableNamingRule', 'preserve');
text_cols = intersect(opts.VariableNames, text_cols, 'stable');
opts = setvartype(opts, text_cols, 'string');
opts = setvartype(opts, setdiff(opts.VariableNames, text_cols, 'stable'), 'double');
runs = readtable(runs_csv, opts);

runs.run_id = runs.test_id;
runs.run_short = regexp(runs.test_id, '^P\d{3}-R\d{3}', 'match', 'once');
runs = movevars(runs, {'run_id', 'run_short'}, 'Before', 1);
keep_text = {'run_id', 'run_short', 'test_id', 'notes', 'plan_hash', 'mat_file'};
runs = to_categorical(runs, keep_text);

cmds = read_cmds(fullfile(db_root, 'index', 'cmds.csv'));
if ~isempty(cmds) && height(cmds) > 0
    runs = join_cmds(runs, cmds);
end
end

% --------------------------------------------------------------------------

function cmds = read_cmds(path)
% cmds.csv: the "first test campaing" tab of the F1tenth testing sheet,
% exported by hand. Columns (header text as in the sheet):
%   Cmd #, Command text, file, obstacle, LLM OK, Translator OK, Rep, Success
cmds = table();
if ~isfile(path)
    return
end
raw = readtable(path, 'TextType', 'string', 'VariableNamingRule', 'preserve');
map = containers.Map( ...
    {'cmd', 'cmdno', 'commandtext', 'file', 'obstacle', 'llmok', 'translatorok', 'rep', 'success'}, ...
    {'cmd_no', 'cmd_no', 'command_text', 'file', 'obstacle', 'llm_ok', 'translator_ok', 'rep', 'success_sheet'});
names = raw.Properties.VariableNames;
for k = 1:numel(names)
    key = lower(regexprep(names{k}, '[^A-Za-z0-9]', ''));
    if isKey(map, key)
        names{k} = map(key);
    else
        names{k} = matlab.lang.makeValidName(names{k});
    end
end
raw.Properties.VariableNames = matlab.lang.makeUniqueStrings(names);
cmds = raw;

% The sheet writes Cmd #, command text and obstacle once per block of
% repetitions: fill Cmd # down the whole sheet, the other two within a block.
if ismember('cmd_no', cmds.Properties.VariableNames)
    cmds.cmd_no = fill_down(cmds.cmd_no);
    blocks = block_ids(cmds.cmd_no);
    for col = {'command_text', 'obstacle'}
        if ismember(col{1}, cmds.Properties.VariableNames)
            v = cmds.(col{1});
            for b = unique(blocks)'
                rows = blocks == b;
                v(rows) = fill_down(v(rows));
            end
            cmds.(col{1}) = v;
        end
    end
end
if ismember('file', cmds.Properties.VariableNames)
    cmds.run_short = regexp(string(cmds.file), 'P\d{3}-R\d{3}', 'match', 'once');
end
cmds = to_categorical(cmds, {'file', 'run_short', 'command_text'});
end

function v = fill_down(v)
if isstring(v)
    v(strlength(v) == 0) = missing;
end
v = fillmissing(v, 'previous');
end

function ids = block_ids(cmd_no)
% One id per run of equal consecutive Cmd # values.
s = string(cmd_no);
s(ismissing(s)) = "";
ids = cumsum([true; s(2:end) ~= s(1:end-1)]);
end

function runs = join_cmds(runs, cmds)
if ~ismember('run_short', cmds.Properties.VariableNames)
    warning('f1db:cmdsNoFile', 'cmds.csv has no usable "file" column; not joined.');
    return
end
cmds = cmds(~ismissing(cmds.run_short), :);
[~, first] = unique(cmds.run_short, 'stable');
if numel(first) < height(cmds)
    dup = setdiff(1:height(cmds), first);
    warning('f1db:cmdsDuplicate', 'cmds.csv lists %s more than once; the first row is used.', ...
        strjoin(unique(cmds.run_short(dup)), ', '));
    cmds = cmds(first, :);
end
right = removevars(cmds, intersect({'file'}, cmds.Properties.VariableNames));
clash = setdiff(intersect(right.Properties.VariableNames, runs.Properties.VariableNames), {'run_short'});
for k = 1:numel(clash)
    right = renamevars(right, clash{k}, ['cmd_' clash{k}]);
end
order = runs.run_id;
runs = outerjoin(runs, right, 'Keys', 'run_short', 'MergeKeys', true, 'Type', 'left');
[~, back] = ismember(order, runs.run_id);  % outerjoin sorts by key; restore order
runs = runs(back, :);
end

function t = to_categorical(t, keep)
for k = 1:width(t)
    name = t.Properties.VariableNames{k};
    if isstring(t.(name)) && ~ismember(name, keep)
        t.(name) = categorical(t.(name));
    end
end
end
