function db = open(db_root)
%F1DB.OPEN  Open the MATLAB run database.
%   db = f1db.open()          uses f1db.default_root() (~/matlab_data)
%   db = f1db.open(db_root)
%
%   db.root   the database folder
%   db.runs   one row per campaign run (index/runs.csv): run_id (full test
%             ID), run_short ("P003-R009"), the campaign_results.csv columns,
%             has_bag, bag_coverage_pct, ..., joined with the matching
%             index/cmds.csv row (command text, obstacle, LLM OK, ...)
%   db.cmds   index/cmds.csv as read (fill-down applied), or an empty table
%
%   Uses index/index.mat (written by f1db.build) when it is newer than
%   runs.csv and cmds.csv, else reads the CSVs directly.
if nargin < 1 || isempty(db_root)
    db_root = f1db.default_root();
end
db_root = char(db_root);
if ~isfolder(db_root)
    error('f1db:noDatabase', 'No database folder %s. Run export_matlab first.', db_root);
end
idx = fullfile(db_root, 'index', 'index.mat');
sources = {fullfile(db_root, 'index', 'runs.csv'), fullfile(db_root, 'index', 'cmds.csv')};
if isfile(idx) && ~any_newer(sources, idx)
    S = load(idx, 'runs', 'cmds');
else
    if isfile(idx)
        warning('f1db:staleIndex', ...
            'index.mat is older than runs.csv/cmds.csv; reading the CSVs. Run f1db.build to refresh it.');
    end
    [S.runs, S.cmds] = read_index(db_root);
end
db = struct('root', db_root, 'runs', S.runs, 'cmds', S.cmds);
end

function tf = any_newer(files, ref)
r = dir(ref);
tf = false;
for k = 1:numel(files)
    if isfile(files{k})
        d = dir(files{k});
        tf = tf || d.datenum > r.datenum;
    end
end
end
