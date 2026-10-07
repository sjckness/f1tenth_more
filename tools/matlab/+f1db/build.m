function idx = build(db_root)
%F1DB.BUILD  Rebuild index/index.mat (db.runs + db.cmds) for fast opening.
%   idx = f1db.build()          uses f1db.default_root()
%   idx = f1db.build(db_root)
%
%   Re-run after export_matlab or after replacing index/cmds.csv. Writes only
%   index/index.mat; the runs/*.mat files are the exporter's and are never
%   modified from MATLAB.
if nargin < 1 || isempty(db_root)
    db_root = f1db.default_root();
end
db_root = char(db_root);
[runs, cmds] = read_index(db_root); %#ok<ASGLU> saved below by name
idx = fullfile(db_root, 'index', 'index.mat');
save(idx, 'runs', 'cmds');
fprintf('f1db.build: %d runs, %d cmds rows -> %s\n', height(runs), height(cmds), idx);
end
