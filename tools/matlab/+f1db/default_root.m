function root = default_root()
%F1DB.DEFAULT_ROOT  Database root: $F1TENTH_MATLAB_DATA, else ~/matlab_data.
%   Mirrors the exporter's get_db_root() (minus its --db-root flag and the
%   logger YAML key, which MATLAB does not read).
root = getenv('F1TENTH_MATLAB_DATA');
if isempty(root)
    if ispc
        home = getenv('USERPROFILE');
    else
        home = getenv('HOME');
    end
    root = fullfile(home, 'matlab_data');
end
end
