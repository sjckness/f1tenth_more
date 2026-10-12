function out = to_timetable(s)
%TO_TIMETABLE  One exported topic struct -> timetable with RowTimes = seconds(t_rel).
%   Every field with one row per message becomes a variable (vectors, n x k
%   matrices such as scan ranges, n x 1 cells such as polygons or horizons).
%   Per-topic constants (source, time_source, topic, info) go to
%   tt.Properties.UserData. A struct without a t_rel vector, or the map (one
%   grid), is returned unchanged.
if ~isstruct(s) || ~isfield(s, 't_rel') || isfield(s, 'grid')
    out = s;
    return
end
n = numel(s.t_rel);
T = table();
meta = struct();
names = fieldnames(s);
for k = 1:numel(names)
    name = names{k};
    v = s.(name);
    if strcmp(name, 't_rel')
        continue
    end
    if ischar(v) || isstruct(v) || (isstring(v) && isscalar(v)) || size(v, 1) ~= n
        meta.(name) = v;
    else
        T.(name) = v;
    end
end
if width(T) == 0
    T = array2table(zeros(n, 0));
end
out = table2timetable(T, 'RowTimes', seconds(double(s.t_rel(:))));
out.Properties.DimensionNames{1} = 't_rel';
out.Properties.UserData = meta;
end
