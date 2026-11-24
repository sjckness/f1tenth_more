import re, csv, sys, collections
log, out = sys.argv[1], sys.argv[2]
runs = {}
pose_mode_order = []
for ln in open(log, encoding='utf-8', errors='replace'):
    m = re.search(r'\[agent [\d:]+\] (p(\d+)_(corridor_mpc|corridor_rrt|p2p_geometric)_r(\d)): (.*)', ln)
    if not m:
        continue
    rid, pose, mode, rep, rest = m.group(1), int(m.group(2)), m.group(3), int(m.group(4)), m.group(5).strip()
    r = runs.setdefault(rid, {'run_id': rid, 'pose': pose, 'mode': mode, 'rep': rep,
                              'status': '', 'mission': '', 'straight': '', 'door': '', 'reset_fail': ''})
    if 'FAIL' in rest:
        if 'standstill' in rest: r['reset_fail'] = 'nan_speed'
        elif 'slam' in rest: r['reset_fail'] = 'slam_no_pose'
        elif '"speed": false' in rest: r['reset_fail'] = 'speed_check_false'
        elif 'wheel_odom' in rest: r['reset_fail'] = 'no_sim_reset_service'
    for st in ('reset_failed', 'stack_not_ready', 'isaac_failed', 'start_failed', 'load_failed'):
        if rest.startswith(st) or f'marked {st}' in rest:
            r['status'] = st
    if rest.startswith('ended'):
        r['status'] = 'flew'
        mm = re.search(r'mission (\w+)', rest)
        r['mission'] = mm.group(1) if mm else ''
        s = re.search(r'move_0_straight_3m: ([^;]*)', rest)
        r['straight'] = s.group(1).strip() if s else ''
        d = re.search(r'move_1_enter_left_door: ([^;]*)', rest)
        r['door'] = d.group(1).strip() if d else ''
rows = sorted(runs.values(), key=lambda r: (r['pose'], r['mode'], r['rep']))
for r in rows:
    if not r['status']:
        r['status'] = 'never_ran'
with open(out, 'w', newline='', encoding='utf-8') as f:
    w = csv.DictWriter(f, fieldnames=['run_id','pose','mode','rep','status','reset_fail','mission','straight','door'])
    w.writeheader(); w.writerows(rows)
print('rows:', len(rows))
print('status:', dict(collections.Counter(r['status'] for r in rows)))
flew = [r for r in rows if r['status']=='flew']
print('flew:', len(flew))
print('straight:', dict(collections.Counter(r['straight'] for r in flew)))
print('door:', dict(collections.Counter(r['door'] for r in flew)))
print('COMPLETE:', sum(r['mission']=='COMPLETE' for r in flew))
print('lost per pose:', dict(sorted(collections.Counter(r['pose'] for r in rows if r['status']!='flew').items())))
