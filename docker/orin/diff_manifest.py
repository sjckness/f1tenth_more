#!/usr/bin/env python3
"""Diff an Orin Jazzy image's resolved versions against the manifest.

usage: diff_manifest.py jazzy_versions.manifest thor_ros_jazzy_dpkg.txt image_versions.txt
Prints Markdown (output/phase5d_logs/d1_image_diff_<tag>.md): every pinned
component with the manifest value, the image value and = / differs / n-a.
Versions are compared on the upstream part (no epoch, no Debian revision);
the ros-jazzy-* list is also compared on the full Debian version.
"""
import re
import sys


def sections(path, comment='#'):
    out, cur = {}, None
    for raw in open(path, errors='replace'):
        line = raw.rstrip('\n')
        m = re.match(r'^\[(\w+)\]', line)
        if m:
            cur = out.setdefault(m.group(1), [])
            continue
        if cur is not None and line.strip() and not line.lstrip().startswith(comment):
            cur.append(line)
    return out


def upstream(v):
    m = re.match(r'^(?:\d+:)?([^-+~]+)', v or '')
    return m.group(1) if m else v


def norm(name):
    return re.sub(r'[-_.]+', '-', name).lower()


manifest = sections(sys.argv[1])
thor = dict(l.split()[:2] for l in open(sys.argv[2]) if len(l.split()) >= 2)
image = sections(sys.argv[3], comment='\x00')
img_ros = dict(l.split()[:2] for l in image.get('ros_dpkg', []) if len(l.split()) >= 2)
img_apt = dict(l.split()[:2] for l in image.get('apt', []) if len(l.split()) >= 2)
img_pip = {}
for l in image.get('pip', []):
    if '==' in l:
        k, v = l.split('==', 1)
        img_pip[norm(k)] = v.strip()
    elif ' @ ' in l:
        img_pip[norm(l.split(' @ ')[0])] = 'direct:' + l.split(' @ ')[1]


def mval(line):
    parts = line.split('#')[0].split()
    return (parts[0], ' '.join(parts[1:])) if parts else (None, None)


rows = []
# ---- ROS: named pins
special = {'nav2_*': None, 'apt_sync': None, 'ros_distro': None, 'rmw_implementation': None,
           'everything_else': None, 'opencv_system': None}
for line in manifest.get('ros', []):
    key, want = mval(line)
    if key is None or not want:
        continue
    if key in special:
        if key == 'nav2_*':
            got = sorted({upstream(v) for p, v in img_ros.items() if p.startswith('ros-jazzy-nav2-')})
            rows.append(('ros', key, want, ', '.join(got) or 'not installed',
                         '=' if got == [want] else 'differs'))
        elif key == 'opencv_system':
            got = [upstream(v) for p, v in img_apt.items() if p.startswith('libopencv-core')]
            rows.append(('ros', key, want, ', '.join(got) or 'not installed',
                         '=' if got and all(g == want for g in got) else 'differs'))
        continue
    pkg = 'ros-jazzy-' + key.replace('_', '-')
    got = img_ros.get(pkg)
    rows.append(('ros', key, want, upstream(got) if got else 'not installed',
                 '=' if got and upstream(got) == want else 'differs'))
# ---- Python
pip_names = {'pyyaml': 'pyyaml', 'colcon_core': 'colcon-core', 'jetson_stats': 'jetson-stats',
             'opencv_python': 'opencv-python'}
for line in manifest.get('python', []):
    key, want = mval(line)
    if key is None:
        continue
    got = img_pip.get(norm(pip_names.get(key, key)))
    if want.startswith('NONE'):
        status = 'n/a (not in the Thor reference)'
    else:
        status = '=' if got and upstream(got) == want else 'differs'
    rows.append(('python', key, want, got or 'not installed', status))
# ---- ZED
zed = {l.split()[0]: l.split()[1] for l in image.get('zed', []) if l.startswith('ZED_SDK_')}
zed_v = '.'.join(zed.get(f'ZED_SDK_{k}_VERSION', '?') for k in ('MAJOR', 'MINOR', 'PATCH'))
for line in manifest.get('zed', []):
    key, want = mval(line)
    if key == 'zed_sdk':
        rows.append(('zed', key, want, zed_v, '=' if zed_v == want else 'differs'))
# ---- submodules
srcs = {}
for l in image.get('workspace', []):
    p = l.split()
    if p and p[0] == 'submodule' and len(p) >= 3:
        srcs[p[1].rstrip('/').split('/')[-1]] = p[2]
for line in manifest.get('submodules', []):
    key, want = mval(line)
    if key is None or not re.match(r'^[0-9a-f]{40}$', want or ''):
        continue
    name = {'zed-ros2-interfaces': 'zed-ros2-interfaces', 'zed_ros2_wrapper': 'zed_ros2_wrapper'}.get(key, key)
    got = srcs.get(name)
    if key == 'husarion_gz_worlds':
        rows.append(('submodule', key, want[:12], 'not built (sim only)', 'n/a'))
        continue
    rows.append(('submodule', key, want[:12], (got or 'missing')[:12], '=' if got == want else 'differs'))

print('| Area | Component | Manifest (Thor) | Image | Status |')
print('|---|---|---|---|---|')
for r in rows:
    print('| ' + ' | '.join(str(x) for x in r) + ' |')

# ---- full ros-jazzy-* comparison
common = sorted(set(thor) & set(img_ros))
diff = [(p, thor[p], img_ros[p]) for p in common if thor[p] != img_ros[p]]
only_thor = sorted(set(thor) - set(img_ros))
only_img = sorted(set(img_ros) - set(thor))
print()
print(f'**ros-jazzy-\\* against the Thor\'s dpkg list (full Debian version):** '
      f'image {len(img_ros)}, Thor {len(thor)}, common {len(common)}, '
      f'version differs {len(diff)}, image-only {len(only_img)}, Thor-only {len(only_thor)}.')
for p, a, b in diff:
    print(f'- DIFFERS `{p}`: Thor `{a}`, image `{b}`')
if only_img:
    print('- image-only: ' + ', '.join(f'`{p[10:]}`' for p in only_img))
print('- Thor-only (not installed in the image): ' + ', '.join(f'`{p[10:]}`' for p in only_thor))
