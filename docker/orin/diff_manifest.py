#!/usr/bin/env python3
"""
Diff an Orin Jazzy image's resolved versions against the manifest.

usage: diff_manifest.py jazzy_versions.manifest thor_ros_jazzy_dpkg.txt image_versions.txt

Prints Markdown (image_report.sh writes it to
output/orin_container/logs/image_diff_<tag>.md): every pinned component with
the manifest value, the image value and = / differs / n/a. Versions are
compared on the upstream part (no epoch, no Debian revision); the ros-jazzy-*
list is also compared on the full Debian version. Sections: [ros], [python],
[zed], [submodules], [snapshots], [rust], [gateway_venv] (the pins are the
requirements file named there, next to the manifest).
"""

import os
import re
import sys


def sections(path, comment='#'):
    """Return {section: [non-empty, non-comment lines]} of an INI-like file."""
    out, cur = {}, None
    with open(path, errors='replace') as f:
        for raw in f:
            line = raw.rstrip('\n')
            m = re.match(r'^\[(\w+)\]', line)
            if m:
                cur = out.setdefault(m.group(1), [])
                continue
            if cur is not None and line.strip() and not line.lstrip().startswith(comment):
                cur.append(line)
    return out


def upstream(v):
    """Return the upstream part of a Debian or pip version."""
    m = re.match(r'^(?:\d+:)?([^-+~]+)', v or '')
    return m.group(1) if m else v


def norm(name):
    """Normalise a Python distribution name (PEP 503)."""
    return re.sub(r'[-_.]+', '-', name).lower()


def mval(line):
    """Return (key, value) of a manifest line, comment removed."""
    parts = line.split('#')[0].split()
    return (parts[0], ' '.join(parts[1:])) if parts else (None, None)


def kv(lines):
    """Return {first word: second word} of 'key value' lines."""
    return dict(line.split()[:2] for line in lines if len(line.split()) >= 2)


def pins(lines):
    """Return {normalised name: version} of 'name==version' lines."""
    out = {}
    for line in lines:
        line = line.split('#')[0].strip()
        if '==' in line:
            k, v = line.split('==', 1)
            out[norm(k)] = v.strip()
    return out


def status(got, want):
    """Return '=' when got equals want on the upstream version, else 'differs'."""
    return '=' if got and upstream(got) == want else 'differs'


def ros_rows(manifest, img_ros, img_apt):
    """Rows of the [ros] section."""
    rows = []
    special = {'nav2_*', 'apt_sync', 'ros_distro', 'rmw_implementation', 'everything_else',
               'opencv_system'}
    for line in manifest.get('ros', []):
        key, want = mval(line)
        if key is None or not want:
            continue
        if key in special:
            if key == 'nav2_*':
                got = sorted({upstream(v) for p, v in img_ros.items()
                              if p.startswith('ros-jazzy-nav2-')})
                rows.append(('ros', key, want, ', '.join(got) or 'not installed',
                             '=' if got == [want] else 'differs'))
            elif key == 'opencv_system':
                got = [upstream(v) for p, v in img_apt.items() if p.startswith('libopencv-core')]
                rows.append(('ros', key, want, ', '.join(got) or 'not installed',
                             '=' if got and all(g == want for g in got) else 'differs'))
            continue
        got = img_ros.get('ros-jazzy-' + key.replace('_', '-'))
        rows.append(('ros', key, want, upstream(got) if got else 'not installed',
                     status(got, want)))
    return rows


def python_rows(manifest, img_pip):
    """Rows of the [python] section."""
    rows = []
    pip_names = {'pyyaml': 'pyyaml', 'colcon_core': 'colcon-core', 'jetson_stats': 'jetson-stats',
                 'opencv_python': 'opencv-python'}
    for line in manifest.get('python', []):
        key, want = mval(line)
        if key is None:
            continue
        got = img_pip.get(norm(pip_names.get(key, key)))
        st = 'n/a (not in the Thor reference)' if want.startswith('NONE') else status(got, want)
        rows.append(('python', key, want, got or 'not installed', st))
    return rows


def zed_rows(manifest, image):
    """Rows of the [zed] section."""
    zed = {line.split()[0]: line.split()[1] for line in image.get('zed', [])
           if line.startswith('ZED_SDK_')}
    zed_v = '.'.join(zed.get(f'ZED_SDK_{k}_VERSION', '?') for k in ('MAJOR', 'MINOR', 'PATCH'))
    rows = []
    for line in manifest.get('zed', []):
        key, want = mval(line)
        if key == 'zed_sdk':
            rows.append(('zed', key, want, zed_v, '=' if zed_v == want else 'differs'))
    return rows


def submodule_rows(manifest, image):
    """Rows of the [submodules] section (the image's SOURCES)."""
    srcs = {}
    for line in image.get('workspace', []):
        p = line.split()
        if p and p[0] == 'submodule' and len(p) >= 3:
            srcs[p[1].rstrip('/').split('/')[-1]] = p[2]
    rows = []
    for line in manifest.get('submodules', []):
        key, want = mval(line)
        if key is None or not re.match(r'^[0-9a-f]{40}$', want or ''):
            continue
        got = srcs.get(key)
        rows.append(('submodule', key, want[:12], (got or 'missing')[:12],
                     '=' if got == want else 'differs'))
    return rows


def snapshot_rows(manifest, image):
    """Rows of the [snapshots] section."""
    img = kv(image.get('snapshots', []))
    rows = []
    for line in manifest.get('snapshots', []):
        key, want = mval(line)
        if key is None:
            continue
        got = img.get(key)
        rows.append(('snapshot', key, want, got or 'unknown', '=' if got == want else 'differs'))
    return rows


def rust_rows(manifest, image, img_ros):
    """Rows of the [rust] section."""
    img = kv(image.get('rust', []))
    rows = []
    for line in manifest.get('rust', []):
        key, want = mval(line)
        if key is None:
            continue
        if key == 'rosidl_generator_rs':
            got = img_ros.get('ros-jazzy-rosidl-generator-rs')
            rows.append(('rust', key, want, upstream(got) if got else 'not installed',
                         status(got, want)))
        elif key == 'cargo_lock':
            rows.append(('rust', key, want, (img.get('cargo_lock_sha256') or 'missing')[:16],
                         'info (sha256; the build checked it equal to the commit)'))
        else:
            got = img.get(key)
            if img.get('rustc') == 'none':
                rows.append(('rust', key, want, 'none (RUST=skip image)', 'n/a'))
            else:
                rows.append(('rust', key, want, got or 'missing',
                             '=' if got == want else 'differs'))
    return rows


def venv_rows(manifest, image, manifest_dir):
    """Rows of [gateway_venv]: every pin of the requirements file, then extras."""
    rows = []
    img = pins(image.get('gateway_venv', []))
    for line in manifest.get('gateway_venv', []):
        key, want = mval(line)
        if key != 'requirements':
            continue
        path = os.path.join(manifest_dir, want)
        try:
            with open(path) as f:
                req = pins(f.readlines())
        except OSError as exc:
            rows.append(('gateway_venv', want, 'unreadable', str(exc), 'differs'))
            continue
        for name, ver in sorted(req.items()):
            got = img.get(name)
            rows.append(('gateway_venv', name, ver, got or 'not installed',
                         '=' if got == ver else 'differs'))
        for name in sorted(set(img) - set(req)):
            rows.append(('gateway_venv', name, 'not pinned', img[name], 'differs (extra)'))
    return rows


def render(rows, thor, img_ros):
    """Return the Markdown report."""
    out = ['| Area | Component | Manifest | Image | Status |', '|---|---|---|---|---|']
    out += ['| ' + ' | '.join(str(x) for x in r) + ' |' for r in rows]
    common = sorted(set(thor) & set(img_ros))
    diff = [(p, thor[p], img_ros[p]) for p in common if thor[p] != img_ros[p]]
    only_thor = sorted(set(thor) - set(img_ros))
    only_img = sorted(set(img_ros) - set(thor))
    out.append('')
    out.append("**ros-jazzy-\\* against the Thor's dpkg list (full Debian version):** "
               f'image {len(img_ros)}, Thor {len(thor)}, common {len(common)}, '
               f'version differs {len(diff)}, image-only {len(only_img)}, '
               f'Thor-only {len(only_thor)}.')
    out += [f'- DIFFERS `{p}`: Thor `{a}`, image `{b}`' for p, a, b in diff]
    if only_img:
        out.append('- image-only: ' + ', '.join(f'`{p[10:]}`' for p in only_img))
    out.append('- Thor-only (not installed in the image): '
               + ', '.join(f'`{p[10:]}`' for p in only_thor))
    n_diff = sum(1 for r in rows if str(r[4]).startswith('differs'))
    out.append('')
    out.append(f'**Pinned components:** {len(rows)}, differing {n_diff}.')
    return '\n'.join(out)


def main(argv):
    """Print the report for the three files named in argv."""
    if len(argv) != 4:
        print(__doc__.strip().splitlines()[2], file=sys.stderr)
        return 2
    manifest = sections(argv[1])
    with open(argv[2]) as f:
        thor = dict(line.split()[:2] for line in f if len(line.split()) >= 2)
    image = sections(argv[3], comment='\x00')
    img_ros = kv(image.get('ros_dpkg', []))
    img_apt = kv(image.get('apt', []))
    img_pip = {}
    for line in image.get('pip', []):
        if '==' in line:
            k, v = line.split('==', 1)
            img_pip[norm(k)] = v.strip()
        elif ' @ ' in line:
            img_pip[norm(line.split(' @ ')[0])] = 'direct:' + line.split(' @ ')[1]
    rows = (ros_rows(manifest, img_ros, img_apt) + python_rows(manifest, img_pip)
            + zed_rows(manifest, image) + submodule_rows(manifest, image)
            + snapshot_rows(manifest, image) + rust_rows(manifest, image, img_ros)
            + venv_rows(manifest, image, os.path.dirname(os.path.abspath(argv[1]))))
    print(render(rows, thor, img_ros))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
