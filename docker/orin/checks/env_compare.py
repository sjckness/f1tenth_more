#!/usr/bin/env python3
"""
The native stack's environment next to the container's, as one table.

Runs on: anywhere (stdlib; reads two env_capture.sh files). Plan P8
"environment table vs native". Every variable that differs, or exists on one
side only, is listed; the ones that decide how the stack talks and runs are
marked and compared with what the design expects (output/phase5d_analysis.md
section 6.5, design.md):

  ROS_DISCOVERY_SERVER          127.0.0.1:11811 on both
  ROS_DOMAIN_ID                 unset or 0 on both
  RMW_IMPLEMENTATION            rmw_fastrtps_cpp in the container (Jazzy
                                jazzy.sh); unset natively (Humble default)
  ROS_SUPER_CLIENT              TRUE natively (ctl.env); unset in the container
                                (the probes and the logger set it themselves)
  FASTRTPS_DEFAULT_PROFILES_FILE  the shipped fastdds_profile.xml in the
                                container; unset natively (no 5 s lease there)
  F1T_PROCESS_BACKEND           proc in the container
  LD_LIBRARY_PATH / PYTHONPATH / AMENT_PREFIX_PATH / PATH: distro paths, listed
                                but expected to differ

  env_compare.py NATIVE.env CONTAINER.env [--markdown OUT.md] [--strict]

--strict: exit 1 when a marked variable deviates from the expectation above.
"""
import argparse
import sys

EXPECT = {
    'ROS_DISCOVERY_SERVER': ('127.0.0.1:11811', '127.0.0.1:11811'),
    'ROS_DOMAIN_ID': ({None, '0'}, {None, '0'}),
    'RMW_IMPLEMENTATION': ({None, 'rmw_fastrtps_cpp'}, 'rmw_fastrtps_cpp'),
    'ROS_SUPER_CLIENT': ({'TRUE', 'true', '1'}, None),
    'FASTRTPS_DEFAULT_PROFILES_FILE': (None, '*fastdds_profile.xml'),
    'F1T_PROCESS_BACKEND': ({None, 'systemd', 'auto'}, 'proc'),
}
DISTRO_PATHS = {'LD_LIBRARY_PATH', 'PYTHONPATH', 'AMENT_PREFIX_PATH', 'PATH', 'CMAKE_PREFIX_PATH',
                'COLCON_PREFIX_PATH', 'ROS_DISTRO', 'ROS_PYTHON_VERSION', 'ROS_VERSION',
                'PKG_CONFIG_PATH', 'ROS_PACKAGE_PATH'}


def read_env(path):
    out = {}
    with open(path) as f:
        for line in f:
            line = line.rstrip('\n')
            if not line or line.startswith('#') or '=' not in line:
                continue
            k, v = line.split('=', 1)
            out[k] = v
    return out


def matches(value, want):
    if isinstance(want, set):
        return value in want
    if want is None:
        return value is None
    if want.startswith('*'):
        return value is not None and value.endswith(want[1:])
    return value == want


def compare(native, container):
    """(rows, deviations): rows = (name, native, container, mark)."""
    rows, deviations = [], []
    for k in sorted(set(native) | set(container)):
        n, c = native.get(k), container.get(k)
        mark = ''
        if k in EXPECT:
            ok = matches(n, EXPECT[k][0]) and matches(c, EXPECT[k][1])
            mark = 'expected' if ok else 'DEVIATES'
            if not ok:
                deviations.append(k)
        elif k in DISTRO_PATHS:
            mark = 'distro path'
        if n != c or k in EXPECT:
            rows.append((k, n, c, mark))
    for k in EXPECT:
        if k not in native and k not in container:
            ok = matches(None, EXPECT[k][0]) and matches(None, EXPECT[k][1])
            rows.append((k, None, None, 'expected' if ok else 'DEVIATES'))
            if not ok:
                deviations.append(k)
    return rows, deviations


def fmt(v, width=70):
    if v is None:
        return '(unset)'
    return v if len(v) <= width else v[:width - 3] + '...'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('  ROS_DISCOVERY_SERVER'):])
    ap.add_argument('native')
    ap.add_argument('container')
    ap.add_argument('--markdown')
    ap.add_argument('--strict', action='store_true')
    a = ap.parse_args(argv)
    rows, deviations = compare(read_env(a.native), read_env(a.container))
    lines = ['| variable | native | container | |', '|---|---|---|---|']
    lines += ['| %s | `%s` | `%s` | %s |' % (k, fmt(n), fmt(c), m) for k, n, c, m in rows]
    text = '\n'.join(lines)
    print(text)
    print('\n%d variables differ; marked deviations: %s' % (len(rows), deviations or 'none'))
    if a.markdown:
        with open(a.markdown, 'w') as f:
            f.write(text + '\n')
    return 1 if (a.strict and deviations) else 0


if __name__ == '__main__':
    sys.exit(main())
