#!/usr/bin/env python3
"""Compare what detection.launch.py / costmap.launch.py start, a base commit vs the worktree.

Every Node each launch file would start (OpaqueFunctions executed), with its
package, executable, name, prefix, condition, resolved parameters and resolved
additional_env, for a set of launch-argument overrides. Prints the differences;
exit 1 when a default-path case differs. R3 (2026-10-23): the *_cpu_threads
OpaqueFunctions must start exactly what the base started.

Usage (workspace install sourced): python3 tools/r3_launch_diff.py [--base cb5e3847]
"""
import os
import subprocess
import sys

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

WS = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
BASE = 'cb5e3847'


def load(path, rev):
    real = f'{WS}/{path}'
    src = (subprocess.check_output(['git', '-C', WS, 'show', f'{rev}:{path}'], text=True)
           if rev else open(real).read())
    g = {'__file__': real, '__name__': f'launch_{rev or "new"}'}
    exec(compile(src, real, 'exec'), g)
    return g['generate_launch_description']()


def nodes(ld, overrides):
    ctx = LaunchContext()
    ctx.launch_configurations.update(overrides)
    found = []
    for e in ld.entities:
        if isinstance(e, DeclareLaunchArgument):
            e.visit(ctx)
        elif isinstance(e, OpaqueFunction):
            found.extend(e.execute(ctx))
        elif isinstance(e, Node):
            found.append(e)
    out = {}
    for n in found:
        params = evaluate_parameters(ctx, n._Node__parameters or [])
        flat = {}
        for p in params:
            flat.update(p if isinstance(p, dict) else {'file': str(p)})
        env = {perform_substitutions(ctx, k): perform_substitutions(ctx, v)
               for k, v in (n.additional_env or [])}
        prefix = n.process_description.prefix if hasattr(n, 'process_description') else None
        prefix = perform_substitutions(ctx, prefix) if prefix else ''
        cond = n.condition.evaluate(ctx) if n.condition else True
        out[n.node_executable] = {'package': n.node_package, 'prefix': prefix,
                                  'condition': cond, 'params': flat, 'env': env}
    return out


def main():
    global BASE
    if len(sys.argv) > 2 and sys.argv[1] == '--base':
        BASE = sys.argv[2]
    cases = {
        'src/f1tenth_perception/launch/detection.launch.py': [
            {}, {'yolo_model': 'yolo26s.engine'}, {'yolo_cpu_threads': '0'}],
        'src/f1tenth_costmap/launch/costmap.launch.py': [{}, {'enable_slam': 'false'}],
    }
    bad = 0
    for path, overrides_list in cases.items():
        for ov in overrides_list:
            old = nodes(load(path, BASE), ov)
            new = nodes(load(path, None), ov)
            same = old == new
            bad += not same
            print(f'{path} {ov or "defaults"}: {len(old)} nodes old, {len(new)} new, '
                  f'{"IDENTICAL" if same else "DIFFERENT"}')
            if not same:
                for k in sorted(set(old) | set(new)):
                    if old.get(k) != new.get(k):
                        o, n = old.get(k, {}), new.get(k, {})
                        for f in sorted(set(o) | set(n)):
                            if o.get(f) != n.get(f):
                                print(f'   {k}.{f}: {o.get(f)!r} -> {n.get(f)!r}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
