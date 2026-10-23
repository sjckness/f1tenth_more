#!/usr/bin/env python3
"""Resolve everything a launch would start, WITHOUT starting it, as normalised JSON.

Evidence tool for the supervisor's sim mode (output/jazzy_port/phase3.md): run it on the
same workspace before and after a launch change and diff the JSON. It walks a launch
description the way `ros2 launch` does (declared defaults + k:=v, conditions, includes,
groups, OpaqueFunctions, timers, event-handler action lists), executes only the actions
that change the launch context (arguments, launch configurations, global parameters,
remaps, environment), and for every Node / LifecycleNode / ComposableNodeContainer /
ComposableNode / ExecuteProcess records what would run: package, executable, node name,
namespace, the full argument list with every generated --params-file inlined as its
parsed content (temporary file names removed), prefix (taskset ...), remappings and the
effective use_sim_time. Nothing is spawned.

  launch_snapshot.py launch <pkg> <file> [k:=v ...]
      one launch file, as `ros2 launch <pkg> <file> k:=v` would run it
  launch_snapshot.py supervisor [k:=v ...]
      supervisor_bringup.launch.py with k:=v, then the component_supervisor_node it
      starts is constructed in-process with exactly those parameters (child process
      start, the stale-pgid sweep and the singleton lock are stubbed out), and every
      `ros2 launch` command it would run for its auto-started components is resolved
      the same way. The output contains the supervisor's commands per component and,
      for each command, its resolved processes.
"""
import json
import os
import re
import sys
import tempfile

import yaml

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from ament_index_python.packages import get_package_share_directory

_SKIP = ('LogInfo', 'EmitEvent', 'Shutdown', 'UnregisterEventHandler')
_TMP = re.compile(r'/tmp/launch_params_[A-Za-z0-9_]+')


def _perf(ctx, x):
    if x is None:
        return None
    if isinstance(x, str):
        return x
    return perform_substitutions(ctx, normalize_to_list_of_substitutions(x))


def _inline_params(cmd):
    """Replace generated --params-file paths by their parsed content (stable across runs)."""
    out = []
    i = 0
    while i < len(cmd):
        c = cmd[i]
        if c == '--params-file' and i + 1 < len(cmd):
            path = cmd[i + 1]
            try:
                with open(path) as f:
                    content = yaml.load(f, Loader=yaml.UnsafeLoader)
            except OSError:
                content = '<unreadable>'
            label = 'generated' if _TMP.match(path) else path
            out += ['--params-file', {'file': label, 'content': content}]
            i += 2
            continue
        out.append(c)
        i += 1
    return out


def _use_sim_time(cmd, node_name):
    """Effective use_sim_time per rcl: the last value on the command line wins."""
    val = None
    short = (node_name or '').split('/')[-1]
    for i, c in enumerate(cmd):
        if c == '-p' and isinstance(cmd[i + 1], str) and cmd[i + 1].startswith('use_sim_time:='):
            val = cmd[i + 1].split(':=', 1)[1]
        elif c == '--params-file' and isinstance(cmd[i + 1], dict):
            for key, body in (cmd[i + 1]['content'] or {}).items():
                k = str(key).lstrip('/')
                if key in ('/**', '**') or k == short or k.endswith('/' + short):
                    params = (body or {}).get('ros__parameters', {}) or {}
                    if 'use_sim_time' in params:
                        val = str(params['use_sim_time'])
    return val


class Walker:

    def __init__(self, ctx):
        self.ctx = ctx
        self.records = []

    def walk(self, entities, where, on_event=False):
        for e in entities:
            if e is None:
                continue
            if isinstance(e, LaunchDescription):
                self.walk(e.entities, where, on_event)
                continue
            if isinstance(e, DeclareLaunchArgument):
                if e.name not in self.ctx.launch_configurations and e.default_value is not None:
                    self.ctx.launch_configurations[e.name] = _perf(self.ctx, e.default_value)
                continue
            cond = getattr(e, 'condition', None)
            if cond is not None and not cond.evaluate(self.ctx):
                continue
            if isinstance(e, Node):
                self._node(e, where, on_event)
                continue
            if isinstance(e, LoadComposableNodes):
                self._composables(e._LoadComposableNodes__composable_node_descriptions, where,
                                  on_event, container=_perf(self.ctx, e._LoadComposableNodes__final_target_container_name
                                                            if hasattr(e, '_LoadComposableNodes__final_target_container_name') else None))
                continue
            if isinstance(e, ExecuteProcess):
                cmd = [_perf(self.ctx, c) for c in e.cmd]
                self.records.append({'where': where, 'on_event': on_event, 'kind': 'process',
                                     'name': _perf(self.ctx, e.name) if e.name else None, 'cmd': cmd})
                continue
            if isinstance(e, TimerAction):
                self.walk(e.actions, where, on_event)
                continue
            if isinstance(e, RegisterEventHandler):
                for _desc, ents in e.describe_conditional_sub_entities():
                    self.walk(ents, where, True)
                continue
            if type(e).__name__ in _SKIP:
                continue
            if isinstance(e, IncludeLaunchDescription):
                src = e.launch_description_source
                out = e.execute(self.ctx) or []
                # the source knows its expanded path only once it has been loaded (execute)
                inner = os.path.basename(str(src.location))
                self.walk(out, inner, on_event)
                continue
            if isinstance(e, (OpaqueFunction, GroupAction)):
                self.walk(e.execute(self.ctx) or [], where, on_event)
                continue
            # context actions (SetLaunchConfiguration, SetParameter(s FromFile), SetRemap,
            # SetEnvironmentVariable, Push/Pop*, ...): executed into the context only
            out = e.execute(self.ctx)
            if out:
                self.walk(out, where, on_event)

    def _node(self, e, where, on_event):
        try:
            self._node_resolved(e, where, on_event)
        except Exception as exc:  # noqa: BLE001 - e.g. a hardware package not installed here
            self.records.append({
                'where': where, 'on_event': on_event, 'kind': type(e).__name__,
                'package': _perf(self.ctx, e.node_package),
                'executable': _perf(self.ctx, e.node_executable),
                'name': getattr(e, '_Node__node_name', None),
                # first clause only: the package search path in the message depends on the host
                'unresolved': f'{type(exc).__name__}: {str(exc).splitlines()[0].split(", searching")[0][:200]}'})

    def _node_resolved(self, e, where, on_event):
        e._perform_substitutions(self.ctx)
        name = getattr(e, '_Node__expanded_node_name', None)
        if name in (None, Node.UNSPECIFIED_NODE_NAME):
            name = None
        ros_args = {}
        if name is not None:
            ros_args['name'] = f'__node:={name}'
        if e.expanded_node_namespace not in (None, Node.UNSPECIFIED_NODE_NAMESPACE):
            ros_args['ns'] = f'__ns:={e.expanded_node_namespace}'
        self.ctx.extend_locals({'ros_specific_arguments': ros_args})
        cmd = _inline_params([_perf(self.ctx, x) for x in e.cmd])
        prefix = getattr(getattr(e, 'process_description', None), 'prefix', None)
        rec = {'where': where, 'on_event': on_event, 'kind': type(e).__name__,
               'package': _perf(self.ctx, e.node_package),
               'executable': _perf(self.ctx, e.node_executable),
               'name': name, 'namespace': e.expanded_node_namespace,
               'prefix': _perf(self.ctx, prefix) if prefix else None,
               'cmd': cmd[1:], 'use_sim_time': _use_sim_time(cmd, name)}
        if isinstance(e, ComposableNodeContainer):
            descs = e._ComposableNodeContainer__composable_node_descriptions or []
            rec['components'] = self._load_requests(descs)
        self.records.append(rec)

    def _load_requests(self, descs):
        from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
        comps = []
        for d in descs:
            req = get_composable_node_load_request(d, self.ctx)
            params = {p.name: _param_value(p.value) for p in req.parameters}
            comps.append({'package': req.package_name, 'plugin': req.plugin_name,
                          'name': req.node_name, 'namespace': req.node_namespace,
                          'remaps': list(req.remap_rules), 'parameters': params,
                          'use_sim_time': params.get('use_sim_time')})
        return comps

    def _composables(self, descs, where, on_event, container):
        self.records.append({'where': where, 'on_event': on_event, 'kind': 'LoadComposableNodes',
                             'container': container, 'components': self._load_requests(descs or [])})


def _param_value(v):
    from rcl_interfaces.msg import ParameterType as T
    return {T.PARAMETER_BOOL: v.bool_value, T.PARAMETER_INTEGER: v.integer_value,
            T.PARAMETER_DOUBLE: v.double_value, T.PARAMETER_STRING: v.string_value,
            T.PARAMETER_STRING_ARRAY: list(v.string_array_value),
            T.PARAMETER_DOUBLE_ARRAY: list(v.double_array_value),
            T.PARAMETER_INTEGER_ARRAY: list(v.integer_array_value),
            T.PARAMETER_BOOL_ARRAY: list(v.bool_array_value)}.get(v.type, None)


def snapshot_launch(pkg, launch_file, cli):
    ctx = LaunchContext()
    ctx.launch_configurations.update(cli)
    path = os.path.join(get_package_share_directory(pkg), 'launch', launch_file)
    ld = get_launch_description_from_any_launch_file(path)
    w = Walker(ctx)
    w.walk([ld], launch_file)
    return w.records


def _parse_cli(args):
    return dict(a.split(':=', 1) for a in args)


def _normalise_cmd(cmd, log_dir):
    return [c.replace(log_dir, '<log_dir>') if isinstance(c, str) else c for c in cmd]


def snapshot_supervisor(cli):
    top = snapshot_launch('f1tenth_bringup', 'supervisor_bringup.launch.py', cli)
    sup = [r for r in top if r.get('name') == 'component_supervisor_node']
    assert len(sup) == 1, top
    params = {}
    for i, c in enumerate(sup[0]['cmd']):
        if c == '--params-file' and isinstance(sup[0]['cmd'][i + 1], dict):
            for body in (sup[0]['cmd'][i + 1]['content'] or {}).values():
                params.update((body or {}).get('ros__parameters', {}) or {})
    log_dir = tempfile.mkdtemp(prefix='launch_snapshot_')
    params = dict(params, log_dir=log_dir)

    import rclpy
    from f1tenth_bringup import component_supervisor_node as csn

    started = []

    class _FakePopen:
        pid = 0

        def poll(self):
            return None

    def _start(proc):
        started.append(proc)
        proc.popen = _FakePopen()
    csn._ComponentProcess.start = _start
    csn.ComponentSupervisorNode._sweep_stale_pgids = lambda self: None
    csn.ComponentSupervisorNode._write_pgid_file = lambda self: None

    ros_args = ['--ros-args']
    for k, v in params.items():
        ros_args += ['-p', f'{k}:={json.dumps(v) if isinstance(v, bool) else v}']
    ros_args += ['-r', '__node:=component_supervisor_node']
    rclpy.init(args=['snapshot'] + ros_args)
    try:
        node = csn.ComponentSupervisorNode()
        components = {}
        for name in sorted(node._processes):
            components[name] = []
            for proc in node._processes[name]:
                cmd = proc.cmd
                assert cmd[:2] == ['ros2', 'launch'], cmd
                pkg, lf, args = cmd[2], cmd[3], _parse_cli(cmd[4:])
                try:
                    resolved = snapshot_launch(pkg, lf, args)
                except Exception as exc:  # noqa: BLE001 - launch file needs a package not installed here
                    resolved = {'error': f'{type(exc).__name__}: '
                                         f'{str(exc).splitlines()[0].split(", searching")[0][:200]}'}
                components[name].append({'cmd': _normalise_cmd(cmd, log_dir), 'resolved': resolved})
        registry = {k: v for k, v in sorted(node._registry.items())}
        node.destroy_node()
    finally:
        rclpy.shutdown()
    return {'supervisor_bringup': top, 'auto_started_components': components,
            'registry': registry}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('launch', 'supervisor'):
        print(__doc__)
        return 2
    if sys.argv[1] == 'launch':
        out = snapshot_launch(sys.argv[2], sys.argv[3], _parse_cli(sys.argv[4:]))
    else:
        out = snapshot_supervisor(_parse_cli(sys.argv[2:]))
    print(json.dumps(out, indent=1, sort_keys=True, default=str))
    return 0


if __name__ == '__main__':
    sys.exit(main())
