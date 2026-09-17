#!/usr/bin/env python3
"""Check that the RUNNING stack uses the 2026-09-18 floor values -- read live, not from files.

Run after every restart, before any mission. Nothing moves: no mission is
started, and the one mission it loads is rejected by design.

1. mpc_corr's live parameters, through its own /mpc_corr/get_parameters
   service (the ros2 CLI's `param get` is blind under this stack's Discovery
   Server, so this is an rclpy client):
       max_forward_speed_mps   0.5
       min_moving_speed_mps    0.4
       object_stop_distance_m  0.14
       object_reach_tol_m      0.10
   plus object_a_dec, which must NOT be declared -- it is, only if an mpc_corr
   from before the floor change is still running.

2. The behaviour tree's LOADER, through /mission/load_mission. The executor
   reads its object_* values through a bootstrap node it destroys at startup,
   so they are not readable live. What IS checkable live is the code that
   uses them: a go_to_object mission at 0.3 m/s must be REJECTED with
   "below the operating floor" (new loader), and go_to_person_floor.json must
   load (it is installed). Loading replaces any loaded mission; nothing starts.

Exit code 0 when everything matches. Usage:
    source install/setup.bash && python3 scripts/check_floor_values.py
"""

import json
import os
import sys
import tempfile

EXPECTED_MPC = {
    'max_forward_speed_mps': 0.5,
    'min_moving_speed_mps': 0.4,
    'object_stop_distance_m': 0.14,
    'object_reach_tol_m': 0.10,
}
MUST_NOT_EXIST = ('object_a_dec',)
TIMEOUT_S = 15.0


def _below_floor_mission():
    return {
        'mission_id': 'check_floor_values_reject',
        'schema_version': '5.0',
        'moves': [{
            'id': 'go', 'terminal': True, 'timeout_sec': 10, 'on_timeout': 'abort',
            'go_to_object': {'target_class': 'person', 'speed': 0.3, 'acquire_timeout_sec': 1.0},
            'stop_condition': {'type': 'object_reached'}}],
    }


def main():
    """Query mpc_corr's parameters and probe the loader; print PASS/FAIL per check."""
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters, ListParameters

    from f1tenth_messages.srv import LoadMission

    rclpy.init()
    node = rclpy.create_node('check_floor_values')
    failures = 0

    def call(client, request):
        if not client.wait_for_service(timeout_sec=TIMEOUT_S):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=TIMEOUT_S)
        return future.result()

    def report(ok, text):
        nonlocal failures
        failures += 0 if ok else 1
        print(('PASS  ' if ok else 'FAIL  ') + text, flush=True)

    # ---- 1. mpc_corr, live
    # Only DECLARED names: in Humble one undeclared name in the request empties
    # the whole reply. Absence is checked through list_parameters below.
    names = list(EXPECTED_MPC)
    got = call(node.create_client(GetParameters, '/mpc_corr/get_parameters'),
               GetParameters.Request(names=names))
    if got is None:
        report(False, 'mpc_corr: /mpc_corr/get_parameters did not answer '
                      f'within {TIMEOUT_S:.0f} s (is navigation up?)')
    else:
        values = dict(zip(names, got.values))
        for name, want in EXPECTED_MPC.items():
            v = values.get(name)
            if v is None:
                report(False, f'mpc_corr {name} is not declared (an old mpc_corr?)')
                continue
            have = (v.double_value if v.type == ParameterType.PARAMETER_DOUBLE
                    else float(v.integer_value) if v.type == ParameterType.PARAMETER_INTEGER
                    else None)
            report(have is not None and abs(have - want) < 1e-9,
                   f'mpc_corr {name} = {have} (want {want})')
        listed = call(node.create_client(ListParameters, '/mpc_corr/list_parameters'),
                      ListParameters.Request(depth=0))
        declared = set(listed.result.names) if listed is not None else set()
        for name in MUST_NOT_EXIST:
            report(listed is not None and name not in declared,
                   f'mpc_corr does not declare {name} (an old mpc_corr would)')

    # ---- 2. the loader, live
    loader = node.create_client(LoadMission, '/mission/load_mission')
    with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as handle:
        json.dump(_below_floor_mission(), handle)
        reject_path = handle.name
    try:
        res = call(loader, LoadMission.Request(path=reject_path))
        report(res is not None and not res.success and 'below the operating floor' in res.message,
               'loader rejects go_to_object speed 0.3: '
               + ('no answer' if res is None else repr(res.message)[:160]))
    finally:
        os.unlink(reject_path)
    floor_path = os.path.join(get_package_share_directory('f1tenth_behavior'),
                              'missions', 'go_to_person_floor.json')
    res = call(loader, LoadMission.Request(path=floor_path))
    report(res is not None and res.success,
           'loader loads go_to_person_floor.json: '
           + ('no answer' if res is None else repr(res.message)[:160]))

    node.destroy_node()
    rclpy.shutdown()
    print('ALL PASS' if failures == 0 else f'{failures} FAILED', flush=True)
    return 0 if failures == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
