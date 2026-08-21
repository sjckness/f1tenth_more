"""slam_toolbox_params.yaml config regression guard.

Config-only, no live launch/nodes -- same "hardware-independent substitute
for a live check" convention f1tenth_localization/test/test_ekf_global_config.py
already established (see that file's own module docstring), used here for
the same reason: VESC/lidar are disconnected this pass, so this is what's
staticly verifiable.

Run standalone: python3 -m pytest test/test_slam_toolbox_params.py -v
"""

import os

import yaml

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
_PARAMS_YAML = os.path.join(_THIS_DIR, '..', 'config', 'slam_toolbox_params.yaml')


def _load_params():
    with open(_PARAMS_YAML) as f:
        return yaml.safe_load(f)['slam_toolbox']['ros__parameters']


def test_scan_queue_size_is_explicitly_set_above_the_vendor_default():
    # Regression guard for the confirmed-live 100%-scan-drop bug this key
    # fixes (see this file's own header comment, next to scan_queue_size
    # itself, for the full live evidence): slam_toolbox's own vendor
    # default is 1 -- effectively zero slack against a ~40Hz scan stream,
    # confirmed live via `ros2 param get /slam_toolbox scan_queue_size`
    # returning 1 while this key was still unset here. This only guards
    # against silently losing the override again (e.g. a future edit
    # deleting the line, or a base-file merge dropping it) -- it does NOT
    # replace live re-verification that scans are actually being processed
    # once VESC/lidar are reconnected (queue depth alone can't be confirmed
    # sufficient from a config file).
    params = _load_params()
    assert 'scan_queue_size' in params
    assert params['scan_queue_size'] > 1


def test_transform_publish_period_still_zero():
    # Regression guard, unrelated to the scan_queue_size fix but living in
    # the same file: transform_publish_period must stay 0.0 -- this is the
    # actual mechanism keeping slam_toolbox from ever broadcasting map ->
    # odom itself (see this file's own header comment) -- map -> odom is
    # owned by ekf_global_filter_node (f1tenth_localization/launch/
    # ekf_global.launch.py). A future edit bumping scan_queue_size back up
    # should not casually "fix" this alongside it -- that would be a
    # separate, deliberate architecture change, not a queue-depth tweak.
    params = _load_params()
    assert params['transform_publish_period'] == 0.0
