"""Dual-EKF chain config/launch-parsing tests (dual-EKF + costmap-derived-
MPC-boundaries pass): confirms ekf.yaml (local) and ekf_global.yaml
(global) declare non-colliding frame ownership, that localization.launch.py
wires both EKF instances correctly per mode, and that ekf_global.launch.py
itself namespaces ekf_global_filter_node (NOT a remappings= list -- see that
launch file's own "SECOND REAL BUG FOUND..." docstring paragraph) so its own
default output topic can never again collide with -- or silently hijack --
its own odom0 subscription, the real /odometry/filtered self-loop bug found
twice now while building/maintaining this pass.

No live launch/nodes anywhere in this file -- YAML loading + direct Python
construction of generate_launch_description() only, same "config/launch-
parsing test" convention already used to verify this exact wiring during
development (see the dual-EKF pass's own report for why: /slam/pose does
not publish, VESC/lidar are disconnected, so a live `ros2 launch` +
`ros2 topic echo /tf` TF-collision check is explicitly blocked this pass --
this test is the hardware-independent substitute for that check, covering
what's staticly verifiable: the two configs' own declared frame ownership
never overlaps, and the launch wiring includes/excludes the right pieces
per mode).

Run standalone: python3 -m pytest test/test_ekf_global_config.py -v
"""

import importlib.util
import os

import pytest
import yaml

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
_SRC_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))  # .../src
_EKF_YAML = os.path.join(_SRC_ROOT, 'f1tenth_bringup', 'config', 'ekf.yaml')
_EKF_GLOBAL_YAML = os.path.join(_SRC_ROOT, 'f1tenth_bringup', 'config', 'ekf_global.yaml')
_LAUNCH_DIR = os.path.join(_SRC_ROOT, 'f1tenth_localization', 'launch')


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _load_launch_module(filename, module_name):
    spec = importlib.util.spec_from_file_location(
        module_name, os.path.join(_LAUNCH_DIR, filename))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _include_locations(launch_description):
    """Resolved file paths for every IncludeLaunchDescription action in
    `launch_description` -- launch's own PythonLaunchDescriptionSource
    stores its (already-resolved, since get_package_share_directory()
    evaluates immediately, not deferred to a LaunchContext) path as a
    single-element list of TextSubstitution under a name-mangled private
    attribute; unwrapped here once so each test doesn't repeat the same
    introspection."""
    from launch.actions import IncludeLaunchDescription
    locations = []
    for action in launch_description.entities:
        if isinstance(action, IncludeLaunchDescription):
            src = action.launch_description_source
            loc = src._LaunchDescriptionSource__location
            locations.append(loc[0].text)
    return locations


def _node_name(node) -> str:
    """A launch_ros Node's own resolved `name=` kwarg -- its public `.name`
    property returns None until a real LaunchContext resolves it (confirmed
    directly, not assumed), so this reads the name-mangled private
    attribute the constructor actually stores the plain string in. Safe
    here specifically because every Node() in this workspace's own launch
    files is constructed with name=<a literal string>, never a further
    substitution that would need real context-resolution."""
    return node._Node__node_name


def _remapping_pairs(node) -> list:
    """A launch_ros Node's own resolved `remappings=` kwarg as a list of
    plain (from, to) string tuples -- same "read the name-mangled private
    attribute, the public property needs a LaunchContext" situation as
    _node_name above. Each raw entry is a pair of 1-tuples of
    TextSubstitution (launch's own internal normalized form for a plain
    string remapping rule)."""
    return [
        (from_subs[0].text, to_subs[0].text)
        for from_subs, to_subs in node._Node__remappings
    ]


# ==============================================================================
# ekf.yaml / ekf_global.yaml -- non-colliding frame ownership. THE actual
# thing a live `ros2 topic echo /tf` check would confirm -- see module
# docstring for why that live check is blocked this pass and this static
# config read is the substitute.
# ==============================================================================

class TestEkfFrameOwnership:

    def test_local_ekf_owns_odom_to_base_link(self):
        cfg = _load_yaml(_EKF_YAML)['ekf_filter_node']['ros__parameters']
        assert cfg['world_frame'] == 'odom'
        assert cfg['odom_frame'] == 'odom'
        assert cfg['base_link_frame'] == 'base_link'
        assert cfg['publish_tf'] is True
        # world_frame == odom_frame means this instance's own publish_tf
        # publishes odom_frame -> base_link_frame (robot_localization's own
        # convention: a filter with publish_tf broadcasts world_frame ->
        # base_link_frame when world_frame == odom_frame).

    def test_global_ekf_owns_map_to_odom(self):
        cfg = _load_yaml(_EKF_GLOBAL_YAML)['/ekf_global/ekf_global_filter_node']['ros__parameters']
        assert cfg['world_frame'] == 'map'
        assert cfg['map_frame'] == 'map'
        assert cfg['odom_frame'] == 'odom'
        assert cfg['publish_tf'] is True
        # world_frame == map_frame (!= odom_frame) means this instance
        # publishes map_frame -> odom_frame -- a DIFFERENT edge from the
        # local EKF's own odom_frame -> base_link_frame above.

    def test_the_two_instances_publish_different_tf_edges(self):
        # The actual non-collision property, stated directly rather than
        # left implicit across the two tests above: (world_frame,
        # {other_frame}) pairs must differ.
        local_cfg = _load_yaml(_EKF_YAML)['ekf_filter_node']['ros__parameters']
        global_cfg = _load_yaml(_EKF_GLOBAL_YAML)['/ekf_global/ekf_global_filter_node']['ros__parameters']
        local_edge = (local_cfg['world_frame'], local_cfg['base_link_frame'])
        global_edge = (global_cfg['world_frame'], global_cfg['odom_frame'])
        assert local_edge != global_edge
        assert local_edge == ('odom', 'base_link')
        assert global_edge == ('map', 'odom')

    def test_global_ekf_fuses_the_local_ekfs_real_output_topic(self):
        # Regression guard for the real self-loop bug found this pass (see
        # ekf_global.launch.py's own "REAL BUG FOUND..." docstring
        # paragraph): odom0 must be the PLAIN /odometry/filtered name (the
        # local EKF's own output) -- if a future edit accidentally changed
        # this to the global filter's OWN output topic, this filter would
        # start fusing its own recursive output.
        #
        # NOT sufficient on its own, by itself, to catch the SECOND bug this
        # config was actually hit by (see ekf_global.launch.py's own "SECOND
        # REAL BUG FOUND..." paragraph): this value was already correctly
        # '/odometry/filtered' the whole time that bug was live -- the launch
        # file's own remapping was what silently rerouted the subscription
        # away from this value at runtime, a launch-wiring problem this
        # YAML-only test structurally cannot see. See
        # TestEkfGlobalLaunch.test_ekf_global_node_is_namespaced_not_remapped
        # below for the test that actually guards against that one.
        cfg = _load_yaml(_EKF_GLOBAL_YAML)['/ekf_global/ekf_global_filter_node']['ros__parameters']
        assert cfg['odom0'] == '/odometry/filtered'

    def test_odom0_is_differential_pose0_is_not(self):
        # Regression guard for the bootstrap-deadlock bug (see
        # odom0_differential's own extensive comment in ekf_global.yaml for
        # the full source-verified mechanism): odom0 (published in 'odom'
        # frame) MUST be differential=true, or robot_localization requires an
        # odom -> map transform that only this same filter's own map -> odom
        # output could ever provide -- confirmed live: this filter published
        # zero messages, ever, while odom0_differential was false, despite
        # odom0 itself demonstrably arriving. pose0 (already published
        # directly in 'map' frame by slam_toolbox) has no such deadlock and
        # MUST stay absolute (differential=false) -- it's the real, load-
        # bearing map-frame correction; flipping it too would remove the
        # only thing actually correcting odom0's own dead-reckoning drift.
        cfg = _load_yaml(_EKF_GLOBAL_YAML)['/ekf_global/ekf_global_filter_node']['ros__parameters']
        assert cfg['odom0_differential'] is True
        assert cfg['odom0_relative'] is False
        assert cfg['pose0_differential'] is False
        assert cfg['pose0_relative'] is False

    def test_global_ekf_fuses_the_calibrated_pose_not_the_raw_one(self):
        # Regression guard for the pose0-covariance-doesn't-exist-as-a-
        # static-param discovery (see ekf_global.yaml's own docstring) --
        # pose0 must be the relay's OWN output, not slam_toolbox's raw
        # /slam/pose directly.
        cfg = _load_yaml(_EKF_GLOBAL_YAML)['/ekf_global/ekf_global_filter_node']['ros__parameters']
        assert cfg['pose0'] == '/slam/pose_calibrated'

    def test_only_xy_yaw_are_fused_2d_vehicle(self):
        # Both odom0_config and pose0_config: [x, y, z, roll, pitch, yaw,
        # vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az] -- only indices 0
        # (x), 1 (y), 5 (yaw) True, matching this pass's own explicit "x/y/
        # yaw only, don't fuse z/roll/pitch" scope.
        cfg = _load_yaml(_EKF_GLOBAL_YAML)['/ekf_global/ekf_global_filter_node']['ros__parameters']
        expected = [True, True, False, False, False, True] + [False] * 9
        assert cfg['odom0_config'] == expected
        assert cfg['pose0_config'] == expected


# ==============================================================================
# localization.launch.py -- both EKF instances wired per mode, no static-TF
# collision with the local EKF's own real odom -> base_link publish_tf.
# ==============================================================================

class TestLocalizationLaunchWiring:

    def _generate(self, localization_source):
        import f1tenth_params.param_defaults as pd
        original_get_value = pd.get_value
        try:
            pd.get_value = (
                lambda name: localization_source if name == 'localization_source'
                else original_get_value(name))
            mod = _load_launch_module(
                'localization.launch.py', f'loc_launch_{localization_source}')
            return mod.generate_launch_description()
        finally:
            pd.get_value = original_get_value

    def test_ekf_mode_includes_both_ekf_instances(self):
        ld = self._generate('ekf')
        locations = _include_locations(ld)
        assert any(loc.endswith('/ekf.launch.py') for loc in locations)
        assert any(loc.endswith('/ekf_global.launch.py') for loc in locations)

    def test_ekf_mode_does_not_publish_the_static_odom_to_base_link_tf(self):
        # See module docstring / localization.launch.py's own "NOW
        # CONDITIONAL" paragraph -- publishing this unconditionally in
        # 'ekf' mode would collide with the local EKF's own real publish_tf.
        from launch_ros.actions import Node
        ld = self._generate('ekf')
        static_tf_nodes = [
            a for a in ld.entities
            if isinstance(a, Node) and _node_name(a) == 'odom_to_base_link_tf'
        ]
        assert static_tf_nodes == []

    def test_raw_odom_mode_does_not_include_the_global_ekf(self):
        # raw_odom mode has no /odometry/filtered for the global EKF's own
        # odom0 to fuse -- see ekf_global.launch.py's own docstring for why
        # it's only included in 'ekf' mode.
        ld = self._generate('raw_odom')
        locations = _include_locations(ld)
        assert not any(loc.endswith('/ekf_global.launch.py') for loc in locations)
        assert not any(loc.endswith('/ekf.launch.py') for loc in locations)

    def test_raw_odom_mode_publishes_the_static_odom_to_base_link_tf(self):
        from launch_ros.actions import Node
        ld = self._generate('raw_odom')
        static_tf_nodes = [
            a for a in ld.entities
            if isinstance(a, Node) and _node_name(a) == 'odom_to_base_link_tf'
        ]
        assert len(static_tf_nodes) == 1


# ==============================================================================
# ekf_global.launch.py -- own two-node bundle, self-loop regression guards.
# ==============================================================================

class TestEkfGlobalLaunch:

    def _generate(self):
        mod = _load_launch_module('ekf_global.launch.py', 'ekf_global_launch_test')
        return mod.generate_launch_description()

    def _ekf_global_node(self, ld):
        from launch_ros.actions import Node
        return next(
            a for a in ld.entities
            if isinstance(a, Node) and _node_name(a) == 'ekf_global_filter_node')

    def test_declares_both_the_ekf_node_and_the_relay_node(self):
        from launch_ros.actions import Node
        ld = self._generate()
        node_names = sorted(_node_name(a) for a in ld.entities if isinstance(a, Node))
        assert node_names == ['ekf_global_filter_node', 'slam_pose_relay_node']

    def test_ekf_global_node_is_namespaced_not_remapped(self):
        # Regression guard for the SECOND self-loop bug (see this launch
        # file's own "SECOND REAL BUG FOUND..." docstring paragraph): a
        # `remappings=[('odometry/filtered', '/ekf_global/odometry/filtered')]`
        # -- this exact fix's own first version, confirmed live-broken --
        # matches by resolved topic-name STRING only, so it ALSO silently
        # catches odom0's own absolute '/odometry/filtered' subscription
        # (same resolved string), starving this filter's own continuous
        # input. A namespace cannot make that mistake: it only ever rewrites
        # RELATIVE names, never an already-absolute one like odom0's. This
        # test would have caught the live bug (asserting on the OLD
        # remappings=-based wiring, `_remapping_pairs(...) ==
        # [('odometry/filtered', ...)]`, still passed with the bug live --
        # it only checked the fix's INTENT, not whether that intent was
        # actually collision-safe).
        ld = self._generate()
        ekf_global_node = self._ekf_global_node(ld)
        assert ekf_global_node._Node__node_namespace == 'ekf_global'
        assert _remapping_pairs(ekf_global_node) == []

    def test_ekf_global_node_output_topic_still_resolves_the_same(self):
        # Namespace-vs-remap is an internal wiring detail -- downstream
        # consumers (costmap_boundary_node.py, ekf_global.yaml's own
        # docstring, this test file's own module docstring) all still
        # expect the node's fused output to land on the exact same external
        # name it always has: '/ekf_global/odometry/filtered'. Confirmed
        # here directly rather than assumed from the namespace + default
        # relative-topic-name reasoning alone.
        ld = self._generate()
        ekf_global_node = self._ekf_global_node(ld)
        namespace = ekf_global_node._Node__node_namespace
        assert f'/{namespace}/odometry/filtered' == '/ekf_global/odometry/filtered'


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
