"""
Hydra scene graph snapshots into the mission run folder (log_scene_graph, default off).

While a run is recording, the latest /hydra/backend/dsg graph is saved to
<run_dir>/scene_graph/dsg_t<seconds>.sparkdsg every scene_graph_period_sec, and to
dsg_final.sparkdsg when the mission ends. If tools/hydra_cost is running on this host (its
`latest` samples.csv written in the last 10 s), its CSV and summary are copied into
<run_dir>/scene_graph/hydra_cost/ at the end as well.

hydra_msgs and spark_dsg come from the Hydra workspace (~/hydra_ws, docs/hydra_hil.md) and are
imported only when the feature is on: the logger never needs them otherwise. Without the
spark_dsg bindings the raw graph bytes are saved instead (dsg_*.bin, readable with
spark_dsg.DynamicSceneGraph.from_binary or hydra_dsg_viewer --file).
Open a saved graph: scripts/hydra/open_graph.sh <run_dir or file>.
"""

import os
import shutil
import time


class SceneGraphSaver:
    """Owned by MissionLoggerNode; all methods run on the node's executor thread."""

    def __init__(self, node, topic, period_sec, hydra_cost_dir):
        self._node = node
        self._log = node.get_logger()
        self._period = max(1.0, float(period_sec))
        self._cost_dir = os.path.expanduser(hydra_cost_dir) if hydra_cost_dir else ''
        self._latest = None          # bytes of the newest graph
        self._dir = None             # <run_dir>/scene_graph while a run is open
        self._t0 = 0.0
        self._saved = 0
        self._dsg = None
        try:
            from hydra_msgs.msg import DsgUpdate
        except ImportError as exc:
            self.enabled = False
            self._log.error(
                f'log_scene_graph: hydra_msgs not importable ({exc}) -- no scene graphs will '
                'be saved. Source ~/hydra_ws (scripts/hydra/hydra_env.sh) before the logger.')
            return
        try:
            import spark_dsg
            self._dsg = spark_dsg
        except ImportError:
            self._log.warn('log_scene_graph: spark_dsg bindings not importable -- saving raw '
                           'graph bytes (dsg_*.bin) instead of .sparkdsg.')
        self.enabled = True
        self._sub = node.create_subscription(DsgUpdate, topic, self._on_graph, 1)
        self._timer = node.create_timer(self._period, self._on_timer)
        self._log.info(f'log_scene_graph: {topic} -> <run>/scene_graph every '
                       f'{self._period:.0f} s and at mission end')

    def _on_graph(self, msg):
        if msg.full_update:
            self._latest = msg.layer_contents.tobytes()

    def _on_timer(self):
        if self._dir is not None:
            self._save(f'dsg_t{time.monotonic() - self._t0:05.0f}')

    def start(self, run_dir):
        if not self.enabled:
            return
        self._dir = os.path.join(run_dir, 'scene_graph')
        self._t0 = time.monotonic()
        self._saved = 0
        try:
            os.makedirs(self._dir, exist_ok=True)
        except OSError as exc:
            self._log.error(f'log_scene_graph: cannot create {self._dir}: {exc}')
            self._dir = None

    def finish(self):
        """Mission end: the final graph and the hydra_cost CSV. Never raises."""
        if not self.enabled or self._dir is None:
            return
        try:
            self._save('dsg_final')
            self._copy_cost()
            self._log.info(f'log_scene_graph: {self._saved} graphs in {self._dir}')
        except Exception as exc:  # noqa: BLE001 - a logger must not kill anything
            self._log.error(f'log_scene_graph: finishing failed: {exc}')
        finally:
            self._dir = None

    def _save(self, name):
        data = self._latest
        if data is None:
            self._log.warn(f'log_scene_graph: no graph received yet, {name} skipped',
                           throttle_duration_sec=30.0)
            return
        ext = '.sparkdsg' if self._dsg is not None else '.bin'
        final = os.path.join(self._dir, name + ext)
        tmp = os.path.join(self._dir, '.tmp_' + name + ext)   # keep the extension: it is the format
        try:
            if self._dsg is not None:
                self._dsg.DynamicSceneGraph.from_binary(data).save(tmp, False)
            else:
                with open(tmp, 'wb') as f:
                    f.write(data)
            os.replace(tmp, final)
            self._saved += 1
        except Exception as exc:  # noqa: BLE001
            self._log.error(f'log_scene_graph: saving {final} failed: {exc}')

    def _copy_cost(self):
        if not self._cost_dir:
            return
        csv_path = os.path.join(self._cost_dir, 'samples.csv')
        try:
            fresh = time.time() - os.path.getmtime(csv_path) < 10.0
        except OSError:
            return
        if not fresh:
            return
        dest = os.path.join(self._dir, 'hydra_cost')
        os.makedirs(dest, exist_ok=True)
        for fname in ('samples.csv', 'summary.md'):
            src = os.path.join(self._cost_dir, fname)
            if os.path.isfile(src):
                shutil.copy2(src, os.path.join(dest, fname))
