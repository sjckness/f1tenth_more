"""A small Hydra scene graph viewer: nodes, edges and labels, live or from a file.

Graph only, no mesh, so it works with hydra_profile:=jetson_min. Open3D when installed (one
window, labels in 3D), matplotlib otherwise. Layers are lifted by --z-offset so they don't
overlap: objects and mesh places at their height, places +1, rooms +2, buildings +3 offsets.

    ros2 run f1tenth_hydra hydra_dsg_viewer                       # live /hydra/backend/dsg
    ros2 run f1tenth_hydra hydra_dsg_viewer --file run/scene_graph/dsg_final.sparkdsg
    ros2 run f1tenth_hydra hydra_dsg_viewer --file X.sparkdsg --summary   # counts, no window
    ros2 run f1tenth_hydra hydra_dsg_viewer --file X.sparkdsg --png out.png

Needs the spark_dsg bindings (source scripts/hydra/hydra_env.sh); live mode also hydra_msgs.
"""

import argparse
import colorsys
import os
import sys
import threading

from f1tenth_hydra.dsg_tools import graph_geometry, graph_summary, import_spark_dsg

LAYER_STYLE = {   # name: (rgb 0-1, z offset in units of --z-offset, node size)
    'OBJECTS': ((0.95, 0.45, 0.1), 0, 9.0),
    'AGENTS': ((0.9, 0.1, 0.1), 0, 5.0),
    'MESH_PLACES': ((0.4, 0.75, 0.4), 0, 4.0),
    'PLACES': ((0.25, 0.45, 0.95), 1, 6.0),
    'ROOMS': ((0.75, 0.3, 0.85), 2, 14.0),
    'BUILDINGS': ((0.2, 0.2, 0.2), 3, 18.0),
}
DEFAULT_STYLE = ((0.5, 0.5, 0.5), 0, 5.0)


def label_color(label):
    if label is None:
        return None
    h = (label * 0.618033988749895) % 1.0
    return colorsys.hsv_to_rgb(h, 0.75, 0.95)


def load_label_names(path=None):
    """Hydra label space names {id: name}; '' if the label space file is not found."""
    import yaml
    if not path:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('hydra'), 'config', 'label_spaces',
                                'ade20k_mp3d_label_space.yaml')
        except Exception:  # noqa: BLE001
            return {}
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        return {int(e['label']): str(e['name']) for e in data.get('label_names', [])}
    except OSError:
        return {}


def scene(G, label_names, z_offset):
    """Drawable scene: per layer points/colors, edge segments, labels."""
    nodes, edges = graph_geometry(G, label_names)
    lift = {name: style[1] * z_offset for name, style in LAYER_STYLE.items()}

    def lifted(p, layer):
        return (p[0], p[1], p[2] + lift.get(layer, 0.0))

    layers, at, layer_of = {}, {}, {}
    for n in nodes:
        style = LAYER_STYLE.get(n['layer'], DEFAULT_STYLE)
        p = lifted(n['pos'], n['layer'])
        at[n['key']], layer_of[n['key']] = p, n['layer']
        color = (label_color(n['label']) if n['layer'] == 'OBJECTS' else None) or style[0]
        entry = layers.setdefault(n['layer'], {'points': [], 'colors': [], 'size': style[2]})
        entry['points'].append(p)
        entry['colors'].append(color)
    segments = []
    for src, dst, kind in edges:
        color = ((0.6, 0.6, 0.6) if kind == 'inter'
                 else LAYER_STYLE.get(layer_of[src], DEFAULT_STYLE)[0])
        segments.append((at[src], at[dst], color, kind))
    labels = [(lifted(n['pos'], n['layer']), n['text'] or n['id']) for n in nodes
              if n['layer'] in ('OBJECTS', 'ROOMS', 'BUILDINGS')]
    return {'layers': layers, 'segments': segments, 'labels': labels}


def format_summary(summary):
    rows = [f"nodes {summary['nodes']}, interlayer edges {summary['interlayer_edges']}, "
            f"mesh vertices {summary['mesh_vertices']}"]
    for name, c in sorted(summary['layers'].items()):
        rows.append(f'  {name:12s} {c["nodes"]:6d} nodes {c["edges"]:6d} edges')
    return '\n'.join(rows)


# ------------------------------------------------------------------ renderers
class Open3DView:
    def __init__(self, title):
        import open3d as o3d
        self.o3d = o3d
        self.gui = o3d.visualization.gui
        self.app = self.gui.Application.instance
        self.app.initialize()
        self.vis = o3d.visualization.O3DVisualizer(title, 1280, 800)
        self.vis.show_settings = True
        self.vis.point_size = 8
        self.app.add_window(self.vis)
        self.names = []
        self.first = True

    def _draw(self, sc):
        o3d, vis = self.o3d, self.vis
        for name in self.names:
            vis.remove_geometry(name)
        self.names = []
        vis.clear_3d_labels()
        for layer, d in sc['layers'].items():
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(d['points']))
            pc.colors = o3d.utility.Vector3dVector(d['colors'])
            vis.add_geometry(f'nodes_{layer}', pc)
            self.names.append(f'nodes_{layer}')
        if sc['segments']:
            pts, lines, colors = [], [], []
            for i, (a, b, c, _) in enumerate(sc['segments']):
                pts += [a, b]
                lines.append([2 * i, 2 * i + 1])
                colors.append(c)
            ls = o3d.geometry.LineSet(o3d.utility.Vector3dVector(pts),
                                      o3d.utility.Vector2iVector(lines))
            ls.colors = o3d.utility.Vector3dVector(colors)
            vis.add_geometry('edges', ls)
            self.names.append('edges')
        for p, text in sc['labels']:
            vis.add_3d_label(p, text)
        if self.first and self.names:
            vis.reset_camera_to_default()
            self.first = False

    def show(self, sc):
        self.app.post_to_main_thread(self.vis, lambda: self._draw(sc))

    def run(self):
        self.app.run()


class MatplotlibView:
    def __init__(self, title, png=None):
        import matplotlib
        if png:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        self.plt = plt
        self.png = png
        self.fig = plt.figure(title, figsize=(11, 8))
        self.ax = self.fig.add_subplot(projection='3d')
        self.pending = None
        self.lock = threading.Lock()

    def show(self, sc):
        with self.lock:
            self.pending = sc

    def _draw(self, sc):
        ax = self.ax
        ax.cla()
        for (a, b, c, kind) in sc['segments']:
            ax.plot(*zip(a, b), color=c, linewidth=0.5 if kind == 'inter' else 1.0,
                    alpha=0.5 if kind == 'inter' else 0.9)
        for layer, d in sc['layers'].items():
            xs, ys, zs = zip(*d['points'])
            ax.scatter(xs, ys, zs, c=d['colors'], s=d['size'] * 3, label=layer, depthshade=False)
        for p, text in sc['labels']:
            ax.text(*p, text, fontsize=7)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m] + layer offset')
        if sc['layers']:
            ax.legend(loc='upper left', fontsize=8)
        self.fig.canvas.draw_idle()

    def poll(self):
        with self.lock:
            sc, self.pending = self.pending, None
        if sc is not None:
            self._draw(sc)
            if self.png:
                self.fig.savefig(self.png, dpi=110)

    def run(self, spin_once=None):
        if self.png:
            self.poll()
            return
        while self.plt.fignum_exists(self.fig.number):
            if spin_once:
                spin_once()
            self.poll()
            self.plt.pause(0.1)


# ------------------------------------------------------------------ main
def main(argv=None):
    p = argparse.ArgumentParser(description='Hydra scene graph viewer (graph only).')
    p.add_argument('--file', help='.sparkdsg / .json graph file (default: live ROS topic)')
    p.add_argument('--topic', default='/hydra/backend/dsg')
    p.add_argument('--z-offset', type=float, default=3.0, help='metres between layers')
    p.add_argument('--labelspace', default=None, help='Hydra label space yaml (names)')
    p.add_argument('--backend', choices=('auto', 'open3d', 'matplotlib'), default='auto')
    p.add_argument('--summary', action='store_true', help='print layer counts and exit')
    p.add_argument('--png', help='render once to this PNG (matplotlib, no window) and exit')
    full = list(sys.argv if argv is None else argv)
    try:
        from rclpy.utilities import remove_ros_args
        plain = remove_ros_args(full)
    except ImportError:
        plain = full
    a = p.parse_args(plain[1:])

    dsg, err = import_spark_dsg()
    if dsg is None:
        print(err, file=sys.stderr)
        return 2
    names = load_label_names(a.labelspace)

    if a.file:
        path = os.path.expanduser(a.file)
        if path.endswith('.bin'):        # raw DsgUpdate payload (logger without spark_dsg)
            with open(path, 'rb') as f:
                G = dsg.DynamicSceneGraph.from_binary(f.read())
        else:
            G = dsg.DynamicSceneGraph.load(path)
        print(format_summary(graph_summary(G)))
        if a.summary:
            return 0
        view = make_view(a, f'Hydra DSG - {os.path.basename(a.file)}')
        view.show(scene(G, names, a.z_offset))
        view.run()
        return 0

    import rclpy
    from rclpy.node import Node
    try:
        from hydra_msgs.msg import DsgUpdate
    except ImportError as exc:
        print(f'live mode needs hydra_msgs ({exc}): source scripts/hydra/hydra_env.sh',
              file=sys.stderr)
        return 2
    rclpy.init(args=full)
    node = Node('hydra_dsg_viewer')
    view = None if a.summary else make_view(a, f'Hydra DSG - {a.topic}')
    state = {'graph': None, 'n': 0}

    def on_msg(msg):
        data = msg.layer_contents.tobytes()
        if state['graph'] is None:
            state['graph'] = dsg.DynamicSceneGraph.from_binary(data)
        else:
            state['graph'].update_from_binary(data)
        state['n'] += 1
        s = graph_summary(state['graph'])
        node.get_logger().info(f'graph #{state["n"]}: {s["nodes"]} nodes, '
                               f'{len(data) / 1024:.0f} kB',
                               throttle_duration_sec=5.0)
        if view is None:
            print(format_summary(s))
            raise SystemExit(0)
        view.show(scene(state['graph'], names, a.z_offset))

    node.create_subscription(DsgUpdate, a.topic, on_msg, 1)
    try:
        if view is None:
            rclpy.spin(node)
        elif isinstance(view, MatplotlibView):
            view.run(spin_once=lambda: rclpy.spin_once(node, timeout_sec=0.0))
        else:
            threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
            view.run()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def make_view(a, title):
    if a.png or a.backend == 'matplotlib':
        return MatplotlibView(title, png=a.png)
    try:
        return Open3DView(title)
    except Exception as exc:  # noqa: BLE001 - no open3d, or no display for it
        if a.backend == 'open3d':
            raise
        print(f'Open3D unavailable ({exc}): matplotlib viewer', file=sys.stderr)
        return MatplotlibView(title)


if __name__ == '__main__':
    sys.exit(main())
