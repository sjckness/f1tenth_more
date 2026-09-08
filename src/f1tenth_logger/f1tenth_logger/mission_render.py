"""
ROS-free half of the mission replay renderer.

THE POINT OF THIS FILE: it must import and run on a machine with no ROS
installed at all. linus renders reports from *.extract.parquet and never sees
a rosbag2 database, so nothing here may import rclpy, rosbag2_py,
rosidl_runtime_py or any message package, directly or transitively. The
ROS-dependent half lives in mission_extract.py, which imports FROM this module
and never the other way round.

The drawing code below is the original mission_replay_video.py code, moved
verbatim rather than rewritten: the split is a cut along a seam the file
already had (deserialize, then draw), not a redesign. That is enforced by test
rather than by intent, since "I did not change the drawing" is exactly the
kind of claim that is worth checking: rendering the reference bag through
extract to parquet to render must produce an MP4 with the same md5 the
pre-split path produced.

INPUT: a *.extract.parquet written by mission_extract.py. It carries the same
Stream samples read_bag() used to build in memory, at full recorded rate and
NOT pre-resampled onto a fixed grid, so --dt still selects the resample period
here exactly as it did when this code read the bag directly, and any dt
reproduces the original frame for frame. See mission_extract.py's own EXTRACT
FORMAT section for the column layout.

Run standalone, no ROS needed:

    python3 -m f1tenth_logger.mission_render <run>.extract.parquet --out-dir DIR

That MP4 md5 check only proves same-machine, same-ffmpeg-build equivalence --
VBR H.264 is not bit-identical across ffmpeg builds/architectures even from
pixel-identical input, so it cannot validate the drawing path CROSS-machine.
For that, use --frames-dir instead of --out-dir: it dumps raw
frame_0001.png.. with no video codec anywhere in the path, so `md5sum
frame_*.png` diffs line for line between machines with no codec confound.

    python3 -m f1tenth_logger.mission_render <run>.extract.parquet --frames-dir DIR
"""

import argparse
import json
import math
import sys
import time
import zlib
from bisect import bisect_right
from pathlib import Path

import numpy as np

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402
from matplotlib.patches import Circle, Ellipse, Polygon as MplPolygon  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

import pyarrow.parquet as pq  # noqa: E402

DEFAULT_BAG_ROOT = Path.home() / '.ros' / 'mission_bags'
# Videos land in the workspace root (gitignored), NOT next to the bags: the
# bag root lives on the scratch volume and is what gets rsync'd/pruned, while
# this is the directory anyone working in the repo already has open. Resolved
# from this file's own location so it does not depend on the cwd the script
# happens to be run from.
#
# The walk up to the workspace root replaces a plain parents[1], which was
# correct only while this file lived at <workspace>/scripts/. It now lives at
# <workspace>/src/f1tenth_logger/f1tenth_logger/, so a fixed index would point
# at the package directory and silently write videos somewhere new. Walking to
# the parent of the enclosing 'src' keeps the SAME output directory the 24
# already-rendered videos are in, and keeps working if the package is ever
# nested differently. resolve() first: --symlink-install makes the installed
# copy a symlink back into src/, and src/ is the tree with a workspace root
# above it.


def _default_out_dir(start: Path):
    """Workspace-root mission_videos/, or None if there is no workspace root.

    Returns None rather than guessing when this module has no src/ ancestor,
    which is the case under a plain (non --symlink-install) colcon build: the
    installed copy is a real file under install/<pkg>/lib/python3.X/site-
    packages/ with no source tree above it. Guessing at Path.cwd() there would
    scatter videos into whatever directory the caller happened to be in, and
    silently -- so main() turns None into an explicit "pass --out-dir" error
    instead. Under --symlink-install (this workspace's default) resolve()
    follows the symlink back into src/ and the walk succeeds as normal.
    """
    for parent in start.parents:
        if parent.name == 'src':
            return parent.parent / 'mission_videos'
    return None


DEFAULT_OUT_DIR = _default_out_dir(Path(__file__).resolve())

# Palette: dark surface + the reference categorical slots 1-3, which are the
# ones validated for ALL-pairs separation (obstacle classes appear side by side
# on the map, so adjacent-pair validation would be the wrong gate). A 4th class
# folds into OTHER grey rather than inventing a hue; every track is also
# direct-labelled with its class name, so identity is never colour-alone.
SURFACE = '#1a1a19'
PANEL = '#121211'
INK = '#ffffff'
INK_SECONDARY = '#c3c2b7'
INK_MUTED = '#898781'
GRID = '#2c2c2a'
CLASS_COLORS = ['#3987e5', '#d95926', '#199e70']
CLASS_OTHER = '#898781'
STATUS = {'good': '#0ca30c', 'warning': '#fab219', 'serious': '#ec835a',
          'critical': '#d03b3b'}
HARD_COLOR = '#c3c2b7'          # hard constraint: neutral ink, reads as structure
CORRIDOR_COLOR = '#3987e5'      # MPC reference corridor: the funnel band
# Predicted horizon: magenta, deliberately NOT white and NOT the corridor's
# blue -- the executed trail is white and the corridor band is blue, and a
# prediction that looks like either of those is a prediction being read as
# fact.
HORIZON_COLOR = '#d55181'
CAR_COLOR = '#ffffff'
MAP_CMAP = LinearSegmentedColormap.from_list(
    'slam_occ', ['#242423', '#3a3a37', '#7e7c74'])

# Per-stream max age for the zero-order hold, seconds. 0.5 for boundaries is
# not a guess: it is the same staleness gate MPC_corr.py applies to them
# (odom_stale_timeout_sec, reused by _get_live_boundaries) -- past it the
# solver itself would have dropped them, so the video must too.
MAX_AGE = {
    'pose': 0.5,
    'boundaries': 0.5,
    'clearance': 0.5,
    'obstacles': 0.5,
    'detections': 0.5,
    'markers': 1.0,
    # The corridor is rebuilt at 0.5 * ts (see MPC_corr's corridor_update_
    # period) and the horizon comes with every solve, so both are as live as
    # the control loop itself; 0.5s only bridges a dropped message.
    'corridor': 0.5,
    'map_to_odom': 0.5,
    'tree': 0.5,
    'solver': 0.5,
    'drive': 0.5,
    'stop': 0.25,
}


# --------------------------------------------------------------------------
# bag discovery
# --------------------------------------------------------------------------
def discover_bags(bag_root: Path, count: int):
    """Newest `count` mission bags by MANIFEST start_time (not directory
    mtime): the manifest is the mission's own record of when the run began,
    and a bag directory's mtime moves whenever anything touches it."""
    entries = []
    for manifest_path in sorted(bag_root.glob('*.manifest.json')):
        try:
            manifest = json.loads(manifest_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            print(f'  skipping {manifest_path.name}: {exc}', file=sys.stderr)
            continue
        bag_dir = bag_root / manifest.get('run_id', manifest_path.stem)
        if not (bag_dir / 'metadata.yaml').exists():
            # bag_path in the manifest can point at the recording machine's own
            # path (~/.ros/...) even when the bags now live somewhere else, so
            # resolve against bag_root and skip if there is genuinely no bag.
            continue
        entries.append((manifest.get('start_time', ''), bag_dir, manifest))
    entries.sort(key=lambda e: e[0], reverse=True)
    return entries[:count]


def load_manifest_for(bag_dir: Path):
    """
    Find and load this bag's manifest.json -- two layouts, tried in order.

    OLD (flat, still what ~/.ros/mission_bags/ and mission_replay_video's live
    one-step workflow use): <bag_root>/<run_id>/ sits next to
    <bag_root>/<run_id>.manifest.json, so the manifest is named after the bag
    directory itself.

    NEW (runs_migrate's one-folder-per-run archive, <archive>/complete/
    <run_id>/): the bag always lives at <run_id>/bag/ -- literally named
    "bag" in every run -- so the manifest is named after the bag directory's
    PARENT instead. Without this second candidate, extract_bag() silently
    finds no manifest for any migrated run, and padding_from_params()
    (fed from manifest['params_snapshot_path']) falls back to stack defaults
    with no error -- exactly the "silently redraws history" failure its own
    docstring warns about, just one layer upstream of where that docstring
    looks.
    """
    candidates = [bag_dir.parent / f'{bag_dir.name}.manifest.json']
    if bag_dir.name == 'bag':
        candidates.append(bag_dir.parent / f'{bag_dir.parent.name}.manifest.json')
    for manifest_path in candidates:
        if manifest_path.exists():
            try:
                return json.loads(manifest_path.read_text())
            except (OSError, json.JSONDecodeError):
                pass
    return {}


# --------------------------------------------------------------------------
# small geometry helpers
# --------------------------------------------------------------------------
def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def body_to_map(px, py, pose):
    """(x, y) in base_link -> map, using the frame's own vehicle pose."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * px - s * py, y + s * px + c * py


def odom_points_to_map(points, map_to_odom):
    """Transform a polyline from the MPC's odom frame into map with the live
    map->odom TF (the pose of odom expressed in map). Identity when the caller
    is already drawing in odom (--pose-source local) or when no TF is fresh --
    the latter is reported once per bag rather than silently drawn wrong."""
    if map_to_odom is None:
        return None
    ox, oy, oyaw = map_to_odom
    c, s_ = math.cos(oyaw), math.sin(oyaw)
    return [(ox + c * px - s_ * py, oy + s_ * px + c * py) for px, py in points]


def boundary_to_map(nx, ny, offset, pose):
    """Halfspace `n . p <= offset` from base_link into map. Same derivation as
    MPC_corr.py's own _boundary_to_world (which transforms into the MPC's
    odom-frame world instead) -- kept identical so the drawn line is the line
    the solver constrained against, not a lookalike."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    nwx = c * nx - s * ny
    nwy = s * nx + c * ny
    return nwx, nwy, offset + nwx * x + nwy * y


def clip_polygon_halfspace(poly, nx, ny, c):
    """Sutherland-Hodgman clip of `poly` down to {p : n . p <= c}. Used to
    shade the EXCLUDED side by clipping the view rectangle with the flipped
    halfspace, which stays correct for any line/rectangle intersection
    (including a line that misses the view entirely -> empty polygon)."""
    out = []
    n = len(poly)
    for i in range(n):
        cur = poly[i]
        nxt = poly[(i + 1) % n]
        d_cur = nx * cur[0] + ny * cur[1] - c
        d_nxt = nx * nxt[0] + ny * nxt[1] - c
        if d_cur <= 0:
            out.append(cur)
        if (d_cur <= 0) != (d_nxt <= 0):
            t = d_cur / (d_cur - d_nxt)
            out.append((cur[0] + t * (nxt[0] - cur[0]),
                        cur[1] + t * (nxt[1] - cur[1])))
    return out


def segment_near(nx, ny, c, cx, cy, span):
    """The piece of `n . p = c` within `span` metres (along the line) of the
    foot of the perpendicular from (cx, cy) -- i.e. the stretch of wall beside
    the car, not the infinite line."""
    norm = math.hypot(nx, ny)
    if norm < 1e-9:
        return None
    ux, uy = nx / norm, ny / norm
    cc = c / norm
    d = ux * cx + uy * cy - cc               # signed distance car -> line
    fx, fy = cx - d * ux, cy - d * uy        # foot of the perpendicular
    dx, dy = -uy, ux
    return ((fx - dx * span, fy - dy * span), (fx + dx * span, fy + dy * span))


# --------------------------------------------------------------------------
# bag reading
# --------------------------------------------------------------------------
class Stream:
    """Timestamped samples plus a zero-order-hold lookup with a max age."""

    def __init__(self, max_age):
        self.t = []
        self.v = []
        self.max_age = max_age

    def add(self, t, v):
        self.t.append(t)
        self.v.append(v)

    def at(self, t):
        """Value in force at time t, or None if nothing is fresh enough."""
        i = bisect_right(self.t, t) - 1
        if i < 0:
            return None
        if self.max_age is not None and (t - self.t[i]) > self.max_age:
            return None
        return self.v[i]

    def __len__(self):
        return len(self.t)


def static_chain_to_base(static_tf, frame, base='base_link'):
    """Compose the static transform `frame` -> `base` (rotation, translation),
    or None if the chain does not reach base. Camera detections arrive in
    zed2_left_camera_frame; the chain up to base_link is entirely static
    (/tf_static), so no time-varying lookup is needed."""
    rot = np.eye(3)
    trans = np.zeros(3)
    cur = frame
    for _ in range(16):
        if cur == base:
            return rot, trans
        entry = static_tf.get(cur)
        if entry is None:
            return None
        parent, t_pc, r_pc = entry
        rot = r_pc @ rot
        trans = r_pc @ trans + t_pc
        cur = parent
    return None


# --------------------------------------------------------------------------
# rendering
# --------------------------------------------------------------------------
def class_color(class_id, order):
    if class_id not in order:
        if len(order) < len(CLASS_COLORS):
            order[class_id] = len(order)
        else:
            order[class_id] = None
    slot = order[class_id]
    return CLASS_COLORS[slot] if slot is not None else CLASS_OTHER


def compute_view(bag, dt):
    """Fixed view box covering the whole run: the car never leaves frame and
    the eye has a stable reference, which a per-frame autoscale destroys."""
    xs, ys = [], []
    for (x, y, _yaw, _v) in bag['streams']['pose'].v:
        xs.append(x)
        ys.append(y)
    for tracks in bag['streams']['markers'].v:
        for (kind, _ns, x, y, _r, _txt) in tracks:
            if kind == 'disk':
                xs.append(x)
                ys.append(y)
    if not xs:
        return (-5, 5), (-5, 5)
    pad = 2.5
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad
    # Square the box so equal-aspect axes do not letterbox unpredictably.
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    half = 0.5 * max(x1 - x0, y1 - y0)
    return (cx - half, cx + half), (cy - half, cy + half)


def draw_frame(axes, bag, t, rel_t, cfg, state):
    ax_map, ax_hud, ax_time = axes
    streams = bag['streams']
    ax_map.clear()
    ax_hud.clear()
    ax_time.clear()

    pose = streams['pose'].at(t)
    xlim, ylim = state['view']
    if cfg.follow and pose is not None:
        h = cfg.follow
        xlim = (pose[0] - h, pose[0] + h)
        ylim = (pose[1] - h, pose[1] + h)

    ax_map.set_facecolor(SURFACE)
    ax_map.set_xlim(*xlim)
    ax_map.set_ylim(*ylim)
    ax_map.set_aspect('equal')
    ax_map.grid(True, color=GRID, linewidth=0.5, alpha=0.6)
    ax_map.tick_params(colors=INK_MUTED, labelsize=7)
    for spine in ax_map.spines.values():
        spine.set_color(GRID)
    ax_map.set_xlabel('x [m]', color=INK_MUTED, fontsize=8)
    ax_map.set_ylabel('y [m]', color=INK_MUTED, fontsize=8)

    # --- SLAM occupancy grid, recessive: context, never a subject ---------
    grid_entry = streams['map'].at(t)
    if grid_entry is not None and not cfg.no_map:
        grid, extent = grid_entry
        # Unknown (-1) -> NaN -> fully transparent, so "never mapped" reads as
        # the page's own background rather than as free space. Free -> barely
        # above the surface, occupied -> light: on a dark surface the WALLS
        # must be the bright thing (matplotlib's own 'Greys' would do the
        # opposite and turn free space into a glowing slab).
        occ = np.where(grid < 0, np.nan, grid.astype(float))
        ax_map.imshow(occ, origin='lower', extent=extent, cmap=MAP_CMAP,
                      vmin=0, vmax=100, alpha=0.9, zorder=0,
                      interpolation='nearest')

    # --- hard-constraint corridor ----------------------------------------
    bounds = streams['boundaries'].at(t)
    if bounds and pose is not None:
        for (nx, ny, offset) in bounds:
            nwx, nwy, c_raw = boundary_to_map(nx, ny, offset, pose[:3])
            norm = math.hypot(nwx, nwy)
            if norm < 1e-9:
                continue
            c_pad = c_raw - (cfg.car_radius + cfg.avoidance_margin) * norm
            # Shade the whole excluded halfspace (it IS infinite -- the solver
            # applies it everywhere), but draw the lines only within
            # --corridor-span of the car: an unbounded line pair per constraint
            # crossing the whole map reads as three random diagonals, and the
            # part that decides anything is the part beside the vehicle.
            excluded = clip_polygon_halfspace(
                [(xlim[0], ylim[0]), (xlim[1], ylim[0]),
                 (xlim[1], ylim[1]), (xlim[0], ylim[1])],
                -nwx, -nwy, -c_raw)
            if len(excluded) >= 3:
                ax_map.add_patch(MplPolygon(excluded, closed=True, facecolor=HARD_COLOR,
                                            alpha=0.13, edgecolor='none', zorder=1))
            for c_line, width, style, alpha in ((c_raw, 2.2, 'solid', 0.95),
                                                (c_pad, 1.2, (0, (5, 4)), 0.7)):
                seg = segment_near(nwx, nwy, c_line, pose[0], pose[1], cfg.corridor_span)
                if seg:
                    ax_map.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]],
                                color=HARD_COLOR, linewidth=width, linestyle=style,
                                alpha=alpha, zorder=4, solid_capstyle='round')

    # --- MPC reference corridor: the widening funnel ----------------------
    # Drawn as a filled band between the left and right Bezier walls, NOT as
    # two outlines: the whole point of this layer is that the corridor starts
    # narrow at the car and opens out ahead, and a filled region makes that
    # legible in a single frame where two diverging lines do not.
    walls = streams['corridor'].at(t)
    if walls:
        m2o = state['map_to_odom_fn'](t)
        left = odom_points_to_map(walls.get('corridor_left', []), m2o)
        right = odom_points_to_map(walls.get('corridor_right', []), m2o)
        center = odom_points_to_map(walls.get('corridor_centerline', []), m2o)
        if left and right:
            band = left + right[::-1]
            ax_map.add_patch(MplPolygon(band, closed=True,
                                        facecolor=CORRIDOR_COLOR, alpha=0.16,
                                        edgecolor='none', zorder=1.5))
            for wall in (left, right):
                ax_map.plot([px for px, _ in wall], [py for _, py in wall],
                            color=CORRIDOR_COLOR, linewidth=1.4, alpha=0.85,
                            zorder=3.5, solid_capstyle='round')
        if center:
            ax_map.plot([px for px, _ in center], [py for _, py in center],
                        color=CORRIDOR_COLOR, linewidth=0.9, alpha=0.5,
                        linestyle=(0, (4, 4)), zorder=3.5)

    # --- soft-cost obstacles (camera path) --------------------------------
    obstacles = streams['obstacles'].at(t)
    if obstacles and pose is not None:
        activation = cfg.car_radius + cfg.avoidance_margin
        for (ox, oy, r) in obstacles:
            mx, my = body_to_map(ox, oy, pose[:3])
            # Glow: concentric rings out to the radius at which mpc_solver's
            # softplus penalty starts biting -- a gradient, because the
            # mechanism is a gradient, unlike the hard lines above.
            for k in range(6):
                frac = 1.0 - k / 6.0
                ax_map.add_patch(Circle(
                    (mx, my), r + activation * frac, facecolor=STATUS['serious'],
                    alpha=0.055, edgecolor='none', zorder=2))
            ax_map.add_patch(Circle((mx, my), r + activation, facecolor='none',
                                    edgecolor=STATUS['serious'], linewidth=0.9,
                                    linestyle=(0, (2, 3)), alpha=0.8, zorder=3))
            ax_map.add_patch(Circle((mx, my), max(r, 0.05),
                                    facecolor=STATUS['serious'], alpha=0.85,
                                    edgecolor='none', zorder=5))

    # --- raw detections: uncertainty ellipses ------------------------------
    det_entry = streams['detections'].at(t)
    if det_entry is not None and pose is not None and state['cam_to_base'] is not None:
        rot, trans = state['cam_to_base']
        _frame, dets = det_entry
        for (class_id, score, dx, dy, dz, sigma) in dets:
            p_base = rot @ np.array([dx, dy, dz]) + trans
            mx, my = body_to_map(p_base[0], p_base[1], pose[:3])
            color = class_color(class_id, state['class_order'])
            if sigma:
                for k in (1.0, 2.0):
                    ax_map.add_patch(Ellipse(
                        (mx, my), 2 * k * sigma, 2 * k * sigma, facecolor=color,
                        alpha=0.10 if k == 2.0 else 0.16, edgecolor=color,
                        linewidth=0.8, linestyle=(0, (1, 2)), zorder=3))
            ax_map.plot([mx], [my], marker='x', markersize=7, color=color,
                        alpha=0.85, markeredgewidth=1.6, zorder=6)

    # --- confirmed semantic tracks ----------------------------------------
    tracks = streams['markers'].at(t)
    if tracks:
        for (kind, ns, tx, ty, tr, txt) in tracks:
            if kind != 'disk':
                continue
            color = class_color(ns, state['class_order'])
            ax_map.add_patch(Circle((tx, ty), max(tr, 0.12) * 2.2, facecolor=color,
                                    alpha=0.13, edgecolor='none', zorder=6))
            ax_map.add_patch(Circle((tx, ty), max(tr, 0.12), facecolor=color,
                                    alpha=0.95, edgecolor=SURFACE, linewidth=1.5,
                                    zorder=7))
        for (kind, ns, tx, ty, _tr, txt) in tracks:
            if kind != 'label':
                continue
            ax_map.text(tx + 0.18, ty + 0.18, txt or ns, color=INK_SECONDARY,
                        fontsize=7.5, zorder=8,
                        path_effects=None)

    # --- car + fading trail ------------------------------------------------
    trail = [(tt, v) for tt, v in zip(streams['pose'].t, streams['pose'].v)
             if t - cfg.trail_seconds <= tt <= t]
    if len(trail) >= 2:
        pts = np.array([[v[0], v[1]] for _tt, v in trail])
        segs = np.stack([pts[:-1], pts[1:]], axis=1)
        alphas = np.linspace(0.04, 0.65, len(segs))
        lc = LineCollection(segs, colors=[(1, 1, 1, a) for a in alphas],
                            linewidths=1.4, zorder=8)
        ax_map.add_collection(lc)
    if pose is not None:
        x, y, yaw, _v = pose
        c, s = math.cos(yaw), math.sin(yaw)
        body = [(0.26, 0.0), (-0.12, 0.13), (-0.05, 0.0), (-0.12, -0.13)]
        pts = [(x + c * bx - s * by, y + s * bx + c * by) for bx, by in body]
        ax_map.add_patch(MplPolygon(pts, closed=True, facecolor=CAR_COLOR,
                                    edgecolor=SURFACE, linewidth=1.0, zorder=10))
        ax_map.add_patch(Circle((x, y), cfg.car_radius, facecolor='none',
                                edgecolor=CAR_COLOR, alpha=0.35, linewidth=0.9,
                                linestyle=(0, (2, 3)), zorder=9))

    # --- MPC predicted horizon --------------------------------------------
    # Fresh every frame (it is re-solved every tick), starting at the car's
    # own position so the ghost reads as "from here, this is where I think I
    # am going". Tapered width + fading alpha toward the far end: the far end
    # of an MPC horizon is the least trustworthy part of it, and the styling
    # should say so rather than drawing it as confidently as the executed
    # trail (which is solid white) or the corridor walls (which are blue).
    solver_frame = streams['solver'].at(t)
    if solver_frame is not None and solver_frame[9] and pose is not None:
        m2o = state['map_to_odom_fn'](t)
        horizon = odom_points_to_map(list(zip(solver_frame[9], solver_frame[10])), m2o)
        if horizon:
            pts = np.array([[pose[0], pose[1]]] + horizon)
            segs = np.stack([pts[:-1], pts[1:]], axis=1)
            n_seg = len(segs)
            fade = np.linspace(0.95, 0.15, n_seg)
            widths = np.linspace(2.4, 0.7, n_seg)
            rgb = matplotlib.colors.to_rgb(HORIZON_COLOR)
            ax_map.add_collection(LineCollection(
                segs, colors=[(*rgb, a) for a in fade], linewidths=widths,
                zorder=11, capstyle='round'))
            # One dot per predicted STATE (not a smooth curve): the horizon is
            # a discrete sequence of states at ts intervals, and their spacing
            # is itself the predicted speed profile.
            ax_map.plot(pts[1:, 0], pts[1:, 1], linestyle='none', marker='o',
                        markersize=2.6, color=HORIZON_COLOR, alpha=0.75,
                        zorder=11)
            ax_map.plot([pts[-1, 0]], [pts[-1, 1]], marker='o', markersize=5.0,
                        markerfacecolor='none', markeredgecolor=HORIZON_COLOR,
                        markeredgewidth=1.2, alpha=0.9, zorder=11)

    # --- HUD ---------------------------------------------------------------
    stop = streams['stop'].at(t)
    tree = streams['tree'].at(t)
    solver = streams['solver'].at(t)
    drive = streams['drive'].at(t)
    clearance = streams['clearance'].at(t)

    ax_hud.set_facecolor(PANEL)
    ax_hud.set_xlim(0, 1)
    ax_hud.set_ylim(0, 1)
    ax_hud.set_xticks([])
    ax_hud.set_yticks([])
    for spine in ax_hud.spines.values():
        spine.set_color(GRID)

    lines = []
    lines.append((f"{state['mission_id']}", INK, 10.5, 'bold'))
    lines.append((f"outcome {state['outcome']}   {state['run_id'][:19]}",
                  INK_MUTED, 7.5, 'normal'))
    lines.append((f"t = {rel_t:6.2f} s / {state['duration']:.2f} s", INK_SECONDARY, 9, 'normal'))
    lines.append(('', INK, 4, 'normal'))

    if stop is not None:
        src = stop[0] or 'unknown'
        lines.append((f"SAFETY STOP  ({src})", STATUS['critical'], 11, 'bold'))
    elif tree is not None and tree[4]:
        lines.append((f"SAFETY STOP  ({tree[5] or 'unknown'})", STATUS['critical'], 11, 'bold'))
    else:
        lines.append(('driving', STATUS['good'], 10, 'bold'))

    if tree is not None:
        active, names, statuses, trip, _sa, _src = tree
        lines.append((f"BT lane: {active or '(root failed)'}", INK_SECONDARY, 8.5, 'normal'))
        lines.append(('  ' + '  '.join(f'{n}:{s[:4]}' for n, s in zip(names, statuses)),
                      INK_MUTED, 7, 'normal'))
        if trip:
            lines.append((f"  emergency trip: {trip}", STATUS['critical'], 8.5, 'normal'))
    else:
        lines.append(('BT lane: (no tick)', INK_MUTED, 8.5, 'normal'))

    lines.append(('', INK, 4, 'normal'))
    if solver is not None:
        (ok, status, status_msg, dt_s, period, cost, backend, nb, nobs,
         pred_x, _pred_y, _pred_yaw, pred_v, _pred_frame) = solver
        color = STATUS['good'] if ok else STATUS['critical']
        lines.append((f"MPC: {'solved' if ok else 'FAILED'}  [{status_msg}]", color, 9, 'bold'))
        late = dt_s > period
        lines.append((f"  solve {dt_s * 1000:5.1f} ms / {period * 1000:.0f} ms budget"
                      + ('  LATE' if late else ''),
                      STATUS['warning'] if late else INK_SECONDARY, 8, 'normal'))
        lines.append((f"  cost {cost:8.3f}   backend {backend}", INK_SECONDARY, 8, 'normal'))
        lines.append((f"  hard constraints {nb}   obstacle disks {nobs}",
                      INK_SECONDARY, 8, 'normal'))
        if pred_x:
            v_end = f"   v_end {pred_v[-1]:.2f} m/s" if pred_v else ''
            lines.append((f"  horizon {len(pred_x)} steps"
                          f" ({len(pred_x) * period:.1f} s){v_end}",
                          INK_SECONDARY, 8, 'normal'))
        else:
            lines.append(('  horizon: not published in this bag',
                          INK_MUTED, 7.5, 'normal'))
    else:
        lines.append(('MPC: (no solve this frame)', INK_MUTED, 9, 'normal'))

    lines.append(('', INK, 4, 'normal'))
    if drive is not None:
        lines.append((f"cmd  v {drive[0]:5.2f} m/s   steer {math.degrees(drive[1]):6.1f} deg",
                      INK_SECONDARY, 8, 'normal'))
    if pose is not None:
        lines.append((f"pose ({pose[0]:6.2f}, {pose[1]:6.2f}) m  "
                      f"yaw {math.degrees(pose[2]):6.1f} deg",
                      INK_SECONDARY, 8, 'normal'))
        lines.append((f"     v_meas {pose[3]:5.2f} m/s   [{state['pose_frame']}]",
                      INK_MUTED, 7.5, 'normal'))
    if clearance is not None:
        lines.append((f"front clearance {clearance:5.2f} m", INK_SECONDARY, 8, 'normal'))

    lines.append(('', INK, 6, 'normal'))
    lines.append(('LEGEND', INK_MUTED, 7.5, 'bold'))
    lines.append(('  filled band = MPC reference corridor (funnel)', CORRIDOR_COLOR, 7, 'normal'))
    lines.append(('  tapering ghost = predicted horizon (this tick)', HORIZON_COLOR, 7, 'normal'))
    lines.append(('  solid/dashed line = hard boundary halfspace (raw / padded)',
                  HARD_COLOR, 7, 'normal'))
    lines.append(('  glow disk = soft-cost obstacle (camera)', STATUS['serious'], 7, 'normal'))
    lines.append(('  dotted ellipse = detection 1/2-sigma', INK_MUTED, 7, 'normal'))
    for cls, slot in sorted(state['class_order'].items(), key=lambda kv: (kv[1] is None, kv[1])):
        col = CLASS_COLORS[slot] if slot is not None else CLASS_OTHER
        lines.append((f'  {cls}', col, 7, 'normal'))

    y = 0.975
    for text, color, size, weight in lines:
        if text:
            ax_hud.text(0.04, y, text, color=color, fontsize=size, weight=weight,
                        va='top', ha='left', family='monospace',
                        transform=ax_hud.transAxes)
        y -= (size + 4.5) / 460.0

    # --- timeline strip ----------------------------------------------------
    ax_time.set_facecolor(PANEL)
    ax_time.set_xlim(0, state['duration'])
    ax_time.set_ylim(0, 1)
    ax_time.set_yticks([])
    ax_time.tick_params(colors=INK_MUTED, labelsize=7)
    for spine in ax_time.spines.values():
        spine.set_color(GRID)
    for (t_a, t_b) in state['stop_spans']:
        ax_time.axvspan(t_a, t_b, color=STATUS['critical'], alpha=0.55, lw=0)
    for tf in state['solver_fail_times']:
        ax_time.plot([tf, tf], [0.0, 0.35], color=STATUS['warning'], linewidth=1.0)
    ax_time.axvline(rel_t, color=INK, linewidth=1.4)
    ax_time.set_xlabel('mission time [s]   (red = safety stop, amber = solver failure)',
                       color=INK_MUTED, fontsize=7.5)


def build_stop_spans(bag, dt):
    """Contiguous [start, end) windows in mission time where a safety stop was
    active, from /safety_stop itself (the recorded fact) rather than from the
    BT's own flag -- both are recorded, and the topic is what actually reached
    the mux."""
    t0 = bag['t0']
    stops = bag['streams']['stop']
    spans = []
    gap = 0.5
    for tt in stops.t:
        rel = tt - t0
        if spans and rel - spans[-1][1] <= gap:
            spans[-1][1] = rel
        else:
            spans.append([rel, rel])
    return [(a, max(b, a + dt)) for a, b in spans]


def _resolve_padding(bag, cfg, label):
    """
    Fill cfg.car_radius / cfg.avoidance_margin from THIS RUN unless overridden.

    These two decide where the tightened (dashed) boundary line sits, so they
    are properties of the run, not viewing preferences. They used to be
    hard-coded argparse defaults (0.20 / 0.12), which meant re-rendering a
    historical run after a config change would silently draw padding that run
    never flew with -- and the values were a third independent copy of numbers
    that live in stack_params.yaml, agreeing only by coincidence. The extract
    now carries the run's own values, read from its params snapshot.

    An explicit --car-radius/--avoidance-margin still wins, because comparing
    a run against different padding is a legitimate thing to want; it is just
    no longer what happens by default. The override is announced, so a frame
    drawn with anything other than the run's own numbers says so.
    """
    padding = bag.get('padding') or {}
    for name, flag in (('car_radius', '--car-radius'),
                       ('avoidance_margin', '--avoidance-margin')):
        override = getattr(cfg, name, None)
        if override is not None:
            recorded = padding.get(name)
            if recorded is not None and override != recorded:
                print(f'[{label}] {flag}={override} overrides this run\'s own '
                      f'{name}={recorded}', file=sys.stderr)
            continue
        if name not in padding:
            raise ValueError(
                f'{label}: extract carries no {name} and none was passed; '
                f'pass {flag} explicitly rather than guessing at this run\'s '
                'geometry')
        setattr(cfg, name, padding[name])
    source = padding.get('source')
    if source == 'fallback' or source == 'unset':
        print(f'[{label}] no params snapshot for this run: boundary padding '
              f'falls back to car_radius={cfg.car_radius} '
              f'avoidance_margin={cfg.avoidance_margin}', file=sys.stderr)


def render_bag(bag, manifest, label, cfg):
    """
    Draw one already-loaded run to MP4.

    Split out of the old render_bag(directory, cfg): the bag, manifest and
    label are passed in now rather than read here, because loading needs ROS
    and this half must not. Everything below is the original code, moved
    verbatim; the md5 check is what enforces that.
    """
    read_start = time.time()
    _resolve_padding(bag, cfg, label)
    if len(bag['streams']['pose']) == 0:
        print(f'[{label}] no pose samples on {bag["pose_topic"]}; skipped',
              file=sys.stderr)
        return None
    duration = bag['t1'] - bag['t0']
    print(f'[{label}] {duration:.1f}s, '
          f'{len(bag["streams"]["pose"])} poses, '
          f'{len(bag["streams"]["boundaries"])} boundary msgs, '
          f'{len(bag["streams"]["obstacles"])} obstacle msgs, '
          f'{len(bag["streams"]["stop"])} safety_stop msgs, '
          f'{len(bag["streams"]["corridor"])} corridor msgs '
          f'(load {time.time() - read_start:.1f}s)')
    n_horizon = sum(1 for v in bag['streams']['solver'].v if v[9])
    if bag['legacy_solver_msgs']:
        print(f'[{label}] {bag["legacy_solver_msgs"]} solver_status msgs '
              'read with the pre-horizon layout (bag predates the horizon '
              'fields) -- no predicted trajectory to draw')
    else:
        print(f'[{label}] {n_horizon}/{len(bag["streams"]["solver"])} '
              'solves carry a predicted horizon')
    if len(bag['streams']['corridor']) == 0:
        print(f'[{label}] no /mpc/corridor_markers in this bag -- the '
              'reference corridor funnel cannot be drawn (bag predates it '
              'being recorded)', file=sys.stderr)
    elif cfg.pose_source != 'local' and len(bag['streams']['map_to_odom']) == 0:
        print(f'[{label}] no map->odom TF -- odom-frame layers '
              '(corridor, horizon) will be skipped', file=sys.stderr)
    for topic, count in bag['dropped'].items():
        print(f'[{label}] dropped {count} undecodable msgs on {topic}',
              file=sys.stderr)

    cam_to_base = None
    det_entry = bag['streams']['detections'].v[0] if len(bag['streams']['detections']) else None
    if det_entry is not None:
        cam_to_base = static_chain_to_base(bag['static_tf'], det_entry[0])
        if cam_to_base is None:
            print(f'[{label}] no static tf chain {det_entry[0]} -> base_link; '
                  'uncertainty ellipses disabled', file=sys.stderr)

    solver_fail_times = [tt - bag['t0']
                         for tt, v in zip(bag['streams']['solver'].t,
                                          bag['streams']['solver'].v)
                         if not v[0]]

    # In map-frame mode every odom-frame layer (corridor, horizon) needs the
    # live map->odom TF; in local mode the render IS in odom, so the transform
    # is the identity and no TF lookup is needed at all.
    if cfg.pose_source == 'local':
        def map_to_odom_fn(_t):
            return (0.0, 0.0, 0.0)
    else:
        def map_to_odom_fn(t):
            return bag['streams']['map_to_odom'].at(t)

    state = {
        'view': compute_view(bag, cfg.dt),
        'map_to_odom_fn': map_to_odom_fn,
        'class_order': {},
        'cam_to_base': cam_to_base,
        'mission_id': manifest.get('mission_id', label),
        'outcome': manifest.get('outcome', '?'),
        'run_id': manifest.get('run_id', label),
        'duration': duration,
        'pose_frame': bag['pose_frame'] or '?',
        'stop_spans': build_stop_spans(bag, cfg.dt),
        'solver_fail_times': solver_fail_times,
    }

    fig = plt.figure(figsize=(12.8, 7.2), dpi=cfg.dpi)
    fig.patch.set_facecolor(SURFACE)
    ax_map = fig.add_axes([0.045, 0.16, 0.60, 0.79])
    ax_hud = fig.add_axes([0.665, 0.16, 0.315, 0.79])
    ax_time = fig.add_axes([0.045, 0.055, 0.935, 0.055])

    n_frames = max(1, int(duration / cfg.dt) + 1)
    fps = max(1, int(round(cfg.speed / cfg.dt)))
    render_start = time.time()

    frames_dir = getattr(cfg, 'frames_dir', None)
    if frames_dir is not None:
        # Raw PNGs, one savefig() per frame, NO video codec anywhere in this
        # path. The point: VBR H.264 is not bit-identical across ffmpeg
        # builds/architectures even from pixel-identical input, so an MP4 md5
        # cannot validate the drawing code cross-machine -- only cross-run on
        # the SAME machine/ffmpeg build (that check is real and stays valid;
        # this is the additional, codec-free one). matplotlib's Agg PNG
        # backend has no such confound: same figure content in, same bytes
        # out, whatever machine runs it. Naming is frame_0001.png.. (1-
        # indexed, 4-digit) so a plain `md5sum frame_*.png | sort` on two
        # machines diffs line for line.
        frames_dir.mkdir(parents=True, exist_ok=True)
        print(f'[{label}] rendering {n_frames} frames @ dt={cfg.dt}s '
              f'-> {frames_dir}/frame_%04d.png')
        for k in range(n_frames):
            rel_t = k * cfg.dt
            draw_frame((ax_map, ax_hud, ax_time), bag, bag['t0'] + rel_t, rel_t,
                       cfg, state)
            fig.savefig(frames_dir / f'frame_{k + 1:04d}.png',
                        facecolor=SURFACE, dpi=cfg.dpi)
            if cfg.progress and (k % 25 == 0 or k == n_frames - 1):
                print(f'  frame {k + 1}/{n_frames}', end='\r', flush=True)
        plt.close(fig)
        print(f'\n[{label}] done in {time.time() - render_start:.1f}s '
              f'-> {frames_dir} ({n_frames} PNGs)')
        return frames_dir

    out_path = cfg.out_dir / f'{state["run_id"]}.mp4'
    cfg.out_dir.mkdir(parents=True, exist_ok=True)
    writer = FFMpegWriter(fps=fps, bitrate=cfg.bitrate,
                          metadata={'title': state['run_id'],
                                    'comment': 'mission_replay_video.py'})
    print(f'[{label}] rendering {n_frames} frames @ {fps} fps '
          f'({cfg.speed}x) -> {out_path}')
    with writer.saving(fig, str(out_path), cfg.dpi):
        for k in range(n_frames):
            rel_t = k * cfg.dt
            draw_frame((ax_map, ax_hud, ax_time), bag, bag['t0'] + rel_t, rel_t,
                       cfg, state)
            writer.grab_frame(facecolor=SURFACE)
            if cfg.progress and (k % 25 == 0 or k == n_frames - 1):
                print(f'  frame {k + 1}/{n_frames}', end='\r', flush=True)
    plt.close(fig)
    print(f'\n[{label}] done in {time.time() - render_start:.1f}s -> {out_path}')
    return out_path


# --------------------------------------------------------------------------
# extract reading -- the parquet equivalent of what read_bag() returned
# --------------------------------------------------------------------------
def read_extract(path):
    """
    Rebuild the in-memory bag dict from a *.extract.parquet.

    Returns exactly the structure read_bag() returned, so every function above
    is untouched by the split -- which is what lets the post-split render match
    the pre-split md5 byte for byte rather than merely look the same. (The JSON
    payloads are lossless too, but so is Parquet's own double; JSON is there
    for ragged nesting, not for precision. See EXTRACT_SCHEMA.md.)
    """
    table = pq.read_table(path)
    columns = {name: table.column(name).to_pylist()
               for name in ('kind', 't', 'payload', 'blob')}

    meta = None
    grids = {}
    static_tf = {}
    raw_samples = {}
    for kind, t, payload, blob in zip(columns['kind'], columns['t'],
                                      columns['payload'], columns['blob']):
        if kind == 'meta':
            meta = json.loads(payload)
        elif kind == 'grid':
            spec = json.loads(payload)
            grids[spec['grid_id']] = np.frombuffer(
                zlib.decompress(blob), dtype=np.int8).reshape(spec['shape'])
        elif kind == 'static_tf':
            spec = json.loads(payload)
            static_tf[spec['child']] = (spec['parent'],
                                        np.array(spec['translation']),
                                        np.array(spec['matrix']))
        elif kind.startswith('sample:'):
            raw_samples.setdefault(kind.split(':', 1)[1], []).append(
                (t, json.loads(payload)))

    if meta is None:
        raise ValueError(f'{path}: no meta row, so this is not a mission extract')

    streams = {}
    for name, max_age in meta['max_age'].items():
        stream = Stream(max_age)
        for t, value in raw_samples.get(name, []):
            stream.add(t, _rehydrate(name, value, grids))
        streams[name] = stream

    return {'streams': streams, 'static_tf': static_tf,
            't0': meta['t0'], 't1': meta['t1'],
            'pose_frame': meta['pose_frame'], 'pose_topic': meta['pose_topic'],
            'legacy_solver_msgs': meta['legacy_solver_msgs'],
            'dropped': meta['dropped'], 'manifest': meta.get('manifest') or {},
            'run_id': meta.get('run_id'),
            'padding': meta.get('padding') or {}}


def _rehydrate(name, value, grids):
    """
    Undo the JSON flattening for the values that are not plain JSON.

    `map` holds a numpy raster, stored once per UNIQUE grid and referenced by
    id here, because a full occupancy raster per tick would dominate the file
    for a picture that changes every few seconds at most. Everything else just
    needs its tuples back: JSON turns them into lists, and draw_frame unpacks
    several of them positionally.
    """
    if name == 'map':
        return (grids[value['grid_id']], tuple(value['extent']))
    if name == 'corridor':
        return {ns: [tuple(pt) for pt in pts] for ns, pts in value.items()}
    if isinstance(value, list):
        return tuple(value)
    return value


def main(argv=None):
    """Render one or more *.extract.parquet files to MP4. No ROS required."""
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('extracts', nargs='+', type=Path,
                    help='*.extract.parquet files written by mission_extract')
    ap.add_argument('--out-dir', type=Path, default=None,
                    help='required unless --frames-dir is given')
    ap.add_argument('--frames-dir', type=Path, default=None,
                    help='debug: dump raw frame_0001.png.. here instead of '
                         "encoding an MP4 -- no video codec in the path, so "
                         "this is what proves the drawing itself is pixel-"
                         "identical across machines (an MP4 md5 can't: VBR "
                         "H.264 is not bit-identical across ffmpeg builds/"
                         "architectures even from identical input). One "
                         "extract at a time: a second run's frames would "
                         "land in the same directory and overwrite the first.")
    ap.add_argument('--dt', type=float, default=0.1,
                    help="resample period [s]; default 0.1 = the MPC's control period")
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--follow', type=float, default=None, metavar='HALF_WIDTH_M')
    ap.add_argument('--trail-seconds', type=float, default=3.0)
    ap.add_argument('--corridor-span', type=float, default=5.0, metavar='M')
    ap.add_argument('--pose-source', choices=('global', 'local'), default='global',
                    help='must match what the extract was written with')
    # default None, NOT a number: the run's own value from the extract is
    # used unless one of these is passed explicitly.
    ap.add_argument('--car-radius', type=float, default=None,
                    help="override this run's own recorded car radius [m]")
    ap.add_argument('--avoidance-margin', type=float, default=None,
                    help="override this run's own recorded avoidance margin [m]")
    ap.add_argument('--dpi', type=int, default=100)
    ap.add_argument('--bitrate', type=int, default=4000)
    ap.add_argument('--no-map', action='store_true')
    ap.add_argument('--no-progress', dest='progress', action='store_false',
                    default=True)
    cfg = ap.parse_args(argv)
    if cfg.frames_dir is None:
        if cfg.out_dir is None:
            ap.error('--out-dir is required unless --frames-dir is given')
        cfg.out_dir = cfg.out_dir.expanduser()
    else:
        cfg.frames_dir = cfg.frames_dir.expanduser()
        if len(cfg.extracts) > 1:
            ap.error('--frames-dir takes one extract at a time '
                     '(frames from each run would overwrite the last)')

    outputs = []
    for path in cfg.extracts:
        bag = read_extract(path)
        label = bag.get('run_id') or path.stem.replace('.extract', '')
        out = render_bag(bag, bag.get('manifest') or {}, label, cfg)
        if out:
            outputs.append(out)
    if outputs:
        print('\nwrote:')
        for out in outputs:
            print(f'  {out}')
    return 0 if outputs else 1


if __name__ == '__main__':
    sys.exit(main())
