#!/usr/bin/env python3
"""Can the office world run a given LLM mission? Answered from the SDF, no simulator.

    sim/isaac/tools/office_missions.py [--sdf PATH] [--grid 0.05] [--json DIR]
                                       [--only M-A,M-B] [--no-tables]

WHY THIS EXISTS NEXT TO pick_spawn_poses.py. That tool answers "where does the car
fit and where can it drive", which is enough to choose a spawn pose and nothing
more. A mission asks harder questions, and all of them are geometric:

  "approach the wall"            is there a wall ahead, far enough that the move
                                 actually drives? guard `wall` stops at thresh,
                                 so the leg is (wall distance - thresh).
  "turn left"                    does the car's own footprint fit through a 90 deg
                                 arc at R_min from the pose the previous move ends in?
  "go to the chair"              is a chair of the world inside the camera's FOV, in
                                 range, and NOT hidden behind a 0.81 m desk as seen
                                 from a lens 0.22 m off the floor?
  "enter the door on the left"   is there an opening in the WALLS (not the SDF's one
                                 Door model, which is a closed leaf) of a width the
                                 door detector admits, on that side, inside
                                 enter_door's 15..135 deg bearing sector, and is it
                                 the one `side` would pick?

Answering them by eye in the Isaac GUI is neither reproducible nor possible on a
machine without Isaac, so they are computed from the same SDF the world is built
from, exactly as the spawn poses are. Re-run after regenerating the world.

WHAT IT BUILDS, all on the lidar plane (base_link + 0.19 m, a 0.3 m band):

  wall     the structural collision geometry only (Surfaces' wall_1/wall_2, the two
           Glass_wall boxes, the Door leaf, the Stairs). This is what a doorway is a
           gap IN, and what guard `wall` measures to.
  obst     every collision geometry, structure and furniture. This is what the car
           must not drive through.
  room     the floor patches, rasterised per triangle and kept SEPARATE: the SDF's
           floor_1..floor_14 are one mesh per room, which is the only room
           decomposition this world carries. A cell with no floor under it is not
           drivable however free it looks (outside the building, where the car falls).
  hmax     the top height of any collision geometry per cell, for the occlusion test.

Mesh bounds come from the mesh files themselves (sdf_to_usd.mesh_triangles), so the
numbers are in the same metres as Gazebo and the generated USD. Geometry is
rasterised triangle by triangle because the office's walls and floors are single
building-wide meshes; primitives use their oriented bounding box, which
OVER-estimates occupancy for round and diagonal furniture -- a pose this tool calls
free is free, a pose it rejects may not be.

NO scipy: the clearance map is a Felzenszwalb exact EDT in numpy, because the Isaac
container has numpy and nothing else of the stack does.

THE STACK NUMBERS IT ENFORCES are read from the real sources, not guessed:

  R_min 0.96 m           mpc_wheelbase_m 0.305 / tan(mpc_steering_angle_min_rad),
                         the worst (right) side; turns are driven at 1.25 x that.
  half width 0.186 m     swept_clearance_body_half_width_m 0.136 + front_beside_body_margin_m 0.05
  front 0.405 m          the URDF's base_front
  wall thresh 3.0 / 2.0  plan_translate.WALL_THRESH_BEFORE_TURN_M / WALL_THRESH_DEFAULT_M
  turn 90 deg            mission_translator_turn_magnitude_deg
  gap chair 0.5 m        object_geometry.gap_min(0.20 + 0.12 + 0.0 + 0.10) -> 0.42, up to 0.1
  gap person 1.0 m       the same with obstacle_class_margin_m {person: 0.5} -> 0.92
  door width 0.6..1.5 m  freespace_doors_width_min/max_detect_m (nominal 0.7..1.4)
  door side 15..135 deg  f1tenth_behavior/behaviours/enter_door.py DOOR_SIDE_MIN/MAX_DEG
  camera                 640x360, fx 262.43, lens at z 0.22 (sim/isaac/config/car_spec.yaml)

WHAT IT DOES NOT ANSWER: whether YOLO actually fires. A chair this tool calls
visible is geometrically visible; whether the renderer puts enough light on it is a
separate question, and `--json` dumps the scene's own sphere_lights next to every
waypoint so the distances can be read off. See output/office_missions/.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sdf_to_usd import (DEFAULT_FUEL, DEFAULT_MODELS, DEFAULT_SDF, mesh_triangles,  # noqa: E402
                        parse_world, resolve_uri)

RES_DEFAULT = 0.05
LIDAR_Z = (0.05, 0.35)          # the scan plane is 0.19 m up; a 0.3 m band around it
HALF_W = 0.186                  # body half width + beside margin
FRONT_X = 0.405                 # base_link -> front bumper (URDF base_front)
REAR_X = -0.15
R_MIN = 0.96                    # worst-side minimum turning radius
TURN_R = R_MIN * 1.25           # radius the 90 deg turn is checked at
WALL_THRESH_BEFORE_TURN = 3.0
WALL_THRESH = 2.0
GAP = {'chair': 0.5, 'person': 1.0}
CAM_Z = 0.22
CAM_FOV_DEG = 50.0              # half of the usable horizontal FOV (full is 101 deg)
CAM_RANGE_M = 7.0
DOOR_DETECT_W = (0.6, 1.5)
DOOR_NOMINAL_W = (0.7, 1.4)
DOOR_SIDE_DEG = (15.0, 135.0)
LIDAR_FOV_DEG = 135.0           # sensors.yaml angle_min/max = +-2.356 rad
PERSON_R = 0.175                # scene person/people collider radius
PERSON_KEEP = PERSON_R + 0.5 + 0.20 + 0.12   # + class margin + car_radius + avoidance

# The structural models: a doorway is a gap in THESE, and guard `wall` measures to them.
STRUCTURE = ('Surfaces', 'Glass_wall_1', 'Glass_wall_2', 'OpenRobotics/Door', 'Stairs')

# Model-name prefix -> the COCO class the detector would report. Only the classes a
# mission can name are mapped; everything else is furniture the car must avoid and
# cannot be asked to drive at.
CLASSES = (
    ('chair', ('ConfOfficeChairBlack', 'DeskChair', 'OfficeChairBlack', 'WoodenChair')),
    ('dining table', ('AdjTable', 'ConferenceTable', 'CoffeeTable', 'Table')),
    ('laptop', ('Laptop',)), ('tv', ('Tv',)), ('refrigerator', ('Fridge',)),
    ('sink', ('BathroomSink',)), ('toilet', ('Toilet',)),
    ('cabinet*', ('WhiteCabinet', 'SquaredShelf', 'Shelf', 'Drawer')),
    ('bin*', ('TrashBin',)), ('cart*', ('MopCart', 'WaterDispenser', 'Kitchen')),
    ('robot*', ('_Rosbot',)),
)

# Human names for the SDF's floor patches, which are this world's only room
# decomposition. Read off the geometry once (floor_N bounds + what stands in them).
ROOM_NAMES = {
    1: 'corridoio SW', 2: 'sala SW', 3: 'ufficio SE', 4: 'corridoio centrale N',
    5: 'bagno NE', 6: 'cucina / sala NE', 7: 'stanza NW', 8: 'sala riunioni W',
    9: 'passaggio W', 10: 'stanza W', 11: 'open space centrale', 12: 'ufficio E',
    13: 'ripostiglio N', 14: 'stanza N-centro',
}

# office.yaml's own fill lights, so a waypoint's distance to the nearest one can be
# reported: a geometrically visible chair in the dark is still not detected.
SPHERE_LIGHTS = ((16.8, 0.0), (16.8, -4.0), (16.8, -8.0), (16.8, -12.0),
                 (15.4, -11.6), (10.0, -19.4), (8.0, -19.4))

# --------------------------------------------------------------------------- the missions
# One entry per campaign command. `plan` is the intent the LLM would have to emit
# (schemas/intent_v1.json) and `steps` is the same thing with the ambiguity removed:
# the door and the go_to target are named explicitly, because this tool checks the
# geometry of ONE chain and `side` / "nearest" resolution is checked separately.
#
# THE IDS HERE ARE NOT THE CAMPAIGN'S. These are the four commands as they were first
# written, plus one variant, kept so the feasibility report stays readable next to the
# question it answered. The campaign that came out of it runs three of them, renumbered
# in its own prompts.yaml (the recorder derives a mission folder from that file alone):
#
#   M-A  -> office_hil_campaign M01_door_left_chair     (Isaac preset campaign_ma)
#   M-B  -> office_hil_campaign M02_wall_left_wall      (Isaac preset campaign_mb)
#   M-D  -> office_hil_campaign M03_person_door_right   (Isaac preset campaign_md)
#   M-A2 variant, not in the campaign: the mirrored door into the NE kitchen
#   M-C  dropped: its last move cannot complete from ANY pose in this world -- go_to
#        stops the car 0.5 m from the chair facing it, so the following "2 metres
#        straight" has an obstacle 0.40 m ahead and times out. That is a property of
#        the command, not of the map, which is why it is still listed and still run.
#        Since 2026-11-23 its target is a PERSON, not a chair: the same person, start
#        and scene as M-D (Isaac preset campaign_md), so M-C and M-D differ only in
#        their last move. With a STANDING person the last move is blocked the same way
#        (gap 1.0 m, and the person leaves 0.45 / 0.80 m beside it on that leg against
#        the 1.18 m needed to pass): the person has to walk away for it to complete.
#
# The campaign texts also drop the ordinal ("the FIRST door on the left"): the planner
# prompt routes an ordinal to `unsupported`, which forces an interactive confirmation.
# `side: left|right` already means the nearest door on that side.
MISSIONS = (
    {'id': 'M-A', 'cmd': 'go straight, enter the first door on the left and then approach the chair',
     'start': (9.33, -15.87, 90.0),
     'plan': [{'mode': 'straight', 'guard': 'distance', 'thresh': 2.6},
              {'mode': 'enter_door', 'side': 'left'},
              {'mode': 'go_to', 'target': 'chair'}],
     'steps': [('straight_dist', 2.6), ('enter_door', 8.08, -13.07, '-x', 'left'),
               ('go_to', 'chair', 'ConfOfficeChairBlack5')]},
    {'id': 'M-A2', 'cmd': 'variant: ... the first door on the RIGHT ... (kitchen NE, 6 chairs)',
     'start': (17.33, -6.37, 90.0),
     'plan': [{'mode': 'straight', 'guard': 'distance', 'thresh': 3.6},
              {'mode': 'enter_door', 'side': 'right'},
              {'mode': 'go_to', 'target': 'chair'}],
     'steps': [('straight_dist', 3.6), ('enter_door', 18.68, -2.52, '+x', 'right'),
               ('go_to', 'chair', 'WoodenChair_4')]},
    {'id': 'M-B', 'cmd': 'go straight at the wall, turn left and then approach the wall',
     'start': (16.83, -9.87, 180.0),
     'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0},
              {'mode': 'turn', 'dir': 'left'},
              {'mode': 'straight', 'guard': 'wall', 'thresh': 2.0}],
     'steps': [('straight_wall', WALL_THRESH_BEFORE_TURN), ('turn', 'left'),
               ('straight_wall', WALL_THRESH)]},
    {'id': 'M-C', 'cmd': 'at the wall turn left, then go to the person, then go straight 2 meters',
     'start': (16.83, -9.87, 180.0),
     'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0},
              {'mode': 'turn', 'dir': 'left'},
              {'mode': 'go_to', 'target': 'person'},
              {'mode': 'straight', 'guard': 'distance', 'thresh': 2.0}],
     'add_person': (11.63, -13.30),     # M-D's person (campaign_md)
     'steps': [('straight_wall', WALL_THRESH_BEFORE_TURN), ('turn', 'left'),
               ('go_to', 'person', None), ('straight_dist', 2.0)]},
    {'id': 'M-D', 'cmd': 'at the wall turn left, then go to the person, then enter the first door on the right',
     'start': (16.83, -9.87, 180.0),
     'plan': [{'mode': 'straight', 'guard': 'wall', 'thresh': 3.0},
              {'mode': 'turn', 'dir': 'left'},
              {'mode': 'go_to', 'target': 'person'},
              {'mode': 'enter_door', 'side': 'right'}],
     # The person does not exist in the SDF: it is added by scene config, and this is
     # where. people.characters with path [[x,y],[x,y]] stands still (people.py's
     # total <= 0 branch), so any number of them can be placed.
     'add_person': (11.63, -13.30),
     'steps': [('straight_wall', WALL_THRESH_BEFORE_TURN), ('turn', 'left'),
               ('go_to', 'person', None), ('enter_door', 8.08, -13.07, '-x', 'right')]},
)

# Where a person may be put on each mission's nominal path, for the obstacle tests:
# (x, y, heading of the leg it sits on [deg]). The heading matters because the only
# gaps that count are the LATERAL ones -- free space straight ahead of or behind the
# person is not a way past.
OBSTACLE_PROBES = {
    'M-A': ((9.33, -15.07, 90.0), (6.78, -13.07, 180.0), (5.78, -13.07, 180.0)),
    'M-A2': ((17.33, -5.37, 90.0), (17.33, -4.37, 90.0), (20.48, -2.52, 0.0)),
    'M-B': ((14.83, -9.87, 180.0), (11.63, -12.57, -90.0), (11.63, -14.07, -90.0),
            (11.63, -17.07, -90.0)),
    'M-C': ((14.83, -9.87, 180.0),),
    'M-D': ((14.83, -9.87, 180.0),),
}


# --------------------------------------------------------------------------- the grids
class Grid:
    """The office rasterised on the lidar plane, plus the floor and height fields."""

    def __init__(self, models, models_dir: str, fuel_cache: str, res: float) -> None:
        self.res = res
        lo = np.array([1e9, 1e9])
        hi = np.array([-1e9, -1e9])
        pieces = []                      # (lo, hi, model, link, is_struct)
        tri_cache: dict = {}
        for m in models:
            is_struct = m.name in STRUCTURE
            for link in m.links:
                for g in link.geoms:
                    if g.kind != 'collision':
                        continue
                    tf = m.pose @ link.pose @ g.pose
                    box = self._bounds(g, tf, models_dir, fuel_cache, tri_cache)
                    if box is None:
                        continue
                    b_lo, b_hi = box
                    pieces.append((b_lo, b_hi, m.name, link.name, is_struct))
                    lo = np.minimum(lo, b_lo.min(axis=0)[:2])
                    hi = np.maximum(hi, b_hi.max(axis=0)[:2])
        if not pieces:
            raise SystemExit('no collision geometry found')
        self.origin = lo - 0.2
        nx = int(math.ceil((hi[0] - self.origin[0] + 0.2) / res)) + 1
        ny = int(math.ceil((hi[1] - self.origin[1] + 0.2) / res)) + 1
        self.shape = (nx, ny)
        self.wall = np.zeros(self.shape, bool)
        self.obst = np.zeros(self.shape, bool)
        self.room = np.zeros(self.shape, np.int16)
        self.hmax = np.zeros(self.shape, float)
        self.objects: list[dict] = []
        extents: dict = {}
        for b_lo, b_hi, name, link, is_struct in pieces:
            is_floor = is_struct and link.startswith('floor')
            if is_floor:
                rid = int(link.split('_')[1])
                for p_lo, p_hi in zip(b_lo, b_hi):
                    self._mark(self.room, p_lo, p_hi, rid, only_empty=True)
                continue
            keep = (b_hi[:, 2] >= LIDAR_Z[0]) & (b_lo[:, 2] <= LIDAR_Z[1])
            for p_lo, p_hi in zip(b_lo, b_hi):
                if p_hi[2] > 0.02:
                    self._mark(self.hmax, p_lo, p_hi, p_hi[2], maximum=True)
            for p_lo, p_hi in zip(b_lo[keep], b_hi[keep]):
                self._mark(self.obst, p_lo, p_hi, True)
                if is_struct:
                    self._mark(self.wall, p_lo, p_hi, True)
            if not is_struct:
                e = extents.setdefault(name, [b_lo.min(axis=0), b_hi.max(axis=0)])
                e[0] = np.minimum(e[0], b_lo.min(axis=0))
                e[1] = np.maximum(e[1], b_hi.max(axis=0))
        for name, (e_lo, e_hi) in extents.items():
            c = ((e_lo[0] + e_hi[0]) / 2.0, (e_lo[1] + e_hi[1]) / 2.0)
            self.objects.append({
                'name': name, 'short': name.split('/')[-1], 'cls': self._cls(name),
                'lo': [round(float(v), 3) for v in e_lo], 'hi': [round(float(v), 3) for v in e_hi],
                'c': [round(c[0], 3), round(c[1], 3)], 'room': int(self.room_at(*c))})
        self.inside = self.room > 0
        self.clear_obst = edt(self.obst, res)
        self.clear_wall = edt(self.wall, res)

    # ---- geometry
    @staticmethod
    def _bounds(g, tf, models_dir, fuel_cache, tri_cache):
        """Per-piece world AABBs: one row per triangle for a mesh, one row otherwise."""
        if g.gtype == 'mesh':
            if g.uri not in tri_cache:
                path, _ = resolve_uri(g.uri, models_dir, fuel_cache)
                tri_cache[g.uri] = mesh_triangles(path) if os.path.isfile(path) else None
            tris = tri_cache[g.uri]
            if tris is None:
                return None
            v = tris * np.array(g.scale, dtype=float)
            w = (tf[:3, :3] @ v.reshape(-1, 3).T).T + tf[:3, 3]
            w = w.reshape(-1, 3, 3)
            return w.min(axis=1), w.max(axis=1)
        if g.gtype == 'box':
            s = np.array(g.size, dtype=float) / 2.0
            mn, mx = -s, s
        elif g.gtype in ('cylinder', 'sphere'):
            r = float(g.size[0]) if g.size else 0.5
            h = float(g.size[1]) if g.gtype == 'cylinder' and len(g.size) > 1 else 2 * r
            mn, mx = np.array([-r, -r, -h / 2]), np.array([r, r, h / 2])
        elif g.gtype == 'plane':
            s = np.array([g.size[0] if g.size else 20.0,
                          g.size[1] if len(g.size) > 1 else 20.0]) / 2.0
            mn, mx = np.array([-s[0], -s[1], -0.01]), np.array([s[0], s[1], 0.01])
        else:
            return None
        corners = np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1])
                            for z in (mn[2], mx[2])])
        w = (tf[:3, :3] @ corners.T).T + tf[:3, 3]
        return w.min(axis=0)[None, :], w.max(axis=0)[None, :]

    @staticmethod
    def _cls(name: str) -> str:
        base = name.split('/')[-1]
        for cls, prefixes in CLASSES:
            if any(base.startswith(p) for p in prefixes):
                return cls
        return 'other'

    def _mark(self, field, lo, hi, value, maximum: bool = False, only_empty: bool = False) -> None:
        i0, j0 = self.ij(lo[0], lo[1])
        i1, j1 = self.ij(hi[0], hi[1])
        if i1 < i0 or j1 < j0:
            return
        sub = field[i0:i1 + 1, j0:j1 + 1]
        if maximum:
            np.maximum(sub, value, out=sub)
        elif only_empty:
            sub[sub == 0] = value
        else:
            sub[:] = value

    def ij(self, x, y):
        i = int(np.clip((x - self.origin[0]) / self.res, 0, self.shape[0] - 1))
        j = int(np.clip((y - self.origin[1]) / self.res, 0, self.shape[1] - 1))
        return i, j

    def xy(self, i, j):
        return self.origin[0] + (i + 0.5) * self.res, self.origin[1] + (j + 0.5) * self.res

    def at(self, field, x, y):
        i, j = self.ij(x, y)
        return field[i, j]

    def room_at(self, x, y) -> int:
        return int(self.at(self.room, x, y))

    # ---- queries the primitives are built from
    def ray(self, field, x, y, yaw, maxd=25.0, half=HALF_W, from_x=FRONT_X):
        """Distance from the FRONT BUMPER to the first `field` cell in the swept corridor."""
        c, s = math.cos(yaw), math.sin(yaw)
        offs = np.linspace(-half, half, 7)
        for k in range(int(maxd / self.res) + 1):
            r = from_x + k * self.res
            for o in offs:
                if self.at(field, x + r * c - o * s, y + r * s + o * c):
                    return k * self.res
        return math.inf

    def strip_free(self, x, y, yaw, length, half=HALF_W, from_x=REAR_X) -> bool:
        """Is the rectangle from from_x to from_x+length (body frame) free of obstacle?"""
        c, s = math.cos(yaw), math.sin(yaw)
        for k in range(int(max(0.0, length) / self.res) + 1):
            r = from_x + k * self.res
            for o in np.linspace(-half, half, 7):
                if self.at(self.obst, x + r * c - o * s, y + r * s + o * c):
                    return False
        return True

    def turn90(self, x, y, yaw, sign, radius=TURN_R):
        """Sweep the car's footprint through a 90 deg arc. sign +1 = left (CCW)."""
        c, s = math.cos(yaw), math.sin(yaw)
        cx, cy = x - sign * radius * s, y + sign * radius * c
        ok = True
        for t in np.linspace(0.0, math.pi / 2, 19):
            a = yaw + sign * t
            px = cx + sign * radius * math.sin(a)
            py = cy - sign * radius * math.cos(a)
            ca, sa = math.cos(a), math.sin(a)
            for r in np.linspace(REAR_X, FRONT_X, 6):
                for o in np.linspace(-HALF_W, HALF_W, 5):
                    qx, qy = px + r * ca - o * sa, py + r * sa + o * ca
                    if self.at(self.obst, qx, qy) or not self.at(self.inside, qx, qy):
                        ok = False
        a = yaw + sign * math.pi / 2
        return ok, cx + sign * radius * math.sin(a), cy - sign * radius * math.cos(a), a

    def sees(self, x, y, yaw, tx, ty, top=1.0):
        """(range, bearing_deg, blocking_height or None) for a target of height `top`.

        The occlusion test is HEIGHT AWARE on purpose: the lens is 0.22 m up, so a
        0.81 m desk halfway to a 1.26 m chair hides it, while the same desk right
        next to the chair does not. A plain 2D line of sight gets this wrong in both
        directions.
        """
        d = math.hypot(tx - x, ty - y)
        b = math.degrees((math.atan2(ty - y, tx - x) - yaw + math.pi) % (2 * math.pi) - math.pi)
        if d < 1e-6:
            return 0.0, 0.0, None
        for k in range(1, int(d / self.res)):
            f = k * self.res / d
            if f > 0.85:                 # the last stretch is the target itself
                break
            h = float(self.at(self.hmax, x + f * (tx - x), y + f * (ty - y)))
            if h > CAM_Z + (top - CAM_Z) * f + 0.02:
                return round(d, 2), round(b, 1), round(h, 2)
        return round(d, 2), round(b, 1), None

    def visible_from(self, x, y, tx, ty, field=None) -> bool:
        """Plain 2D line of sight against `field` (default: the walls)."""
        field = self.wall if field is None else field
        d = math.hypot(tx - x, ty - y)
        for k in range(int(d / self.res)):
            f = min(k * self.res / d, max(0.0, (d - 0.3) / d))
            if self.at(field, x + f * (tx - x), y + f * (ty - y)):
                return False
        return True

    def free_run(self, x, y, nx, ny, maxd=4.0) -> float:
        """Free distance from (x,y) along (nx,ny) until obstacle or off-floor."""
        d = 0.0
        while d < maxd:
            d += self.res
            px, py = x + nx * d, y + ny * d
            if self.at(self.obst, px, py) or not self.at(self.inside, px, py):
                return round(d - self.res, 2)
        return maxd


def edt(mask: np.ndarray, res: float) -> np.ndarray:
    """Exact euclidean distance to the nearest True cell (Felzenszwalb), in metres.

    scipy.ndimage.distance_transform_edt would do this in one line and is NOT
    available in the Isaac container.
    """
    inf = 1e12

    def dt1(fr):
        n = len(fr)
        d = np.empty(n)
        v = np.zeros(n, dtype=int)
        z = np.empty(n + 1)
        k = 0
        z[0], z[1] = -inf, inf
        for q in range(1, n):
            while True:
                s = ((fr[q] + q * q) - (fr[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
                if s <= z[k]:
                    k -= 1
                else:
                    break
            k += 1
            v[k], z[k], z[k + 1] = q, s, inf
        k = 0
        for q in range(n):
            while z[k + 1] < q:
                k += 1
            d[q] = (q - v[k]) ** 2 + fr[v[k]]
        return d

    f = np.where(mask, 0.0, inf)
    for i in range(f.shape[0]):
        f[i] = dt1(f[i])
    for j in range(f.shape[1]):
        f[:, j] = dt1(f[:, j])
    return np.sqrt(f) * res


# --------------------------------------------------------------------------- doorways
def openings(grid: Grid, width_range=(0.5, 2.6)) -> list[dict]:
    """Openings in the WALLS: a gap with a jamb on each side and floor through it.

    NOT the SDF's `Door` model: that leaf is CLOSED and a flight of stairs stands
    0.6 m in front of it, so for every sensor it is wall. The doors a car can drive
    at are the gaps left between the wall meshes, which is also the geometry
    f1tenth_costmap's detectors are written for (two jambs and a run of beams that
    reaches further).
    """
    nx, ny = grid.shape
    res = grid.res
    free = (~grid.wall) & grid.inside

    def run(i, j, di, dj, limit):
        n = 0
        while n * res < limit:
            i += di
            j += dj
            if not (0 <= i < nx and 0 <= j < ny) or grid.wall[i, j]:
                break
            n += 1
        return n * res

    cands: dict = {}
    for axis in (0, 1):
        jd = (1, 0) if axis == 0 else (0, 1)        # jambs across this
        td = (0, 1) if axis == 0 else (1, 0)        # passage along this
        for i in range(1, nx - 1):
            for j in range(1, ny - 1):
                if not free[i, j]:
                    continue
                d_pos = run(i, j, *jd, limit=3.0)
                d_neg = run(i, j, -jd[0], -jd[1], limit=3.0)
                if d_pos >= 3.0 or d_neg >= 3.0:    # no second jamb: not a gap
                    continue
                width = d_pos + d_neg + res
                if not (width_range[0] <= width <= width_range[1]):
                    continue
                f1 = run(i, j, *td, limit=3.0)
                f2 = run(i, j, -td[0], -td[1], limit=3.0)
                if min(f1, f2) < 1.2:               # nothing to drive into
                    continue
                ci = i + int(round((d_pos - d_neg) / 2 / res)) * jd[0]
                cj = j + int(round((d_pos - d_neg) / 2 / res)) * jd[1]
                x, y = grid.xy(ci, cj)
                s = int(1.2 / res)
                a = (ci + td[0] * s, cj + td[1] * s)
                b = (ci - td[0] * s, cj - td[1] * s)
                if not (0 <= a[0] < nx and 0 <= a[1] < ny and 0 <= b[0] < nx and 0 <= b[1] < ny):
                    continue
                key = (round(x * 2) / 2, round(y * 2) / 2, axis)
                rec = {'x': round(x, 2), 'y': round(y, 2), 'w': round(width, 2),
                       'through': 'y' if axis == 0 else 'x',
                       'rooms': sorted({int(grid.room[a]), int(grid.room[b])}),
                       'clear_in': round(float(min(grid.clear_obst[a], grid.clear_obst[b])), 2)}
                if key not in cands or rec['w'] < cands[key]['w']:
                    cands[key] = rec
    out: list[dict] = []
    for r in sorted(cands.values(), key=lambda d: (d['through'], d['x'], d['y'])):
        for prev in out:
            if (prev['through'] == r['through'] and abs(prev['x'] - r['x']) < 1.2
                    and abs(prev['y'] - r['y']) < 1.2):
                prev['w'] = min(prev['w'], r['w'])
                prev['clear_in'] = max(prev['clear_in'], r['clear_in'])
                break
        else:
            out.append(dict(r))
    for r in out:
        r['detectable'] = bool(DOOR_DETECT_W[0] <= r['w'] <= DOOR_DETECT_W[1])
        r['nominal'] = bool(DOOR_NOMINAL_W[0] <= r['w'] <= DOOR_NOMINAL_W[1])
        r['drivable'] = bool(r['w'] >= 0.55 and r['clear_in'] >= 0.30)
    return out


def door_side_pick(grid: Grid, ops, x, y, yaw, side):
    """Which opening enter_door{side} resolves to, and the whole visible list.

    enter_door picks the nearest STRONG door whose bearing off the nose is inside
    DOOR_SIDE_MIN_DEG..DOOR_SIDE_MAX_DEG on that side (enter_door.py). The wall
    detectors publish every confirmed door strong, so "strong" here is "seen from
    two poses >= 1.0 m apart", which any straight leg before the move provides.
    """
    rows = []
    for o in ops:
        if not o['detectable']:
            continue
        d = math.hypot(o['x'] - x, o['y'] - y)
        b = math.degrees((math.atan2(o['y'] - y, o['x'] - x) - yaw + math.pi) % (2 * math.pi) - math.pi)
        if d > 9.0 or abs(b) > LIDAR_FOV_DEG:
            continue
        if not grid.visible_from(x, y, o['x'], o['y']):
            continue
        rows.append({'x': o['x'], 'y': o['y'], 'w': o['w'], 'd': round(d, 2), 'bearing': round(b)})
    rows.sort(key=lambda r: r['d'])
    lo, hi = DOOR_SIDE_DEG
    if side == 'left':
        sector = [r for r in rows if lo <= r['bearing'] <= hi]
    else:
        sector = [r for r in rows if -hi <= r['bearing'] <= -lo]
    return (sector[0] if sector else None), rows


# --------------------------------------------------------------------------- mission chain
def run_mission(grid: Grid, ops, mission, out=print) -> dict:
    """Walk one mission's primitive chain and report every step.

    The mission's add_person is an obstacle for the whole chain (its collider, PERSON_R): it is
    in no SDF, so without this a move AFTER go_to person would drive straight through it.
    """
    if not mission.get('add_person'):
        return _run_mission(grid, ops, mission, out)
    px, py = mission['add_person']
    saved = grid.obst.copy()
    try:
        i0, j0 = grid.ij(px - PERSON_R, py - PERSON_R)
        i1, j1 = grid.ij(px + PERSON_R, py + PERSON_R)
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                cx, cy = grid.xy(i, j)
                if math.hypot(cx - px, cy - py) <= PERSON_R + grid.res / 2:
                    grid.obst[i, j] = True
        return _run_mission(grid, ops, mission, out)
    finally:
        grid.obst = saved


def _run_mission(grid: Grid, ops, mission, out=print) -> dict:
    by_short = {o['short']: o for o in grid.objects}
    x, y, yaw = mission['start'][0], mission['start'][1], math.radians(mission['start'][2])
    out(f"\n### {mission['id']}  {mission['cmd']}")
    out(f"    start ({x:.2f}, {y:.2f}) yaw {mission['start'][2]:.0f} deg"
        f"  room {mission['start'] and grid.room_at(x, y)} "
        f"({ROOM_NAMES.get(grid.room_at(x, y), '?')})")
    if mission.get('add_person'):
        out(f"    requires a person at {mission['add_person']} (none in the SDF: add it "
            f"through scene config)")
    ok_all, waypoints = True, [(round(x, 2), round(y, 2))]
    for step in mission['steps']:
        kind = step[0]
        if kind == 'straight_wall':
            thresh = step[1]
            d_wall = grid.ray(grid.wall, x, y, yaw)
            d_obst = grid.ray(grid.obst, x, y, yaw)
            leg = d_wall - thresh
            clear = grid.strip_free(x, y, yaw, leg) if leg > 0 else False
            ok = math.isfinite(d_wall) and leg > 0.5 and clear
            out(f"  straight -> wall at {thresh} m : wall {d_wall:.2f} m, nearest obstacle "
                f"{d_obst:.2f} m, drive {leg:.2f} m, corridor clear {clear}  "
                f"=> {'OK' if ok else 'FAIL'}")
            if not ok:
                ok_all = False
                break
            x, y = x + leg * math.cos(yaw), y + leg * math.sin(yaw)
        elif kind == 'straight_dist':
            n = step[1]
            clear = grid.strip_free(x, y, yaw, n)
            d_obst = grid.ray(grid.obst, x, y, yaw)
            out(f"  straight {n} m : nearest obstacle ahead {d_obst:.2f} m, corridor clear "
                f"{clear}  => {'OK' if clear else 'FAIL'}")
            if not clear:
                ok_all = False
                break
            x, y = x + n * math.cos(yaw), y + n * math.sin(yaw)
        elif kind == 'turn':
            sign = 1 if step[1] == 'left' else -1
            ok, x2, y2, yaw2 = grid.turn90(x, y, yaw, sign)
            out(f"  turn {step[1]} 90 deg (R={TURN_R:.2f} m) : swept footprint clear {ok} -> "
                f"({x2:.2f}, {y2:.2f}) yaw {math.degrees(yaw2):.0f}  => {'OK' if ok else 'FAIL'}")
            if not ok:
                ok_all = False
                break
            x, y, yaw = x2, y2, yaw2
        elif kind == 'go_to':
            cls, name = step[1], step[2]
            if cls == 'person':
                tx, ty = mission['add_person']
                top = 1.6
            else:
                obj = by_short[name]
                tx, ty = obj['c']
                top = min(1.0, obj['hi'][2])
            d, b, blocker = grid.sees(x, y, yaw, tx, ty, top)
            in_fov = abs(b) <= CAM_FOV_DEG
            travel = max(0.0, d - 0.30 - GAP[cls] - FRONT_X)
            approach = math.atan2(ty - y, tx - x)
            clear = grid.strip_free(x, y, approach, travel)
            ok = in_fov and blocker is None and d <= CAM_RANGE_M and clear
            out(f"  go_to {cls} {name or ''} @({tx:.2f},{ty:.2f}) : range {d} m, bearing {b} deg "
                f"(FOV +-{CAM_FOV_DEG:.0f}: {in_fov}), occluder {blocker}, approach {travel:.2f} m "
                f"clear {clear}  => {'OK' if ok else 'FAIL'}")
            if not ok:
                ok_all = False
            yaw = approach
            x, y = x + travel * math.cos(yaw), y + travel * math.sin(yaw)
        elif kind == 'enter_door':
            dx, dy, through, side = step[1], step[2], step[3], step[4]
            picked, rows = door_side_pick(grid, ops, x, y, yaw, side)
            right = picked is not None and abs(picked['x'] - dx) < 0.4 and abs(picked['y'] - dy) < 0.4
            d = math.hypot(dx - x, dy - y)
            out(f"  enter_door side={side} : {len(rows)} detectable door(s) visible; "
                f"picks {('(%.2f,%.2f) at %.2f m, bearing %d deg' % (picked['x'], picked['y'], picked['d'], picked['bearing'])) if picked else 'NOTHING (door_not_found)'}"
                f"  => {'OK, that is the intended one' if right else 'WRONG door / none'}")
            for r in rows:
                sector = ''
                if DOOR_SIDE_DEG[0] <= abs(r['bearing']) <= DOOR_SIDE_DEG[1]:
                    sector = '  [%s sector]' % ('left' if r['bearing'] > 0 else 'right')
                out(f"       ({r['x']:6.2f},{r['y']:7.2f}) w{r['w']:.2f}  {r['d']:4.2f} m  "
                    f"bearing {r['bearing']:4d} deg{sector}")
            if not right:
                ok_all = False
            tyaw = {'+x': 0.0, '-x': math.pi, '+y': math.pi / 2, '-y': -math.pi / 2}[through]
            out(f"       the move ENDS IN the opening ({dx}, {dy}) facing {through}, "
                f"{d:.2f} m away, not beyond it")
            x, y, yaw = dx, dy, tyaw
        waypoints.append((round(x, 2), round(y, 2)))
        out(f"       pose now ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f}")
    for (wx, wy) in waypoints:
        near = min(math.hypot(wx - lx, wy - ly) for lx, ly in SPHERE_LIGHTS)
        if near >= 5.0:
            out(f"  LIGHT: waypoint ({wx}, {wy}) is {near:.1f} m from the nearest sphere_light "
                f"-- add one, or the camera has nothing to detect")
    out(f"  == {mission['id']}: {'FEASIBLE' if ok_all else 'NOT feasible as written'}")
    return {'id': mission['id'], 'feasible': ok_all, 'waypoints': waypoints}


def obstacle_probes(grid: Grid, mission, out=print) -> list[dict]:
    """Can the car get past a person standing at each probe point of this mission?"""
    need = PERSON_KEEP + HALF_W
    rows = []
    for (px, py, head_deg) in OBSTACLE_PROBES.get(mission['id'], ()):
        yaw = math.radians(head_deg)
        left = (-math.sin(yaw), math.cos(yaw))
        d_left = grid.free_run(px, py, left[0], left[1])
        d_right = grid.free_run(px, py, -left[0], -left[1])
        sides = [s for s, d in (('left', d_left), ('right', d_right)) if d >= need]
        rows.append({'xy': [px, py], 'heading_deg': head_deg, 'left_m': d_left,
                     'right_m': d_right, 'need_m': round(need, 2), 'pass_on': sides})
        verdict = ('pass on ' + ' and '.join(f'{s} ({(d_left if s == "left" else d_right) - need:.2f} m spare)'
                                             for s in sides)) if sides \
            else 'BLOCKS (neither side reaches %.2f m)' % need
        out(f"  person at ({px:6.2f},{py:7.2f}) on the {head_deg:.0f} deg leg: "
            f"free left {d_left:4.2f} / right {d_right:4.2f}  -> {verdict}")
    return rows


# --------------------------------------------------------------------------- reports
def print_inventory(grid: Grid, out=print) -> None:
    by_room: dict = {}
    for o in grid.objects:
        by_room.setdefault(o['room'], []).append(o)
    out('\n## objects per room (SDF model names; no object carries a semantic label)')
    for rid in sorted(by_room):
        counts: dict = {}
        for o in by_room[rid]:
            counts.setdefault(o['cls'], []).append(o['short'])
        out(f"  room {rid:2d} {ROOM_NAMES.get(rid, '?'):22s} " +
            ', '.join(f'{c} x{len(v)}' for c, v in sorted(counts.items())))
    chairs = [o for o in grid.objects if o['cls'] == 'chair']
    out(f'\n## the {len(chairs)} chairs (the only go_to targets this world actually has)')
    for o in sorted(chairs, key=lambda d: (d['room'], d['c'][0])):
        out(f"  {o['short']:24s} ({o['c'][0]:6.2f},{o['c'][1]:7.2f})  h {o['hi'][2]:.2f} m  "
            f"room {o['room']:2d} {ROOM_NAMES.get(o['room'], '?')}")


def print_openings(ops, out=print) -> None:
    out(f'\n## {len(ops)} wall openings (detect band {DOOR_DETECT_W[0]}-{DOOR_DETECT_W[1]} m, '
        f'nominal {DOOR_NOMINAL_W[0]}-{DOOR_NOMINAL_W[1]})')
    for o in sorted(ops, key=lambda d: (d['x'], d['y'])):
        flags = []
        if o['detectable']:
            flags.append('detectable' if o['nominal'] else 'detectable/marginal')
        else:
            flags.append('NOT detectable')
        flags.append('drivable' if o['drivable'] else 'not drivable')
        rooms = ' <-> '.join(ROOM_NAMES.get(r, f'#{r}') for r in o['rooms'])
        out(f"  ({o['x']:6.2f},{o['y']:7.2f}) w {o['w']:.2f} m  through {o['through']}  "
            f"{rooms:42s} {', '.join(flags)}")


def write_json(grid: Grid, ops, results, out_dir: str) -> None:
    """Dump what output/office_missions/office_hil.html reads."""
    os.makedirs(out_dir, exist_ok=True)
    res = grid.res
    ox, oy = grid.origin
    nx, ny = grid.shape

    def runs(mask, cell):
        step = max(1, int(round(cell / res)))
        got = []
        for j in range(0, ny, step):
            row = mask[:, j:j + step].any(axis=1)
            i = 0
            while i < nx:
                if row[i]:
                    k = i
                    while k < nx and row[k:min(nx, k + step)].any():
                        k += step
                    got.append([round(ox + i * res, 2), round(oy + j * res, 2),
                                round((k - i) * res, 2), round(step * res, 2)])
                    i = k
                else:
                    i += step
        return got

    centers = {}
    for rid in range(1, 15):
        idx = np.argwhere(grid.room == rid)
        if len(idx):
            centers[str(rid)] = [round(float(ox + idx[:, 0].mean() * res), 2),
                                 round(float(oy + idx[:, 1].mean() * res), 2)]
    geo = {
        'bounds': [round(float(ox), 2), round(float(oy), 2),
                   round(float(ox + nx * res), 2), round(float(oy + ny * res), 2)],
        'floor': runs(grid.inside & ~grid.wall, 0.20),
        'wall': runs(grid.wall, 0.10),
        'objects': [{'n': o['short'], 'c': o['cls'], 'lo': [round(o['lo'][0], 2), round(o['lo'][1], 2)],
                     'hi': [round(o['hi'][0], 2), round(o['hi'][1], 2)],
                     'h': round(o['hi'][2], 2), 'r': o['room']} for o in grid.objects],
        'openings': ops,
        'lights': [list(p) for p in SPHERE_LIGHTS],
        'rooms': {str(k): v for k, v in ROOM_NAMES.items()},
        'room_centers': centers,
    }
    with open(os.path.join(out_dir, 'map_geo.json'), 'w') as f:
        json.dump(geo, f, separators=(',', ':'))
    with open(os.path.join(out_dir, 'results.json'), 'w') as f:
        json.dump({'missions': results,
                   'free_area_m2': round(float((grid.inside & ~grid.obst).sum()) * res * res, 1)},
                  f, indent=1)
    print(f"\nwrote map_geo.json and results.json to {out_dir}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sdf', default=DEFAULT_SDF)
    ap.add_argument('--models-dir', default=DEFAULT_MODELS)
    ap.add_argument('--fuel-cache', default=DEFAULT_FUEL)
    ap.add_argument('--grid', type=float, default=RES_DEFAULT, help='grid resolution [m]')
    ap.add_argument('--json', default='', help='write map_geo.json / results.json here')
    ap.add_argument('--only', default='', help='comma-separated mission ids')
    ap.add_argument('--no-tables', action='store_true', help='skip the inventory and door tables')
    args = ap.parse_args()

    models, meta = parse_world(args.sdf)
    grid = Grid(models, args.models_dir, args.fuel_cache, args.grid)
    print(f"{meta['world']}: {meta['models']} models, grid {grid.shape} at {grid.res} m, "
          f"origin ({grid.origin[0]:.2f}, {grid.origin[1]:.2f})")
    print(f"drivable floor {float((grid.inside & ~grid.obst).sum()) * grid.res ** 2:.1f} m2, "
          f"{len(grid.objects)} furniture models, "
          f"{len([o for o in grid.objects if o['cls'] == 'chair'])} chairs, "
          f"{len([o for o in grid.objects if o['cls'] == 'person'])} people")
    ops = openings(grid)
    if not args.no_tables:
        print_inventory(grid)
        print_openings(ops)
    want = {s.strip() for s in args.only.split(',') if s.strip()}
    results = []
    for m in MISSIONS:
        if want and m['id'] not in want:
            continue
        results.append(run_mission(grid, ops, m))
        if OBSTACLE_PROBES.get(m['id']):
            print(f"  -- a person on this path (keep-out {PERSON_KEEP:.2f} m + half width "
                  f"{HALF_W} m = {PERSON_KEEP + HALF_W:.2f} m of gap needed to slip past):")
            results[-1]['obstacles'] = obstacle_probes(grid, m)
    if args.json:
        write_json(grid, ops, results, args.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
