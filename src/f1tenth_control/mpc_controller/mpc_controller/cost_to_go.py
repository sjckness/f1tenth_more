"""
Wall-aware terminal cost-to-go V(x, y, theta, gear) on a local lattice.

Pure Python + numpy/scipy (no rclpy). reverse_mpc.ReversePlanner uses it as
every direction candidate's terminal cost and to score its predicted x_N.

WHAT V IS. The cost of the cheapest drivable path from (x, y, theta), with
the car last moving in `gear`, to the goal: a backward Dijkstra over motion
primitives on a lattice of 0.10 m cells x 36 headings inside a square window
(4 x 4 m by default) around the car and the goal. Edge cost is the
primitive's length, times k_rev when it is driven in reverse, plus
gear_switch_cost when it changes direction -- so the GEAR is part of the
state (two lattice copies), which is what lets a direction change cost
something. A state that cannot reach the goal gets unreachable_cost, a large
FINITE number, so a candidate ending there still has a score.

PRIMITIVES. From each heading bin, forward and reverse at steering {-max,
-max/2, 0, +max/2, +max} (the real, asymmetric limits from config), each
`primitive_length_m` long, integrated with the MPC's own model
(f1tenth_state_fcn_dt_beta) and snapped to the nearest cell and heading bin.
They are computed once per configuration.

COLLISION. One occupancy grid from the wall segments, the boundary
half-planes (the same source the MPC's boundary rows use), the detected
obstacle disks and, optionally, everything outside the corridor; then one
configuration-space slice per heading bin by correlating that grid with the
car's footprint (bumper to bumper, body half-width, plus a margin) rotated
into the bin. A primitive is valid when its start, its end and every sample
along it are free in the slice of the heading at that sample.

VECTORISED. Edges are built per primitive with shifted boolean slices (no
Python loop over cells), duplicates are reduced to their cheapest, and the
search is scipy.sparse.csgraph.dijkstra from one super-source wired to the
goal states. The build time is measured and reported (build_ms); if it is
too slow on the Orin the right move is a compiled implementation (C++ or
Rust, see output/reverse/report.md), not a coarser lattice.

QUERY. value() interpolates trilinearly in (x, y, theta) with theta wrapping;
outside the window it clamps to the window and adds the distance outside.
gradient() is a central difference over grad_step_m / one heading bin, wide
enough to see through the lattice's snapping noise.
"""

from dataclasses import dataclass
import math
import time
from typing import Dict, List, Optional, Tuple

from mpc_controller.vehicle_model import f1tenth_state_fcn_dt_beta
import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

__all__ = ['CostToGo', 'LatticeConfig', 'LatticeField', 'build_field',
           'occupancy_grid', 'window_center']

FORWARD_GEAR = 0
REVERSE_GEAR = 1


@dataclass(frozen=True)
class LatticeConfig:
    """Lattice, cost and geometry parameters (MPC_corr fills them from params)."""

    window_m: float = 4.0
    resolution_m: float = 0.10
    n_theta: int = 36
    primitive_length_m: float = 0.30
    k_rev: float = 3.0
    gear_switch_cost: float = 1.0
    # Extra cost per metre at full lock, pro rata |steer| / max: every
    # primitive is the same length, so without it a zigzag through the
    # lattice costs exactly what a straight line does and the greedy policy
    # breaks the tie at random (measured: a wiggling path in an empty
    # corridor). Small against k_rev and gear_switch_cost, so it only picks
    # between equally long manoeuvres.
    steer_cost_per_m: float = 0.15
    unreachable_cost: float = 50.0
    goal_tolerance_m: float = 0.15
    goal_heading_tol_rad: float = math.radians(15.0)
    # Footprint, metres from base_link along / across the heading.
    front_x: float = 0.405
    rear_x: float = -0.082
    half_width: float = 0.136
    footprint_margin_m: float = 0.10
    wheelbase: float = 0.305
    lr: float = 0.17
    steer_min: float = -0.309
    steer_max: float = 0.319
    sample_step_m: float = 0.05
    # Central-difference step of gradient(): one primitive length, so the
    # lattice's period cancels (see build_field's ALIASING note).
    grad_step_m: float = 0.30
    # Window placement: the car keeps at least this much window around it.
    car_edge_margin_m: float = 1.2
    out_of_window_cost_per_m: float = 1.0


def window_center(car_xy, goal_xy, cfg: LatticeConfig) -> Tuple[float, float]:
    """
    Return the window centre: the car-goal midpoint, or as far toward the goal as allowed.

    The car must stay car_edge_margin_m inside the window (room to reverse or
    turn); when the goal is too far for both to fit, the window slides from
    the car toward it and the goal is reached through the window's border.
    """
    car = np.asarray(car_xy, dtype=float)
    goal = np.asarray(goal_xy, dtype=float)
    d = goal - car
    dist = float(np.hypot(*d))
    reach = max(cfg.window_m / 2.0 - cfg.car_edge_margin_m, 0.0)
    if dist < 1e-9:
        return float(car[0]), float(car[1])
    step = min(dist / 2.0, reach)
    c = car + d / dist * step
    return float(c[0]), float(c[1])


def occupancy_grid(origin, shape, res, segments=(), halfplanes=(), circles=(),
                   corridor=None, segment_thickness=None) -> np.ndarray:
    """
    Rasterise the obstacles onto the cell centres of the window.

    origin is the centre of cell (0, 0); shape (nx, ny); x indexes axis 0.
    segments ((x1, y1), (x2, y2)) are walls; halfplanes (nx, ny, offset) mark
    n . p > offset as occupied (the MPC's boundary convention, free side
    n . p <= offset); circles (x, y, r). corridor, when given, marks every
    cell OUTSIDE its nearest-index slab occupied (the QP's own lateral rule).
    """
    nx, ny = shape
    xs = origin[0] + np.arange(nx) * res
    ys = origin[1] + np.arange(ny) * res
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    occ = np.zeros(shape, dtype=bool)
    thick = (res * math.sqrt(0.5)) if segment_thickness is None else segment_thickness
    for (x1, y1), (x2, y2) in segments:
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        if L2 < 1e-12:
            t = np.zeros_like(X)
        else:
            t = np.clip(((X - x1) * dx + (Y - y1) * dy) / L2, 0.0, 1.0)
        occ |= np.hypot(X - (x1 + t * dx), Y - (y1 + t * dy)) <= thick
    for nxh, nyh, off in halfplanes:
        if not math.isfinite(off) or (nxh == 0.0 and nyh == 0.0):
            continue
        occ |= (nxh * X + nyh * Y) > off
    for cx, cy, r in circles:
        occ |= np.hypot(X - cx, Y - cy) <= r
    if corridor is not None:
        xc = np.asarray(corridor['xc'], dtype=float)
        yc = np.asarray(corridor['yc'], dtype=float)
        pts = np.stack([X.ravel(), Y.ravel()], axis=1)
        d2 = (pts[:, 0:1] - xc[None, :]) ** 2 + (pts[:, 1:2] - yc[None, :]) ** 2
        idx = np.argmin(d2, axis=1)
        lat = (np.asarray(corridor['nx'])[idx] * (pts[:, 0] - xc[idx])
               + np.asarray(corridor['ny'])[idx] * (pts[:, 1] - yc[idx]))
        outside = np.abs(lat) > np.asarray(corridor['halfWidth'])[idx]
        occ |= outside.reshape(shape)
    return occ


def _footprint_kernels(cfg: LatticeConfig) -> List[np.ndarray]:
    """One boolean kernel per heading bin: cells the footprint covers, base_link at the centre."""
    res = cfg.resolution_m
    m = cfg.footprint_margin_m + res * math.sqrt(0.5)   # cell-centre sampling margin
    fx, rx, hw = cfg.front_x + m, cfg.rear_x - m, cfg.half_width + m
    reach = math.hypot(max(abs(fx), abs(rx)), hw)
    R = int(math.ceil(reach / res))
    off = np.arange(-R, R + 1) * res
    DX, DY = np.meshgrid(off, off, indexing='ij')
    kernels = []
    for it in range(cfg.n_theta):
        th = 2.0 * math.pi * it / cfg.n_theta
        c, s = math.cos(th), math.sin(th)
        bx = c * DX + s * DY
        by = -s * DX + c * DY
        kernels.append((bx <= fx) & (bx >= rx) & (np.abs(by) <= hw))
    return kernels


@dataclass
class _Primitive:
    """
    Every primitive from one heading that snaps onto the same end, one direction.

    They cost the same, so they are ONE edge, which exists when ANY of them
    is collision-free: `samples` holds one check-point array per member. This
    is also what keeps the edge list free of duplicates (coo -> csr would sum
    them).
    """

    it: int
    direction: int          # +1 forward, -1 reverse
    end: Tuple[int, int, int]   # (di, dj, it_end)
    samples: List[np.ndarray]   # per member: (k, 3) int di, dj, it of each check point
    cost: float


def _primitives(cfg: LatticeConfig) -> List[_Primitive]:
    res, nth = cfg.resolution_m, cfg.n_theta
    bin_w = 2.0 * math.pi / nth
    steers = (cfg.steer_min, 0.5 * cfg.steer_min, 0.0, 0.5 * cfg.steer_max, cfg.steer_max)
    n_sub = max(int(math.ceil(cfg.primitive_length_m / 0.01)), 1)
    h = cfg.primitive_length_m / n_sub
    every = max(int(round(cfg.sample_step_m / h)), 1)
    groups: Dict[Tuple[int, int, Tuple[int, int, int]], _Primitive] = {}
    for it in range(nth):
        th0 = it * bin_w
        for d in (1, -1):
            for delta in steers:
                x = [0.0, 0.0, th0, float(d)]
                pts = [(0.0, 0.0, th0)]
                for k in range(n_sub):
                    x = f1tenth_state_fcn_dt_beta(x, [delta, 0.0], h, cfg.wheelbase, cfg.lr)
                    if (k + 1) % every == 0 or k == n_sub - 1:
                        pts.append((x[0], x[1], x[2]))
                arr = np.array(pts)
                cells = np.column_stack([
                    np.rint(arr[:, 0] / res), np.rint(arr[:, 1] / res),
                    np.mod(np.rint(arr[:, 2] / bin_w), nth)]).astype(int)
                end = tuple(int(v) for v in cells[-1])
                if end[0] == 0 and end[1] == 0 and end[2] == it:
                    continue
                samples = np.unique(cells, axis=0)
                key = (it, d, end)
                lock = abs(delta) / max(abs(cfg.steer_min), abs(cfg.steer_max), 1e-9)
                cost = cfg.primitive_length_m * ((1.0 if d > 0 else cfg.k_rev)
                                                 + cfg.steer_cost_per_m * lock)
                if key in groups:
                    groups[key].samples.append(samples)
                    groups[key].cost = min(groups[key].cost, cost)
                else:
                    groups[key] = _Primitive(it=it, direction=d, end=end,
                                             samples=[samples], cost=cost)
    return list(groups.values())


@dataclass
class LatticeField:
    """A built V: values per (gear, theta bin, x cell, y cell) and its window."""

    cfg: LatticeConfig
    origin: Tuple[float, float]
    shape: Tuple[int, int]
    V: np.ndarray            # (2, n_theta, nx, ny), smoothed (see build_field)
    V_raw: np.ndarray        # the Dijkstra values themselves (Bellman-consistent)
    free: np.ndarray         # (n_theta, nx, ny) configuration-space free cells
    build_ms: float
    n_edges: int
    goal: Tuple[float, float, Optional[float]]
    n_goal_states: int

    def _lookup(self, x, y, psi, gear):
        cfg = self.cfg
        res, nth = cfg.resolution_m, cfg.n_theta
        nx, ny = self.shape
        fx = (np.asarray(x, dtype=float) - self.origin[0]) / res
        fy = (np.asarray(y, dtype=float) - self.origin[1]) / res
        cfx = np.clip(fx, 0.0, nx - 1.0)
        cfy = np.clip(fy, 0.0, ny - 1.0)
        outside = np.hypot(fx - cfx, fy - cfy) * res
        ft = np.mod(np.asarray(psi, dtype=float), 2.0 * math.pi) / (2.0 * math.pi / nth)
        i0 = np.minimum(np.floor(cfx).astype(int), nx - 2)
        j0 = np.minimum(np.floor(cfy).astype(int), ny - 2)
        t0 = np.floor(ft).astype(int) % nth
        ax, ay, at = cfx - i0, cfy - j0, ft - np.floor(ft)
        t1 = (t0 + 1) % nth
        Vg = self.V[REVERSE_GEAR if gear < 0 else FORWARD_GEAR]
        val = 0.0
        for ti, wt in ((t0, 1.0 - at), (t1, at)):
            for ii, wx in ((i0, 1.0 - ax), (i0 + 1, ax)):
                for jj, wy in ((j0, 1.0 - ay), (j0 + 1, ay)):
                    val = val + wt * wx * wy * Vg[ti, ii, jj]
        return val + cfg.out_of_window_cost_per_m * outside

    def value(self, x: float, y: float, psi: float, gear: int) -> float:
        """Return V at a continuous pose in gear (+1 forward / -1 reverse)."""
        return float(self._lookup(x, y, psi, gear))

    def policy_path(self, x: float, y: float, psi: float, gear: int,
                    follow_m: float, direction: Optional[int] = None
                    ) -> List[Tuple[float, float, float, int]]:
        """
        Return the poses along V's optimal path from (x, y, psi) in `gear`, for follow_m.

        Greedy Bellman descent on the RAW Dijkstra values through the valid
        primitives: from the snapped state, take the successor minimising
        edge cost + V, repeatedly, until follow_m of path is covered or the
        goal is reached. Every step is a collision-checked edge, so the path
        is CONNECTED and footprint-free -- unlike a nearest-low-V search,
        which happily picks a cell on the far side of a wall. `direction`
        (+1 / -1) restricts the descent to primitives driven that way (the
        best continuation for a candidate whose schedule forbids the other);
        None allows both. Each entry is (x, y, psi unwrapped from the input
        psi, gear after the step); the first is the snapped start.
        """
        cfg = self.cfg
        res, nth = cfg.resolution_m, cfg.n_theta
        nx, ny = self.shape
        bin_w = 2.0 * math.pi / nth
        i = int(round((x - self.origin[0]) / res))
        j = int(round((y - self.origin[1]) / res))
        it = int(round(psi / bin_w)) % nth
        g = REVERSE_GEAR if gear < 0 else FORWARD_GEAR
        if not (0 <= i < nx and 0 <= j < ny):
            return [(float(x), float(y), float(psi), int(gear))]
        psi_out = round(psi / bin_w) * bin_w     # bin `it`'s heading on psi's own branch
        out = [(self.origin[0] + i * res, self.origin[1] + j * res, psi_out, int(gear))]
        travelled = 0.0
        by_it = _PRIMS_BY_IT.get(cfg)
        while travelled < follow_m - 1e-9:
            if self.V_raw[g, it, i, j] <= 1e-6 or self.V_raw[g, it, i, j] >= cfg.unreachable_cost:
                break
            best = None
            for p in by_it[it]:
                if direction is not None and p.direction != direction:
                    continue
                di, dj, te = p.end
                i2, j2 = i + di, j + dj
                if not (0 <= i2 < nx and 0 <= j2 < ny) or not self.free[te, i2, j2]:
                    continue
                ok = False
                for member in p.samples:
                    ii = i + member[:, 0]
                    jj = j + member[:, 1]
                    inside = (ii >= 0) & (ii < nx) & (jj >= 0) & (jj < ny)
                    if inside.all() and self.free[member[:, 2], ii, jj].all():
                        ok = True
                        break
                if not ok:
                    continue
                g2 = FORWARD_GEAR if p.direction > 0 else REVERSE_GEAR
                switch = cfg.gear_switch_cost if g2 != g else 0.0
                c = p.cost + switch + self.V_raw[g2, te, i2, j2]
                if best is None or c < best[0]:
                    best = (c, i2, j2, te, g2)
            if best is None:
                break
            _c, i, j, te, g = best
            psi_out += (((te - it + nth // 2) % nth) - nth // 2) * bin_w
            it = te
            travelled += cfg.primitive_length_m
            out.append((self.origin[0] + i * res, self.origin[1] + j * res, psi_out,
                        1 if g == FORWARD_GEAR else -1))
        return out

    def policy_target(self, x: float, y: float, psi: float, gear: int,
                      follow_m: float, direction: Optional[int] = None
                      ) -> Tuple[float, float, float, int]:
        """Return the last pose of policy_path (x, y, psi, gear)."""
        last = self.policy_path(x, y, psi, gear, follow_m, direction)[-1]
        return float(last[0]), float(last[1]), float(last[2]), int(last[3])

    def gradient(self, x: float, y: float, psi: float, gear: int) -> np.ndarray:
        """Return (dV/dx, dV/dy, dV/dpsi), central differences."""
        h = self.cfg.grad_step_m
        ht = 2.0 * math.pi / self.cfg.n_theta
        xs = np.array([x + h, x - h, x, x, x, x])
        ys = np.array([y, y, y + h, y - h, y, y])
        ps = np.array([psi, psi, psi, psi, psi + ht, psi - ht])
        v = self._lookup(xs, ys, ps, gear)
        return np.array([(v[0] - v[1]) / (2 * h), (v[2] - v[3]) / (2 * h),
                         (v[4] - v[5]) / (2 * ht)])


_PRIMITIVE_CACHE: Dict[LatticeConfig, List[_Primitive]] = {}
_KERNEL_CACHE: Dict[LatticeConfig, List[np.ndarray]] = {}
_PRIMS_BY_IT: Dict[LatticeConfig, Dict[int, List[_Primitive]]] = {}


def build_field(cfg: LatticeConfig, car_xy, goal, segments=(), halfplanes=(),
                circles=(), corridor=None) -> LatticeField:
    """
    Build V for goal (x, y, heading or None) in a window placed by window_center.

    A goal heading of None accepts any heading at the goal (position-only
    arrival, which is what goal_pose moves are). A goal outside the window is
    reached through the window border, each border state starting at its
    straight-line distance to the goal.
    """
    t0 = time.perf_counter()
    if cfg not in _PRIMITIVE_CACHE:
        _PRIMITIVE_CACHE[cfg] = _primitives(cfg)
        _KERNEL_CACHE[cfg] = _footprint_kernels(cfg)
        by_it: Dict[int, List[_Primitive]] = {}
        for p in _PRIMITIVE_CACHE[cfg]:
            by_it.setdefault(p.it, []).append(p)
        _PRIMS_BY_IT[cfg] = by_it
    prims, kernels = _PRIMITIVE_CACHE[cfg], _KERNEL_CACHE[cfg]
    res, nth = cfg.resolution_m, cfg.n_theta
    n_cells = int(round(cfg.window_m / res))
    nx = ny = n_cells
    cx, cy = window_center(car_xy, goal[:2], cfg)
    origin = (cx - (n_cells - 1) * res / 2.0, cy - (n_cells - 1) * res / 2.0)
    # Shift the grid (by less than a cell) so the CAR sits on a cell centre:
    # every policy path starts by snapping the car's pose, and with an even
    # cell count the car would otherwise be half a cell off from the start.
    origin = tuple(o + res * (((c - o) / res) - round((c - o) / res))
                   for o, c in zip(origin, (float(car_xy[0]), float(car_xy[1]))))

    occ = occupancy_grid(origin, (nx, ny), res, segments, halfplanes, circles, corridor)
    occ_u8 = occ.astype(np.uint8)
    free = np.stack([ndimage.correlate(occ_u8, k.astype(np.uint8), mode='constant', cval=0) == 0
                     for k in kernels])                       # (nth, nx, ny)

    pad = int(math.ceil(cfg.primitive_length_m / res)) + 2
    free_pad = np.zeros((nth, nx + 2 * pad, ny + 2 * pad), dtype=bool)
    free_pad[:, pad:pad + nx, pad:pad + ny] = free

    n_layer = nth * nx * ny
    n_states = 2 * n_layer
    rows, cols, wts = [], [], []
    ii, jj = np.meshgrid(np.arange(nx), np.arange(ny), indexing='ij')
    for p in prims:
        ok = np.zeros((nx, ny), dtype=bool)
        for member in p.samples:
            ok_m = free[p.it].copy()
            for di, dj, ti in member:
                ok_m &= free_pad[ti, pad + di:pad + di + nx, pad + dj:pad + dj + ny]
            ok |= ok_m
        di, dj, te = p.end
        ok &= free_pad[te, pad + di:pad + di + nx, pad + dj:pad + dj + ny]
        src_i, src_j = ii[ok], jj[ok]
        if src_i.size == 0:
            continue
        src = (p.it * nx + src_i) * ny + src_j
        dst = (te * nx + src_i + di) * ny + (src_j + dj)
        g_to = FORWARD_GEAR if p.direction > 0 else REVERSE_GEAR
        for g_from in (FORWARD_GEAR, REVERSE_GEAR):
            w = p.cost + (cfg.gear_switch_cost if g_from != g_to else 0.0)
            # REVERSED graph for the backward search: an edge dst -> src.
            rows.append(g_to * n_layer + dst)
            cols.append(g_from * n_layer + src)
            wts.append(np.full(src.size, w))

    # Goal states: cells within the tolerance (and heading tolerance, if
    # any), free, either gear -- or the free border cells if the goal is
    # outside the window.
    gx, gy = float(goal[0]), float(goal[1])
    g_th = goal[2] if len(goal) > 2 else None
    X = origin[0] + ii * res
    Y = origin[1] + jj * res
    inside = (origin[0] - res / 2 <= gx <= origin[0] + (nx - 0.5) * res
              and origin[1] - res / 2 <= gy <= origin[1] + (ny - 0.5) * res)
    super_node = n_states
    if inside:
        near = np.hypot(X - gx, Y - gy) <= max(cfg.goal_tolerance_m, res * 0.75)
        th = np.arange(nth) * 2.0 * math.pi / nth
        if g_th is None:
            th_ok = np.ones(nth, dtype=bool)
        else:
            dth = np.abs(np.arctan2(np.sin(th - g_th), np.cos(th - g_th)))
            th_ok = dth <= cfg.goal_heading_tol_rad + 1e-9
        goal_mask = free & near[None, :, :] & th_ok[:, None, None]
        init = np.zeros(int(goal_mask.sum()))
    else:
        # One primitive DEEP, not one cell: a 0.3 m primitive lands on every
        # third column, so a one-cell border is reachable straight on from
        # only a third of the cells (see the ALIASING note below).
        k = max(int(math.ceil(cfg.primitive_length_m / res)), 1)
        border = np.zeros((nx, ny), dtype=bool)
        border[:k, :] = border[-k:, :] = border[:, :k] = border[:, -k:] = True
        goal_mask = free & border[None, :, :]
        init = np.broadcast_to(np.hypot(X - gx, Y - gy)[None], goal_mask.shape)[goal_mask]
    goal_idx = np.flatnonzero(goal_mask)
    for g in (FORWARD_GEAR, REVERSE_GEAR):
        rows.append(np.full(goal_idx.size, super_node))
        cols.append(g * n_layer + goal_idx)
        # A tiny floor: some scipy versions read a stored 0 as "no edge".
        wts.append(np.maximum(init, 1e-9))

    r = np.concatenate(rows) if rows else np.zeros(0, dtype=int)
    c = np.concatenate(cols) if cols else np.zeros(0, dtype=int)
    w = np.concatenate(wts) if wts else np.zeros(0)
    # No duplicate (row, col) by construction (see _Primitive), so coo ->
    # csr's summing of duplicates never fires.
    G = coo_matrix((w, (r, c)), shape=(n_states + 1, n_states + 1)).tocsr()
    dist = dijkstra(G, directed=True, indices=super_node)[:n_states]
    dist = np.where(np.isfinite(dist), dist, cfg.unreachable_cost)
    dist = np.minimum(dist, cfg.unreachable_cost)
    V = dist.reshape(2, nth, nx, ny)
    V_raw = V.copy()
    # ALIASING. Primitives move a whole primitive length, so along a straight
    # line V is a staircase whose step is one primitive (0.3 m): the raw
    # field's slope alternates between 0 and several times the true one. A
    # box mean over one primitive length in x and y removes that period,
    # which is what the candidates' scores need. The mean is over REACHABLE
    # cells only (a normalised convolution): an unreachable cell stays
    # unreachable and does not leak into its free neighbours, which would
    # otherwise inflate exactly the wall-hugging cells the best paths through
    # a door use.
    k = max(int(round(cfg.primitive_length_m / res)), 1)
    if k > 1:
        reach = (V < cfg.unreachable_cost).astype(float)
        num = ndimage.uniform_filter(V * reach, size=(1, 1, k, k), mode='nearest')
        den = ndimage.uniform_filter(reach, size=(1, 1, k, k), mode='nearest')
        V = np.where(reach > 0.0, num / np.maximum(den, 1e-9), cfg.unreachable_cost)
    return LatticeField(cfg=cfg, origin=origin, shape=(nx, ny), V=V, V_raw=V_raw, free=free,
                        build_ms=(time.perf_counter() - t0) * 1e3, n_edges=int(w.size),
                        goal=(gx, gy, g_th), n_goal_states=int(goal_idx.size))


class CostToGo:
    """
    The cost-to-go the planner reads, rebuilt only when its inputs change.

    Rebuilds on: no field yet; the goal moving more than goal_change_m (or
    its heading more than goal_change_rad); the walls, obstacles or corridor
    changing by more than wall_change_m; the car (or a queried x_N) leaving
    the inner part of the window. Rebuilds other than for a goal change are
    rate-limited to min_rebuild_period_s. Every build's time is kept in
    build_times_ms for the timing report.
    """

    def __init__(self, cfg: LatticeConfig, goal_change_m: float = 0.10,
                 goal_change_rad: float = math.radians(5.0), wall_change_m: float = 0.10,
                 min_rebuild_period_s: float = 0.5, recenter_margin_m: float = 0.6):
        self.cfg = cfg
        self.goal_change_m = goal_change_m
        self.goal_change_rad = goal_change_rad
        self.wall_change_m = wall_change_m
        self.min_rebuild_period_s = min_rebuild_period_s
        self.recenter_margin_m = recenter_margin_m
        self.field: Optional[LatticeField] = None
        self._goal = None
        self._inputs = None
        self._built_at = -math.inf
        self.build_times_ms: List[float] = []
        self.last_rebuild_reason = ''

    @property
    def ready(self) -> bool:
        return self.field is not None

    def clear(self):
        """Forget the goal and the field (a move without a goal pose)."""
        self.field = None
        self._goal = None
        self._inputs = None

    def update(self, now_sec: float, car_xy, goal, segments=(), halfplanes=(),
               circles=(), corridor=None) -> bool:
        """Rebuild if needed; return True when a new field was built."""
        goal = (float(goal[0]), float(goal[1]),
                None if len(goal) < 3 or goal[2] is None else float(goal[2]))
        inputs = (_norm_segments(segments), _norm_rows(halfplanes), _norm_rows(circles),
                  _corridor_sig(corridor))
        reason = None
        if self.field is None:
            reason = 'first'
        elif self._goal_changed(goal):
            reason = 'goal'
        elif (now_sec - self._built_at) >= self.min_rebuild_period_s:
            if self._inputs_changed(inputs):
                reason = 'walls'
            elif not self._well_inside(car_xy):
                reason = 'recenter'
        if reason is None:
            return False
        self.field = build_field(self.cfg, car_xy, goal, segments, halfplanes, circles,
                                 corridor)
        self._goal, self._inputs, self._built_at = goal, inputs, now_sec
        self.build_times_ms.append(self.field.build_ms)
        self.last_rebuild_reason = reason
        return True

    def value(self, x, y, psi, gear) -> float:
        return self.field.value(x, y, psi, gear)

    def gradient(self, x, y, psi, gear) -> np.ndarray:
        return self.field.gradient(x, y, psi, gear)

    def policy_target(self, x, y, psi, gear, follow_m, direction=None):
        return self.field.policy_target(x, y, psi, gear, follow_m, direction)

    def policy_path(self, x, y, psi, gear, follow_m, direction=None):
        return self.field.policy_path(x, y, psi, gear, follow_m, direction)

    def _goal_changed(self, goal) -> bool:
        old = self._goal
        if old is None:
            return True
        if math.hypot(goal[0] - old[0], goal[1] - old[1]) > self.goal_change_m:
            return True
        if (goal[2] is None) != (old[2] is None):
            return True
        if goal[2] is not None:
            d = math.atan2(math.sin(goal[2] - old[2]), math.cos(goal[2] - old[2]))
            return abs(d) > self.goal_change_rad
        return False

    def _inputs_changed(self, inputs) -> bool:
        old = self._inputs
        for a, b in zip(inputs, old):
            if a.shape != b.shape:
                return True
            if a.size and np.max(np.abs(a - b)) > self.wall_change_m:
                return True
        return False

    def _well_inside(self, car_xy) -> bool:
        f = self.field
        res = self.cfg.resolution_m
        lo = np.array(f.origin) + self.recenter_margin_m
        hi = np.array(f.origin) + (np.array(f.shape) - 1) * res - self.recenter_margin_m
        p = np.asarray(car_xy, dtype=float)
        return bool(np.all(p >= lo) and np.all(p <= hi))


def _norm_segments(segments) -> np.ndarray:
    arr = np.array([[a[0], a[1], b[0], b[1]] for a, b in segments], dtype=float)
    return arr.reshape(-1, 4)


def _norm_rows(rows) -> np.ndarray:
    arr = np.array([list(r) for r in rows if all(math.isfinite(float(v)) for v in r)],
                   dtype=float)
    return arr.reshape(len(arr), -1) if arr.size else np.zeros((0, 3))


def _corridor_sig(corridor) -> np.ndarray:
    if corridor is None:
        return np.zeros((0,))
    return np.array([corridor['xc'][0], corridor['yc'][0], corridor['xc'][-1],
                     corridor['yc'][-1], float(np.mean(corridor['halfWidth']))])
