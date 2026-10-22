"""
The Wi-Fi coverage grid: samples binned into square cells of the map frame.

Session only. The grid starts empty, lives in memory and is cleared by
/comms/coverage_reset; nothing is written to disk.

A cell is the square [ix*res, (ix+1)*res) x [iy*res, (iy+1)*res) of the map
frame, so a cell keeps its place however far the grid grows. Per cell it
keeps: the sample count, how many of those had no link, mean and min RSSI
over the associated samples, the RTT samples (for the p95), the heartbeat
rounds answered and lost while the car was in it, and the time of the last
sample.

A sample has NO LINK when the Wi-Fi is not associated, or when a fresh
/comms/link_status says LOST. A stale or missing link_status is "link
unknown", never "no link": the RSSI is still recorded.

OccupancyGrid encoding (both layers; int8, higher is worse):
    -1        UNKNOWN: never visited (RSSI layer), or no RTT sample (RTT layer)
    0..100    RSSI: 0 at rssi_good_dbm or better, 100 at rssi_bad_dbm or worse,
              linear in dBm between; of the cell's mean RSSI
              RTT: 0 at 0 ms, 100 at rtt_bad_ms or worse, linear; of the
              cell's p95 RTT
    -128      NO_LINK: at least no_link_fraction of the cell's samples had
              no link
"""

from array import array
from collections import deque
from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple

UNKNOWN = -1
NO_LINK = -128


@dataclass
class Sample:
    x: float
    y: float
    t: float                        # node clock [s]
    associated: bool
    rssi_dbm: Optional[float] = None
    link_known: bool = False        # a fresh /comms/link_status was there
    link_lost: bool = False         # ... and it said LOST
    rtt_ms: Optional[float] = None
    answered: int = 0               # heartbeat rounds answered since the previous sample
    lost: int = 0                   # heartbeat rounds lost since the previous sample

    @property
    def no_link(self) -> bool:
        return (not self.associated) or (self.link_known and self.link_lost)


@dataclass
class Scale:
    rssi_good_dbm: float = -40.0
    rssi_bad_dbm: float = -90.0
    rtt_bad_ms: float = 200.0
    no_link_fraction: float = 0.5

    def __post_init__(self):
        if self.rssi_good_dbm <= self.rssi_bad_dbm:
            raise ValueError('rssi_good_dbm must be above rssi_bad_dbm')
        if self.rtt_bad_ms <= 0:
            raise ValueError('rtt_bad_ms must be > 0')
        if not 0.0 < self.no_link_fraction <= 1.0:
            raise ValueError('no_link_fraction must be in (0, 1]')


def percentile(values, q: float) -> Optional[float]:
    """Return the nearest-rank q-th percentile of values, q in 0..100; None for no values."""
    if not values:
        return None
    ordered = sorted(values)
    rank = max(1, math.ceil(q / 100.0 * len(ordered)))
    return ordered[rank - 1]


def _clip_level(v: float) -> int:
    return int(min(100, max(0, round(v))))


class CellStats:
    __slots__ = ('samples', 'no_link', 'rssi_n', 'rssi_sum', 'rssi_min', 'rtt',
                 'answered', 'lost', 'first_t', 'last_t')

    def __init__(self, rtt_keep: int):
        self.samples = 0
        self.no_link = 0
        self.rssi_n = 0
        self.rssi_sum = 0.0
        self.rssi_min: Optional[float] = None
        self.rtt = deque(maxlen=rtt_keep)
        self.answered = 0
        self.lost = 0
        self.first_t: Optional[float] = None
        self.last_t: Optional[float] = None

    def add(self, s: Sample) -> None:
        self.samples += 1
        if s.no_link:
            self.no_link += 1
        if s.associated and s.rssi_dbm is not None:
            self.rssi_n += 1
            self.rssi_sum += s.rssi_dbm
            self.rssi_min = (s.rssi_dbm if self.rssi_min is None
                             else min(self.rssi_min, s.rssi_dbm))
        if s.rtt_ms is not None and not s.no_link:
            self.rtt.append(s.rtt_ms)
        self.answered += max(0, s.answered)
        self.lost += max(0, s.lost)
        if self.first_t is None:
            self.first_t = s.t
        self.last_t = s.t

    @property
    def rssi_mean(self) -> Optional[float]:
        return self.rssi_sum / self.rssi_n if self.rssi_n else None

    @property
    def rtt_p95(self) -> Optional[float]:
        return percentile(self.rtt, 95.0)

    @property
    def loss_pct(self) -> Optional[float]:
        decided = self.answered + self.lost
        return 100.0 * self.lost / decided if decided else None

    @property
    def no_link_fraction(self) -> float:
        return self.no_link / self.samples if self.samples else 0.0

    def is_no_link(self, scale: Scale) -> bool:
        return self.samples > 0 and self.no_link_fraction >= scale.no_link_fraction

    def rssi_level(self, scale: Scale) -> int:
        if self.is_no_link(scale):
            return NO_LINK
        mean = self.rssi_mean
        if mean is None:
            return UNKNOWN
        span = scale.rssi_good_dbm - scale.rssi_bad_dbm
        return _clip_level(100.0 * (scale.rssi_good_dbm - mean) / span)

    def rtt_level(self, scale: Scale) -> int:
        if self.is_no_link(scale):
            return NO_LINK
        p95 = self.rtt_p95
        if p95 is None:
            return UNKNOWN
        return _clip_level(100.0 * p95 / scale.rtt_bad_ms)


@dataclass
class GridImage:
    """One layer, ready for nav_msgs/OccupancyGrid (row-major, row 0 at origin_y)."""

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    data: array

    def key(self) -> Tuple:
        return (self.resolution, self.origin_x, self.origin_y, self.width, self.height,
                self.data.tobytes())


class CoverageGrid:
    def __init__(self, resolution: float = 1.0, rtt_keep: int = 200,
                 max_extent_m: float = 200.0):
        if resolution <= 0:
            raise ValueError('resolution must be > 0')
        self.resolution = float(resolution)
        self.rtt_keep = int(rtt_keep)
        self.max_extent_m = float(max_extent_m)
        self.cells: Dict[Tuple[int, int], CellStats] = {}
        self.version = 0            # bumped on every accepted sample and on reset
        self.rejected = 0           # samples outside max_extent_m

    def cell_of(self, x: float, y: float) -> Tuple[int, int]:
        return (math.floor(x / self.resolution), math.floor(y / self.resolution))

    def add(self, s: Sample) -> bool:
        """Bin one sample; False (and not kept) when it is outside max_extent_m."""
        if not (math.isfinite(s.x) and math.isfinite(s.y)) or \
                abs(s.x) > self.max_extent_m or abs(s.y) > self.max_extent_m:
            self.rejected += 1
            return False
        key = self.cell_of(s.x, s.y)
        cell = self.cells.get(key)
        if cell is None:
            cell = self.cells[key] = CellStats(self.rtt_keep)
        cell.add(s)
        self.version += 1
        return True

    def reset(self) -> int:
        n = len(self.cells)
        self.cells.clear()
        self.version += 1
        return n

    def bounds(self) -> Optional[Tuple[int, int, int, int]]:
        if not self.cells:
            return None
        xs = [k[0] for k in self.cells]
        ys = [k[1] for k in self.cells]
        return min(xs), min(ys), max(xs), max(ys)

    def encode(self, layer: str, scale: Scale) -> GridImage:
        """Return the 'rssi' or 'rtt' layer; an empty grid is one UNKNOWN cell at 0,0."""
        level = {'rssi': CellStats.rssi_level, 'rtt': CellStats.rtt_level}[layer]
        b = self.bounds()
        if b is None:
            return GridImage(self.resolution, 0.0, 0.0, 1, 1, array('b', [UNKNOWN]))
        x0, y0, x1, y1 = b
        width, height = x1 - x0 + 1, y1 - y0 + 1
        data = array('b', [UNKNOWN]) * (width * height)
        for (ix, iy), cell in self.cells.items():
            data[(iy - y0) * width + (ix - x0)] = level(cell, scale)
        return GridImage(self.resolution, x0 * self.resolution, y0 * self.resolution,
                         width, height, data)

    def cells_report(self, scale: Scale) -> List[Dict]:
        """Return every cell's statistics, for /comms/coverage_cells."""
        out = []
        res = self.resolution
        for (ix, iy), c in sorted(self.cells.items()):
            out.append({
                'ix': ix, 'iy': iy,
                'x': round((ix + 0.5) * res, 3), 'y': round((iy + 0.5) * res, 3),
                'samples': c.samples, 'no_link': c.no_link,
                'no_link_cell': c.is_no_link(scale),
                'rssi_mean_dbm': _r(c.rssi_mean), 'rssi_min_dbm': _r(c.rssi_min),
                'rtt_p95_ms': _r(c.rtt_p95), 'rtt_samples': len(c.rtt),
                'loss_pct': _r(c.loss_pct), 'rounds_answered': c.answered,
                'rounds_lost': c.lost, 'first_t': _r(c.first_t, 3),
                'last_t': _r(c.last_t, 3),
            })
        return out


def _r(v: Optional[float], digits: int = 1) -> Optional[float]:
    return None if v is None else round(v, digits)


class LinkDeltas:
    """Heartbeat rounds decided between two samples, from link_status's cumulative counters."""

    def __init__(self):
        self._prev: Optional[Tuple[int, int]] = None

    def update(self, answered: int, lost: int) -> Tuple[int, int]:
        """Return (answered, lost) since the previous call; (0, 0) when the counters restart."""
        prev, self._prev = self._prev, (answered, lost)
        if prev is None or answered < prev[0] or lost < prev[1]:
            return 0, 0
        return answered - prev[0], lost - prev[1]

    def forget(self) -> None:
        """Drop the baseline: the next update only sets it (link_status went stale)."""
        self._prev = None
