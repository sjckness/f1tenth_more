"""coverage.py: cells, statistics (mean/min/p95/loss), no-link cells, encoding, reset."""

from f1tenth_comms.coverage import (CoverageGrid, LinkDeltas, NO_LINK, percentile, Sample,
                                    Scale, UNKNOWN)
import pytest

SCALE = Scale(rssi_good_dbm=-40.0, rssi_bad_dbm=-90.0, rtt_bad_ms=200.0, no_link_fraction=0.5)


def ok(x, y, rssi=-50.0, rtt=None, t=0.0, answered=0, lost=0):
    return Sample(x=x, y=y, t=t, associated=True, rssi_dbm=rssi, link_known=True,
                  link_lost=False, rtt_ms=rtt, answered=answered, lost=lost)


def test_cells_are_floor_of_the_map_coordinates():
    g = CoverageGrid(resolution=1.0)
    assert g.cell_of(0.0, 0.0) == (0, 0)
    assert g.cell_of(0.99, 1.0) == (0, 1)
    assert g.cell_of(-0.01, -1.0) == (-1, -1)
    assert CoverageGrid(resolution=0.5).cell_of(1.2, -0.2) == (2, -1)


def test_mean_and_min_rssi_per_cell():
    g = CoverageGrid()
    for rssi in (-50.0, -60.0, -70.0):
        g.add(ok(0.2, 0.3, rssi=rssi))
    g.add(ok(1.5, 0.5, rssi=-45.0))
    c = g.cells[(0, 0)]
    assert c.samples == 3
    assert c.rssi_mean == pytest.approx(-60.0)
    assert c.rssi_min == -70.0
    assert g.cells[(1, 0)].rssi_mean == -45.0


def test_p95_is_nearest_rank():
    assert percentile(list(range(1, 101)), 95) == 95
    assert percentile([10.0], 95) == 10.0
    assert percentile([5.0, 1.0, 3.0], 95) == 5.0
    assert percentile([], 95) is None
    g = CoverageGrid()
    for rtt in range(1, 21):                # 1..20 ms: nearest rank 19
        g.add(ok(0.5, 0.5, rtt=float(rtt)))
    assert g.cells[(0, 0)].rtt_p95 == 19.0


def test_rtt_keeps_only_the_last_n_per_cell():
    g = CoverageGrid(rtt_keep=5)
    for rtt in (900.0, 900.0, 1.0, 2.0, 3.0, 4.0, 5.0):
        g.add(ok(0.5, 0.5, rtt=rtt))
    assert g.cells[(0, 0)].rtt_p95 == 5.0


def test_loss_pct_from_rounds_answered_and_lost_in_the_cell():
    g = CoverageGrid()
    g.add(ok(0.5, 0.5, answered=2, lost=0))
    g.add(ok(0.5, 0.5, answered=1, lost=1))
    g.add(ok(3.5, 0.5, answered=0, lost=0))
    assert g.cells[(0, 0)].loss_pct == pytest.approx(25.0)
    assert g.cells[(3, 0)].loss_pct is None     # nothing decided there yet


def test_no_link_is_recorded_not_dropped():
    g = CoverageGrid()
    # Not associated: no link, even with no link_status at all.
    g.add(Sample(x=0.5, y=0.5, t=1.0, associated=False))
    # Associated, fresh link_status says LOST: no link.
    g.add(Sample(x=0.5, y=0.5, t=2.0, associated=True, rssi_dbm=-80.0, link_known=True,
                 link_lost=True, rtt_ms=None, lost=2))
    c = g.cells[(0, 0)]
    assert (c.samples, c.no_link) == (2, 2)
    assert c.rssi_mean == -80.0 and c.loss_pct == 100.0
    assert c.last_t == 2.0
    for layer in ('rssi', 'rtt'):
        img = g.encode(layer, SCALE)
        assert list(img.data) == [NO_LINK]


def test_stale_link_status_is_unknown_not_no_link():
    g = CoverageGrid()
    g.add(Sample(x=0.5, y=0.5, t=1.0, associated=True, rssi_dbm=-40.0, link_known=False,
                 link_lost=True))
    c = g.cells[(0, 0)]
    assert c.no_link == 0
    assert list(g.encode('rssi', SCALE).data) == [0]
    assert list(g.encode('rtt', SCALE).data) == [UNKNOWN]     # no RTT sample


def test_no_link_cell_needs_the_configured_share_of_samples():
    g = CoverageGrid()
    g.add(ok(0.5, 0.5, rssi=-40.0))
    g.add(ok(0.5, 0.5, rssi=-40.0))
    g.add(Sample(x=0.5, y=0.5, t=0.0, associated=False))
    assert g.cells[(0, 0)].no_link_fraction == pytest.approx(1 / 3)
    assert list(g.encode('rssi', SCALE).data) == [0]
    g.add(Sample(x=0.5, y=0.5, t=0.0, associated=False))       # now 2 of 4
    assert list(g.encode('rssi', SCALE).data) == [NO_LINK]


def test_levels_are_linear_and_clipped():
    g = CoverageGrid()
    g.add(ok(0.5, 0.5, rssi=-30.0, rtt=500.0))     # better than good, worse than bad
    g.add(ok(1.5, 0.5, rssi=-65.0, rtt=100.0))     # half way on both
    g.add(ok(2.5, 0.5, rssi=-95.0, rtt=0.0))
    assert list(g.encode('rssi', SCALE).data) == [0, 50, 100]
    assert list(g.encode('rtt', SCALE).data) == [100, 50, 0]


def test_grid_geometry_and_unvisited_cells():
    g = CoverageGrid(resolution=0.5)
    g.add(ok(-0.7, 0.2, rssi=-40.0))       # cell (-2, 0)
    g.add(ok(0.3, 1.1, rssi=-90.0))        # cell (0, 2)
    img = g.encode('rssi', SCALE)
    assert (img.origin_x, img.origin_y, img.width, img.height) == (-1.0, 0.0, 3, 3)
    assert list(img.data) == [0, UNKNOWN, UNKNOWN,
                              UNKNOWN, UNKNOWN, UNKNOWN,
                              UNKNOWN, UNKNOWN, 100]


def test_empty_grid_is_one_unknown_cell():
    img = CoverageGrid().encode('rtt', SCALE)
    assert (img.width, img.height, list(img.data)) == (1, 1, [UNKNOWN])


def test_reset_clears_every_cell_and_changes_the_version():
    g = CoverageGrid()
    g.add(ok(0.5, 0.5))
    g.add(ok(5.5, 0.5))
    v = g.version
    assert g.reset() == 2
    assert g.cells == {} and g.bounds() is None and g.version > v
    assert list(g.encode('rssi', SCALE).data) == [UNKNOWN]


def test_samples_far_from_the_origin_are_dropped():
    g = CoverageGrid(max_extent_m=50.0)
    assert not g.add(ok(1e6, 0.0))
    assert not g.add(ok(float('nan'), 0.0))
    assert g.add(ok(49.0, -49.0))
    assert g.rejected == 2 and len(g.cells) == 1


def test_link_deltas_survive_a_link_manager_restart():
    d = LinkDeltas()
    assert d.update(10, 1) == (0, 0)        # first value only sets the baseline
    assert d.update(12, 2) == (2, 1)
    assert d.update(1, 0) == (0, 0)         # counters went back: a restart
    assert d.update(3, 0) == (2, 0)
    d.forget()
    assert d.update(9, 9) == (0, 0)


def test_cells_report_has_every_number():
    g = CoverageGrid()
    g.add(ok(0.5, 0.5, rssi=-50.0, rtt=10.0, t=5.0, answered=3, lost=1))
    g.add(Sample(x=0.5, y=0.5, t=6.0, associated=False))
    (cell,) = g.cells_report(SCALE)
    assert cell == {
        'ix': 0, 'iy': 0, 'x': 0.5, 'y': 0.5, 'samples': 2, 'no_link': 1,
        'no_link_cell': True, 'rssi_mean_dbm': -50.0, 'rssi_min_dbm': -50.0,
        'rtt_p95_ms': 10.0, 'rtt_samples': 1, 'loss_pct': 25.0, 'rounds_answered': 3,
        'rounds_lost': 1, 'first_t': 5.0, 'last_t': 6.0}


def test_scale_rejects_nonsense():
    with pytest.raises(ValueError):
        Scale(rssi_good_dbm=-90.0, rssi_bad_dbm=-40.0)
    with pytest.raises(ValueError):
        Scale(no_link_fraction=0.0)
