"""The two-step planner flow: plan, then start, with every refusal."""

from f1tenth_telegram import config, planner
from f1tenth_telegram.flow import Flow, PLAN_MAX_AGE_S
import pytest

from test_controller import FakeTransport, PLANNER_ERROR


class Clock:
    def __init__(self):
        self.t = 1_800_000_000.0

    def __call__(self):
        return self.t


@pytest.fixture
def setup(monkeypatch):
    monkeypatch.setattr(planner, 'POLL_S', 0)
    clock = Clock()
    fake = FakeTransport()
    fake.state['planner'] = 'running'
    fake.clock = clock
    flow = Flow(fake, config.from_dict({}), clock=clock)
    return flow, fake, clock


def plan(flow, fake, text='vai dritto per due metri'):
    r = flow.propose(text)
    assert r['kind'] == 'ask', r
    ex, why = flow.begin_plan(r['nonce'])
    assert why is None
    while not ex.done:
        ex.poll(fake, 5)
    return flow.finish_plan(ex), ex


def test_plan_then_start(setup):
    flow, fake, clock = setup
    offer, _ = plan(flow, fake)
    assert offer.id == '20261020T153012-abc123' and fake.sent == ['vai dritto per due metri']
    # Planning alone starts nothing.
    assert not any(c[0] in ('mission-load', 'mission-start') for c in fake.calls)
    r = flow.offer_start(offer.token)
    assert r['kind'] == 'ask'
    res = flow.start(r['nonce'])
    assert res['ok'] and res['countdown_s'] == 15.0
    assert ['mission-load', offer.id] in fake.calls
    assert ['mission-start', '--id', offer.id] in fake.calls
    # One start per plan.
    assert flow.offer_start(offer.token)['kind'] == 'gone'


def test_discard(setup):
    flow, fake, _ = setup
    offer, _ = plan(flow, fake)
    assert flow.discard(offer.token)
    assert flow.offer_start(offer.token)['kind'] == 'gone'


def test_text_is_collapsed_and_checked(setup):
    flow, fake, _ = setup
    assert flow.propose('vai\n dritto  \n')['text'] == 'vai dritto'
    assert 'at most 500' in flow.propose('x' * 501)['why']
    assert flow.propose('  \n ')['kind'] == 'invalid'


def test_stale_taps(setup):
    flow, fake, clock = setup
    r = flow.propose('vai dritto')
    clock.t += 11
    ex, why = flow.begin_plan(r['nonce'])
    assert ex is None and 'ask again' in why and fake.sent == []
    offer, _ = plan(flow, fake)
    r = flow.offer_start(offer.token)
    clock.t += 11
    res = flow.start(r['nonce'])
    assert not res['ok'] and 'ask again' in res['why']
    assert not any(c[0] == 'mission-start' for c in fake.calls)


def test_a_plan_older_than_5_min_cannot_start(setup):
    flow, fake, clock = setup
    offer, _ = plan(flow, fake)
    clock.t += PLAN_MAX_AGE_S + 1
    r = flow.offer_start(offer.token)
    assert r['kind'] == 'expired' and 'plan again' in r['why']
    # ...nor when it ages between the two questions.
    offer, _ = plan(flow, fake)
    clock.t += PLAN_MAX_AGE_S - 5
    r = flow.offer_start(offer.token)
    assert r['kind'] == 'ask'          # still young enough to ask...
    clock.t += 6
    flow.starts.ttl_s = 1e9            # (the 10 s rule aside, to test the age alone)
    res = flow.start(r['nonce'])       # ...but too old when the yes comes
    assert not res['ok'] and 'plan again' in res['why']
    assert not any(c[0] == 'mission-load' for c in fake.calls)


def test_no_new_plan_while_planning_or_while_a_mission_runs(setup):
    flow, fake, _ = setup
    first = flow.propose('vai dritto')
    ex, _ = flow.begin_plan(first['nonce'])
    assert flow.propose('fermati')['why'].startswith('still planning «vai dritto»')
    # A newer question replaces an older one: only the last "Plan?" counts.
    a = flow.asks.ask('a')
    flow.asks.ask('b')
    assert flow.begin_plan(a)[1] == 'this confirmation is no longer valid'
    while not ex.done:
        ex.poll(fake, 5)
    flow.finish_plan(ex)
    fake.mission = {'state': 'running', 'id': 'm1'}
    r = flow.propose('vai dritto')
    assert r['kind'] == 'refused' and 'a mission is running (m1)' in r['why']


def test_planner_not_running_or_unreachable(setup):
    flow, fake, _ = setup
    fake.state['planner'] = 'stopped'
    assert flow.propose('vai')['kind'] == 'not_running'
    fake.online = False
    assert flow.propose('vai')['kind'] == 'error'


def test_a_planner_error_offers_nothing(setup):
    flow, fake, _ = setup
    fake.planner_batches = [PLANNER_ERROR]
    fake.answer_next = PLANNER_ERROR
    offer, ex = plan(flow, fake, 'boh')
    assert offer is None and flow.offers == {}
    text = planner.render_failure(ex, as_html=False)
    assert 'refused (EmptyPlanError: ambiguous command)' in text
    assert 'No mission, nothing to start.' in text


@pytest.mark.parametrize('step, answer, reason', [
    ('mission-start', {'ok': False, 'refused': 'not READY: scan'}, 'not READY: scan'),
    ('mission-start', {'ok': False, 'refused': 'the e-stop is latched (estop-reset first)'},
     'the e-stop is latched'),
    ('mission-start', {'ok': False, 'refused': 'another mission is running (RUNNING)'},
     'another mission is running'),
    ('mission-load', {'ok': False, 'refused': 'invalid mission: MissionConfigError: x'},
     'invalid mission'),
])
def test_start_refusals_from_f1tenth_ctl(setup, step, answer, reason):
    flow, fake, _ = setup
    offer, _ = plan(flow, fake)
    fake.answers[step] = answer
    r = flow.offer_start(offer.token)
    res = flow.start(r['nonce'])
    assert not res['ok'] and res['why'].startswith(reason)
    if step == 'mission-load':
        assert not any(c[0] == 'mission-start' for c in fake.calls)


def test_the_offer_shows_the_summary_and_the_json(setup):
    flow, fake, _ = setup
    offer, _ = plan(flow, fake)
    text = planner.render_offer(offer.text, offer.entry)
    assert '🧭 Plan for «vai dritto per due metri» (3.1 s)' in text
    assert '1 move(s) · max speed 0.5 m/s · reverse: no' in text
    assert 'length: ~2 m, at most 30 s (timeouts)' in text
    assert '1. straight at 0.5 m/s, until distance_reached 2.0 (30 s → abort)' in text
    assert '<blockquote expandable>{' in text and '&quot;llm_go_ahead_2m&quot;' in text


def test_no_plan_unless_the_planner_is_plan_only(setup):
    flow, fake, _ = setup
    fake.plan_only = False             # e.g. started by an older f1tenth_ctl
    r = flow.propose('vai dritto')
    assert r['kind'] == 'refused' and 'not plan-only' in r['why']
    fake.plan_only = None              # an older f1tenth_ctl does not even say
    assert flow.propose('vai dritto')['kind'] == 'refused'
    # ...and checked again at the 🧭 Plan tap.
    fake.plan_only = True
    nonce = flow.propose('vai dritto')['nonce']
    fake.plan_only = False
    ex, why = flow.begin_plan(nonce)
    assert ex is None and 'not plan-only' in why and fake.sent == []
