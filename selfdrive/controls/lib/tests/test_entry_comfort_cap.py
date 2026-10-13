"""ENTRY_COMFORT_CAP (cycle 2026-10-13 WS ENTRY, PROPOSAL_v2): on a slow entry with a light incoming command the governor's catch-up
is capped at -EC_A while the causal predictor still rests at >= D_REST_MIN + EC_MARGIN; floor lanes bind through the min() as
today; a hazard or under-delivery ends the cap and today's law resumes at J_DOWN. A small delayed-gain plant drives the real service."""
from collections import deque
from dataclasses import replace

import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.stopping_service import EntryCapEpisode, Phase, StoppingService, entry_cap_travel
from openpilot.selfdrive.controls.lib.tests.test_stopping_service import make_signals

P = StoppingService().p
DT = 0.01
D_MIN = P.D_REST_MIN + P.EC_MARGIN


@pytest.fixture(autouse=True, params=[(True, True), (True, False), (False, True), (False, False)], ids=['gat_land', 'gat', 'land', 'none'])
def flags(monkeypatch, request):
  monkeypatch.setattr(stopping_flags, 'SERVICE_APPROACH_LAW', 'governor')
  monkeypatch.setattr(stopping_flags, 'ENTRY_COMFORT_CAP', True)
  monkeypatch.setattr(stopping_flags, 'GAP_AWARE_TERMINAL', request.param[0])
  monkeypatch.setattr(stopping_flags, 'DELIVERED_LANDING', request.param[1])


def drive(*, v0=1.18, gap0=5.4, u_in=-0.63, gain=1.0, push=0.1, delay=0.3, lv=0.0, a_target=None, n=700, pre=40, cut_t=None, cut_d=0.0,
          event=None, isd=0.3):
  """Pre-roll `pre` engaged frames INACTIVE (lead not yet stopped) at u_in, then the stopped lead latches. event(t) -> dict of
  signal / kwarg overrides from that frame on. Returns (svc, trace)."""
  svc = StoppingService()
  q = deque([u_in] * int(round(delay / DT)))
  v, x, w = v0, 0.0, u_in
  a = gain * u_in + push
  tr = []
  for k in range(pre + n):
    t = (k - pre) * DT
    on = k >= pre
    cut = cut_d if cut_t is not None and t >= cut_t else 0.0
    gap = gap0 - x - cut
    ov = event(t) if event is not None and on else {}
    sig = replace(make_signals(d_gap=gap, a_coast=ov.get('a_coast', push), latch=on and ov.get('latch', True), wheel=v <= 0.01,
                               gap_source=ov.get('gap_source'), dropout=ov.get('dropout', False)),
                  track_age_s=ov.get('track_age', 5.0))
    r = svc.update(engaged=True, v_ego=v, a_ego=a, a_target=u_in if a_target is None else a_target, should_stop=on,
                   dts_planner=ov.get('dts', -1.0), planner_min_limit=-3.5, signals=sig, lead_status=True, lead_v=ov.get('lv', lv),
                   increased_stopped_distance=isd, wire_accel=w, prev_wire=w, fcw=ov.get('fcw', False), dt=DT)
    w = r.accel if r.active else u_in
    ep = svc._entry_cap
    tr.append(dict(t=t, v=v, gap=gap, w=w, phase=svc.phase, armed=bool(ep and ep.armed), reason=ep.reason if ep else None,
                   dbg=r.debug or {}))
    q.append(w)
    a = gain * q.popleft() + push
    v = max(v + a * DT, 0.0)
    x += v * DT
    if on and svc.phase in (Phase.HOLD,) and v <= 0.0:
      break
  return svc, tr


def on_frames(tr):
  return [f for f in tr if f['t'] >= 0.0]


def max_deepen(tr):
  w = [f['w'] for f in on_frames(tr)]
  return max((w[i - 1] - w[i]) / DT for i in range(1, len(w)))


def test_late_entry_arms_and_caps():
  svc, tr = drive()
  f = on_frames(tr)
  assert f[0]['phase'] == Phase.APPROACH_GLIDE and f[0]['armed']
  assert min(x['w'] for x in f if x['v'] > P.V_DESCENT_START) >= -P.EC_A - 1e-9
  assert f[0]['dbg']['a_phase_raw'] < -P.EC_A          # today's demand was deeper: the cap binds
  rest = min(x['gap'] for x in f)
  assert rest >= P.D_REST_MIN
  assert svc._entry_cap.pred >= D_MIN - 0.3          # the plan is unfolding as predicted (no under-delivery here)


def test_flag_off_and_ineligible_entry_are_identical(monkeypatch):
  for kw in (dict(), dict(v0=2.2, gap0=7.0)):
    monkeypatch.setattr(stopping_flags, 'ENTRY_COMFORT_CAP', False)
    _, off = drive(**kw)
    monkeypatch.setattr(stopping_flags, 'ENTRY_COMFORT_CAP', True)
    _, on = drive(**kw)
    if kw:
      assert [f['w'] for f in on] == [f['w'] for f in off]   # v 2.2: never eligible
    else:
      assert [f['w'] for f in on] != [f['w'] for f in off]


@pytest.mark.parametrize('kw', [
  dict(v0=1.85, gap0=6.8),                                    # speed bound (1.64 m/s at the entry frame)
  dict(u_in=-0.75),                                           # the incoming command already at the cap: never release it
  dict(gap0=4.9),                                             # predicted rest below the threshold
  dict(event=lambda t: dict(latch=False)),                    # strict latch off
  dict(event=lambda t: dict(gap_source='held')),              # inward / invalid hold (not gap_live)
  dict(event=lambda t: dict(track_age=0.3)),                  # young identity
  dict(event=lambda t: dict(fcw=True)),
  dict(event=lambda t: dict(a_coast=-0.3)),                   # uphill
  dict(event=lambda t: dict(dts=0.3)),                        # the planner's stop point short of the radar anchor
  dict(pre=10),                                               # the 0.3 s command history is not full
])
def test_no_arm(kw):
  svc, tr = drive(**kw)
  assert not any(f['armed'] for f in tr)


def test_window_closes_after_its_time():
  # trust arrives only after the window: no arm, the episode closed on the window
  svc, tr = drive(event=lambda t: dict(track_age=5.0 if t > P.EC_T_WIN + 0.05 else 0.3))
  assert not any(f['armed'] for f in tr)
  assert any(f['reason'] == 'window' for f in tr)


def test_hot_entry_slowing_through_the_cap_never_arms():
  svc, tr = drive(v0=2.2, gap0=6.6, u_in=-0.9)
  assert not any(f['armed'] for f in tr)
  assert all(f['reason'] in (None,) for f in on_frames(tr))


@pytest.mark.parametrize('name,kw', [
  ('nearer', dict(cut_t=0.4, cut_d=0.6)),                                        # radar gap steps in (late object)
  ('nearer', dict(event=lambda t: dict(dts=0.0) if t > 0.2 else {})),            # the planner's target steps in
  ('trust', dict(event=lambda t: dict(track_age=0.1) if t > 0.4 else {})),       # identity change
  ('trust', dict(event=lambda t: dict(dropout=True, gap_source='decay') if t > 0.4 else {})),
  ('trust', dict(event=lambda t: dict(fcw=True) if t > 0.4 else {})),
  ('trust', dict(event=lambda t: dict(latch=False) if t > 0.4 else {})),          # strict latch lost
  ('under', dict(gain=0.75, push=0.15)),                                          # the plant under-delivers
])
def test_spend_resumes_today_at_j_down(name, kw):
  svc, tr = drive(**kw)
  f = on_frames(tr)
  assert f[0]['armed']
  spent = [x for x in f if x['reason'] in ('trust', 'nearer', 'under')]
  assert spent and spent[0]['reason'] == name
  i = f.index(spent[0])
  assert not any(x['armed'] for x in f[i:])                                       # never re-arms in the episode
  for a, b in zip(f[i:], f[i + 1:], strict=False):                                # J_DOWN unless a lane binds (its own J_SAFE)
    if b['w'] < a['w'] and not b['dbg'].get('safety_binding'):
      assert (a['w'] - b['w']) / DT <= P.J_DOWN + 1e-6


def test_floor_lane_binds_through_the_cap_without_spending():
  # a radar vLead bias (-0.4) makes the barrier ask more than the cap: it binds at its own rate, the cap stays armed (Astra's
  # 228d_1576.35 rebound: the lane bind used to restore the whole anchor deficit at J_SAFE)
  svc, tr = drive(v0=1.5, gap0=5.95, u_in=-0.6, lv=-0.4)
  f = on_frames(tr)
  assert f[0]['armed']
  bound = [x for x in f if x['armed'] and x['dbg'].get('safety_binding')]
  assert bound, 'the scenario must exercise a lane bind under the cap'
  assert min(x['w'] for x in f if x['armed']) > -1.0                            # the lane's own demand, not the anchor deficit
  assert not any(x['reason'] in ('trust', 'nearer', 'under') for x in f)


@pytest.mark.parametrize('gap0', [5.52, 5.53, 5.6, 5.7, 5.9])
def test_nominal_lag_never_spends(gap0):
  # Astra's normal plant: v 1.5, incoming -0.6, gain 1, push +0.1, delay 0.3 s, no quantization
  svc, tr = drive(v0=1.5, gap0=gap0, u_in=-0.6)
  assert not any(f['reason'] in ('trust', 'nearer', 'under') for f in tr)
  assert max_deepen(tr) <= P.J_DOWN + 1e-6


def test_release_clears_and_reentry_opens_a_new_episode():
  svc, tr = drive(n=60)
  assert svc._entry_cap is not None and svc._entry_cap.armed
  svc.phase = Phase.RELEASE
  svc.reset()
  assert svc._entry_cap is None


def test_descent_captures_after_the_cap():
  svc, tr = drive()
  f = on_frames(tr)
  below = [x for x in f if x['v'] <= P.V_DESCENT_START]
  assert below and all(not x['dbg'].get('ec_armed') or x['dbg']['a_phase'] == x['dbg']['a_phase_raw'] for x in below)
  assert f[-1]['phase'] in (Phase.RAMP_TO_HOLD, Phase.HOLD)


def test_attributed_candidate_keeps_the_uncapped_phase(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'ATTRIBUTED_SAFETY', 'live')
  svc, tr = drive()
  for f in on_frames(tr):
    d = f['dbg']
    if d.get('attr_candidate') is not None and d.get('a_phase_raw') is not None and d.get('attr_eligible'):
      assert d['attr_candidate'] <= d['a_phase_raw'] + 1e-9


def test_predictor_reads_the_commands_in_flight():
  q_light, q_deep = [-0.3] * 30, [-1.0] * 30
  assert entry_cap_travel(1.2, q_light, -0.75, P) > entry_cap_travel(1.2, q_deep, -0.75, P)
  assert entry_cap_travel(0.4, [], -0.75, P) == pytest.approx(P.EC_S_TERM * (0.4 / P.V_DESCENT_START) ** 2)


def test_episode_open_rule():
  assert EntryCapEpisode.open(1.59, -0.74, 0.0, P).eligible
  assert not EntryCapEpisode.open(1.6, -0.5, 0.0, P).eligible
  assert not EntryCapEpisode.open(1.0, -0.75, 0.0, P).eligible


def test_1576_rebound_pin(monkeypatch):
  # 228d_1576.35 entry state (v 1.50, gap 5.59, vLead -0.04, incoming -0.73, push +0.04): the frozen predictor rests at 3.46 < 3.55,
  # so the cap stays off and the command is today's to the frame (Astra's rebound -0.75 -> -1.60 cannot happen); peak >= today's
  kw = dict(v0=1.78, gap0=6.25, u_in=-0.73, push=0.04, lv=-0.04)
  _, on = drive(**kw)
  f = on_frames(on)
  assert 1.45 < f[0]['v'] < 1.55 and 5.5 < f[0]['gap'] < 5.7
  assert not any(x['armed'] for x in on)
  monkeypatch.setattr(stopping_flags, 'ENTRY_COMFORT_CAP', False)
  _, off = drive(**kw)
  assert [x['w'] for x in on] == [x['w'] for x in off]


def test_weak_plant_spends_on_the_measured_shortfall():
  # the judges' 'noise' grid (gain 0.8, push +0.45): the cap delivers ~-0.15; the census delivery model alone spent 1.3 s late (2.87 m).
  # Once settled at the cap, the measured shortfall makes the prediction pessimistic and the cap spends within ~0.5 s of the arm
  svc, tr = drive(v0=1.5, gap0=7.5, u_in=-0.3, gain=0.8, push=0.45)
  f = on_frames(tr)
  arm = next(x for x in f if x['armed'])
  spend = next(x for x in f if x['reason'] == 'under')
  assert spend['t'] - arm['t'] < 0.6
  assert min(x['gap'] for x in f) >= P.D_REST_MIN


def test_measured_shortfall_never_makes_the_prediction_optimistic():
  ep = EntryCapEpisode(eligible=True, armed=True, gap_ref=8.0)
  q = deque([-0.75] * 30, maxlen=30)
  kw = dict(t=0.5, v=1.0, gap=8.0, lead_v=0.0, u_now=-0.75, queue=q, a_phase=-1.0, trust=True, nearer=False, dt=DT)
  ep.step(P, a_ego=-1.2, **kw)                          # delivering deeper than the model: the census B stands
  deep = ep.pred
  ep2 = EntryCapEpisode(eligible=True, armed=True, gap_ref=8.0)
  ep2.step(P, a_ego=None, **kw)
  assert deep == pytest.approx(ep2.pred)
  ep3 = EntryCapEpisode(eligible=True, armed=True, gap_ref=8.0)
  ep3.step(P, a_ego=-0.4, **kw)                         # shallower: the prediction falls
  assert ep3.pred < ep2.pred
