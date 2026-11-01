"""RELEASE_END_STOPPED_LEAD_REHOLD (stopping_flags, 2026-10-04; route 00002232 at 4904.04).

The chain: at rest behind a stopped lead, radar range drift (4.6 -> 5.1 m with Doppler 0) armed gap_grew; the acc-mode planner asked
to launch; the service HOLD released on that planner go; the RELEASE ended at rest and the service went INACTIVE; on the next frame
LongControl entered `starting` (the Hyundai sender drops StopReq outside `stopping`) while the service re-entered the same stopped
lead. With the flag the RELEASE end re-holds instead: LongControl stays `stopping` (StopReq held at rest), and the planner go is
refused until the lead leaves the stopped window or creeps away measurably. Once per wheel stop."""
import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.stop_context import StopContext
from openpilot.selfdrive.controls.lib.stopping_service import Phase, ServiceParams, StoppingService
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles

P = ServiceParams()
DT = 0.01
LIMITS = (-3.5, 2.0)
GO = 0.42      # the acc-mode planner's launch request at 4904 (aTarget 0.21..0.56, shouldStop 0)


@pytest.fixture(autouse=True)
def _flag_on(monkeypatch):
  monkeypatch.setattr(stopping_flags, "RELEASE_END_STOPPED_LEAD_REHOLD", True)


def travel(tau, a=1.0, v_max=1.5):
  """Lead travel tau s after it starts at a m/s^2 up to v_max."""
  tau = max(tau, 0.0)
  t_up = v_max / a
  return 0.5 * a * tau * tau if tau <= t_up else 0.5 * a * t_up * t_up + v_max * (tau - t_up)


class Radar:
  """20 Hz radar: range quantised to 0.1 m; lead_v(t) is the reported Doppler (a flicker is a Doppler-only artifact)."""

  def __init__(self, gap, lead_v=lambda t: 0.0):
    self.gap, self.lead_v, self.meas = gap, lead_v, None

  def read(self, t):
    if self.meas is None or round(t / DT) % 5 == 0:
      self.meas = (round(self.gap(t) / 0.1) * 0.1, self.lead_v(t))
    return self.meas


class Seam:
  """StopContext + StoppingService with the ego held at rest (every command here is <= 0)."""

  def __init__(self, radar):
    self.radar, self.ctx, self.svc = radar, StopContext(), StoppingService()
    self.t, self.cmd, self.phases, self.cmds = 0.0, 0.0, [], []

  def step(self, a_target, should_stop):
    d, lv = self.radar.read(self.t)
    sig = self.ctx.update(v_ego=0.0, a_ego=0.0, a_cmd=self.cmd, lead_status=True, lead_v=lv, lead_d_rel=d, lead_track_id=7,
                          standstill=True, dt=DT)
    r = self.svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=a_target, should_stop=should_stop, dts_planner=None,
                        planner_min_limit=LIMITS[0], signals=sig, lead_status=True, lead_v=lv, dt=DT, wire_accel=self.cmd,
                        a_target_trajectory=a_target)
    self.cmd = r.accel
    self.phases.append(r.phase)
    self.cmds.append(r.accel)
    self.t += DT
    return r, sig

  def run(self, seconds, a_target, should_stop):
    for _ in range(int(round(seconds / DT))):
      r, sig = self.step(a_target, should_stop)
    return r, sig

  def reholds(self):
    return sum(1 for a, b in zip(self.phases, self.phases[1:], strict=False) if a == Phase.RELEASE and b == Phase.RAMP_TO_HOLD)


def drift_then_go(lead_v=lambda t: 0.0, gap_after=None):
  """HOLD at 4.6 m, standstill range drift to 5.1 m (Doppler 0) arming gap_grew, then the planner go: the RELEASE ends at rest."""
  def gap(t):
    g = 4.6 + 0.1 * min(max(t - 3.0, 0.0), 5.0)
    return gap_after(t, g) if gap_after is not None else g
  s = Seam(Radar(gap, lead_v))
  s.run(8.5, -0.3, True)
  assert s.svc.phase == Phase.HOLD and s.cmd == pytest.approx(P.A_HOLD_SECURE)
  assert s.svc.ev.hold_entry_gap == pytest.approx(4.6)
  return s


def run_to_release_end(s):
  """Planner go until the RELEASE ends (re-hold with the flag, INACTIVE without)."""
  for _ in range(200):
    s.step(GO, False)
    if s.phases[-1] in (Phase.RAMP_TO_HOLD, Phase.INACTIVE) and Phase.RELEASE in s.phases[-80:]:
      return s.phases[-1]
  raise AssertionError("the RELEASE never ended")


def test_release_end_behind_a_stopped_lead_re_holds_and_refuses_the_stale_go():
  s = drift_then_go()
  assert run_to_release_end(s) == Phase.RAMP_TO_HOLD
  assert s.svc.ev.hold_entry_gap == pytest.approx(4.6), "the hold anchor must be kept"
  assert s.svc.ev.rehold_gap == pytest.approx(5.1)
  r, sig = s.run(3.0, GO, False)                        # the go persists 3 s toward the stopped lead (4904: 3.2 s)
  assert sig.lead_confirmed_stopped
  assert all(p in (Phase.RAMP_TO_HOLD, Phase.HOLD) for p in s.phases[-300:]), "the stale go released the re-hold"
  assert r.accel == pytest.approx(P.A_HOLD_SECURE), "the re-hold builds the secure hold"


def test_flag_off_release_end_goes_inactive_as_the_car_build(monkeypatch):
  monkeypatch.setattr(stopping_flags, "RELEASE_END_STOPPED_LEAD_REHOLD", False)
  s = drift_then_go()
  assert run_to_release_end(s) == Phase.INACTIVE
  assert s.svc.ev.rehold_gap is None
  # car build: the stopped lead re-enters on the next service frame with a fresh anchor (the stale gap evidence erased); in
  # LongControl that frame already ran the legacy transition with the service INACTIVE (the race, see the LongControl pins)
  r, _ = s.step(GO, False)
  assert r.phase == Phase.RAMP_TO_HOLD and s.svc.ev.hold_entry_gap == pytest.approx(5.1)


def test_flag_off_identity_until_the_release_end(monkeypatch):
  on = drift_then_go()
  run_to_release_end(on)
  monkeypatch.setattr(stopping_flags, "RELEASE_END_STOPPED_LEAD_REHOLD", False)
  off = drift_then_go()
  run_to_release_end(off)
  n = len(off.cmds) - 1
  assert on.cmds[:n] == off.cmds[:n] and on.phases[:n] == off.phases[:n]


def test_real_departure_after_the_re_hold_releases_at_once():
  depart = 13.0
  s = drift_then_go(lead_v=lambda t: min(max(t - depart, 0.0), 1.5), gap_after=lambda t, g: g + travel(t - depart))
  assert run_to_release_end(s) == Phase.RAMP_TO_HOLD
  while s.t < depart:
    s.step(GO, False)
  assert s.svc.phase == Phase.HOLD
  t_go = None
  for _ in range(300):
    r, sig = s.step(GO, False)
    if t_go is None and r.phase == Phase.RELEASE:
      t_go = s.t
      assert not sig.lead_confirmed_stopped
    if r.phase == Phase.INACTIVE:
      break
  assert r.phase == Phase.INACTIVE
  # the strict latch drops once the Doppler leaves the stopped window (0.3 m/s, 0.3 s after the start + one radar period),
  # then the RELEASE ramps the secure hold off at J_GO
  assert t_go - depart <= 0.3 + 0.05 + 2 * DT
  assert s.t - t_go <= -P.A_HOLD_SECURE / P.J_GO + 0.05
  assert s.reholds() == 1


def test_lead_departing_during_the_release_is_unchanged(monkeypatch):
  def make():
    s = drift_then_go(lead_v=lambda t: min(max(t - 8.6, 0.0), 1.5), gap_after=lambda t, g: g + travel(t - 8.6))
    run_to_release_end(s)
    s.run(1.0, GO, False)
    return s
  on = make()
  monkeypatch.setattr(stopping_flags, "RELEASE_END_STOPPED_LEAD_REHOLD", False)
  off = make()
  assert on.reholds() == 0
  assert on.cmds == off.cmds and on.phases == off.phases


def test_doppler_flicker_re_holds_at_every_release_end():
  # a stopped lead whose Doppler flickers to +0.44 for one radar period every 1.5 s, under a persistent stale go: a flicker drops
  # the strict latch, which lets the go through; every RELEASE end that still reads the same stopped lead re-holds again (Astra
  # 2026-10-04 P1: handing the second end back INACTIVE re-created the 4904 race). The cycle runs at rest; nothing goes INACTIVE.
  s = drift_then_go(lead_v=lambda t: 0.44 if t > 10.0 and (t - 10.0) % 1.5 < 0.1 else 0.0)
  assert run_to_release_end(s) == Phase.RAMP_TO_HOLD
  s.run(8.0, GO, False)
  releases = sum(1 for a, b in zip(s.phases, s.phases[1:], strict=False) if a != Phase.RELEASE and b == Phase.RELEASE)
  assert releases >= 2 and s.reholds() == releases
  assert Phase.INACTIVE not in s.phases[-900:]
  assert max(s.cmds[-900:]) <= 0.0
  assert s.svc.ev.hold_entry_gap == pytest.approx(4.6), "the hold anchor is kept through every re-hold"


@pytest.mark.parametrize("creep", [0.2, 0.25])
def test_slow_creeper_inside_the_stopped_window_is_not_trapped(creep):
  # a lead creeping away below the 0.3 m/s stopped window keeps the strict latch set; fresh gap growth while it reads measurably
  # moving (> MON_LEAD_RECEDE_MPS) is departure evidence again: the go passes once the gap grew RELEASE_GAP_GROW_M past the re-hold
  start = 12.0
  s = drift_then_go(lead_v=lambda t: creep if t >= start else 0.0, gap_after=lambda t, g: g + creep * max(t - start, 0.0))
  assert run_to_release_end(s) == Phase.RAMP_TO_HOLD
  while s.t < start:
    s.step(GO, False)
  t_go = None
  for _ in range(800):
    r, sig = s.step(GO, False)
    if t_go is None and r.phase == Phase.RELEASE:
      t_go = s.t
      assert sig.lead_confirmed_stopped
    if r.phase == Phase.INACTIVE:
      break
  assert r.phase == Phase.INACTIVE, "trapped behind a slow creeper"
  # 0.3 m of fresh growth (+ one 0.1 m radar quantum) + outward persistence (0.25 s) + radar timing
  assert t_go - start <= (P.RELEASE_GAP_GROW_M + 0.1) / creep + 0.5
  assert s.reholds() == 1
  # the continuation (Astra 2026-10-04 P2): the ego is held at rest here, so the service keeps re-entering behind the creeper as
  # the car build does; a lead that measurably creeps away is never re-held again
  s.run(38.0, GO, False)
  assert s.reholds() == 1


def test_re_hold_state_is_episode_and_wheel_stop_scoped():
  s = drift_then_go()
  run_to_release_end(s)
  assert s.svc.ev.rehold_gap is not None
  s.svc.ev.on_wheel_latch(0.0, 5.1)                     # a new hold anchor is the creep reference again
  assert s.svc.ev.rehold_gap is None
  s.svc.ev.rehold_gap = 5.1
  s.svc.reset()                                         # INACTIVE, disengage, gas, out of band, the LIVE fault path
  assert s.svc.ev.rehold_gap is None and s.svc.phase == Phase.INACTIVE


# --- LongControl seam: the 4904 frame sequence ----------------------------------------------------------------------------------
def lc_4904(depart=None, flicker=False):
  """The 4904 chain through LongControl (LIVE service ownership): HOLD at 4.6 m, range drift to 5.1 m (Doppler 0), then the acc
  planner's go (shouldStop 0, aTarget +0.42) toward the still-stopped lead; optionally the lead departs at `depart` s."""
  cp = DummyCarParams()
  cp.startingState = True
  lc = LongControl(cp)
  toggles = DummyFrogPilotToggles()
  radar = Radar(lambda t: 4.6 + 0.1 * min(max(t - 3.0, 0.0), 5.0) + (travel(t - depart) if depart else 0.0),
                lambda t: min(max(t - depart, 0.0), 1.5) if depart else (0.44 if flicker and t > 10.0 and (t - 10.0) % 1.5 < 0.1 else 0.0))
  rows = []
  for i in range(int(16.0 / DT)):
    t = i * DT
    go = t >= 8.5
    d, lv = radar.read(t)
    out = lc.update(active=True, CS=DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True), a_target=GO if go else -0.3,
                    should_stop=not go, distance_to_stop_target_m=-1.0 if go else 0.3, accel_limits=LIMITS, frogpilot_toggles=toggles,
                    lead_status=True, lead_v=lv, lead_d_rel=d, lead_track_id=7)
    rows.append((t, lc.long_control_state, lc._service_shadow_svc.phase, float(out),
                 bool(lc._service_signals is not None and lc._service_signals.lead_confirmed_stopped)))
  return rows


def test_4904_frame_sequence_keeps_stopping_and_the_wire_while_the_lead_is_stopped():
  rows = lc_4904()
  go = [r for r in rows if r[0] >= 8.5]
  assert any(r[2] == Phase.RELEASE for r in go) and any(r[2] == Phase.RAMP_TO_HOLD for r in go)
  # StopReq precondition at rest: `stopping` on every frame (the Hyundai sender clears StopReq only on a state exit, gas or v > 0.1)
  assert all(r[1] == LongCtrlState.stopping for r in go), "a legacy `starting` frame escaped under the re-hold"
  assert all(r[4] for r in go)
  assert all(r[2] != Phase.INACTIVE for r in go)
  assert max(r[3] for r in go) <= 0.0
  assert go[-1][3] == pytest.approx(P.A_HOLD_SECURE)


def test_4904_flag_off_pin_reproduces_the_race(monkeypatch):
  monkeypatch.setattr(stopping_flags, "RELEASE_END_STOPPED_LEAD_REHOLD", False)
  rows = lc_4904()
  k = next(i for i, r in enumerate(rows) if r[0] >= 8.5 and r[2] == Phase.INACTIVE)
  assert rows[k][1] == LongCtrlState.stopping           # the RELEASE-end frame itself stays stopping (service phase RELEASE)
  assert rows[k + 1][1] == LongCtrlState.starting and rows[k + 1][4], "car build: `starting` while the lead still reads stopped"


def test_4904_real_departure_launches_after_the_latch_drop():
  rows = lc_4904(depart=12.0)
  pre = [r for r in rows if 8.5 <= r[0] < 12.0]
  assert all(r[1] == LongCtrlState.stopping for r in pre)
  t_drop = next(r[0] for r in rows if r[0] >= 12.0 and not r[4])
  t_start = next(r[0] for r in rows if r[0] >= 12.0 and r[1] == LongCtrlState.starting)
  assert t_start - t_drop <= -P.A_HOLD_SECURE / P.J_GO + 0.05


def test_4904_doppler_flicker_keeps_stopping_through_every_release():
  # Astra 2026-10-04 P1 reproduction: after the first re-hold a 0.1 s Doppler flicker lets the stale go through again; the next
  # RELEASE end must not hand the wire to a legacy launcher while the lead still reads stopped
  rows = lc_4904(flicker=True)
  go = [r for r in rows if r[0] >= 8.5]
  assert sum(1 for a, b in zip(go, go[1:], strict=False) if a[2] != Phase.RELEASE and b[2] == Phase.RELEASE) >= 2
  assert all(r[1] == LongCtrlState.stopping for r in go), "a legacy `starting` frame escaped after a flicker"
  assert all(r[2] != Phase.INACTIVE for r in go)
  assert max(r[3] for r in go) <= 0.0
