"""HOLD_DEPARTURE_RELEASE (stopping_flags, cycle 2026-10-10 C6; drives 0000225d 1229.52 and 0000203a 553.38).

At rest in blended mode the planner's MPC follows the model's flat trajectory: aTarget stays 0..+0.2 with shouldStop set while the
model itself says go, so the HOLD's planner go never comes. LongControl's own chain had released the stop and ran pid under the hold.
The physical-go dwell restarted on every outward-held radar step (StopContext outward persistence), so it never fired, and a release
alone re-entered on the same standstill shouldStop. With the flag, under a pid hold only: the dwell counts live geometry with the
model's stop bit clear, a planner that does not brake and a lead that does not brake; the RELEASE ends in pid on the planner's target, and the standstill
shouldStop does not re-enter while the chain stays released, the lead keeps moving and the model bit stays clear."""
import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.stop_context import StopContext
from openpilot.selfdrive.controls.lib.stopping_service import Phase, ServiceParams, StoppingService
from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles
from openpilot.selfdrive.controls.lib.tests.test_release_end_rehold import Radar

P = ServiceParams()
DT = 0.01
LIMITS = (-3.5, 2.0)
DEP = 3.0          # s: the lead starts to leave
A_T = 0.05         # the blended planner's standstill aTarget behind a departing lead (225d: +0.02..+0.08, shouldStop set)


@pytest.fixture(autouse=True)
def _flag_on(monkeypatch):
  monkeypatch.setattr(stopping_flags, "HOLD_DEPARTURE_RELEASE", True)


def leaving(v_lead=1.0, d0=7.0, stop_at=None):
  """Range (quantised by Radar) and Doppler of a lead that leaves at v_lead from DEP (and stops at stop_at, when given)."""
  def v(t):
    return v_lead if DEP <= t and (stop_at is None or t < stop_at) else 0.0

  def gap(t):
    t_end = t if stop_at is None else min(t, stop_at)
    return d0 + v_lead * max(t_end - DEP, 0.0)
  return Radar(gap, v)


class Seam:
  """StopContext + StoppingService with the ego at rest, the planner's standstill stop (shouldStop, A_T) and the model / chain inputs."""

  def __init__(self, radar):
    self.radar, self.ctx, self.svc = radar, StopContext(), StoppingService()
    self.t, self.cmd, self.phases = 0.0, 0.0, []

  def step(self, model_stop=False, chain_released=True, a_target=A_T, should_stop=True, lead_a=0.0):
    d, lv = self.radar.read(self.t)
    sig = self.ctx.update(v_ego=0.0, a_ego=0.0, a_cmd=self.cmd, lead_status=True, lead_v=lv, lead_d_rel=d, lead_track_id=7,
                          standstill=True, dt=DT)
    r = self.svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=a_target, should_stop=should_stop, dts_planner=None,
                        planner_min_limit=LIMITS[0], signals=sig, lead_status=True, lead_v=lv, dt=DT, wire_accel=self.cmd,
                        a_target_trajectory=a_target, lead_a=lead_a, model_stop=model_stop, chain_released=chain_released)
    # INACTIVE: the wire is LongControl's (pid on the planner's target)
    self.cmd = r.accel if r.active else a_target
    self.phases.append(r.phase)
    self.t += DT
    return r

  def run(self, seconds, **kw):
    for _ in range(int(round(seconds / DT))):
      r = self.step(**kw)
    return r

  def held(self):
    """Settle the hold first: the planner's standstill stop until DEP (the lead still waits)."""
    self.run(DEP - 0.01, a_target=-0.05)
    assert self.svc.phase == Phase.HOLD and self.cmd == pytest.approx(P.A_HOLD_SECURE)
    return self

  def first(self, phase):
    return next((round(k * DT, 2) for k, p in enumerate(self.phases) if p == phase), None)


def test_departure_under_a_pid_hold_releases_and_does_not_re_enter():
  s = Seam(leaving()).held()
  s.run(5.0)
  t_rel = s.first(Phase.RELEASE)
  # gap growth past the anchor + 0.3 m (~0.3-0.4 s at 1 m/s with 0.1 m range steps) + the 0.5 s dwell over outward-held frames
  assert t_rel is not None and DEP + 0.5 <= t_rel <= DEP + 1.2
  k_end = next(k for k, p in enumerate(s.phases) if k * DT > t_rel and p == Phase.INACTIVE)
  assert (k_end * DT - t_rel) <= -P.A_HOLD_SECURE / P.J_GO + 0.05, "the RELEASE ramp is today's J_GO"
  assert all(p == Phase.INACTIVE for p in s.phases[k_end:]), "the standstill shouldStop re-entered the service"


def test_flag_off_keeps_the_hold_while_the_lead_leaves(monkeypatch):
  monkeypatch.setattr(stopping_flags, "HOLD_DEPARTURE_RELEASE", False)
  s = Seam(leaving()).held()
  s.run(5.0)
  assert s.first(Phase.RELEASE) is None, "car build: the measured-only dwell restarts at every outward radar step"
  assert s.svc.ev.lead_departure_s < P.RELEASE_LEAD_CONFIRM_S


@pytest.mark.parametrize("kw", [dict(model_stop=True), dict(chain_released=False), dict(a_target=-0.02), dict(a_target=0.0)],
                         ids=["model_stop", "stopping_hold", "planner_brakes", "planner_zero"])
def test_no_release_while_anything_asks_to_stop(kw):
  s = Seam(leaving()).held()
  s.run(5.0, **kw)
  assert s.first(Phase.RELEASE) is None


@pytest.mark.parametrize("lead_a", [-0.5, float("nan")], ids=["braking", "no_estimate"])
def test_no_release_while_the_lead_brakes(lead_a):
  # judges' sgs grid: the lead pulled away and was already braking to a stop (-2.4..-3.0) when the dwell completed; the release
  # dipped the hold to -0.02 at rest and re-held 0.6 s later. aLeadK below ATTR_LEAD_BRAKING (or none) is no departure
  s = Seam(leaving()).held()
  s.run(5.0, lead_a=lead_a)
  assert s.first(Phase.RELEASE) is None


@pytest.mark.parametrize("v_lead", [0.0, 0.3, 0.5])
def test_no_release_toward_a_stationary_or_crawling_lead(v_lead):
  # Doppler at or below the pull bar (RELEASE_LEAD_PULL_MPS 0.5): a crawler is not a departure
  s = Seam(leaving(v_lead=v_lead)).held()
  s.run(6.0)
  assert s.first(Phase.RELEASE) is None


def test_no_release_on_standstill_range_drift_with_doppler_zero():
  # 00002232 4904: range drift 4.6 -> 5.1 m with Doppler 0 arms gap_grew; without Doppler it is no departure
  s = Seam(Radar(lambda t: 4.6 + 0.1 * min(max(t - DEP, 0.0), 5.0))).held()
  s.run(6.0)
  assert s.first(Phase.RELEASE) is None


def test_no_release_on_a_doppler_burst_without_range_growth():
  s = Seam(Radar(lambda t: 7.0, lambda t: 0.9 if DEP <= t < DEP + 0.4 else 0.0)).held()
  s.run(4.0)
  assert s.first(Phase.RELEASE) is None


def test_model_stop_after_the_release_re_enters_the_hold():
  s = Seam(leaving()).held()
  s.run(2.0)
  assert s.svc.phase == Phase.INACTIVE
  r = s.step(model_stop=True)
  assert r.phase == Phase.RAMP_TO_HOLD, "the model's stop must re-enter at once"


def test_braking_lead_after_the_release_re_enters_the_hold():
  s = Seam(leaving()).held()
  s.run(2.0)
  assert s.svc.phase == Phase.INACTIVE
  r = s.step(lead_a=-0.5)
  assert r.phase == Phase.RAMP_TO_HOLD, "a lead braking after the release ends the mask: the standstill stop re-enters at once"


def test_lead_stopping_again_ends_the_mask():
  # a queue that rolls 1.5 m and stops: once the lead reads stopped the standstill shouldStop re-enters the service
  s = Seam(leaving(stop_at=DEP + 1.5)).held()
  s.run(4.0)
  assert s.first(Phase.RELEASE) is not None
  assert s.phases[-1] in (Phase.RAMP_TO_HOLD, Phase.HOLD)


def test_chain_back_in_stopping_ends_the_mask():
  s = Seam(leaving()).held()
  s.run(2.0)
  assert s.svc.phase == Phase.INACTIVE
  s.step()
  r = s.step(chain_released=False)
  assert r.phase == Phase.RAMP_TO_HOLD


def test_mask_is_episode_scoped():
  s = Seam(leaving()).held()
  s.run(2.0)
  assert s.svc._departed
  s.svc.reset()
  assert not s.svc._departed


# --- LongControl seam: the 225d / 203a class (the chain released the stop and runs pid under the service's hold) ---------------
def make_lc():
  """LongControl in pid (the chain's departing-lead release, human acceleration on) under an owned service HOLD at rest, 7.0 m
  behind the lead."""
  cp = DummyCarParams()
  cp.startingState = True
  lc = LongControl(cp)
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  toggles = DummyFrogPilotToggles()
  toggles.human_acceleration = True
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = P.A_HOLD_SECURE
  lc._service_live_owning = True
  svc = lc._service_shadow_svc
  svc.phase, svc._last_cmd, svc._hold_entry_gap = Phase.HOLD, P.A_HOLD_SECURE, 7.0
  return lc, toggles


def lc_step(lc, toggles, v=0.0, a_target=A_T, should_stop=True, lv=0.0, gap=7.0, lead_a=0.0, track=7, model_stop=False):
  a = float(lc.update(True, DummyCarState(v_ego=v, a_ego=0.0, standstill=v < 0.005), a_target, should_stop, None, LIMITS, toggles,
                      experimental_mode=True, lead_status=True, lead_v=lv, lead_d_rel=gap, lead_a=lead_a, lead_track_id=track,
                      lead_model_prob=0.99, increased_stopped_distance=0.3, a_target_trajectory=a_target, model_should_stop=model_stop))
  return (lc.long_control_state, lc._service_shadow_svc.phase, a)


def lc_pid_hold(depart=True, model_stop=False, v_lead=1.0, veto=None, t_veto=None):
  """make_lc(); from DEP the lead leaves at v_lead while the blended planner keeps its standstill stop (shouldStop, +0.05).
  veto from t_veto: "model" sets the model's stop bit, "planner" brakes (-0.05), "lead" brakes at -1.0 m/s^2 (range and Doppler;
  aLeadK from the first radar frame at or after t_veto)."""
  lc, toggles = make_lc()
  radar = leaving(v_lead) if depart else Radar(lambda t: 7.0, lambda t: 0.2)
  if veto == "lead":
    def gap(t):
      tb = min(max(t - t_veto, 0.0), v_lead)
      return 7.0 + v_lead * max(min(t, t_veto) - DEP, 0.0) + v_lead * tb - 0.5 * tb * tb
    radar = Radar(gap, lambda t: 0.0 if t < DEP else max(v_lead - max(t - t_veto, 0.0), 0.0))
  rows, lead_a = [], 0.0
  for k in range(int(7.0 / DT)):
    t = k * DT
    vetoed = veto is not None and t >= t_veto - 1e-9
    d, lv = radar.read(t)
    if k % 5 == 0:   # aLeadK arrives with the radar frame
      lead_a = -1.0 if veto == "lead" and vetoed else 0.0
    rows.append((t, *lc_step(lc, toggles, a_target=-0.05 if veto == "planner" and vetoed else A_T, lv=lv, gap=d, lead_a=lead_a,
                             model_stop=model_stop or (veto == "model" and vetoed))))
  return rows


@pytest.mark.parametrize("c4", [False, True], ids=["pid_hold", "c4_stopping_hold"])
def test_pid_hold_departure_hands_the_car_to_pid_without_a_starting_step(monkeypatch, c4):
  # c4: SERVICE_HOLD_ENTERS_STOPPING turns the same hold into a `stopping` hold (StopReq); its latch counts as the released chain and
  # its at-rest bridge yields to the departure release (bundle integration, cycle 2026-10-10)
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "SERVICE_HOLD_ENTERS_STOPPING", c4)
  rows = lc_pid_hold()
  k_rel = next(k for k, r in enumerate(rows) if r[2] == Phase.RELEASE)
  assert DEP + 0.5 <= rows[k_rel][0] <= DEP + 1.2
  assert all(r[1] == (LongCtrlState.stopping if c4 else LongCtrlState.pid) for r in rows[1:k_rel])
  after = rows[k_rel:]
  assert all(r[1] != LongCtrlState.starting for r in after), "a departure release must not pass through `starting`"
  # pid from the frame after the release decision (with SERVICE_HOLD_ENTERS_STOPPING the decision frame itself is still `stopping`)
  assert all(r[1] == LongCtrlState.pid for r in rows[k_rel + 1:])
  assert all(r[2] in (Phase.RELEASE, Phase.INACTIVE) for r in after), "the standstill shouldStop re-entered the service"
  assert rows[-1][2] == Phase.INACTIVE and rows[-1][3] == pytest.approx(A_T, abs=0.05), "the wire ends on the planner's target"
  assert all(b[3] >= a[3] - 1e-9 for a, b in zip(after, after[1:], strict=False)), "the release ramp is monotone"


@pytest.mark.parametrize("c4", [False, True], ids=["pid_hold", "c4_stopping_hold"])
def test_pid_hold_flag_off_keeps_the_hold(monkeypatch, c4):
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "SERVICE_HOLD_ENTERS_STOPPING", c4)
  monkeypatch.setattr(stopping_flags, "HOLD_DEPARTURE_RELEASE", False)
  rows = lc_pid_hold()
  assert all(r[2] == Phase.HOLD for r in rows)
  assert rows[-1][3] == pytest.approx(P.A_HOLD_SECURE)


@pytest.mark.parametrize("c4", [False, True], ids=["pid_hold", "c4_stopping_hold"])
@pytest.mark.parametrize("depart,model_stop", [(True, True), (False, False)], ids=["model_stop", "creeper"])
def test_pid_hold_keeps_the_hold_without_a_model_agreed_departure(monkeypatch, depart, model_stop, c4):
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "SERVICE_HOLD_ENTERS_STOPPING", c4)
  rows = lc_pid_hold(depart=depart, model_stop=model_stop)
  assert all(r[2] == Phase.HOLD for r in rows)


BUNDLE_FLAGS = ("RELEASE_GO_EVIDENCE_KEEP", "OWNED_STARTING_TRANSIT", "SERVICE_HOLD_ENTERS_STOPPING", "STOPREQ_SETTLED_SET",
                "HOLD_DEPARTURE_RELEASE")


@pytest.mark.parametrize("all_flags", [False, True], ids=["c6", "all_flags"])
@pytest.mark.parametrize("veto", ["model", "planner", "lead"])
def test_pid_hold_veto_mid_dwell_keeps_the_hold(monkeypatch, veto, all_flags):
  # Astra bundle-2 P1 (reproduce_c6_veto.py): the departure dwell had counted outward-held frames into the measured-only one, so the
  # measured frames after a veto (the model's stop bit or the planner at -0.05 from 4.08 s; the lead braking at -1.0 from 4.08 s, its
  # aLeadK on the 4.10 s radar frame) completed it and released at 4.10 s. Its own dwell restarts on the veto: the hold stays, as
  # flag-off.
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  for f in BUNDLE_FLAGS if all_flags else ():
    monkeypatch.setattr(stopping_flags, f, True)
  rows = lc_pid_hold(v_lead=0.6)
  assert next(r[0] for r in rows if r[2] == Phase.RELEASE) == pytest.approx(4.10), "the un-vetoed departure releases at 4.10 s"
  rows = lc_pid_hold(v_lead=0.6, veto=veto, t_veto=4.08)
  assert all(r[2] == Phase.HOLD for r in rows), f"released at {next(r[0] for r in rows if r[2] != Phase.HOLD):.2f} s"
  assert all(r[3] == pytest.approx(P.A_HOLD_SECURE) for r in rows)


@pytest.mark.parametrize("c4", [False, True], ids=["pid_hold", "c4_stopping_hold"])
def test_band_exit_ends_the_exemption_for_a_replacement_lead(monkeypatch, c4):
  # Astra bundle-2 P2 (reproduce_c6_lifetime.py): LongControl's out-of-band path resets an INACTIVE service's StopContext but not the
  # service; the departure exemption earned behind track 7 passed to track 8 and suppressed its stop. Real LongControl band exit.
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "SERVICE_HOLD_ENTERS_STOPPING", c4)
  lc, toggles = make_lc()
  radar = leaving()
  for k in range(600):
    d, lv = radar.read(k * DT)
    lc_step(lc, toggles, a_target=A_T if k >= 300 else -0.05, lv=lv, gap=d)
  assert lc._service_shadow_svc._departed and lc._service_shadow_svc.phase == Phase.INACTIVE
  for k in range(300):   # out of the band (2.5 m/s) on the planner's go; the lead is replaced above it
    lc_step(lc, toggles, v=k * DT, a_target=0.2, should_stop=False, lv=4.0, gap=10.0 + k * DT)
  for _ in range(100):
    lc_step(lc, toggles, v=3.0, a_target=0.0, should_stop=False, lv=3.0, gap=15.0, track=8)
  assert not lc._service_shadow_svc._departed
  for k in range(1, 296):   # back through the band behind the moving replacement
    v = 3.0 - k * DT
    lc_step(lc, toggles, v=v, a_target=-1.0, should_stop=False, lv=v + 0.65, gap=15.0 + k * 0.0065, track=8)
  for k in range(80):   # the planner stops at rest behind the moving replacement (model bit clear, +0.05)
    r = lc_step(lc, toggles, lv=0.65, gap=17.0 + k * 0.0065, track=8)
  assert r[1] == Phase.HOLD and r[2] == pytest.approx(P.A_HOLD_SECURE)
  if c4:
    assert r[0] == LongCtrlState.stopping
