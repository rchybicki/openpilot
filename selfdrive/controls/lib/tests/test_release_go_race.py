"""RELEASE_GO_EVIDENCE_KEEP (stopping_flags, cycle 2026-10-10 C2): the HOLD -> RELEASE race.

The planner's go pulse (0.3-0.38 s) must coincide with the lead pull (syn_launch). A recent go still pairs with a later pull; the
physical-go dwell is unchanged."""
import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.stop_context import StopSignals
from openpilot.selfdrive.controls.lib.stopping_service import Phase, ServiceParams, StoppingService
from openpilot.selfdrive.controls.lib.tests.test_hold_departure_release import BUNDLE_FLAGS, lc_step, make_lc
from openpilot.selfdrive.controls.lib.tests.test_release_end_rehold import Radar, Seam, travel

P = ServiceParams()
DT = 0.01
DEP = 20.0      # the lead starts at 1 m/s^2


@pytest.fixture(autouse=True)
def _flag_off(monkeypatch):
  monkeypatch.setattr(stopping_flags, "RELEASE_GO_EVIDENCE_KEEP", False)


def departure(pulse=(0.05, 0.40), after=0.10, a=1.0, lead_delay=0.0, drift=0.0, t_end=4.0):
  """HOLD behind a stopped lead at 4.6 m; at DEP the planner asks to go for pulse (aT 0.27) then sits at `after`; the lead departs at
  a m/s^2 from DEP + lead_delay (a = 0: it stays; drift = standstill range drift after the pulse, Doppler 0). Returns (release time
  after DEP, seam)."""
  t_lead = DEP + lead_delay

  def gap(t):
    return 4.6 + (travel(t - t_lead, a=a) if a > 0.0 else 0.0) + drift * min(max(t - DEP - pulse[1], 0.0), 0.5) / 0.5
  s = Seam(Radar(gap, lambda t: min(max(a * (t - t_lead), 0.0), 1.5)))
  s.run(DEP, -0.3, True)
  assert s.svc.phase == Phase.HOLD
  while s.t < DEP + t_end:
    tau = s.t - DEP
    at = 0.27 if pulse[0] <= tau < pulse[1] else (after if tau >= pulse[1] else -0.3)
    r, _ = s.step(at, at <= 0.2)
    if r.phase == Phase.RELEASE:
      return round(s.t - DEP, 2), s
  return None, s


def test_c2_go_pulse_that_ends_before_the_pull_still_releases(monkeypatch):
  t_off, _ = departure()
  assert t_off is None, "car build: the pulse ended 0.1 s before the pull; aT 0.10 never releases (and the dwell keeps restarting)"
  monkeypatch.setattr(stopping_flags, "RELEASE_GO_EVIDENCE_KEEP", True)
  t_on, _ = departure()
  # lv - v > 0.5 on the first radar frame past 0.5 s (20 Hz)
  assert t_on is not None and t_on <= 0.5 + 0.05 + 2 * DT


def test_c2_identity_when_the_pulse_meets_the_pull(monkeypatch):
  off, s_off = departure(pulse=(0.05, 1.5))
  monkeypatch.setattr(stopping_flags, "RELEASE_GO_EVIDENCE_KEEP", True)
  on, s_on = departure(pulse=(0.05, 1.5))
  assert off == on and s_off.cmds == s_on.cmds


@pytest.mark.parametrize("kw", [
  dict(a=0.0),                       # the lead never moves: a go pulse alone is not release evidence
  dict(a=0.0, drift=0.5),            # standstill range drift (Doppler 0) after a refused go: never pairs with the remembered go
  dict(pulse=(0.05, 0.40), after=-0.2),   # the planner brakes again: the remembered go is withdrawn
  dict(lead_delay=0.5, t_end=1.2),   # the pull comes 0.6 s after the pulse ended (> RELEASE_GO_KEEP_S)
])
def test_c2_remembered_go_never_releases_without_a_recent_go_and_a_doppler_pull(monkeypatch, kw):
  monkeypatch.setattr(stopping_flags, "RELEASE_GO_EVIDENCE_KEEP", True)
  t_on, s = departure(**({"t_end": 0.9} | kw))
  assert t_on is None, f"released at DEP +{t_on} s"
  assert all(p == Phase.HOLD for p in s.phases[-90:])


def test_c2_physical_go_dwell_is_unchanged(monkeypatch):
  # no planner go at all (aT 0.10): a smooth departure's outward-held frames restart the dwell with the flag as without it (keeping
  # it through them released 2235_s55/s56 toward a queue lead that rolled 1.7 m and stopped again)
  off, s_off = departure(pulse=(9.0, 9.0), t_end=6.0)
  monkeypatch.setattr(stopping_flags, "RELEASE_GO_EVIDENCE_KEEP", True)
  on, s_on = departure(pulse=(9.0, 9.0), t_end=6.0)
  assert off is None and on is None
  assert s_off.cmds == s_on.cmds


def test_c2_held_frames_alone_never_confirm_the_departure(monkeypatch):
  # test_held_gap_prediction_does_not_confirm_physical_departure under the flag
  monkeypatch.setattr(stopping_flags, "RELEASE_GO_EVIDENCE_KEEP", True)
  svc = StoppingService()
  svc.phase = Phase.HOLD
  svc._last_cmd = P.A_HOLD_SECURE
  svc._hold_entry_gap = 4.0
  held = StopSignals(d_gap=5.0, gap_source="held", gap_hold_outward=True, dropout_active=False, a_coast=0.0,
                     wheel_stop_latched=True, lead_confirmed_stopped=False, lead_stopped_for_entry=False)
  for _ in range(int(2.0 * P.RELEASE_LEAD_CONFIRM_S / DT)):
    r = svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.4, should_stop=True, dts_planner=None, planner_min_limit=-3.5,
                   signals=held, lead_status=True, lead_v=0.8, dt=DT, wire_accel=P.A_HOLD_SECURE)
  assert r.phase == Phase.HOLD
  assert r.debug["lead_departure_confirm_s"] == 0.0


def stop_go_stop(t):
  """(range, Doppler, aLeadK) of a lead 7.0 m ahead that pulls at +1.5 m/s^2 for 0.4 s from 3.0 s, then brakes at -1.0 to rest."""
  tau = t - 3.0
  if tau < 0.0:
    return 7.0, 0.0, 0.0
  if tau < 0.4:
    return 7.0 + 0.75 * tau * tau, 1.5 * tau, 1.5
  tb = min(tau - 0.4, 0.6)
  return 7.12 + 0.6 * tb - 0.5 * tb * tb, max(0.6 - tb, 0.0), -1.0 if tb < 0.6 else 0.0


@pytest.mark.parametrize("all_flags", [False, True], ids=["c2", "all_flags"])
def test_c2_remembered_go_never_releases_toward_a_braking_lead(monkeypatch, all_flags):
  # Astra bundle-2 P2 (reproduce_c2_braking.py): the planner's go (+0.27, 3.05-3.29 s) ends, 0 until 3.45 s, then +0.10. At 3.45 s
  # the Doppler pull (0.55 m/s) meets the remembered go while aLeadK reads -1.0: the release went -0.70 -> -0.004 toward a lead that
  # stopped 0.30 m further. A braking lead contradicts the remembered go: the hold stays, as flag-off.
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "RELEASE_GO_EVIDENCE_KEEP", True)
  for f in BUNDLE_FLAGS if all_flags else ():
    monkeypatch.setattr(stopping_flags, f, True)
  lc, toggles = make_lc()
  radar = Radar(lambda t: stop_go_stop(t)[0], lambda t: stop_go_stop(t)[1])
  lead_a = 0.0
  for k in range(550):
    t = k * DT
    d, lv = radar.read(t)
    if k % 5 == 0:   # aLeadK arrives with the radar frame
      lead_a = stop_go_stop(t)[2]
    a_target = 0.27 if 3.05 <= t < 3.3 else 0.10 if t >= 3.45 else 0.0 if t >= 3.3 else -0.3
    _, phase, wire = lc_step(lc, toggles, a_target=a_target, lv=lv, gap=d, lead_a=lead_a)
    if k == 345:   # the pull and the remembered go coincide: only the lead's braking stands between them
      assert lv > P.RELEASE_LEAD_PULL_MPS and t - lc._service_shadow_svc.ev.planner_go_t <= P.RELEASE_GO_KEEP_S
    assert phase == Phase.HOLD and wire == pytest.approx(P.A_HOLD_SECURE), f"t={t:.2f}"
