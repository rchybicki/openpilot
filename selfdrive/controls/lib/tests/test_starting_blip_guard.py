"""STARTING_BLIP_GUARD (cycle 2026-10-18 WS RELEASE B): a stop-target flicker at speed takes LongControl stopping -> starting -> pid in one
frame each; the 'starting' frame sent startAccel +0.70 (22d4 4490.95 / 4491.09 / 4491.17 at 2.8-3.2 m/s under a -2.25 plan, 22cf 844.05;
19 of 53 such frames on the cached drives reached SCC12). With the flag the frame holds the previous wire at vEgo >= V_SLEW_MAX; below it
the low-speed slew already bounds the frame, and the state machine is the same either way."""
import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.stopping_guard import V_SLEW_MAX
from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles


def _transit(monkeypatch, on, v, owned=False, n=30, k_go=5, wire0=-1.2, mode="LIVE"):
  """A rolling stop at v (the legacy chain, or the service owning with owned=True) whose stop drops at k_go: per frame (state, wire).
  mode SHADOW keeps the service out at every speed (the legacy chain below the service band too)."""
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", mode)
  monkeypatch.setattr(stopping_flags, "STARTING_BLIP_GUARD", on)
  cp = DummyCarParams()
  cp.startingState = True
  lc = LongControl(cp)
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  toggles = DummyFrogPilotToggles()
  lc.long_control_state = LongCtrlState.pid if owned else LongCtrlState.stopping
  lc.last_output_accel = wire0
  rows = []
  for k in range(n):
    stop = k < k_go
    lead = dict(lead_status=True, lead_v=0.0 if stop else v + 1.0, lead_d_rel=5.6, lead_a=0, lead_track_id=7, lead_model_prob=0.99) if owned \
      else dict(lead_status=False)
    w = lc.update(True, DummyCarState(v_ego=v, a_ego=wire0), wire0 if stop else 0.3, stop, 1.5 if stop else None, (-3.5, 2), toggles,
                  experimental_mode=True, increased_stopped_distance=0.3, a_target_trajectory=wire0 if stop else 0.3, **lead)
    rows.append((lc.long_control_state, float(w)))
  return rows


@pytest.mark.parametrize("v, mode", [(V_SLEW_MAX, "SHADOW"), (2.0, "SHADOW"), (3.0, "SHADOW"), (2.6, "LIVE"), (3.2, "LIVE")])
def test_transit_at_speed_holds_the_previous_wire(monkeypatch, v, mode):
  # the legacy chain's transit (the car's blips: the service had not entered, 2.4-3.2 m/s)
  off, on = _transit(monkeypatch, False, v, mode=mode), _transit(monkeypatch, True, v, mode=mode)
  assert [s for s, _ in on] == [s for s, _ in off]                               # the state machine is untouched
  k = next(i for i, (s, _) in enumerate(off) if s == LongCtrlState.starting)
  assert off[k - 1][0] == LongCtrlState.stopping and off[k + 1][0] == LongCtrlState.pid
  assert off[k][1] == pytest.approx(DummyFrogPilotToggles().startAccel)          # today: the launch command for one frame
  assert on[k][1] == pytest.approx(on[k - 1][1])                                 # the guard: the previous wire
  assert [w for _, w in on[:k]] == [w for _, w in off[:k]] and [w for _, w in on[k + 1:]] == [w for _, w in off[k + 1:]]


@pytest.mark.parametrize("mode", ["LIVE", "SHADOW"])
@pytest.mark.parametrize("v", [0.0, 0.5, 0.9, V_SLEW_MAX - 0.01])
def test_below_the_slew_band_nothing_changes(monkeypatch, v, mode):
  # a launch from rest or a slow roll: the low-speed slew bounds the 'starting' frame today; the guard does not touch it
  on = _transit(monkeypatch, True, v, mode=mode)
  assert LongCtrlState.starting in [s for s, _ in on] and on == _transit(monkeypatch, False, v, mode=mode)


@pytest.mark.parametrize("v", [1.25, 2.0])
def test_owned_transit_is_unchanged(monkeypatch, v):
  # OWNED_STARTING_TRANSIT: the service keeps the wire through the transit already; the guard adds nothing
  on = _transit(monkeypatch, True, v, owned=True, n=60, k_go=40)
  assert LongCtrlState.starting in [s for s, _ in on] and on == _transit(monkeypatch, False, v, owned=True, n=60, k_go=40)


def test_transit_holds_a_trimmed_wire(monkeypatch):
  # Astra review P1: after the tracking trim has learned an offset (pure feed-forward gain, a steady -1.2 demand the car under-delivers),
  # the held frame is the published wire including that trim (reset() zeroes the trim, so nothing is added or removed)
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "STARTING_BLIP_GUARD", True)
  cp = DummyCarParams()
  cp.longitudinalTuning.kpV = [0.0]
  cp.startingState = True
  lc = LongControl(cp)
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  toggles = DummyFrogPilotToggles()

  def step(stop, target):
    return lc.update(True, DummyCarState(v_ego=3., a_ego=-.8), target, stop, 1.5 if stop else -1., (-3.5, 2), toggles,
                     experimental_mode=True, a_target_trajectory=target, lead_status=False)
  for _ in range(300):
    step(False, -1.2)
  before = step(True, -1.2)
  assert lc.long_control_state == LongCtrlState.stopping and lc._trim_wire < -.1
  after = step(False, .3)
  assert lc.long_control_state == LongCtrlState.starting and after == pytest.approx(before)
