"""Entry correction boundaries; recorded-input replay and synthetic plants are separate evidence."""
import math
import random
from dataclasses import replace

import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.stop_context import StopSignals
from openpilot.selfdrive.controls.lib.stopping_service import GOV_LAG, Phase, StoppingService
from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles


@pytest.fixture
def entry(monkeypatch):
  monkeypatch.setattr(stopping_flags, "SERVICE_APPROACH_LAW", "governor")
  monkeypatch.setattr(stopping_flags, "GOVERNOR_PROFILE_REFERENCE", True)
  monkeypatch.setattr(stopping_flags, "GOVERNOR_RECOVERY_BRAKE", False)
  monkeypatch.setattr(stopping_flags, "ATTRIBUTED_SAFETY", "off")
  return dict(engaged=True, v_ego=2.22, a_ego=-1.0, a_target=None, should_stop=True,
              dts_planner=2.9, planner_min_limit=-3.5,
              signals=StopSignals(7.2, "measured", False, False, 0.0, False, True, True, 2.0, True),
              lead_status=True, lead_v=-0.2, increased_stopped_distance=0.3, wire_accel=-1.0)


def test_entry_correction_only_reduces_new_braking(entry):
  rng = random.Random(57)
  changed = 0
  for _ in range(1200):
    kw = {**entry, "v_ego": rng.uniform(0.51, 2.49), "a_ego": -rng.uniform(0.01, 3.5),
          "wire_accel": rng.uniform(-3.0, -0.03), "lead_v": rng.uniform(-1.0, 1.0),
          "signals": replace(entry["signals"], d_gap=rng.uniform(3.0, 15.0), a_coast=rng.uniform(-0.5, 0.5))}
    raw = StoppingService().update(**{**kw, "a_ego": 0.0})
    corrected = StoppingService().update(**kw)
    a0, a1 = raw.debug["a_phase"], corrected.debug["a_phase"]
    assert a0 <= a1 <= max(kw["wire_accel"], a0)
    for lane in ("a_kin", "a_plan", "a_monitor", "a_barrier", "a_gov", "gov_v_ref", "gov_d"):
      assert corrected.debug[lane] == raw.debug[lane]
    if raw.debug["safety_binding"]:
      assert corrected.debug["safety_binding"]
    changed += a1 > a0 + 1e-6
  assert changed > 100


@pytest.mark.parametrize("a_ego", [0.0, 0.4, float("nan"), float("inf"), -float("inf"), None])
def test_no_deceleration_evidence_cannot_soften_entry(entry, a_ego):
  expected = StoppingService().update(**{**entry, "a_ego": 0.0})
  actual = StoppingService().update(**{**entry, "a_ego": a_ego})
  assert math.isfinite(actual.accel)
  assert actual == expected


@pytest.mark.parametrize("source,outward,dropout", [("held", False, False), ("decay", False, True), ("none", False, False)])
def test_untrusted_geometry_cannot_earn_entry_credit(entry, source, outward, dropout):
  kw = {**entry, "signals": replace(entry["signals"], gap_source=source, gap_hold_outward=outward, dropout_active=dropout)}
  assert StoppingService().update(**kw) == StoppingService().update(**{**kw, "a_ego": 0.0})


def test_outward_hold_can_earn_the_same_credit_as_its_measured_lower_bound(entry):
  held = {**entry, "signals": replace(entry["signals"], gap_source="held", gap_hold_outward=True)}
  result = StoppingService().update(**held)
  assert result.debug["a_phase"] == StoppingService().update(**entry).debug["a_phase"]
  assert result.debug["a_phase"] > StoppingService().update(**{**held, "a_ego": 0.0}).debug["a_phase"]


@pytest.mark.parametrize("raising", [False, True])
def test_insufficient_braking_or_failed_prediction_cannot_earn_credit(entry, monkeypatch, raising):
  kw = {**entry, "signals": replace(entry["signals"], d_gap=4.5)}
  assert StoppingService().update(**kw) == StoppingService().update(**{**kw, "a_ego": 0.0})
  def failed_prediction(*args, **kwargs):
    if raising:
      raise RuntimeError("prediction unavailable")
    return None

  monkeypatch.setattr("openpilot.selfdrive.controls.lib.stopping_service.predictive_lead_demand", failed_prediction)
  assert StoppingService().update(**entry) == StoppingService().update(**{**entry, "a_ego": 0.0})


def test_correction_expires_and_reset_starts_a_new_entry(entry):
  service = StoppingService()
  first = service.update(**entry)
  raw_phase = StoppingService().update(**{**entry, "a_ego": 0.0}).debug["a_phase"]
  assert first.debug["a_phase"] > raw_phase + 0.05
  for _ in range(450):
    last = service.update(**entry)
  assert abs(last.debug["a_phase"] - raw_phase) < 0.00003
  service.reset()
  assert service.update(**entry) == first


def test_warm_reseed_does_not_restart_the_entry_clock(entry):
  service = StoppingService()
  service.update(**entry)
  for _ in range(450):
    service.update(**entry)
  before = service.ev.entry_t
  service.reseed_takeover(-0.8, -3.5)
  after = service.update(**entry)
  raw_phase = StoppingService().update(**{**entry, "a_ego": 0.0}).debug["a_phase"]
  assert service.ev.entry_t == before
  assert service._t - before > 9 * GOV_LAG
  assert abs(after.debug["a_phase"] - raw_phase) < 0.00003


@pytest.mark.parametrize("hazard", ["planner", "close_lead", "reversal", "dropout"])
def test_safety_can_deepen_immediately_at_entry(entry, hazard):
  kw = {**entry, "wire_accel": -0.3}
  if hazard == "planner":
    kw.update(a_target=-3.0, a_target_trajectory=-3.0)
  elif hazard == "close_lead":
    kw["signals"] = replace(entry["signals"], d_gap=3.3)
  elif hazard == "reversal":
    kw.update(lead_v=-1.5, signals=replace(entry["signals"], d_gap=4.0))
  else:
    kw.update(v_ego=0.8, wire_accel=-0.03, signals=replace(entry["signals"], d_gap=10.0, dropout_active=True))
  result = StoppingService().update(**kw)
  assert result.debug["safety_binding"]
  assert result.accel == pytest.approx(kw["wire_accel"] - 0.08)


@pytest.mark.parametrize("law,lead", [("legacy", True), ("governor", False)])
def test_other_approach_paths_are_unchanged(entry, monkeypatch, law, lead):
  monkeypatch.setattr(stopping_flags, "SERVICE_APPROACH_LAW", law)
  kw = {**entry, "lead_status": lead}
  if not lead:
    kw["signals"] = replace(entry["signals"], d_gap=None, gap_source="none")
  assert StoppingService().update(**kw) == StoppingService().update(**{**kw, "a_ego": 0.0})


def test_armed_terminal_descent_keeps_its_command(entry):
  results = []
  for a in (-2.0, 0.0):
    service = StoppingService()
    service.phase = Phase.APPROACH_GLIDE
    service.ev.on_entry(0.0, 0.8)
    service._last_cmd = -0.45
    service._floor_v_peak = 0.8
    results.append(service.update(**{**entry, "v_ego": 0.4, "a_ego": a}))
  assert results[0] == results[1]


def test_weak_braking_entry_preserves_floor_and_hold(monkeypatch):
  # This stress cell rejects credit for all measured braking: that prototype rests at 2.994 m.
  # Use the actual estimator/controller and one control tick of delay. This is not a vehicle model.
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "SERVICE_APPROACH_LAW", "governor")
  monkeypatch.setattr(stopping_flags, "GOVERNOR_RECOVERY_BRAKE", True)
  monkeypatch.setattr(stopping_flags, "ATTRIBUTED_SAFETY", "live")
  control = LongControl(DummyCarParams())
  control._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  toggles = DummyFrogPilotToggles()
  v, gap, wire = 2.4, 7.0, -2.0
  a = 0.7 * wire + 0.45
  control.last_output_accel = wire
  stopped = None
  for k in range(1200):
    delayed = wire
    plan = -min(v * v / (2 * max(gap - 4.3, 0.3)), 1.5)
    wire = control.update(True, DummyCarState(v_ego=v, a_ego=a, standstill=v < 0.005), plan,
      True, max(gap - 4.3, 0), (-3.5, 2), toggles, experimental_mode=True,
      lead_status=True, lead_v=0, lead_d_rel=gap, lead_a=0, lead_track_id=7, lead_model_prob=0.99,
      increased_stopped_distance=0.3, a_target_trajectory=plan)
    a += (0.7 * delayed + 0.45 - a) * -math.expm1(-0.01 / 0.4)
    next_v = max(v + a * 0.01, 0)
    gap -= (v + next_v) * 0.005
    a, v = (next_v - v) / 0.01, next_v
    assert gap >= 3.0
    if stopped is not None:
      assert v == 0
      if k - stopped >= 100:
        break
    elif v == 0:
      stopped = k
  assert stopped is not None and k - stopped >= 100
