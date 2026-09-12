"""Restrictions on profile recovery; these are controller tests, not a model of the Santa Fe."""
import random
from dataclasses import replace

import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.stop_context import StopSignals
from openpilot.selfdrive.controls.lib.stopping_service import GOV_A_C, StoppingService


def _step(monkeypatch, enabled, v, lv, gap, coast=0.0, source="measured", outward=False, dropout=False,
          lead=True, isd=0.3, planner=-0.4, wheel=False, seed=-0.4):
  monkeypatch.setattr(stopping_flags, "SERVICE_APPROACH_LAW", "governor")
  monkeypatch.setattr(stopping_flags, "GOVERNOR_RECOVERY_BRAKE", enabled)
  sig = StopSignals(gap, source, outward, dropout, coast, wheel, True, True, 2.0, True)
  if not lead:
    sig = replace(sig, d_gap=None, gap_source="none", lead_confirmed_stopped=False, lead_stopped_for_entry=False)
  return StoppingService().update(engaged=True, v_ego=v, a_ego=-0.3, a_target=planner, should_stop=True,
    dts_planner=3.0, planner_min_limit=-3.5, signals=sig, lead_status=lead, lead_v=lv,
    increased_stopped_distance=isd, dt=0.01, wire_accel=seed, a_target_trajectory=planner)


def test_bookmarked_recovery_keeps_a_braking_phase(monkeypatch):
  # 000020bf segment 34: the ego fell below the profile, which then asked it to recover speed.
  old = _step(monkeypatch, False, v=0.717, lv=0.03, gap=6.8, coast=-0.04)
  new = _step(monkeypatch, True, v=0.717, lv=0.03, gap=6.8, coast=-0.04)
  assert old.debug["a_gov"] > 0.0 and old.debug["gov_v_ref"] > 0.717
  assert old.debug["a_phase"] == -0.03
  assert new.debug["a_phase"] < -0.06
  assert new.debug["a_gov"] == old.debug["a_gov"]  # the original shadow profile stays comparable


@pytest.mark.parametrize("profile", [False, True])
def test_recovery_cap_only_deepens_and_has_bounded_authority(monkeypatch, profile):
  monkeypatch.setattr(stopping_flags, "GOVERNOR_PROFILE_REFERENCE", profile)
  rng = random.Random(55)
  changed = 0
  for _ in range(1500):
    kw = dict(v=rng.uniform(0.51, 2.49), lv=rng.uniform(-1.0, 3.0), gap=rng.uniform(2.0, 20.0),
              coast=rng.uniform(-0.5, 0.5), isd=rng.uniform(0.0, 1.0),
              planner=rng.uniform(-1.5, 0.2), seed=rng.uniform(-1.0, -0.03))
    old, new = [_step(monkeypatch, flag, **kw) for flag in (False, True)]
    a0, a1 = old.debug["a_phase"], new.debug["a_phase"]
    assert min(a0, -GOV_A_C - kw["coast"]) - 1e-9 <= a1 <= a0 + 1e-9
    assert new.accel <= old.accel + 1e-9
    assert new.accel >= min(old.accel, kw["seed"]) - 1e-9  # the limit cannot add a downward command step
    for lane in ("a_gov", "a_kin", "a_barrier", "a_plan", "gov_v_ref", "gov_d"):
      assert new.debug[lane] == old.debug[lane]
    changed += a1 < a0 - 1e-9
  assert changed > 100


@pytest.mark.parametrize("kw", [
  dict(v=0.5, lv=0.0, gap=6.8),  # terminal band
  dict(v=0.1, lv=0.0, gap=4.6, wheel=True),  # hold
  dict(v=0.8, lv=0.8, gap=6.0),  # crawler fixed point
  dict(v=0.8, lv=1.2, gap=6.0),  # departing lead
  dict(v=2.0, lv=0.0, gap=4.4),  # hot entry inside the lag-adjusted anchor
  dict(v=1.0, lv=0.0, gap=6.8, source="held"),  # rejected inward jump
  dict(v=1.0, lv=0.0, gap=6.8, source="decay", dropout=True),
  dict(v=1.0, lv=0.0, gap=6.8, lead=False),
])
def test_recovery_exclusions_are_unchanged(monkeypatch, kw):
  old, new = [_step(monkeypatch, flag, **kw) for flag in (False, True)]
  assert new == old


def test_outward_hold_lower_bound_can_keep_recovery_braking(monkeypatch):
  old, new = [_step(monkeypatch, flag, v=0.8, lv=0.0, gap=6.8, source="held", outward=True) for flag in (False, True)]
  assert new.debug["a_phase"] < old.debug["a_phase"]


def test_deeper_planner_braking_and_reversal_safety_remain(monkeypatch):
  for lv in (0.0, -0.5):
    old, new = [_step(monkeypatch, flag, v=0.8, lv=lv, gap=6.8, planner=-1.2) for flag in (False, True)]
    assert new.accel == old.accel
    assert new.debug["a_plan"] == old.debug["a_plan"]
    assert new.debug["a_barrier"] == old.debug["a_barrier"]


def test_warm_takeover_uses_its_new_seed_and_keeps_safety_braking(monkeypatch):
  results = []
  for enabled in (False, True):
    monkeypatch.setattr(stopping_flags, "SERVICE_APPROACH_LAW", "governor")
    monkeypatch.setattr(stopping_flags, "GOVERNOR_RECOVERY_BRAKE", enabled)
    s = StoppingService()
    kw = dict(engaged=True, v_ego=0.8, a_ego=-0.3, a_target=None, should_stop=True, dts_planner=3.7,
              planner_min_limit=-3.5, signals=StopSignals(8.0, "measured", False, False, 0.0, False, True, True, 2.0, True),
              lead_status=True, lead_v=0.0, increased_stopped_distance=0.3, wire_accel=-0.4)
    s.update(**kw)
    s.reseed_takeover(-0.03, -3.5)
    results.append(s.update(**kw))
  # A release limit cannot restore brake already lost at takeover; the existing safety lanes still deepen.
  assert results[0] == results[1]
  assert results[1].debug["a_phase"] == -0.03 and results[1].accel < -0.03


@pytest.mark.parametrize("lag", [0.15, 0.30, 0.45, 0.60])
@pytest.mark.parametrize("push", [0.10, 0.25, 0.40, 0.50])
def test_release_limit_preserves_stop_and_hold_with_lag(monkeypatch, lag, push):
  # A synthetic first-order actuator with a known constant push, not a fitted vehicle predictor.
  # Exercise the retained planner brake, its attributed release, the terminal seed and the subsequent hold.
  monkeypatch.setattr(stopping_flags, "SERVICE_APPROACH_LAW", "governor")
  monkeypatch.setattr(stopping_flags, "ATTRIBUTED_SAFETY", "live")
  monkeypatch.setattr(stopping_flags, "GOVERNOR_PROFILE_REFERENCE", True)
  finishes = []
  for enabled in (False, True):
    monkeypatch.setattr(stopping_flags, "GOVERNOR_RECOVERY_BRAKE", enabled)
    s = StoppingService()
    v, gap, cmd = 1.7, 8.9, -0.49
    a, stopped = cmd + push, None
    for k in range(1200):
      sig = StopSignals(gap, "measured", False, False, push, v < 0.02, True, True, 3.0 + k * 0.01, True)
      plan = -0.64 if k < 100 else -0.3
      r = s.update(engaged=True, v_ego=v, a_ego=a, a_target=plan, should_stop=True, dts_planner=gap - 4.3,
        planner_min_limit=-3.5, signals=sig, lead_status=True, lead_v=0.0, increased_stopped_distance=0.3,
        wire_accel=cmd, lead_a=-0.5 if k < 100 else 0.0, a_target_trajectory=plan)
      assert r.active and r.phase.name not in ("RELEASE", "INACTIVE")
      assert abs(r.accel - cmd) <= 0.080001
      cmd = r.accel
      a += (cmd + push - a) * (0.01 / lag)
      v = max(v + a * 0.01, 0.0)
      gap -= v * 0.01
      assert gap >= 3.0
      if v == 0.0:
        a = 0.0  # the stopped plant cannot integrate backward motion
        if stopped is None:
          stopped = k
        if k - stopped >= 100:
          break
    assert stopped is not None and k - stopped >= 100
    assert r.phase.name == "HOLD" and cmd <= -0.70
    assert 4.0 <= gap <= 5.1
    finishes.append(gap)
  assert abs(finishes[1] - finishes[0]) <= 0.10
