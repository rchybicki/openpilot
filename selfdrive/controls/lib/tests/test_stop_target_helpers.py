import ast
from pathlib import Path

import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import (
  LEAD_STOP_DISTANCE_TARGET,
  STOP_TARGET_CLOSE_HOLD_REMAINING_M,
  STOP_TARGET_LATCH_DURATION_S,
  get_distance_to_stopped_lead_target,
  get_effective_lead_distance,
  get_published_lead_distance,
  get_published_lead_distance_compensation,
  get_stop_target_factor,
  get_stopped_lead_control_target,
  get_stopped_lead_obstacle_offset,
  update_distance_to_stop_target_for_mode,
  update_distance_to_stop_target_with_latch,
)

REPO_ROOT = Path(__file__).resolve().parents[4]
MPC_STOP_DISTANCE = 5.5  # long_mpc.py STOP_DISTANCE (long_mpc is not importable without a scons build)


def test_stop_target_latch_uses_min_positive_candidate() -> None:
  distance_to_stop_target_m, latch_timer_s = update_distance_to_stop_target_with_latch(
    current_distance_to_stop_target_m=-1.0,
    current_latch_timer_s=0.0,
    dt=0.05,
    candidates=(0.82, 0.31),
  )

  assert distance_to_stop_target_m == 0.31
  assert latch_timer_s == STOP_TARGET_LATCH_DURATION_S


def test_stop_target_latch_prefers_close_lead_hold_over_far_secondary_candidate() -> None:
  distance_to_stop_target_m, latch_timer_s = update_distance_to_stop_target_with_latch(
    current_distance_to_stop_target_m=1.4,
    current_latch_timer_s=0.4,
    dt=0.05,
    candidates=(STOP_TARGET_CLOSE_HOLD_REMAINING_M, 2.7),
  )

  assert distance_to_stop_target_m == STOP_TARGET_CLOSE_HOLD_REMAINING_M
  assert latch_timer_s == STOP_TARGET_LATCH_DURATION_S


def test_stop_target_latch_holds_last_positive_value_across_brief_dropout() -> None:
  distance_to_stop_target_m, latch_timer_s = update_distance_to_stop_target_with_latch(
    current_distance_to_stop_target_m=0.44,
    current_latch_timer_s=0.4,
    dt=0.05,
    candidates=(-1.0, -1.0),
  )

  assert distance_to_stop_target_m == 0.44
  assert latch_timer_s == pytest.approx(0.35, abs=1e-12)


def test_stop_target_latch_clears_after_timeout() -> None:
  distance_to_stop_target_m, latch_timer_s = update_distance_to_stop_target_with_latch(
    current_distance_to_stop_target_m=0.44,
    current_latch_timer_s=0.0,
    dt=0.05,
    candidates=(-1.0, -1.0),
  )

  assert distance_to_stop_target_m == -1.0
  assert latch_timer_s == 0.0


def test_stop_target_latch_stays_active_in_blended_mode() -> None:
  distance_to_stop_target_m, latch_timer_s = update_distance_to_stop_target_for_mode(
    mode="blended",
    current_distance_to_stop_target_m=-1.0,
    current_latch_timer_s=0.0,
    dt=0.05,
    candidates=(4.5, -1.0),
  )

  assert distance_to_stop_target_m == 4.5
  assert latch_timer_s == STOP_TARGET_LATCH_DURATION_S


def test_stop_target_latch_clears_for_unsupported_mode() -> None:
  distance_to_stop_target_m, latch_timer_s = update_distance_to_stop_target_for_mode(
    mode="unknown",
    current_distance_to_stop_target_m=0.44,
    current_latch_timer_s=0.4,
    dt=0.05,
    candidates=(0.82, 0.31),
  )

  assert distance_to_stop_target_m == -1.0
  assert latch_timer_s == 0.0


def test_stop_target_factor_keeps_creeping_lead_target_alive_longer() -> None:
  # Route 00000078 event 1 first surfaced the stop target with the lead already
  # down near 5-6 kph. Keep that explicit target materially alive instead of
  # collapsing a real ~3 m remaining target down to roughly ~1 m.
  assert get_stop_target_factor(4.91) == pytest.approx(0.75885, abs=1e-4)
  assert get_stop_target_factor(6.0) == pytest.approx(0.65, abs=1e-12)
  assert get_stop_target_factor(2.66) == pytest.approx(0.8514, abs=1e-4)
  assert get_stop_target_factor(7.6) == pytest.approx(0.0, abs=1e-12)


def test_distance_to_stopped_lead_target_strengthens_recent_wide_route_case() -> None:
  distance_to_stop_target_m = get_distance_to_stopped_lead_target(
    v_lead_raw=1.364,
    v_lead_distance_raw=6.098,
    increased_stopped_distance=0.0,
    lead_stop_distance_target=3.0,
  )

  assert distance_to_stop_target_m == pytest.approx(2.351, abs=1e-3)


def test_distance_to_stopped_lead_target_preserves_good_slow_lead_behavior_inside_cap() -> None:
  distance_to_stop_target_m = get_distance_to_stopped_lead_target(
    v_lead_raw=0.739,
    v_lead_distance_raw=6.900,
    increased_stopped_distance=0.0,
    lead_stop_distance_target=3.0,
  )

  assert distance_to_stop_target_m == pytest.approx(3.320, abs=1e-3)


def test_default_lead_stop_distance_target_moves_closest_stops_back_half_meter() -> None:
  assert LEAD_STOP_DISTANCE_TARGET == pytest.approx(4.0, abs=1e-12)

  distance_to_stop_target_m = get_distance_to_stopped_lead_target(
    v_lead_raw=0.739,
    v_lead_distance_raw=6.900,
    increased_stopped_distance=0.0,
    lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET,
  )

  assert distance_to_stop_target_m == pytest.approx(2.469, abs=1e-3)


def test_distance_to_stopped_lead_target_holds_close_stopped_lead_inside_target_gap() -> None:
  distance_to_stop_target_m = get_distance_to_stopped_lead_target(
    v_lead_raw=0.0,
    v_lead_distance_raw=3.20,
    increased_stopped_distance=0.0,
    lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET,
  )

  assert distance_to_stop_target_m == pytest.approx(STOP_TARGET_CLOSE_HOLD_REMAINING_M, abs=1e-12)


def test_stop_target_sequence_does_not_restore_stale_far_target_inside_gap() -> None:
  distance_to_stop_target_m = -1.0
  latch_timer_s = 0.0
  for lead_distance_m, expected_target_m in (
    (6.70, 2.70),
    (5.40, 1.40),
    (4.20, 0.20),
    (3.20, STOP_TARGET_CLOSE_HOLD_REMAINING_M),
  ):
    candidate = get_distance_to_stopped_lead_target(
      v_lead_raw=0.0,
      v_lead_distance_raw=lead_distance_m,
      increased_stopped_distance=0.0,
      lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET,
    )
    distance_to_stop_target_m, latch_timer_s = update_distance_to_stop_target_with_latch(
      current_distance_to_stop_target_m=distance_to_stop_target_m,
      current_latch_timer_s=latch_timer_s,
      dt=0.05,
      candidates=(candidate, candidate),
    )

    assert distance_to_stop_target_m == pytest.approx(expected_target_m, abs=1e-12)
    assert latch_timer_s == STOP_TARGET_LATCH_DURATION_S


def test_distance_to_stopped_lead_target_ignores_invalid_close_distance() -> None:
  distance_to_stop_target_m = get_distance_to_stopped_lead_target(
    v_lead_raw=0.0,
    v_lead_distance_raw=0.0,
    increased_stopped_distance=0.0,
    lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET,
  )

  assert distance_to_stop_target_m == pytest.approx(0.0, abs=1e-12)


def test_stopped_lead_control_target_matches_live_close_bookmark_seed() -> None:
  assert get_stopped_lead_control_target(
    v_ego=1.59,
    lead_v=0.0,
    lead_d_rel=4.20,
  ) == pytest.approx(1.45, abs=1e-12)
  assert get_stopped_lead_control_target(
    v_ego=1.20,
    lead_v=0.0,
    lead_d_rel=3.20,
  ) == pytest.approx(0.45, abs=1e-12)


def test_stopped_lead_control_target_ignores_departing_lead() -> None:
  assert get_stopped_lead_control_target(
    v_ego=1.20,
    lead_v=1.10,
    lead_d_rel=3.20,
  ) is None


def test_stopped_lead_control_target_arrived_gate_suppresses_low_speed_rest_band() -> None:
  # Crept-to-rest at a STOPPED lead inside the arrived ceiling (v_ego <= 0.35, dRel <= 4.30):
  # the producer must return None so bouncing radar can't re-arm the glide-cap re-grab walk.
  assert stopping_flags.STOPPED_LEAD_ARRIVED_GATE_ENABLED
  assert get_stopped_lead_control_target(v_ego=0.20, lead_v=0.0, lead_d_rel=2.20) is None
  assert get_stopped_lead_control_target(v_ego=0.20, lead_v=0.0, lead_d_rel=4.10) is None


def test_stopped_lead_control_target_arrived_gate_inactive_just_outside_band() -> None:
  # Just above the speed band: gate does not fire, legacy trigger math still produces the target.
  assert get_stopped_lead_control_target(v_ego=0.50, lead_v=0.0, lead_d_rel=3.0) == pytest.approx(0.25, abs=1e-12)
  # Just outside the speed band (v_ego just over the 0.35 arrived ceiling): normal trigger math applies.
  # NOTE: at v_ego <= 0.35 the legacy trigger_gap floors at 3.10 m, well inside the 4.30 m arrived
  # ceiling, so the gate fully shadows the legacy producer there -- the boundary that exposes
  # "gate off, trigger math on" lives just ABOVE the speed band, not at a larger gap.
  assert get_stopped_lead_control_target(v_ego=0.40, lead_v=0.0, lead_d_rel=3.05) == pytest.approx(0.30, abs=1e-12)


def test_stopped_lead_control_target_arrived_gate_kill_switch_restores_legacy(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "STOPPED_LEAD_ARRIVED_GATE_ENABLED", False)
  # With the gate disabled, the qualifying frame falls through to the legacy close-hold value.
  assert get_stopped_lead_control_target(v_ego=0.20, lead_v=0.0, lead_d_rel=2.20) == pytest.approx(0.05, abs=1e-12)


def test_distance_to_stopped_lead_target_stays_off_for_moving_lead() -> None:
  distance_to_stop_target_m = get_distance_to_stopped_lead_target(
    v_lead_raw=2.3,
    v_lead_distance_raw=6.5,
    increased_stopped_distance=0.0,
    lead_stop_distance_target=2.5,
  )

  assert distance_to_stop_target_m == pytest.approx(0.0, abs=1e-12)


# --- ISD single-meaning machinery (stopping redesign §4.2, red-team F12/F25) ---


def _published_gap(true_gap: float, isd: float, publish_true: bool) -> float:
  """What radard publishes for a given TRUE gap in a given flag state."""
  return true_gap if publish_true else true_gap - isd


def _target_path_rest_true_gap(isd: float, publish_true: bool, target: float = LEAD_STOP_DISTANCE_TARGET) -> float:
  """TRUE gap at which the explicit-target path rests (where remaining distance hits zero).

  For a stopped lead (factor == 1.0) the returned remaining distance equals
  true_gap - rest_gap while it stays inside (0, STOP_TARGET_MAX_DISTANCE_M); probe that
  linear band (avoiding the 0.05 close-hold sentinel and the far cutoff to 0.0) and
  recover the rest gap as true_gap - remaining, asserting it is probe-independent.
  """
  rests = []
  for i in range(1, 120):
    true_gap = i * 0.1
    remaining = get_distance_to_stopped_lead_target(
      v_lead_raw=0.0,
      v_lead_distance_raw=_published_gap(true_gap, isd, publish_true),
      increased_stopped_distance=isd,
      lead_stop_distance_target=target,
    )
    if 0.06 < remaining < 4.4:
      rests.append(true_gap - remaining)
  assert rests, "no probe point landed in the linear remaining band"
  assert max(rests) - min(rests) < 1e-9
  return rests[0]


def _mpc_path_rest_true_gap(isd: float, publish_true: bool, target: float = LEAD_STOP_DISTANCE_TARGET,
                            stop_distance: float = MPC_STOP_DISTANCE) -> float:
  """TRUE gap at which the MPC stopped-lead obstacle path rests.

  At standstill behind a stopped lead the MPC settles where
  published_gap == stop_distance - stopped_lead_offset (factor == 1.0 for a stopped lead);
  the TRUE gap adds back the radard mutation when it is active.
  """
  offset = get_stopped_lead_obstacle_offset(stop_distance, isd, target, 1.0)
  rest_published_gap = stop_distance - offset
  return rest_published_gap + (0.0 if publish_true else isd)


@pytest.mark.parametrize("publish_true", [False, True])
@pytest.mark.parametrize("isd", [0.0, 1.5, 3.0])
def test_isd_rest_gap_equality_between_mpc_and_target_paths(monkeypatch, isd: float, publish_true: bool) -> None:
  # §4.2.3 required test: both stop paths rest at the SAME true gap in both flag states --
  # flag off: lead_stop_distance_target (4.0), ISD-independent; flag on: target + ISD.
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", publish_true)
  expected_rest_true_gap = LEAD_STOP_DISTANCE_TARGET + (isd if publish_true else 0.0)

  target_rest = _target_path_rest_true_gap(isd, publish_true)
  mpc_rest = _mpc_path_rest_true_gap(isd, publish_true)

  assert target_rest == pytest.approx(expected_rest_true_gap, abs=1e-6)
  assert mpc_rest == pytest.approx(expected_rest_true_gap, abs=1e-9)
  assert mpc_rest == pytest.approx(target_rest, abs=1e-6)


@pytest.mark.parametrize("isd", [0.0, 1.5, 3.0])
def test_mpc_rest_gap_is_independent_of_stop_distance_constant(monkeypatch, isd: float) -> None:
  # the MPC rest point must come from the offset algebra, not the STOP_DISTANCE constant
  for publish_true in (False, True):
    monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", publish_true)
    assert _mpc_path_rest_true_gap(isd, publish_true, stop_distance=5.5) == pytest.approx(
      _mpc_path_rest_true_gap(isd, publish_true, stop_distance=7.0), abs=1e-9)


def test_target_path_remaining_distance_above_rest_point(monkeypatch) -> None:
  # one direct, non-inverted check of the two flag-state expressions (F25 derivation)
  isd = 1.5
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", False)
  # flag off: published gap = true - ISD; remaining = published + ISD - target = true - target
  assert get_distance_to_stopped_lead_target(0.0, 5.0 - isd, isd, 4.0) == pytest.approx(1.0, abs=1e-12)
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", True)
  # flag on: published gap = true; remaining = true - (target + ISD) -> rests ISD farther back
  assert get_distance_to_stopped_lead_target(0.0, 6.5, isd, 4.0) == pytest.approx(1.0, abs=1e-12)


def test_mpc_stopped_lead_offset_sign_per_f25(monkeypatch) -> None:
  isd = 1.5
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", False)
  assert get_stopped_lead_obstacle_offset(5.5, isd, 4.0, 1.0) == pytest.approx(5.5 + isd - 4.0, abs=1e-12)
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", True)
  # MINUS sign: obstacle appears ISD closer so ego rests ISD farther (a +ISD here would rest 2*ISD closer)
  assert get_stopped_lead_obstacle_offset(5.5, isd, 4.0, 1.0) == pytest.approx(5.5 - isd - 4.0, abs=1e-12)


def test_published_lead_distance_mutation_predicate(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", False)
  assert get_published_lead_distance(10.0, 1.5) == pytest.approx(8.5, abs=1e-12)
  assert get_published_lead_distance(10.0, 0.0) == pytest.approx(10.0, abs=1e-12)
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", True)
  assert get_published_lead_distance(10.0, 1.5) == pytest.approx(10.0, abs=1e-12)


@pytest.mark.parametrize("isd", [0.0, 1.5, 3.0])
def test_planner_cap_compensation_recovers_true_gap_in_both_flag_states(monkeypatch, isd: float) -> None:
  true_gap = 9.0
  for publish_true in (False, True):
    monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", publish_true)
    published = get_published_lead_distance(true_gap, isd)
    assert published + get_published_lead_distance_compensation(isd) == pytest.approx(true_gap, abs=1e-12)


@pytest.mark.parametrize("isd", [0.0, 1.5, 3.0])
def test_effective_lead_distance_invariant_across_flag_flip(monkeypatch, isd: float) -> None:
  # §4.2.4: the longcontrol stopping layer sees the SAME effective distance for the same true
  # gap in both flag states -- the flip is a no-op for it by construction.
  true_gap = 7.0
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", False)
  eff_legacy = get_effective_lead_distance(get_published_lead_distance(true_gap, isd), isd)
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", True)
  eff_flipped = get_effective_lead_distance(get_published_lead_distance(true_gap, isd), isd)
  assert eff_flipped == pytest.approx(eff_legacy, abs=1e-12)
  assert eff_legacy == pytest.approx(true_gap - isd, abs=1e-12)


def _function_calls_in(path: str, caller: str | None = None) -> set[str]:
  tree = ast.parse((REPO_ROOT / path).read_text())
  nodes = [tree]
  if caller is not None:
    nodes = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == caller]
    assert nodes, f"{caller} not found in {path}"
  names: set[str] = set()
  for scope in nodes:
    for node in ast.walk(scope):
      if isinstance(node, ast.Call):
        func = node.func
        if isinstance(func, ast.Name):
          names.add(func.id)
        elif isinstance(func, ast.Attribute):
          names.add(func.attr)
  return names


def test_runtime_call_sites_use_the_isd_helpers() -> None:
  # build-free source pin: the consumers that cannot be imported without a scons build
  # really route their ISD terms through the single-definition-site helpers above
  assert "get_stopped_lead_obstacle_offset" in _function_calls_in(
    "selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py", caller="get_stopped_equivalence_factor")
  assert "get_published_lead_distance" in _function_calls_in(
    "selfdrive/controls/radard.py", caller="get_lead")
  assert "get_published_lead_distance_compensation" in _function_calls_in(
    "selfdrive/controls/lib/longitudinal_planner.py", caller="get_santa_fe_stopped_lead_smooth_approach_cap")
  assert "get_published_lead_distance_compensation" in _function_calls_in(
    "selfdrive/controls/lib/longitudinal_planner.py", caller="get_santa_fe_slowing_lead_smooth_approach_cap")
