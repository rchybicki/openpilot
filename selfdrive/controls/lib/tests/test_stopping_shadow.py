import pytest

from openpilot.selfdrive.controls.lib.stopping_profile_selector import PROFILE_GLIDE_SOFTEN, PROFILE_NO_CHANGE
from openpilot.selfdrive.controls.lib.stopping_shadow import StoppingShadowInput, StoppingShadowOracle, StoppingShadowSimulation


def test_stopping_shadow_oracle_selects_glide_soften_for_overbraked_tail() -> None:
  oracle = StoppingShadowOracle()

  decision = oracle.evaluate(
    StoppingShadowInput(
      output_accel=-0.80,
      last_output_accel=-0.80,
      should_stop=True,
      v_ego=0.15,
      a_ego=-0.70,
      stop_accel=-2.0,
      remaining_m=0.40,
      explicit_target_available=True,
      rollout_m=1.20,
      phase=1,
      release_lock_active=False,
      rebound_arrest_active=False,
      lead_status=False,
      lead_v=0.0,
      lead_d_rel=None,
    )
  )

  assert decision.profile == PROFILE_GLIDE_SOFTEN
  assert decision.score_delta < 0.0
  assert decision.selected.speed_rebound_mps < decision.current.speed_rebound_mps
  assert decision.selected.first_output_accel > decision.current.first_output_accel


def test_stopping_shadow_oracle_rejects_close_lead_gap_risk() -> None:
  oracle = StoppingShadowOracle()

  decision = oracle.evaluate(
    StoppingShadowInput(
      output_accel=-0.45,
      last_output_accel=-0.45,
      should_stop=True,
      v_ego=0.75,
      a_ego=-0.10,
      stop_accel=-2.0,
      remaining_m=0.80,
      explicit_target_available=True,
      rollout_m=0.50,
      phase=1,
      release_lock_active=False,
      rebound_arrest_active=False,
      lead_status=True,
      lead_v=0.0,
      lead_d_rel=1.20,
    )
  )

  assert decision.profile == PROFILE_NO_CHANGE
  assert decision.rejection_reason in ("lead_gap_risk", "lead_gap_regression")


def test_stopping_shadow_decision_writes_debug_payload() -> None:
  oracle = StoppingShadowOracle()
  debug: dict[str, object] = {}

  decision = oracle.evaluate(
    StoppingShadowInput(
      output_accel=-0.80,
      last_output_accel=-0.80,
      should_stop=True,
      v_ego=0.15,
      a_ego=-0.70,
      stop_accel=-2.0,
      remaining_m=0.40,
      explicit_target_available=True,
      rollout_m=1.20,
      phase=1,
      release_lock_active=False,
      rebound_arrest_active=False,
      lead_status=False,
      lead_v=0.0,
      lead_d_rel=None,
    )
  )
  decision.write_debug(debug)

  assert debug["shadow_profile"] == PROFILE_GLIDE_SOFTEN
  assert debug["shadow_score_delta"] == pytest.approx(decision.score_delta)
  assert debug["shadow_first_output_accel"] == pytest.approx(decision.selected.first_output_accel)
  assert debug["shadow_selected_leapfrog"] is False


def test_stopping_shadow_oracle_rejects_unsafe_candidate_even_if_current_is_unsafe() -> None:
  oracle = StoppingShadowOracle()
  shadow_input = StoppingShadowInput(
    output_accel=-0.80,
    last_output_accel=-0.80,
    should_stop=True,
    v_ego=0.15,
    a_ego=-0.70,
    stop_accel=-2.0,
    remaining_m=0.40,
    explicit_target_available=True,
    rollout_m=1.20,
    phase=1,
    release_lock_active=False,
    rebound_arrest_active=False,
    lead_status=False,
    lead_v=0.0,
    lead_d_rel=None,
  )
  current = StoppingShadowSimulation(
    profile="no_change",
    score=2.0,
    first_output_accel=-0.80,
    min_a_ego=-1.2,
    max_abs_jerk=3.8,
    max_cmd_step=0.0,
    max_cmd_jerk=0.0,
    rollout_m=1.2,
    final_v_ego=0.0,
    final_remaining_m=0.0,
    min_lead_gap_m=None,
    speed_rebound_mps=0.08,
    unexpected_accel_mps2=0.0,
    is_harsh=True,
    is_leapfrog=True,
  )
  candidate = StoppingShadowSimulation(
    profile="glide_soften",
    score=1.0,
    first_output_accel=-0.70,
    min_a_ego=-0.8,
    max_abs_jerk=2.0,
    max_cmd_step=0.1,
    max_cmd_jerk=1.0,
    rollout_m=1.1,
    final_v_ego=0.0,
    final_remaining_m=0.0,
    min_lead_gap_m=None,
    speed_rebound_mps=0.06,
    unexpected_accel_mps2=0.0,
    is_harsh=False,
    is_leapfrog=True,
  )

  assert oracle._reject_candidate(current, candidate, shadow_input) == "leapfrog_risk"
