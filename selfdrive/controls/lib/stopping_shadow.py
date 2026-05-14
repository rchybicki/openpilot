from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Any

from openpilot.selfdrive.controls.lib.stopping_profile_selector import (
  PROFILE_GLIDE_SOFTEN,
  PROFILE_NO_CHANGE,
  PROFILE_PRESERVE_BRAKE,
  PROFILE_SOFTEN_THEN_DEEPEN,
  PROFILE_TAIL_DEEPEN,
)


STOPPING_SHADOW_VERSION = "fresh_20260514_profile_oracle_v2"
STOPPING_SHADOW_LOGGING_ENABLED = True
STOPPING_SHADOW_SAMPLE_INTERVAL_FRAMES = 10
STOPPING_SHADOW_LOG_PERIOD_S = 2.0
STOPPING_SHADOW_PROFILE_LOG_PERIOD_S = 0.8

SHADOW_MODEL_DT_S = 0.10
SHADOW_MIN_IMPROVEMENT_SCORE = 0.018

SHADOW_MODEL_COEFFICIENTS = {
  "intercept": -0.003240927224571002,
  "a_ego_prev": 0.8715303653619791,
  "accel_cmd_delayed": 0.37169541723475924,
  "v_ego": 0.013537141414051946,
  "relief": 0.28700393800740076,
  "low_speed": -0.014521878402947764,
  "cmd_x_low_speed": -0.45138720666502313,
}
SHADOW_MODEL_LOW_SPEED_REF = 1.20
SHADOW_MODEL_RELIEF_CMD_THRESHOLD = -0.25

SHADOW_PROFILE_RESIDUALS_MPS2 = {
  PROFILE_NO_CHANGE: (0.0,) * 12,
  PROFILE_PRESERVE_BRAKE: (
    -0.006666666666666672,
    -0.00888888888888889,
    -0.004444444444444445,
    0.06222222222222224,
    0.08000000000000004,
    0.0955555555555556,
    0.08888888888888893,
    0.08444444444444449,
    0.06222222222222224,
    0.05111111111111112,
    0.046666666666666676,
    0.03333333333333333,
  ),
  PROFILE_SOFTEN_THEN_DEEPEN: (
    0.10909090909090911,
    0.12000000000000002,
    0.12000000000000002,
    0.09818181818181819,
    0.11454545454545456,
    0.09818181818181819,
    0.0,
    -0.021818181818181816,
    -0.03818181818181818,
    -0.049090909090909095,
    -0.06545454545454547,
    -0.07090909090909091,
  ),
  PROFILE_TAIL_DEEPEN: (
    -0.04999999999999999,
    0.0,
    -0.09000000000000001,
    -0.04,
    -0.07,
    -0.08999999999999997,
    -0.08999999999999997,
    -0.08999999999999997,
    -0.05999999999999999,
    -0.05999999999999999,
    -0.08999999999999997,
    -0.07999999999999999,
  ),
  PROFILE_GLIDE_SOFTEN: (
    0.03750000000000001,
    0.03750000000000001,
    0.11625000000000008,
    0.11125000000000007,
    0.10375000000000006,
    0.10375000000000006,
    0.10000000000000005,
    0.10000000000000005,
    0.11500000000000009,
    0.11500000000000009,
    0.10500000000000005,
    0.10125000000000005,
  ),
}


def _finite_float(value: Any, default: float = 0.0) -> float:
  try:
    result = float(value)
  except (TypeError, ValueError):
    return default
  return result if isfinite(result) else default


def _clip(value: float, lower: float, upper: float) -> float:
  return max(lower, min(upper, value))


@dataclass(frozen=True)
class StoppingShadowInput:
  output_accel: float
  last_output_accel: float
  should_stop: bool
  v_ego: float
  a_ego: float
  stop_accel: float
  remaining_m: float | None
  explicit_target_available: bool
  rollout_m: float
  phase: int
  release_lock_active: bool
  rebound_arrest_active: bool
  lead_status: bool
  lead_v: float
  lead_d_rel: float | None


@dataclass(frozen=True)
class StoppingShadowSimulation:
  profile: str
  score: float
  first_output_accel: float
  min_a_ego: float
  max_abs_jerk: float
  max_cmd_step: float
  max_cmd_jerk: float
  rollout_m: float
  final_v_ego: float
  final_remaining_m: float | None
  min_lead_gap_m: float | None
  speed_rebound_mps: float
  unexpected_accel_mps2: float
  is_harsh: bool
  is_leapfrog: bool


@dataclass(frozen=True)
class StoppingShadowDecision:
  version: str
  profile: str
  confidence: float
  score_current: float
  score_selected: float
  score_delta: float
  rejection_reason: str
  current: StoppingShadowSimulation
  selected: StoppingShadowSimulation
  residual_template_mps2: tuple[float, ...]

  def write_debug(self, debug: dict[str, object]) -> None:
    debug["shadow_version"] = self.version
    debug["shadow_profile"] = self.profile
    debug["shadow_confidence"] = float(self.confidence)
    debug["shadow_score_current"] = float(self.score_current)
    debug["shadow_score_selected"] = float(self.score_selected)
    debug["shadow_score_delta"] = float(self.score_delta)
    debug["shadow_rejection_reason"] = self.rejection_reason
    debug["shadow_first_output_accel"] = float(self.selected.first_output_accel)
    debug["shadow_current_rollout_m"] = float(self.current.rollout_m)
    debug["shadow_selected_rollout_m"] = float(self.selected.rollout_m)
    debug["shadow_selected_min_a_ego"] = float(self.selected.min_a_ego)
    debug["shadow_selected_final_v_ego"] = float(self.selected.final_v_ego)
    debug["shadow_selected_harsh"] = bool(self.selected.is_harsh)
    debug["shadow_selected_leapfrog"] = bool(self.selected.is_leapfrog)
    debug["shadow_residual_preview_mps2"] = tuple(float(value) for value in self.residual_template_mps2[:4])


class StoppingShadowOracle:
  """Shadow-only learned profile evaluator.

  This evaluates the learned profile templates with the current fitted plant model. It never
  returns an authority command to the caller; the caller only records the decision in debug/log data.
  """

  def evaluate(self, shadow_input: StoppingShadowInput) -> StoppingShadowDecision:
    current = self._simulate(PROFILE_NO_CHANGE, SHADOW_PROFILE_RESIDUALS_MPS2[PROFILE_NO_CHANGE], shadow_input)
    if not shadow_input.should_stop or shadow_input.v_ego > 2.5:
      return self._decision(
        profile=PROFILE_NO_CHANGE,
        current=current,
        selected=current,
        residual_template=SHADOW_PROFILE_RESIDUALS_MPS2[PROFILE_NO_CHANGE],
        rejection_reason="outside_stop_shadow_scope",
      )

    accepted: list[tuple[float, str, StoppingShadowSimulation, tuple[float, ...]]] = []
    first_rejection = "no_candidate_improved"
    for profile, residuals in SHADOW_PROFILE_RESIDUALS_MPS2.items():
      if profile == PROFILE_NO_CHANGE:
        continue
      simulation = self._simulate(profile, residuals, shadow_input)
      rejection = self._reject_candidate(current, simulation, shadow_input)
      if rejection is not None:
        if first_rejection == "no_candidate_improved":
          first_rejection = rejection
        continue
      if simulation.score <= current.score - SHADOW_MIN_IMPROVEMENT_SCORE:
        accepted.append((simulation.score, profile, simulation, residuals))

    if not accepted:
      return self._decision(
        profile=PROFILE_NO_CHANGE,
        current=current,
        selected=current,
        residual_template=SHADOW_PROFILE_RESIDUALS_MPS2[PROFILE_NO_CHANGE],
        rejection_reason=first_rejection,
      )

    _, profile, selected, residual_template = min(accepted, key=lambda item: item[0])
    return self._decision(
      profile=profile,
      current=current,
      selected=selected,
      residual_template=residual_template,
      rejection_reason="accepted",
    )

  def _decision(
    self,
    *,
    profile: str,
    current: StoppingShadowSimulation,
    selected: StoppingShadowSimulation,
    residual_template: tuple[float, ...],
    rejection_reason: str,
  ) -> StoppingShadowDecision:
    score_delta = selected.score - current.score
    confidence = _clip((current.score - selected.score) / 0.45, 0.0, 1.0) if profile != PROFILE_NO_CHANGE else 0.0
    return StoppingShadowDecision(
      version=STOPPING_SHADOW_VERSION,
      profile=profile,
      confidence=float(confidence),
      score_current=float(current.score),
      score_selected=float(selected.score),
      score_delta=float(score_delta),
      rejection_reason=rejection_reason,
      current=current,
      selected=selected,
      residual_template_mps2=residual_template,
    )

  def _simulate(
    self,
    profile: str,
    residual_template: tuple[float, ...],
    shadow_input: StoppingShadowInput,
  ) -> StoppingShadowSimulation:
    v_ego = max(0.0, _finite_float(shadow_input.v_ego))
    a_ego = _clip(_finite_float(shadow_input.a_ego), -2.5, 1.5)
    command_base = _clip(_finite_float(shadow_input.output_accel), _finite_float(shadow_input.stop_accel, -2.0), -0.05)
    last_command = _clip(_finite_float(shadow_input.last_output_accel, command_base), _finite_float(shadow_input.stop_accel, -2.0), 1.5)
    remaining_m = None if shadow_input.remaining_m is None else max(0.0, _finite_float(shadow_input.remaining_m))
    lead_gap = _finite_float(shadow_input.lead_d_rel) if shadow_input.lead_status and shadow_input.lead_d_rel is not None else None

    first_output = command_base
    min_a_ego = a_ego
    max_abs_jerk = 0.0
    max_cmd_step = 0.0
    max_cmd_jerk = 0.0
    rollout_m = max(0.0, _finite_float(shadow_input.rollout_m))
    min_lead_gap = lead_gap
    min_v_seen = v_ego
    speed_rebound = 0.0
    unexpected_accel = 0.0

    for step, residual in enumerate(residual_template):
      command = _clip(command_base + _clip(_finite_float(residual), -0.20, 0.20), _finite_float(shadow_input.stop_accel, -2.0), -0.05)
      if step == 0:
        first_output = command
      cmd_step = command - last_command
      max_cmd_step = max(max_cmd_step, abs(cmd_step))
      max_cmd_jerk = max(max_cmd_jerk, abs(cmd_step) / SHADOW_MODEL_DT_S)
      next_a = _clip(_predict_next_accel(a_ego, command, v_ego), -2.5, 1.5)
      max_abs_jerk = max(max_abs_jerk, abs(next_a - a_ego) / SHADOW_MODEL_DT_S)
      avg_v = max(0.0, v_ego + (0.5 * next_a * SHADOW_MODEL_DT_S))
      next_v = max(0.0, v_ego + (next_a * SHADOW_MODEL_DT_S))
      step_distance = max(0.0, (v_ego + next_v) * 0.5 * SHADOW_MODEL_DT_S)
      rollout_m += step_distance
      if remaining_m is not None:
        remaining_m = max(0.0, remaining_m - step_distance)
      if lead_gap is not None:
        lead_gap += (_finite_float(shadow_input.lead_v) - avg_v) * SHADOW_MODEL_DT_S
        min_lead_gap = lead_gap if min_lead_gap is None else min(min_lead_gap, lead_gap)
      min_a_ego = min(min_a_ego, next_a)
      if v_ego < 0.50:
        unexpected_accel = max(unexpected_accel, next_a)
      min_v_seen = min(min_v_seen, next_v)
      if min_v_seen < 0.35:
        speed_rebound = max(speed_rebound, next_v - min_v_seen)
      a_ego = next_a
      v_ego = next_v
      last_command = command

    is_harsh = min_a_ego < -1.15 or max_abs_jerk > 3.40 or max_cmd_step > 0.23
    is_leapfrog = speed_rebound > 0.055 or unexpected_accel > 0.16
    score = _score_simulation(
      min_a_ego=min_a_ego,
      max_abs_jerk=max_abs_jerk,
      max_cmd_step=max_cmd_step,
      max_cmd_jerk=max_cmd_jerk,
      rollout_m=rollout_m,
      final_v_ego=v_ego,
      final_remaining_m=remaining_m,
      min_lead_gap_m=min_lead_gap,
      speed_rebound_mps=speed_rebound,
      unexpected_accel_mps2=unexpected_accel,
      explicit_target_available=shadow_input.explicit_target_available,
    )
    return StoppingShadowSimulation(
      profile=profile,
      score=float(score),
      first_output_accel=float(first_output),
      min_a_ego=float(min_a_ego),
      max_abs_jerk=float(max_abs_jerk),
      max_cmd_step=float(max_cmd_step),
      max_cmd_jerk=float(max_cmd_jerk),
      rollout_m=float(rollout_m),
      final_v_ego=float(v_ego),
      final_remaining_m=remaining_m,
      min_lead_gap_m=min_lead_gap,
      speed_rebound_mps=float(speed_rebound),
      unexpected_accel_mps2=float(unexpected_accel),
      is_harsh=bool(is_harsh),
      is_leapfrog=bool(is_leapfrog),
    )

  def _reject_candidate(
    self,
    current: StoppingShadowSimulation,
    candidate: StoppingShadowSimulation,
    shadow_input: StoppingShadowInput,
  ) -> str | None:
    if candidate.is_leapfrog and not current.is_leapfrog:
      return "new_leapfrog_risk"
    if candidate.is_harsh and not current.is_harsh:
      return "new_harsh_risk"
    if candidate.rollout_m > current.rollout_m + 0.35:
      return "rollout_regression"
    if shadow_input.explicit_target_available and shadow_input.remaining_m is not None:
      remaining = max(0.0, _finite_float(shadow_input.remaining_m))
      if candidate.rollout_m > remaining + 0.30 and candidate.rollout_m > current.rollout_m + 0.10:
        return "target_overshoot_risk"
    if candidate.min_lead_gap_m is not None and shadow_input.lead_status:
      lead_floor = 1.10 if shadow_input.v_ego < 0.70 else 1.35
      if candidate.min_lead_gap_m < lead_floor:
        return "lead_gap_risk"
      if current.min_lead_gap_m is not None and candidate.min_lead_gap_m < current.min_lead_gap_m - 0.15:
        return "lead_gap_regression"
    if candidate.final_v_ego > current.final_v_ego + 0.20 and candidate.rollout_m > current.rollout_m + 0.12:
      return "late_stop_speed_regression"
    return None


def _predict_next_accel(a_ego_prev: float, accel_cmd_delayed: float, v_ego: float) -> float:
  low_speed = _clip((SHADOW_MODEL_LOW_SPEED_REF - v_ego) / max(SHADOW_MODEL_LOW_SPEED_REF, 1e-6), 0.0, 1.0)
  relief = max(0.0, accel_cmd_delayed - SHADOW_MODEL_RELIEF_CMD_THRESHOLD)
  return (
    SHADOW_MODEL_COEFFICIENTS["intercept"]
    + (SHADOW_MODEL_COEFFICIENTS["a_ego_prev"] * a_ego_prev)
    + (SHADOW_MODEL_COEFFICIENTS["accel_cmd_delayed"] * accel_cmd_delayed)
    + (SHADOW_MODEL_COEFFICIENTS["v_ego"] * v_ego)
    + (SHADOW_MODEL_COEFFICIENTS["relief"] * relief)
    + (SHADOW_MODEL_COEFFICIENTS["low_speed"] * low_speed)
    + (SHADOW_MODEL_COEFFICIENTS["cmd_x_low_speed"] * accel_cmd_delayed * low_speed)
  )


def _score_simulation(
  *,
  min_a_ego: float,
  max_abs_jerk: float,
  max_cmd_step: float,
  max_cmd_jerk: float,
  rollout_m: float,
  final_v_ego: float,
  final_remaining_m: float | None,
  min_lead_gap_m: float | None,
  speed_rebound_mps: float,
  unexpected_accel_mps2: float,
  explicit_target_available: bool,
) -> float:
  decel_cost = 1.25 * max(0.0, -min_a_ego - 0.72)
  jerk_cost = 0.18 * max(0.0, max_abs_jerk - 2.20)
  command_step_cost = 0.60 * max(0.0, max_cmd_step - 0.09)
  command_jerk_cost = 0.08 * max(0.0, max_cmd_jerk - 0.85)
  rollout_limit = 2.00
  if explicit_target_available and final_remaining_m is not None:
    rollout_limit = 1.25
  rollout_cost = 1.10 * max(0.0, rollout_m - rollout_limit)
  final_speed_cost = 0.60 * max(0.0, final_v_ego - 0.18)
  target_left_cost = 0.0
  if explicit_target_available and final_remaining_m is not None:
    target_left_cost = 0.20 * max(0.0, final_remaining_m - 0.75)
  lead_cost = 0.0
  if min_lead_gap_m is not None:
    lead_cost = 2.0 * max(0.0, 1.10 - min_lead_gap_m)
  leapfrog_cost = (4.0 * max(0.0, speed_rebound_mps - 0.03)) + (2.0 * max(0.0, unexpected_accel_mps2 - 0.08))
  return decel_cost + jerk_cost + command_step_cost + command_jerk_cost + rollout_cost + final_speed_cost + target_left_cost + lead_cost + leapfrog_cost


def shadow_log_payload(
  debug: dict[str, object],
  *,
  v_ego: float,
  a_ego: float,
  output_accel: float,
  lead_status: bool,
  lead_v: float,
  lead_d_rel: float,
) -> dict[str, object]:
  return {
    "version": str(debug.get("shadow_version", "")),
    "profile": str(debug.get("shadow_profile", "")),
    "confidence": round(_finite_float(debug.get("shadow_confidence")), 4),
    "score_delta": round(_finite_float(debug.get("shadow_score_delta")), 4),
    "score_current": round(_finite_float(debug.get("shadow_score_current")), 4),
    "score_selected": round(_finite_float(debug.get("shadow_score_selected")), 4),
    "reason": str(debug.get("shadow_rejection_reason", "")),
    "first_output_accel": round(_finite_float(debug.get("shadow_first_output_accel")), 4),
    "actual_output_accel": round(_finite_float(output_accel), 4),
    "v_ego": round(_finite_float(v_ego), 4),
    "a_ego": round(_finite_float(a_ego), 4),
    "remaining_m": round(_finite_float(debug.get("remaining_m")), 4),
    "rollout_m": round(_finite_float(debug.get("rollout_m")), 4),
    "lead_status": bool(lead_status),
    "lead_v": round(_finite_float(lead_v), 4),
    "lead_d_rel": round(_finite_float(lead_d_rel), 4),
    "selected_rollout_m": round(_finite_float(debug.get("shadow_selected_rollout_m")), 4),
    "selected_min_a_ego": round(_finite_float(debug.get("shadow_selected_min_a_ego")), 4),
    "selected_final_v_ego": round(_finite_float(debug.get("shadow_selected_final_v_ego")), 4),
    "selected_harsh": bool(debug.get("shadow_selected_harsh", False)),
    "selected_leapfrog": bool(debug.get("shadow_selected_leapfrog", False)),
  }
