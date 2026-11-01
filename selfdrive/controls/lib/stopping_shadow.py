from __future__ import annotations

from math import isfinite
from typing import Any


STOPPING_SHADOW_LOGGING_ENABLED = True
STOPPING_SHADOW_SAMPLE_INTERVAL_FRAMES = 10
STOPPING_SHADOW_LOG_PERIOD_S = 2.0
STOPPING_SHADOW_PROFILE_LOG_PERIOD_S = 0.8


def _finite_float(value: Any, default: float = 0.0) -> float:
  try:
    result = float(value)
  except (TypeError, ValueError):
    return default
  return result if isfinite(result) else default


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
    "observer_scope": str(debug.get("shadow_observer_scope", "")),
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
