from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, sqrt
from typing import Any


PROFILE_NO_CHANGE = "no_change"
PROFILE_PRESERVE_BRAKE = "preserve_brake"
PROFILE_SOFTEN_THEN_DEEPEN = "soften_then_deepen"
PROFILE_TAIL_DEEPEN = "tail_deepen"
PROFILE_GLIDE_SOFTEN = "glide_soften"

PROFILE_LABELS = (
  PROFILE_NO_CHANGE,
  PROFILE_PRESERVE_BRAKE,
  PROFILE_SOFTEN_THEN_DEEPEN,
  PROFILE_TAIL_DEEPEN,
  PROFILE_GLIDE_SOFTEN,
)

SELECTOR_FEATURE_NAMES = (
  "v_ego_mps",
  "a_ego_mps2",
  "last_output_accel_mps2",
  "rollout_m",
  "remaining_m",
  "remaining_present",
  "lead_d_rel_m",
  "lead_present",
  "lead_v_mps",
  "phase",
  "release_lock_active",
  "rebound_arrest_active",
  "explicit_target_available",
  "should_stop",
)

DEFAULT_FEATURE_SCALES = {
  "v_ego_mps": 0.55,
  "a_ego_mps2": 0.45,
  "last_output_accel_mps2": 0.35,
  "rollout_m": 0.65,
  "remaining_m": 1.00,
  "remaining_present": 1.00,
  "lead_d_rel_m": 3.00,
  "lead_present": 1.00,
  "lead_v_mps": 0.75,
  "phase": 1.00,
  "release_lock_active": 1.00,
  "rebound_arrest_active": 1.00,
  "explicit_target_available": 1.00,
  "should_stop": 1.00,
}

DEFAULT_RESIDUAL_TEMPLATE_MPS2 = (0.0, 0.0, 0.0)


def _finite_float(value: Any, default: float = 0.0) -> float:
  try:
    result = float(value)
  except (TypeError, ValueError):
    return default
  return result if isfinite(result) else default


def _clip(value: float, lower: float, upper: float) -> float:
  return max(lower, min(upper, value))


def _bool_float(value: Any) -> float:
  return 1.0 if bool(value) else 0.0


@dataclass(frozen=True)
class StoppingProfileFeatures:
  v_ego_mps: float
  a_ego_mps2: float
  last_output_accel_mps2: float
  rollout_m: float
  remaining_m: float | None
  lead_d_rel_m: float | None
  lead_v_mps: float
  phase: int
  release_lock_active: bool
  rebound_arrest_active: bool
  explicit_target_available: bool
  should_stop: bool

  def as_dict(self) -> dict[str, float]:
    remaining_present = self.remaining_m is not None and self.explicit_target_available
    lead_present = self.lead_d_rel_m is not None
    return {
      "v_ego_mps": _clip(_finite_float(self.v_ego_mps), 0.0, 2.5),
      "a_ego_mps2": _clip(_finite_float(self.a_ego_mps2), -2.5, 1.5),
      "last_output_accel_mps2": _clip(_finite_float(self.last_output_accel_mps2), -2.5, 1.5),
      "rollout_m": _clip(_finite_float(self.rollout_m), 0.0, 3.5),
      "remaining_m": _clip(_finite_float(self.remaining_m), -1.0, 6.0) if remaining_present else 0.0,
      "remaining_present": _bool_float(remaining_present),
      "lead_d_rel_m": _clip(_finite_float(self.lead_d_rel_m), 0.0, 15.0) if lead_present else 0.0,
      "lead_present": _bool_float(lead_present),
      "lead_v_mps": _clip(_finite_float(self.lead_v_mps), -3.0, 3.0) if lead_present else 0.0,
      "phase": _clip(float(int(self.phase)), 0.0, 2.0),
      "release_lock_active": _bool_float(self.release_lock_active),
      "rebound_arrest_active": _bool_float(self.rebound_arrest_active),
      "explicit_target_available": _bool_float(self.explicit_target_available),
      "should_stop": _bool_float(self.should_stop),
    }

  @classmethod
  def from_mapping(cls, data: dict[str, Any]) -> StoppingProfileFeatures:
    remaining_present = bool(data.get("remaining_present", data.get("explicit_target_available", False)))
    lead_present = bool(data.get("lead_present", data.get("lead_d_rel_m") is not None))
    return cls(
      v_ego_mps=_finite_float(data.get("v_ego_mps")),
      a_ego_mps2=_finite_float(data.get("a_ego_mps2")),
      last_output_accel_mps2=_finite_float(data.get("last_output_accel_mps2")),
      rollout_m=_finite_float(data.get("rollout_m")),
      remaining_m=_finite_float(data.get("remaining_m")) if remaining_present else None,
      lead_d_rel_m=_finite_float(data.get("lead_d_rel_m")) if lead_present else None,
      lead_v_mps=_finite_float(data.get("lead_v_mps")),
      phase=int(_finite_float(data.get("phase"))),
      release_lock_active=bool(data.get("release_lock_active", False)),
      rebound_arrest_active=bool(data.get("rebound_arrest_active", False)),
      explicit_target_available=bool(data.get("explicit_target_available", remaining_present)),
      should_stop=bool(data.get("should_stop", True)),
    )


@dataclass(frozen=True)
class StoppingProfileDecision:
  profile: str
  confidence: float
  distance: float
  second_distance: float | None


class PrototypeStoppingProfileSelector:
  """Small nearest-prototype selector trained offline from horizon-teacher labels."""

  def __init__(
    self,
    *,
    feature_names: tuple[str, ...],
    feature_scales: dict[str, float],
    prototypes: dict[str, dict[str, float]],
    selector_kind: str = "prototype",
    exemplars: list[tuple[str, dict[str, float]]] | None = None,
    knn_k: int = 5,
  ) -> None:
    self.feature_names = feature_names
    self.feature_scales = feature_scales
    self.prototypes = {
      profile: prototype
      for profile, prototype in prototypes.items()
      if profile in PROFILE_LABELS
    }
    self.selector_kind = selector_kind
    self.exemplars = [
      (profile, features)
      for profile, features in (exemplars or [])
      if profile in PROFILE_LABELS
    ]
    self.knn_k = max(1, int(knn_k))

  @classmethod
  def from_json(cls, data: dict[str, Any]) -> PrototypeStoppingProfileSelector:
    feature_names = tuple(str(name) for name in data.get("feature_names", SELECTOR_FEATURE_NAMES))
    feature_scales_raw = data.get("feature_scales", DEFAULT_FEATURE_SCALES)
    feature_scales = {
      name: max(_finite_float(feature_scales_raw.get(name, DEFAULT_FEATURE_SCALES.get(name, 1.0)), DEFAULT_FEATURE_SCALES.get(name, 1.0)), 1e-3)
      for name in feature_names
    }
    prototypes: dict[str, dict[str, float]] = {}
    exemplars: list[tuple[str, dict[str, float]]] = []
    for item in data.get("profiles", []):
      if not isinstance(item, dict):
        continue
      profile = str(item.get("profile", ""))
      prototype_raw = item.get("prototype", {})
      if profile not in PROFILE_LABELS or not isinstance(prototype_raw, dict):
        continue
      prototypes[profile] = {name: _finite_float(prototype_raw.get(name)) for name in feature_names}
      for exemplar in item.get("exemplars", []):
        if not isinstance(exemplar, dict) or not isinstance(exemplar.get("features"), dict):
          continue
        exemplars.append((profile, {name: _finite_float(exemplar["features"].get(name)) for name in feature_names}))
    model_type = str(data.get("model_type", "nearest_profile_prototype"))
    selector_kind = str(data.get("selector_kind", "knn" if model_type == "nearest_profile_knn" else "prototype"))
    return cls(
      feature_names=feature_names,
      feature_scales=feature_scales,
      prototypes=prototypes,
      selector_kind=selector_kind,
      exemplars=exemplars,
      knn_k=int(data.get("knn_k", 5)),
    )

  def _select_knn(self, row: dict[str, float]) -> StoppingProfileDecision | None:
    if not self.exemplars:
      return None

    scored = sorted(
      (
        _normalized_feature_distance(self.feature_names, self.feature_scales, row, features),
        profile,
      )
      for profile, features in self.exemplars
    )
    neighbors = scored[:self.knn_k]
    if not neighbors:
      return None

    weights: dict[str, float] = {}
    best_distance_by_profile: dict[str, float] = {}
    for distance, profile in neighbors:
      weights[profile] = weights.get(profile, 0.0) + (1.0 / max(distance, 0.05))
      best_distance_by_profile[profile] = min(distance, best_distance_by_profile.get(profile, float("inf")))

    ranked_profiles = sorted(weights.items(), key=lambda item: item[1], reverse=True)
    profile, best_weight = ranked_profiles[0]
    second_profile = ranked_profiles[1][0] if len(ranked_profiles) > 1 else None
    total_weight = sum(weights.values())
    best_distance = best_distance_by_profile.get(profile, neighbors[0][0])
    second_distance = best_distance_by_profile.get(second_profile) if second_profile is not None else None
    support = best_weight / max(total_weight, 1e-6)
    distance_factor = 1.0 / (1.0 + best_distance)
    confidence = support * distance_factor
    return StoppingProfileDecision(profile=profile, confidence=float(confidence), distance=float(best_distance), second_distance=second_distance)

  def select(self, features: StoppingProfileFeatures) -> StoppingProfileDecision:
    row = features.as_dict()
    if self.selector_kind == "knn":
      knn_decision = self._select_knn(row)
      if knn_decision is not None:
        return knn_decision

    if not self.prototypes:
      return StoppingProfileDecision(profile=PROFILE_NO_CHANGE, confidence=0.0, distance=float("inf"), second_distance=None)

    scored: list[tuple[float, str]] = []
    for profile, prototype in self.prototypes.items():
      total = 0.0
      for name in self.feature_names:
        scale = max(self.feature_scales.get(name, DEFAULT_FEATURE_SCALES.get(name, 1.0)), 1e-3)
        diff = (row.get(name, 0.0) - prototype.get(name, 0.0)) / scale
        total += diff * diff
      scored.append((sqrt(total / max(len(self.feature_names), 1)), profile))

    scored.sort(key=lambda item: item[0])
    best_distance, profile = scored[0]
    second_distance = scored[1][0] if len(scored) > 1 else None
    margin = 0.0 if second_distance is None else max(0.0, second_distance - best_distance) / max(second_distance, 1e-3)
    confidence = (1.0 / (1.0 + best_distance)) * (0.60 + (0.40 * margin))
    return StoppingProfileDecision(profile=profile, confidence=float(confidence), distance=float(best_distance), second_distance=second_distance)


def _normalized_feature_distance(
  feature_names: tuple[str, ...],
  feature_scales: dict[str, float],
  left: dict[str, float],
  right: dict[str, float],
) -> float:
  total = 0.0
  for name in feature_names:
    scale = max(feature_scales.get(name, DEFAULT_FEATURE_SCALES.get(name, 1.0)), 1e-3)
    diff = (left.get(name, 0.0) - right.get(name, 0.0)) / scale
    total += diff * diff
  return sqrt(total / max(len(feature_names), 1))


def nearest_exemplar_distance_for_profile(
  selector_payload: dict[str, Any],
  profile: str,
  features: dict[str, float] | None,
) -> float | None:
  if profile == PROFILE_NO_CHANGE or features is None:
    return None

  feature_names = tuple(str(name) for name in selector_payload.get("feature_names", SELECTOR_FEATURE_NAMES))
  feature_scales_raw = selector_payload.get("feature_scales", DEFAULT_FEATURE_SCALES)
  feature_scales = {
    name: max(_finite_float(feature_scales_raw.get(name, DEFAULT_FEATURE_SCALES.get(name, 1.0)), DEFAULT_FEATURE_SCALES.get(name, 1.0)), 1e-3)
    for name in feature_names
  }
  normalized_features = StoppingProfileFeatures.from_mapping(features).as_dict()
  best_distance: float | None = None
  for item in selector_payload.get("profiles", []):
    if not isinstance(item, dict) or str(item.get("profile", "")) != profile:
      continue
    exemplars = item.get("exemplars", [])
    if not isinstance(exemplars, list):
      return None
    for exemplar in exemplars:
      if not isinstance(exemplar, dict) or not isinstance(exemplar.get("features"), dict):
        continue
      exemplar_features = StoppingProfileFeatures.from_mapping(exemplar["features"]).as_dict()
      distance = _normalized_feature_distance(feature_names, feature_scales, normalized_features, exemplar_features)
      if best_distance is None or distance < best_distance:
        best_distance = distance
    return best_distance
  return None


def residual_template_for_profile(
  selector_payload: dict[str, Any],
  profile: str,
  features: dict[str, float] | None = None,
  *,
  max_exemplar_distance: float = 0.90,
) -> list[float]:
  if profile == PROFILE_NO_CHANGE:
    return list(DEFAULT_RESIDUAL_TEMPLATE_MPS2)
  feature_names = tuple(str(name) for name in selector_payload.get("feature_names", SELECTOR_FEATURE_NAMES))
  feature_scales_raw = selector_payload.get("feature_scales", DEFAULT_FEATURE_SCALES)
  feature_scales = {
    name: max(_finite_float(feature_scales_raw.get(name, DEFAULT_FEATURE_SCALES.get(name, 1.0)), DEFAULT_FEATURE_SCALES.get(name, 1.0)), 1e-3)
    for name in feature_names
  }
  for item in selector_payload.get("profiles", []):
    if not isinstance(item, dict) or str(item.get("profile", "")) != profile:
      continue
    exemplars = item.get("exemplars", [])
    if features is not None and isinstance(exemplars, list):
      scored: list[tuple[float, list[float]]] = []
      normalized_features = StoppingProfileFeatures.from_mapping(features).as_dict()
      for exemplar in exemplars:
        if not isinstance(exemplar, dict) or not isinstance(exemplar.get("features"), dict):
          continue
        raw_template = exemplar.get("residual_template_mps2", [])
        if not isinstance(raw_template, list) or not raw_template:
          continue
        exemplar_features = StoppingProfileFeatures.from_mapping(exemplar["features"]).as_dict()
        distance = _normalized_feature_distance(feature_names, feature_scales, normalized_features, exemplar_features)
        scored.append((distance, [_clip(_finite_float(value), -0.20, 0.20) for value in raw_template]))
      if scored:
        scored.sort(key=lambda item: item[0])
        if scored[0][0] <= max_exemplar_distance:
          return scored[0][1]
    raw_template = item.get("residual_template_mps2", DEFAULT_RESIDUAL_TEMPLATE_MPS2)
    if not isinstance(raw_template, list):
      return list(DEFAULT_RESIDUAL_TEMPLATE_MPS2)
    return [_clip(_finite_float(value), -0.20, 0.20) for value in raw_template]
  return list(DEFAULT_RESIDUAL_TEMPLATE_MPS2)


def profile_label_from_horizon_teacher(teacher: dict[str, Any], min_improvement: float = 0.01) -> str:
  if teacher.get("status") != "ok":
    return PROFILE_NO_CHANGE
  score_delta = _finite_float(teacher.get("score_delta"))
  if score_delta > -abs(float(min_improvement)):
    return PROFILE_NO_CHANGE

  intent = str(teacher.get("intent", ""))
  if intent in ("deepen_then_soften", "reshape"):
    return PROFILE_PRESERVE_BRAKE
  if intent == "soften_then_deepen":
    return PROFILE_SOFTEN_THEN_DEEPEN
  if intent in ("deepen", "tail_deepen"):
    return PROFILE_TAIL_DEEPEN
  if intent in ("soften", "tail_soften"):
    return PROFILE_GLIDE_SOFTEN
  return PROFILE_NO_CHANGE
