from __future__ import annotations

from dataclasses import asdict, dataclass
from math import sqrt
from statistics import median
from typing import Any

import numpy as np


FEATURE_NAMES = (
  "intercept",
  "a_ego_prev",
  "accel_cmd_delayed",
  "v_ego",
  "relief",
  "low_speed",
  "cmd_x_low_speed",
)
FEATURE_INDEX = {name: idx for idx, name in enumerate(FEATURE_NAMES)}
V_EGO_FEATURE_INDEX = FEATURE_INDEX["v_ego"]
MODEL_KIND_LINEAR = "linear"
MODEL_KIND_SPEED_BAND = "speed_band_linear"
SUPPORTED_MODEL_KINDS = (MODEL_KIND_LINEAR, MODEL_KIND_SPEED_BAND)
SPEED_BAND_NAMES = ("low", "high")
SPEED_BAND_CANDIDATE_QUANTILES = (0.20, 0.35, 0.50, 0.65, 0.80)


@dataclass
class DelayFit:
  delay_frames: int
  sample_count: int
  rmse: float
  mae: float
  r2: float


@dataclass
class ModelParameters:
  coefficients: np.ndarray
  model_kind: str = MODEL_KIND_LINEAR
  speed_split_mps: float | None = None
  band_coefficients: dict[str, np.ndarray] | None = None
  band_sample_counts: dict[str, int] | None = None


def _coefficients_dict(values: np.ndarray) -> dict[str, float]:
  return {name: float(values[idx]) for idx, name in enumerate(FEATURE_NAMES)}


def _coefficients_array(values: dict[str, float]) -> np.ndarray:
  return np.array([float(values.get(name, 0.0)) for name in FEATURE_NAMES], dtype=float)


def _normalized_coefficients(raw: Any) -> dict[str, float]:
  if not isinstance(raw, dict):
    return {name: 0.0 for name in FEATURE_NAMES}
  return {name: float(raw.get(name, 0.0)) for name in FEATURE_NAMES}


def _normalized_band_coefficients(raw: Any) -> dict[str, dict[str, float]] | None:
  if not isinstance(raw, dict):
    return None
  return {str(name): _normalized_coefficients(coefficients) for name, coefficients in raw.items()}


@dataclass
class FittedStoppingModel:
  delay_frames: int
  coefficients: dict[str, float]
  rmse: float
  mae: float
  r2: float
  sample_count: int
  dt_s: float
  relief_cmd_threshold: float
  low_speed_ref: float
  model_kind: str = MODEL_KIND_LINEAR
  speed_split_mps: float | None = None
  band_coefficients: dict[str, dict[str, float]] | None = None
  band_sample_counts: dict[str, int] | None = None

  def as_json(self) -> dict[str, Any]:
    payload = asdict(self)
    payload["feature_names"] = list(FEATURE_NAMES)
    return payload

  @classmethod
  def from_json(cls, data: dict[str, Any]) -> FittedStoppingModel:
    coefficients = _normalized_coefficients(data.get("coefficients", {}))
    band_coefficients = _normalized_band_coefficients(data.get("band_coefficients", data.get("regime_coefficients")))
    band_sample_counts_raw = data.get("band_sample_counts", {})
    band_sample_counts = None
    if isinstance(band_sample_counts_raw, dict):
      band_sample_counts = {str(name): int(count) for name, count in band_sample_counts_raw.items()}
    return cls(
      delay_frames=int(data["delay_frames"]),
      coefficients=coefficients,
      rmse=float(data["rmse"]),
      mae=float(data["mae"]),
      r2=float(data["r2"]),
      sample_count=int(data["sample_count"]),
      dt_s=float(data["dt_s"]),
      relief_cmd_threshold=float(data["relief_cmd_threshold"]),
      low_speed_ref=float(data["low_speed_ref"]),
      model_kind=str(data.get("model_kind", MODEL_KIND_LINEAR)),
      speed_split_mps=float(data["speed_split_mps"]) if data.get("speed_split_mps") is not None else None,
      band_coefficients=band_coefficients,
      band_sample_counts=band_sample_counts,
    )

  def predict_next(self, a_ego_prev: float, accel_cmd_delayed: float, v_ego: float) -> float:
    coefficients = self.effective_coefficients(v_ego)
    relief = max(0.0, accel_cmd_delayed - self.relief_cmd_threshold)
    low_speed = max(0.0, min(1.0, (self.low_speed_ref - v_ego) / max(self.low_speed_ref, 1e-6)))
    row = np.array([
      1.0,
      a_ego_prev,
      accel_cmd_delayed,
      v_ego,
      relief,
      low_speed,
      accel_cmd_delayed * low_speed,
    ], dtype=float)
    coef = _coefficients_array(coefficients)
    return float(np.dot(row, coef))

  def effective_coefficients(self, v_ego: float) -> dict[str, float]:
    coefficients = self.coefficients
    if (
      self.model_kind == MODEL_KIND_SPEED_BAND
      and self.band_coefficients is not None
      and self.speed_split_mps is not None
    ):
      band_name = SPEED_BAND_NAMES[0] if v_ego <= self.speed_split_mps else SPEED_BAND_NAMES[1]
      coefficients = self.band_coefficients.get(band_name, coefficients)
    return coefficients


def _design_row(a_ego_prev: float, accel_cmd_delayed: float, v_ego: float, relief_cmd_threshold: float, low_speed_ref: float) -> np.ndarray:
  relief = max(0.0, accel_cmd_delayed - relief_cmd_threshold)
  low_speed = max(0.0, min(1.0, (low_speed_ref - v_ego) / max(low_speed_ref, 1e-6)))
  return np.array([
    1.0,
    a_ego_prev,
    accel_cmd_delayed,
    v_ego,
    relief,
    low_speed,
    accel_cmd_delayed * low_speed,
  ], dtype=float)


def _score(y_true: np.ndarray, y_pred: np.ndarray) -> tuple[float, float, float]:
  errors = y_pred - y_true
  mse = float(np.mean(np.square(errors)))
  rmse = sqrt(max(mse, 0.0))
  mae = float(np.mean(np.abs(errors)))
  centered = y_true - float(np.mean(y_true))
  denom = float(np.sum(np.square(centered)))
  if denom <= 1e-9:
    r2 = 0.0
  else:
    r2 = 1.0 - (float(np.sum(np.square(errors))) / denom)
  return rmse, mae, r2


def _fit_linear_coefficients(x: np.ndarray, y: np.ndarray) -> np.ndarray:
  coefficients, *_ = np.linalg.lstsq(x, y, rcond=None)
  return coefficients


def _candidate_speed_splits(speeds: np.ndarray, min_rows_per_band: int) -> list[float]:
  if len(speeds) < (2 * min_rows_per_band):
    return []

  candidates = {
    round(float(np.quantile(speeds, quantile)), 6)
    for quantile in SPEED_BAND_CANDIDATE_QUANTILES
  }
  median_speed = round(float(np.median(speeds)), 6)
  candidates.add(median_speed)

  valid_candidates: list[float] = []
  for split in sorted(candidates):
    low_count = int(np.count_nonzero(speeds <= split))
    high_count = len(speeds) - low_count
    if low_count >= min_rows_per_band and high_count >= min_rows_per_band:
      valid_candidates.append(split)
  return valid_candidates


def _fit_speed_band_parameters(
  x: np.ndarray,
  y: np.ndarray,
  linear_coefficients: np.ndarray,
  min_rows_per_band: int,
) -> tuple[ModelParameters, np.ndarray] | None:
  speeds = x[:, V_EGO_FEATURE_INDEX]
  candidates = _candidate_speed_splits(speeds, min_rows_per_band)
  if not candidates:
    return None

  median_speed = float(np.median(speeds))
  best_parameters: ModelParameters | None = None
  best_predictions: np.ndarray | None = None
  best_score: tuple[float, float] | None = None

  for split in candidates:
    low_mask = speeds <= split
    high_mask = np.logical_not(low_mask)
    low_count = int(np.count_nonzero(low_mask))
    high_count = int(np.count_nonzero(high_mask))
    if low_count < min_rows_per_band or high_count < min_rows_per_band:
      continue

    low_coefficients = _fit_linear_coefficients(x[low_mask], y[low_mask])
    high_coefficients = _fit_linear_coefficients(x[high_mask], y[high_mask])
    predictions = np.empty_like(y)
    predictions[low_mask] = x[low_mask] @ low_coefficients
    predictions[high_mask] = x[high_mask] @ high_coefficients
    rmse, _, _ = _score(y, predictions)
    candidate_score = (rmse, abs(float(split) - median_speed))

    if best_score is None or candidate_score < best_score:
      best_score = candidate_score
      best_predictions = predictions
      best_parameters = ModelParameters(
        coefficients=linear_coefficients,
        model_kind=MODEL_KIND_SPEED_BAND,
        speed_split_mps=float(split),
        band_coefficients={
          SPEED_BAND_NAMES[0]: low_coefficients,
          SPEED_BAND_NAMES[1]: high_coefficients,
        },
        band_sample_counts={
          SPEED_BAND_NAMES[0]: low_count,
          SPEED_BAND_NAMES[1]: high_count,
        },
      )

  if best_parameters is None or best_predictions is None:
    return None
  return best_parameters, best_predictions


def estimate_dt_s(samples: list[Any]) -> float:
  deltas: list[float] = []
  for prev, cur in zip(samples, samples[1:], strict=False):
    dt = float(cur.t) - float(prev.t)
    if dt > 1e-6:
      deltas.append(dt)
  if not deltas:
    return 0.01
  return float(median(deltas))


def build_training_matrix(
  samples: list[Any],
  windows: list[tuple[int, int]],
  delay_frames: int,
  min_speed: float,
  max_speed: float,
  relief_cmd_threshold: float,
  low_speed_ref: float,
  require_enabled: bool,
) -> tuple[np.ndarray, np.ndarray]:
  rows: list[np.ndarray] = []
  targets: list[float] = []
  last_index = len(samples) - 1

  for start_idx, end_idx in windows:
    start = max(0, int(start_idx))
    end = min(last_index, int(end_idx))
    if end - start < 2:
      continue

    for idx in range(start, end):
      next_idx = idx + 1
      delayed_idx = idx - delay_frames
      if delayed_idx < start:
        continue
      if next_idx > end:
        break

      sample = samples[idx]
      delayed_sample = samples[delayed_idx]
      next_sample = samples[next_idx]

      if require_enabled:
        # The response model is intended to capture vehicle response to actuator commands.
        # When openpilot is not enabled, commands may be published but not applied, so they corrupt the fit.
        if not bool(getattr(sample, "enabled", True)):
          continue
        if not bool(getattr(delayed_sample, "enabled", True)):
          continue
        if not bool(getattr(next_sample, "enabled", True)):
          continue
      if not bool(sample.should_stop):
        continue
      if delayed_sample.accel_cmd is None:
        continue

      a_prev = float(sample.a_ego)
      cmd_delayed = float(delayed_sample.accel_cmd)
      v_ego = float(sample.v_ego)
      a_next = float(next_sample.a_ego)

      if not (min_speed <= v_ego <= max_speed):
        continue
      if any(np.isnan(value) or np.isinf(value) for value in (a_prev, cmd_delayed, v_ego, a_next)):
        continue

      rows.append(_design_row(a_prev, cmd_delayed, v_ego, relief_cmd_threshold, low_speed_ref))
      targets.append(a_next)

  if not rows:
    return np.zeros((0, len(FEATURE_NAMES)), dtype=float), np.zeros((0,), dtype=float)

  x = np.vstack(rows).astype(float)
  y = np.array(targets, dtype=float)
  return x, y


def fit_with_delay(
  samples: list[Any],
  windows: list[tuple[int, int]],
  delay_frames: int,
  min_speed: float,
  max_speed: float,
  relief_cmd_threshold: float,
  low_speed_ref: float,
  require_enabled: bool,
  model_kind: str = MODEL_KIND_LINEAR,
  speed_band_min_rows: int = 60,
) -> tuple[ModelParameters | np.ndarray | list[float] | None, DelayFit]:
  x, y = build_training_matrix(
    samples=samples,
    windows=windows,
    delay_frames=delay_frames,
    min_speed=min_speed,
    max_speed=max_speed,
    relief_cmd_threshold=relief_cmd_threshold,
    low_speed_ref=low_speed_ref,
    require_enabled=require_enabled,
  )
  if len(y) == 0:
    return None, DelayFit(delay_frames=delay_frames, sample_count=0, rmse=float("inf"), mae=float("inf"), r2=0.0)

  linear_coefficients = _fit_linear_coefficients(x, y)
  parameters = ModelParameters(coefficients=linear_coefficients, model_kind=MODEL_KIND_LINEAR)
  y_pred = x @ linear_coefficients

  if model_kind == MODEL_KIND_SPEED_BAND:
    band_fit = _fit_speed_band_parameters(
      x=x,
      y=y,
      linear_coefficients=linear_coefficients,
      min_rows_per_band=speed_band_min_rows,
    )
    if band_fit is not None:
      parameters, y_pred = band_fit
  elif model_kind != MODEL_KIND_LINEAR:
    raise ValueError(f"Unsupported model_kind: {model_kind}")

  rmse, mae, r2 = _score(y, y_pred)
  return parameters, DelayFit(delay_frames=delay_frames, sample_count=len(y), rmse=rmse, mae=mae, r2=r2)


def fit_stopping_model(
  samples: list[Any],
  windows: list[tuple[int, int]],
  max_delay_frames: int = 20,
  min_speed: float = 0.0,
  max_speed: float = 2.0,
  relief_cmd_threshold: float = -0.25,
  low_speed_ref: float = 1.2,
  min_rows: int = 50,
  delay_min_sample_ratio: float = 0.40,
  delay_rmse_tolerance: float = 0.03,
  require_enabled: bool = True,
  model_kind: str = MODEL_KIND_LINEAR,
  speed_band_min_rows: int = 60,
) -> tuple[FittedStoppingModel, list[DelayFit]]:
  if max_delay_frames < 0:
    raise ValueError("max_delay_frames must be >= 0")
  if not (0.0 < delay_min_sample_ratio <= 1.0):
    raise ValueError("delay_min_sample_ratio must be in (0, 1]")
  if delay_rmse_tolerance < 0.0:
    raise ValueError("delay_rmse_tolerance must be >= 0")
  if model_kind not in SUPPORTED_MODEL_KINDS:
    raise ValueError(f"model_kind must be one of {SUPPORTED_MODEL_KINDS}")
  if speed_band_min_rows <= 0:
    raise ValueError("speed_band_min_rows must be > 0")

  best_parameters: ModelParameters | None = None
  best_fit: DelayFit | None = None
  delay_fits: list[DelayFit] = []
  parameters_by_delay: dict[int, ModelParameters] = {}

  for delay in range(max_delay_frames + 1):
    parameters, fit = fit_with_delay(
      samples=samples,
      windows=windows,
      delay_frames=delay,
      min_speed=min_speed,
      max_speed=max_speed,
      relief_cmd_threshold=relief_cmd_threshold,
      low_speed_ref=low_speed_ref,
      require_enabled=require_enabled,
      model_kind=model_kind,
      speed_band_min_rows=speed_band_min_rows,
    )
    delay_fits.append(fit)
    if parameters is None:
      continue
    if isinstance(parameters, ModelParameters):
      parameters_by_delay[delay] = parameters
    else:
      parameters_by_delay[delay] = ModelParameters(
        coefficients=np.asarray(parameters, dtype=float),
        model_kind=MODEL_KIND_LINEAR,
      )

  if not parameters_by_delay:
    raise RuntimeError("Unable to fit stopping model: no valid training rows")

  valid_fits = [fit for fit in delay_fits if fit.sample_count > 0 and np.isfinite(fit.rmse)]
  max_rows = max(fit.sample_count for fit in valid_fits)
  min_rows_by_ratio = int(max_rows * delay_min_sample_ratio)
  min_candidate_rows = max(min_rows, min_rows_by_ratio)
  candidates = [fit for fit in valid_fits if fit.sample_count >= min_candidate_rows]
  if not candidates:
    # Fallback: if ratio filtering is too strict on sparse datasets, use all valid fits.
    candidates = valid_fits

  best_rmse = min(fit.rmse for fit in candidates)
  rmse_limit = best_rmse * (1.0 + delay_rmse_tolerance)
  near_best = [fit for fit in candidates if fit.rmse <= rmse_limit]
  best_fit = min(near_best, key=lambda fit: (fit.delay_frames, fit.rmse))
  best_parameters = parameters_by_delay.get(best_fit.delay_frames)

  if best_parameters is None:
    raise RuntimeError(f"Unable to fit stopping model: missing coefficients for delay={best_fit.delay_frames}")
  if best_fit.sample_count < min_rows:
    raise RuntimeError(f"Unable to fit stopping model: only {best_fit.sample_count} rows (min_rows={min_rows})")

  coefficients = _coefficients_dict(best_parameters.coefficients)
  band_coefficients = None
  if best_parameters.band_coefficients is not None:
    band_coefficients = {
      name: _coefficients_dict(values) for name, values in best_parameters.band_coefficients.items()
    }
  model = FittedStoppingModel(
    delay_frames=best_fit.delay_frames,
    coefficients=coefficients,
    rmse=best_fit.rmse,
    mae=best_fit.mae,
    r2=best_fit.r2,
    sample_count=best_fit.sample_count,
    dt_s=estimate_dt_s(samples),
    relief_cmd_threshold=relief_cmd_threshold,
    low_speed_ref=low_speed_ref,
    model_kind=best_parameters.model_kind,
    speed_split_mps=best_parameters.speed_split_mps,
    band_coefficients=band_coefficients,
    band_sample_counts=best_parameters.band_sample_counts,
  )
  return model, delay_fits


def simulate_event_with_model(
  samples: list[Any],
  start_idx: int,
  end_idx: int,
  hold_idx: int,
  model: FittedStoppingModel,
) -> dict[str, Any]:
  start = max(0, int(start_idx))
  end = min(len(samples) - 1, int(end_idx))
  hold = max(start, min(int(hold_idx), end))
  if end - start < 2:
    raise ValueError("Event window too short for simulation")

  times = [float(samples[start].t)]
  predicted = [float(samples[start].a_ego)]
  predicted_v = [float(samples[start].v_ego)]
  rollout_distance_m = 0.0
  for idx in range(start, end):
    delayed_idx = max(start, idx - model.delay_frames)
    cmd_delayed = samples[delayed_idx].accel_cmd
    if cmd_delayed is None:
      cmd_delayed = samples[idx].accel_cmd
    if cmd_delayed is None:
      cmd_delayed = -0.1
    next_a = model.predict_next(predicted[-1], float(cmd_delayed), float(samples[idx].v_ego))
    next_a = max(min(next_a, 3.0), -4.0)
    dt = float(samples[idx + 1].t) - float(samples[idx].t)
    if dt <= 1e-6:
      dt = max(model.dt_s, 1e-3)
    prev_v = predicted_v[-1]
    next_v = max(0.0, prev_v + (next_a * dt))
    rollout_distance_m += max(0.0, 0.5 * (prev_v + next_v) * dt)
    predicted.append(next_a)
    predicted_v.append(next_v)
    times.append(float(samples[idx + 1].t))

  pre_hold_indices = [
    i for i, t in enumerate(times)
    if (times[max(0, i - 1)] if i > 0 else t) >= (float(samples[hold].t) - 0.8)
  ]
  if len(pre_hold_indices) < 2:
    pre_hold_indices = list(range(len(times)))

  max_jerk: float | None = None
  for prev_i, cur_i in zip(pre_hold_indices, pre_hold_indices[1:], strict=False):
    dt = times[cur_i] - times[prev_i]
    if dt <= 1e-6:
      continue
    jerk = abs((predicted[cur_i] - predicted[prev_i]) / dt)
    max_jerk = jerk if max_jerk is None else max(max_jerk, jerk)

  pred_min_a = min(predicted[max(0, hold - start - 30):hold - start + 1])
  return {
    "times": times,
    "predicted_a_ego": predicted,
    "predicted_v_ego": predicted_v,
    "pred_rollout_distance_m": rollout_distance_m,
    "pred_end_stop_jerk_mps3": max_jerk,
    "pred_min_a_ego_mps2": pred_min_a,
  }
