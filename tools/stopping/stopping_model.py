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


@dataclass
class DelayFit:
  delay_frames: int
  sample_count: int
  rmse: float
  mae: float
  r2: float


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

  def as_json(self) -> dict[str, Any]:
    payload = asdict(self)
    payload["feature_names"] = list(FEATURE_NAMES)
    return payload

  @classmethod
  def from_json(cls, data: dict[str, Any]) -> FittedStoppingModel:
    coefficients_raw = data.get("coefficients", {})
    coefficients = {name: float(coefficients_raw.get(name, 0.0)) for name in FEATURE_NAMES}
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
    )

  def predict_next(self, a_ego_prev: float, accel_cmd_delayed: float, v_ego: float) -> float:
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
    coef = np.array([self.coefficients[name] for name in FEATURE_NAMES], dtype=float)
    return float(np.dot(row, coef))


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
) -> tuple[np.ndarray | None, DelayFit]:
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

  coefficients, *_ = np.linalg.lstsq(x, y, rcond=None)
  y_pred = x @ coefficients
  rmse, mae, r2 = _score(y, y_pred)
  return coefficients, DelayFit(delay_frames=delay_frames, sample_count=len(y), rmse=rmse, mae=mae, r2=r2)


def fit_stopping_model(
  samples: list[Any],
  windows: list[tuple[int, int]],
  max_delay_frames: int = 20,
  min_speed: float = 0.0,
  max_speed: float = 2.0,
  relief_cmd_threshold: float = -0.25,
  low_speed_ref: float = 1.2,
  min_rows: int = 50,
  require_enabled: bool = True,
) -> tuple[FittedStoppingModel, list[DelayFit]]:
  if max_delay_frames < 0:
    raise ValueError("max_delay_frames must be >= 0")

  best_coefficients: np.ndarray | None = None
  best_fit: DelayFit | None = None
  delay_fits: list[DelayFit] = []

  for delay in range(max_delay_frames + 1):
    coefficients, fit = fit_with_delay(
      samples=samples,
      windows=windows,
      delay_frames=delay,
      min_speed=min_speed,
      max_speed=max_speed,
      relief_cmd_threshold=relief_cmd_threshold,
      low_speed_ref=low_speed_ref,
      require_enabled=require_enabled,
    )
    delay_fits.append(fit)
    if coefficients is None:
      continue
    if best_fit is None or fit.rmse < best_fit.rmse:
      best_fit = fit
      best_coefficients = coefficients

  if best_fit is None or best_coefficients is None:
    raise RuntimeError("Unable to fit stopping model: no valid training rows")
  if best_fit.sample_count < min_rows:
    raise RuntimeError(f"Unable to fit stopping model: only {best_fit.sample_count} rows (min_rows={min_rows})")

  coefficients = {name: float(best_coefficients[idx]) for idx, name in enumerate(FEATURE_NAMES)}
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
