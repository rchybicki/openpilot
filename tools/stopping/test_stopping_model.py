from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping import stopping_model as stopping_model_module
from openpilot.tools.stopping.stopping_model import (
  DelayFit,
  FEATURE_NAMES,
  FittedStoppingModel,
  MODEL_KIND_LINEAR,
  MODEL_KIND_LOW_SPEED_BLEND,
  MODEL_KIND_SPEED_BAND,
  fit_stopping_model,
  simulate_event_with_model,
)


@dataclass
class FakeSample:
  t: float
  a_ego: float
  v_ego: float
  accel_cmd: float | None
  should_stop: bool
  enabled: bool = True


TRUE_COEFFICIENTS = {
  "intercept": 0.015,
  "a_ego_prev": 0.82,
  "accel_cmd_delayed": 0.28,
  "v_ego": -0.11,
  "relief": 0.33,
  "low_speed": -0.06,
  "cmd_x_low_speed": 0.14,
}
HIGH_SPEED_COEFFICIENTS = {
  "intercept": 0.010,
  "a_ego_prev": 0.74,
  "accel_cmd_delayed": 0.18,
  "v_ego": -0.09,
  "relief": 0.12,
  "low_speed": -0.02,
  "cmd_x_low_speed": 0.05,
}
LOW_SPEED_COEFFICIENTS = {
  "intercept": 0.030,
  "a_ego_prev": 0.95,
  "accel_cmd_delayed": 0.40,
  "v_ego": -0.18,
  "relief": 0.56,
  "low_speed": -0.10,
  "cmd_x_low_speed": 0.26,
}
PIECEWISE_SPEED_SPLIT_MPS = 0.55
LOW_SPEED_BLEND_HEAD_COEFFICIENTS = {
  "intercept": 0.020,
  "a_ego_prev": 0.90,
  "accel_cmd_delayed": 0.34,
  "v_ego": -0.15,
  "relief": 0.46,
  "low_speed": -0.09,
  "cmd_x_low_speed": 0.18,
}


def _next_accel_from_coefficients(
  coefficients: dict[str, float],
  a_ego_prev: float,
  cmd_delayed: float,
  v_ego: float,
  relief_threshold: float,
  low_speed_ref: float,
) -> float:
  relief = max(0.0, cmd_delayed - relief_threshold)
  low_speed = max(0.0, min(1.0, (low_speed_ref - v_ego) / max(low_speed_ref, 1e-6)))
  return (
    coefficients["intercept"]
    + (coefficients["a_ego_prev"] * a_ego_prev)
    + (coefficients["accel_cmd_delayed"] * cmd_delayed)
    + (coefficients["v_ego"] * v_ego)
    + (coefficients["relief"] * relief)
    + (coefficients["low_speed"] * low_speed)
    + (coefficients["cmd_x_low_speed"] * cmd_delayed * low_speed)
  )


def _next_accel(a_ego_prev: float, cmd_delayed: float, v_ego: float, relief_threshold: float, low_speed_ref: float) -> float:
  return _next_accel_from_coefficients(TRUE_COEFFICIENTS, a_ego_prev, cmd_delayed, v_ego, relief_threshold, low_speed_ref)


def _next_accel_piecewise(a_ego_prev: float, cmd_delayed: float, v_ego: float, relief_threshold: float, low_speed_ref: float) -> float:
  coefficients = LOW_SPEED_COEFFICIENTS if v_ego <= PIECEWISE_SPEED_SPLIT_MPS else HIGH_SPEED_COEFFICIENTS
  return _next_accel_from_coefficients(coefficients, a_ego_prev, cmd_delayed, v_ego, relief_threshold, low_speed_ref)


def _blend_coefficients(base: dict[str, float], low_speed_head: dict[str, float], blend: float) -> dict[str, float]:
  return {
    name: ((1.0 - blend) * base[name]) + (blend * low_speed_head[name])
    for name in FEATURE_NAMES
  }


def _next_accel_low_speed_blend(a_ego_prev: float, cmd_delayed: float, v_ego: float, relief_threshold: float, low_speed_ref: float) -> float:
  blend = max(0.0, min(1.0, (low_speed_ref - v_ego) / max(low_speed_ref, 1e-6)))
  coefficients = _blend_coefficients(TRUE_COEFFICIENTS, LOW_SPEED_BLEND_HEAD_COEFFICIENTS, blend)
  return _next_accel_from_coefficients(coefficients, a_ego_prev, cmd_delayed, v_ego, relief_threshold, low_speed_ref)


def build_synthetic_samples(delay_frames: int, sample_count: int = 160, dt_s: float = 0.05) -> list[FakeSample]:
  relief_threshold = -0.25
  low_speed_ref = 1.2

  cmds = [-0.72 + (0.48 * math.sin(idx / 11.0)) for idx in range(sample_count)]
  speeds = [max(0.05, 1.6 - (0.009 * idx) + (0.05 * math.cos(idx / 13.0))) for idx in range(sample_count)]
  accel_values = [-0.28]
  samples: list[FakeSample] = []

  for idx in range(sample_count):
    sample = FakeSample(
      t=idx * dt_s,
      a_ego=accel_values[idx],
      v_ego=speeds[idx],
      accel_cmd=cmds[idx],
      should_stop=True,
    )
    samples.append(sample)
    delayed_idx = max(0, idx - delay_frames)
    next_accel = _next_accel(accel_values[idx], cmds[delayed_idx], speeds[idx], relief_threshold, low_speed_ref)
    accel_values.append(max(min(next_accel, 2.0), -3.0))

  samples.append(FakeSample(
    t=sample_count * dt_s,
    a_ego=accel_values[-1],
    v_ego=max(0.01, speeds[-1] - 0.02),
    accel_cmd=cmds[-1],
    should_stop=True,
  ))
  return samples


def build_piecewise_samples(delay_frames: int, sample_count: int = 220, dt_s: float = 0.05) -> list[FakeSample]:
  relief_threshold = -0.25
  low_speed_ref = 1.2

  cmds = [-0.68 + (0.34 * math.sin(idx / 8.0)) for idx in range(sample_count)]
  speeds = [max(0.05, 1.45 - (0.0062 * idx) + (0.03 * math.cos(idx / 10.0))) for idx in range(sample_count)]
  accel_values = [-0.20]
  samples: list[FakeSample] = []

  for idx in range(sample_count):
    samples.append(FakeSample(
      t=idx * dt_s,
      a_ego=accel_values[idx],
      v_ego=speeds[idx],
      accel_cmd=cmds[idx],
      should_stop=True,
    ))
    delayed_idx = max(0, idx - delay_frames)
    next_accel = _next_accel_piecewise(accel_values[idx], cmds[delayed_idx], speeds[idx], relief_threshold, low_speed_ref)
    accel_values.append(max(min(next_accel, 2.0), -3.0))

  samples.append(FakeSample(
    t=sample_count * dt_s,
    a_ego=accel_values[-1],
    v_ego=max(0.01, speeds[-1] - 0.02),
    accel_cmd=cmds[-1],
    should_stop=True,
  ))
  return samples


def build_low_speed_blend_samples(delay_frames: int, sample_count: int = 220, dt_s: float = 0.05) -> list[FakeSample]:
  relief_threshold = -0.25
  low_speed_ref = 1.2

  cmds = [-0.58 + (0.22 * math.sin(idx / 8.0)) for idx in range(sample_count)]
  speeds = [max(0.05, 1.30 - (0.0053 * idx) + (0.03 * math.cos(idx / 12.0))) for idx in range(sample_count)]
  accel_values = [-0.18]
  samples: list[FakeSample] = []

  for idx in range(sample_count):
    samples.append(FakeSample(
      t=idx * dt_s,
      a_ego=accel_values[idx],
      v_ego=speeds[idx],
      accel_cmd=cmds[idx],
      should_stop=True,
    ))
    delayed_idx = max(0, idx - delay_frames)
    next_accel = _next_accel_low_speed_blend(accel_values[idx], cmds[delayed_idx], speeds[idx], relief_threshold, low_speed_ref)
    accel_values.append(max(min(next_accel, 1.5), -2.0))

  samples.append(FakeSample(
    t=sample_count * dt_s,
    a_ego=accel_values[-1],
    v_ego=max(0.01, speeds[-1] - 0.02),
    accel_cmd=cmds[-1],
    should_stop=True,
  ))
  return samples


def test_fit_stopping_model_ignores_disabled_samples_in_delay_search() -> None:
  # Disabled samples can publish accel_cmd but it is not applied to the car; they must not be used for command-response fitting.
  samples = build_synthetic_samples(delay_frames=3, sample_count=180, dt_s=0.05)

  tail_count = 1200
  dt_s = 0.05
  base_t = samples[-1].t + dt_s
  v = max(0.10, float(samples[-1].v_ego))
  a = float(samples[-1].a_ego)
  for idx in range(tail_count):
    cmd = -0.65 + (0.55 * math.sin(idx / 9.0))
    # Simulate command not applied: aEgo evolves independently of accel_cmd.
    a = (0.995 * a) + (0.002 * math.cos(idx / 7.0))
    v = max(0.06, v - 0.0007)
    samples.append(FakeSample(
      t=base_t + (idx * dt_s),
      a_ego=a,
      v_ego=v,
      accel_cmd=cmd,
      should_stop=True,
      enabled=False,
    ))

  model_filtered, _ = fit_stopping_model(
    samples=samples,
    windows=[(0, len(samples) - 1)],
    max_delay_frames=6,
    min_speed=0.0,
    max_speed=2.0,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
    min_rows=120,
  )
  assert model_filtered.delay_frames == 3

  model_including_disabled, _ = fit_stopping_model(
    samples=samples,
    windows=[(0, len(samples) - 1)],
    max_delay_frames=6,
    min_speed=0.0,
    max_speed=2.0,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
    min_rows=120,
    require_enabled=False,
  )
  assert model_including_disabled.delay_frames != 3


def test_fit_stopping_model_recovers_delay_from_synthetic_data() -> None:
  samples = build_synthetic_samples(delay_frames=3)
  model, delay_fits = fit_stopping_model(
    samples=samples,
    windows=[(0, len(samples) - 1)],
    max_delay_frames=6,
    min_speed=0.0,
    max_speed=2.0,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
    min_rows=80,
  )

  assert model.delay_frames == 3
  assert model.sample_count >= 120
  assert model.rmse < 1e-9
  assert model.mae < 1e-9
  assert model.r2 > 0.999999
  assert len(delay_fits) == 7
  assert min(delay_fits, key=lambda item: item.rmse).delay_frames == 3

  for feature in FEATURE_NAMES:
    assert feature in model.coefficients
  assert model.coefficients["a_ego_prev"] == pytest.approx(TRUE_COEFFICIENTS["a_ego_prev"], abs=1e-3)
  assert model.coefficients["accel_cmd_delayed"] == pytest.approx(TRUE_COEFFICIENTS["accel_cmd_delayed"], abs=1e-3)


def test_fit_stopping_model_requires_minimum_rows() -> None:
  samples = build_synthetic_samples(delay_frames=2, sample_count=40)
  with pytest.raises(RuntimeError, match="min_rows"):
    fit_stopping_model(
      samples=samples,
      windows=[(0, len(samples) - 1)],
      max_delay_frames=4,
      min_rows=100,
    )


def test_fit_stopping_model_delay_selection_avoids_sparse_high_delay(monkeypatch: pytest.MonkeyPatch) -> None:
  scripted_fits = {
    0: DelayFit(delay_frames=0, sample_count=100, rmse=0.120, mae=0.090, r2=0.50),
    1: DelayFit(delay_frames=1, sample_count=90, rmse=0.115, mae=0.085, r2=0.55),
    2: DelayFit(delay_frames=2, sample_count=35, rmse=0.090, mae=0.070, r2=0.70),
    3: DelayFit(delay_frames=3, sample_count=20, rmse=0.080, mae=0.060, r2=0.75),
  }

  def fake_fit_with_delay(*args: object, delay_frames: int, **kwargs: object) -> tuple[list[float] | None, DelayFit]:
    fit = scripted_fits[delay_frames]
    coefficients = [float(delay_frames)] * len(FEATURE_NAMES)
    return coefficients, fit

  monkeypatch.setattr(stopping_model_module, "fit_with_delay", fake_fit_with_delay)
  samples = build_synthetic_samples(delay_frames=2, sample_count=80)
  model, _ = fit_stopping_model(
    samples=samples,
    windows=[(0, len(samples) - 1)],
    max_delay_frames=3,
    min_rows=20,
    delay_min_sample_ratio=0.50,
    delay_rmse_tolerance=0.0,
  )
  assert model.delay_frames == 1


def test_fit_stopping_model_delay_selection_prefers_lower_delay_within_rmse_tolerance(monkeypatch: pytest.MonkeyPatch) -> None:
  scripted_fits = {
    0: DelayFit(delay_frames=0, sample_count=100, rmse=0.140, mae=0.100, r2=0.40),
    1: DelayFit(delay_frames=1, sample_count=100, rmse=0.101, mae=0.080, r2=0.65),
    2: DelayFit(delay_frames=2, sample_count=100, rmse=0.100, mae=0.079, r2=0.66),
  }

  def fake_fit_with_delay(*args: object, delay_frames: int, **kwargs: object) -> tuple[list[float] | None, DelayFit]:
    fit = scripted_fits[delay_frames]
    coefficients = [float(delay_frames)] * len(FEATURE_NAMES)
    return coefficients, fit

  monkeypatch.setattr(stopping_model_module, "fit_with_delay", fake_fit_with_delay)
  samples = build_synthetic_samples(delay_frames=1, sample_count=80)
  model, _ = fit_stopping_model(
    samples=samples,
    windows=[(0, len(samples) - 1)],
    max_delay_frames=2,
    min_rows=20,
    delay_min_sample_ratio=0.50,
    delay_rmse_tolerance=0.02,
  )
  assert model.delay_frames == 1


def test_fit_stopping_model_speed_band_recovers_piecewise_response() -> None:
  samples = build_piecewise_samples(delay_frames=2)
  linear_model, _ = fit_stopping_model(
    samples=samples,
    windows=[(0, len(samples) - 1)],
    max_delay_frames=4,
    min_rows=120,
    model_kind=MODEL_KIND_LINEAR,
  )
  speed_band_model, _ = fit_stopping_model(
    samples=samples,
    windows=[(0, len(samples) - 1)],
    max_delay_frames=4,
    min_rows=120,
    model_kind=MODEL_KIND_SPEED_BAND,
    speed_band_min_rows=60,
  )

  assert speed_band_model.delay_frames == 2
  assert speed_band_model.model_kind == MODEL_KIND_SPEED_BAND
  assert speed_band_model.band_coefficients is not None
  assert speed_band_model.band_sample_counts is not None
  assert speed_band_model.speed_split_mps is not None
  assert speed_band_model.band_sample_counts["low"] >= 60
  assert speed_band_model.band_sample_counts["high"] >= 60
  assert abs(speed_band_model.speed_split_mps - PIECEWISE_SPEED_SPLIT_MPS) < 0.20
  assert speed_band_model.rmse < (linear_model.rmse * 0.40)
  assert speed_band_model.r2 > linear_model.r2

  high_prediction = speed_band_model.predict_next(-0.32, -0.52, 0.82)
  low_prediction = speed_band_model.predict_next(-0.32, -0.52, 0.18)
  assert high_prediction == pytest.approx(
    _next_accel_from_coefficients(HIGH_SPEED_COEFFICIENTS, -0.32, -0.52, 0.82, -0.25, 1.2),
    abs=1e-6,
  )
  assert low_prediction < (high_prediction - 0.60)
  assert low_prediction < -0.90


def test_fit_stopping_model_low_speed_blend_recovers_smooth_response() -> None:
  samples = build_low_speed_blend_samples(delay_frames=2)
  linear_model, _ = fit_stopping_model(
    samples=samples,
    windows=[(0, len(samples) - 1)],
    max_delay_frames=4,
    min_rows=120,
    model_kind=MODEL_KIND_LINEAR,
  )
  blend_model, _ = fit_stopping_model(
    samples=samples,
    windows=[(0, len(samples) - 1)],
    max_delay_frames=4,
    min_rows=120,
    model_kind=MODEL_KIND_LOW_SPEED_BLEND,
    low_speed_head_min_rows=80,
  )

  assert blend_model.model_kind == MODEL_KIND_LOW_SPEED_BLEND
  assert blend_model.low_speed_coefficients is not None
  assert blend_model.low_speed_head_sample_count is not None
  assert blend_model.low_speed_head_sample_count >= 80
  assert blend_model.rmse < linear_model.rmse
  assert blend_model.r2 >= linear_model.r2

  blended_low_prediction = blend_model.predict_next(-0.26, -0.38, 0.18)
  blended_mid_prediction = blend_model.predict_next(-0.26, -0.38, 0.36)
  assert abs(blended_low_prediction - blended_mid_prediction) < 0.20


def test_simulate_event_with_model_matches_synthetic_profile() -> None:
  samples = build_synthetic_samples(delay_frames=2)
  model, _ = fit_stopping_model(
    samples=samples,
    windows=[(0, len(samples) - 1)],
    max_delay_frames=4,
    min_rows=80,
  )

  start_idx = 20
  end_idx = 70
  hold_idx = 65
  simulation = simulate_event_with_model(
    samples=samples,
    start_idx=start_idx,
    end_idx=end_idx,
    hold_idx=hold_idx,
    model=model,
  )

  assert len(simulation["times"]) == (end_idx - start_idx + 1)
  assert len(simulation["predicted_a_ego"]) == len(simulation["times"])
  assert simulation["pred_end_stop_jerk_mps3"] is not None
  assert simulation["pred_min_a_ego_mps2"] <= min(simulation["predicted_a_ego"][:hold_idx - start_idx + 1]) + 1e-12
  assert simulation["predicted_a_ego"][-1] == pytest.approx(samples[end_idx].a_ego, abs=1e-4)


def test_simulate_event_with_model_supports_speed_band_model() -> None:
  samples = build_piecewise_samples(delay_frames=2)
  model, _ = fit_stopping_model(
    samples=samples,
    windows=[(0, len(samples) - 1)],
    max_delay_frames=4,
    min_rows=120,
    model_kind=MODEL_KIND_SPEED_BAND,
    speed_band_min_rows=60,
  )

  start_idx = 30
  end_idx = 90
  hold_idx = 82
  simulation = simulate_event_with_model(
    samples=samples,
    start_idx=start_idx,
    end_idx=end_idx,
    hold_idx=hold_idx,
    model=model,
  )

  assert set(simulation) == {
    "times",
    "predicted_a_ego",
    "predicted_v_ego",
    "pred_rollout_distance_m",
    "pred_end_stop_jerk_mps3",
    "pred_min_a_ego_mps2",
  }
  assert len(simulation["times"]) == (end_idx - start_idx + 1)
  assert len(simulation["predicted_a_ego"]) == len(simulation["times"])
  assert len(simulation["predicted_v_ego"]) == len(simulation["times"])
  assert math.isfinite(simulation["pred_rollout_distance_m"])
  assert simulation["pred_end_stop_jerk_mps3"] is not None
  assert simulation["predicted_v_ego"][-1] >= 0.0


def test_model_from_json_fills_missing_coefficients() -> None:
  model = FittedStoppingModel.from_json({
    "delay_frames": 4,
    "coefficients": {
      "a_ego_prev": 0.75,
      "accel_cmd_delayed": 0.22,
    },
    "rmse": 0.12,
    "mae": 0.09,
    "r2": 0.85,
    "sample_count": 200,
    "dt_s": 0.05,
    "relief_cmd_threshold": -0.20,
    "low_speed_ref": 1.0,
  })

  assert model.delay_frames == 4
  assert model.model_kind == MODEL_KIND_LINEAR
  assert model.coefficients["a_ego_prev"] == 0.75
  assert model.coefficients["accel_cmd_delayed"] == 0.22
  assert model.coefficients["intercept"] == 0.0
  assert set(model.coefficients.keys()) == set(FEATURE_NAMES)


def test_model_from_json_preserves_speed_band_coefficients() -> None:
  model = FittedStoppingModel.from_json({
    "delay_frames": 2,
    "coefficients": {
      "intercept": 0.0,
      "a_ego_prev": 0.82,
      "accel_cmd_delayed": 0.25,
    },
    "rmse": 0.05,
    "mae": 0.03,
    "r2": 0.94,
    "sample_count": 300,
    "dt_s": 0.05,
    "relief_cmd_threshold": -0.25,
    "low_speed_ref": 1.2,
    "model_kind": MODEL_KIND_SPEED_BAND,
    "speed_split_mps": 0.5,
    "band_coefficients": {
      "low": {
        "a_ego_prev": 0.95,
        "accel_cmd_delayed": 0.40,
      },
      "high": {
        "a_ego_prev": 0.70,
        "accel_cmd_delayed": 0.18,
      },
    },
    "band_sample_counts": {
      "low": 140,
      "high": 160,
    },
  })
  round_trip = FittedStoppingModel.from_json(model.as_json())

  assert round_trip.model_kind == MODEL_KIND_SPEED_BAND
  assert round_trip.speed_split_mps == 0.5
  assert round_trip.band_coefficients is not None
  assert round_trip.band_sample_counts == {"low": 140, "high": 160}
  assert round_trip.band_coefficients["low"]["intercept"] == 0.0
  assert round_trip.band_coefficients["high"]["a_ego_prev"] == 0.70
  assert round_trip.as_json()["speed_split_mps"] == 0.5
  assert round_trip.as_json()["band_sample_counts"] == {"low": 140, "high": 160}
  assert round_trip.predict_next(-0.30, -0.50, 0.20) != round_trip.predict_next(-0.30, -0.50, 0.80)


def test_model_from_json_preserves_low_speed_blend_coefficients() -> None:
  model = FittedStoppingModel.from_json({
    "delay_frames": 2,
    "coefficients": {
      "intercept": 0.0,
      "a_ego_prev": 0.82,
      "accel_cmd_delayed": 0.25,
    },
    "rmse": 0.05,
    "mae": 0.03,
    "r2": 0.94,
    "sample_count": 300,
    "dt_s": 0.05,
    "relief_cmd_threshold": -0.25,
    "low_speed_ref": 1.2,
    "model_kind": MODEL_KIND_LOW_SPEED_BLEND,
    "low_speed_coefficients": {
      "a_ego_prev": 0.94,
      "accel_cmd_delayed": 0.38,
    },
    "low_speed_head_sample_count": 123,
  })
  round_trip = FittedStoppingModel.from_json(model.as_json())

  assert round_trip.model_kind == MODEL_KIND_LOW_SPEED_BLEND
  assert round_trip.low_speed_coefficients is not None
  assert round_trip.low_speed_head_sample_count == 123
  assert round_trip.low_speed_coefficients["intercept"] == 0.0
  assert round_trip.predict_next(-0.30, -0.50, 0.20) != round_trip.predict_next(-0.30, -0.50, 0.90)
