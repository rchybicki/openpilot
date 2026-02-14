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
from openpilot.tools.stopping.stopping_model import DelayFit, FEATURE_NAMES, FittedStoppingModel, fit_stopping_model, simulate_event_with_model


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


def _next_accel(a_ego_prev: float, cmd_delayed: float, v_ego: float, relief_threshold: float, low_speed_ref: float) -> float:
  relief = max(0.0, cmd_delayed - relief_threshold)
  low_speed = max(0.0, min(1.0, (low_speed_ref - v_ego) / max(low_speed_ref, 1e-6)))
  return (
    TRUE_COEFFICIENTS["intercept"]
    + (TRUE_COEFFICIENTS["a_ego_prev"] * a_ego_prev)
    + (TRUE_COEFFICIENTS["accel_cmd_delayed"] * cmd_delayed)
    + (TRUE_COEFFICIENTS["v_ego"] * v_ego)
    + (TRUE_COEFFICIENTS["relief"] * relief)
    + (TRUE_COEFFICIENTS["low_speed"] * low_speed)
    + (TRUE_COEFFICIENTS["cmd_x_low_speed"] * cmd_delayed * low_speed)
  )


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
  assert model.coefficients["a_ego_prev"] == 0.75
  assert model.coefficients["accel_cmd_delayed"] == 0.22
  assert model.coefficients["intercept"] == 0.0
  assert set(model.coefficients.keys()) == set(FEATURE_NAMES)
