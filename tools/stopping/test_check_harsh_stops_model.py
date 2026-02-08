from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping.stopping_model import FittedStoppingModel
from openpilot.tools.stopping.check_harsh_stops_model import jerk_window_metrics, simulate_event_with_controller


@dataclass
class FakeSample:
  t: float
  v_ego: float
  a_ego: float
  accel_cmd: float | None


def simple_model() -> FittedStoppingModel:
  return FittedStoppingModel(
    delay_frames=3,
    coefficients={
      "intercept": 0.0,
      "a_ego_prev": 0.88,
      "accel_cmd_delayed": 0.12,
      "v_ego": -0.06,
      "relief": 0.15,
      "low_speed": -0.03,
      "cmd_x_low_speed": 0.08,
    },
    rmse=0.1,
    mae=0.08,
    r2=0.9,
    sample_count=300,
    dt_s=0.05,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
  )


def build_samples(count: int = 120, dt_s: float = 0.05) -> list[FakeSample]:
  samples: list[FakeSample] = []
  v_ego = 1.0
  a_ego = -0.2
  for idx in range(count):
    t = idx * dt_s
    accel_cmd = -0.24 + (0.08 * max(0.0, (count - idx) / count))
    samples.append(FakeSample(
      t=t,
      v_ego=max(0.0, v_ego),
      a_ego=a_ego,
      accel_cmd=accel_cmd,
    ))
    a_ego = max(-1.2, (a_ego * 0.92) - 0.004)
    v_ego = max(0.0, v_ego + (a_ego * dt_s))
  return samples


def test_jerk_window_metrics_handles_short_window() -> None:
  times = [0.0, 0.1, 0.2]
  predicted = [-0.1, -0.3, -0.5]
  max_jerk, pred_min_a = jerk_window_metrics(times, predicted, hold_time_s=times[-1])
  assert max_jerk is not None
  assert pred_min_a == -0.5


def test_simulate_event_with_controller_returns_predictions() -> None:
  samples = build_samples()
  model = simple_model()
  result = simulate_event_with_controller(
    samples=samples,
    start_idx=10,
    hold_idx=70,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
  )

  assert len(result["times"]) == 61
  assert len(result["predicted_a_ego"]) == 61
  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_min_a_ego_mps2"] <= min(result["predicted_a_ego"][-30:])
