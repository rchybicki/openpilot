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


def regression_model_20260208() -> FittedStoppingModel:
  return FittedStoppingModel(
    delay_frames=5,
    coefficients={
      "intercept": -0.04320604031092344,
      "a_ego_prev": 1.010074012410447,
      "accel_cmd_delayed": -0.31926101673078167,
      "v_ego": -0.12176045422426689,
      "relief": 0.534523007098092,
      "low_speed": 0.05826100487596623,
      "cmd_x_low_speed": 0.2548958826663355,
    },
    rmse=0.11811660842898317,
    mae=0.07128108502605691,
    r2=0.9019443311679881,
    sample_count=252,
    dt_s=0.09997653450000143,
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


def build_regression_seed_samples() -> list[FakeSample]:
  dt_s = 0.09997653450000143
  start_t = 11.94309850799982
  initial = [
    FakeSample(t=start_t + (0 * dt_s), v_ego=0.335951566696167, a_ego=1.2601673603057861, accel_cmd=-0.20452241599559784),
    FakeSample(t=start_t + (1 * dt_s), v_ego=0.4783416986465454, a_ego=1.3591804504394531, accel_cmd=-0.3499393165111542),
    FakeSample(t=start_t + (2 * dt_s), v_ego=0.5635169744491577, a_ego=1.0225462913513184, accel_cmd=-0.48228707909584045),
    FakeSample(t=start_t + (3 * dt_s), v_ego=0.6258711218833923, a_ego=0.7631985545158386, accel_cmd=-0.6094451546669006),
    FakeSample(t=start_t + (4 * dt_s), v_ego=0.6672426462173462, a_ego=0.5265697240829468, accel_cmd=-0.7339661717414856),
    FakeSample(t=start_t + (5 * dt_s), v_ego=0.6868202090263367, a_ego=0.3043563663959503, accel_cmd=-0.8564475178718567),
  ]
  tail = [
    FakeSample(
      t=start_t + ((6 + idx) * dt_s),
      v_ego=0.6868202090263367,
      a_ego=0.3043563663959503,
      accel_cmd=-0.8564475178718567,
    )
    for idx in range(14)
  ]
  return initial + tail


def build_regression_seed_samples_ce_event6() -> list[FakeSample]:
  dt_s = 0.09997653450000143
  start_t = 38.159
  initial = [
    FakeSample(t=start_t + (0 * dt_s), v_ego=1.1098029613494873, a_ego=0.40280288457870483, accel_cmd=-0.3443150520324707),
    FakeSample(t=start_t + (1 * dt_s), v_ego=1.073291301727295, a_ego=-0.09874717891216278, accel_cmd=-0.480320543050766),
    FakeSample(t=start_t + (2 * dt_s), v_ego=1.0061943531036377, a_ego=-0.47208571434020996, accel_cmd=-0.6147042512893677),
    FakeSample(t=start_t + (3 * dt_s), v_ego=0.9250645637512207, a_ego=-0.6961544752120972, accel_cmd=-0.7462438344955444),
    FakeSample(t=start_t + (4 * dt_s), v_ego=0.8390381932258606, a_ego=-0.8071169853210449, accel_cmd=-0.8743184804916382),
    FakeSample(t=start_t + (5 * dt_s), v_ego=0.7475503087043762, a_ego=-0.8757506608963013, accel_cmd=-0.8995009064674377),
  ]
  tail = [
    FakeSample(
      t=start_t + ((6 + idx) * dt_s),
      v_ego=0.7475503087043762,
      a_ego=-0.8757506608963013,
      accel_cmd=-0.8995009064674377,
    )
    for idx in range(13)
  ]
  return initial + tail


def build_regression_seed_samples_cf_event1() -> list[FakeSample]:
  dt_s = 0.09997653450000143
  start_t = 11.218064344
  initial = [
    FakeSample(t=start_t + (0 * dt_s), v_ego=1.2311971187591553, a_ego=0.4767300486564636, accel_cmd=-0.2991747558116913),
    FakeSample(t=start_t + (1 * dt_s), v_ego=1.2988179922103882, a_ego=0.6378638744354248, accel_cmd=-0.32144904136657715),
    FakeSample(t=start_t + (2 * dt_s), v_ego=1.3541970252990723, a_ego=0.5086413025856018, accel_cmd=-0.32807910442352295),
    FakeSample(t=start_t + (3 * dt_s), v_ego=1.372763752937317, a_ego=0.3411423861980438, accel_cmd=-0.33298081159591675),
    FakeSample(t=start_t + (4 * dt_s), v_ego=1.4262495040893555, a_ego=0.47555384039878845, accel_cmd=-0.3375347852706909),
    FakeSample(t=start_t + (5 * dt_s), v_ego=1.4562633037567139, a_ego=0.3309769034385681, accel_cmd=-0.34269511699676514),
  ]
  tail = [
    FakeSample(
      t=start_t + ((6 + idx) * dt_s),
      v_ego=1.4562633037567139,
      a_ego=0.3309769034385681,
      accel_cmd=-0.34269511699676514,
    )
    for idx in range(40)
  ]
  return initial + tail


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


def test_simulate_event_with_controller_regression_seed_limits_predicted_jerk() -> None:
  # Seeded from route 000006c7--86cecffe81 event 1 to guard against harsh end-stop replay.
  samples = build_regression_seed_samples()
  model = regression_model_20260208()
  result = simulate_event_with_controller(
    samples=samples,
    start_idx=5,
    hold_idx=19,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
  )

  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_end_stop_jerk_mps3"] <= 0.70


def test_simulate_event_with_controller_regression_seed_ce_event6_limits_predicted_jerk() -> None:
  # Seeded from route 000006ce--d41951b402 signal-event 6 (engaged, harsh final stop).
  samples = build_regression_seed_samples_ce_event6()
  model = regression_model_20260208()
  result = simulate_event_with_controller(
    samples=samples,
    start_idx=5,
    hold_idx=18,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
  )

  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_end_stop_jerk_mps3"] <= 0.70


def test_simulate_event_with_controller_regression_seed_cf_event1_limits_predicted_floor() -> None:
  # Seeded from route 000006cf--551c9ecf95 speed-event 1 (engaged; harsh decel floor in replay).
  samples = build_regression_seed_samples_cf_event1()
  model = regression_model_20260208()
  result = simulate_event_with_controller(
    samples=samples,
    start_idx=5,
    hold_idx=45,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
  )

  assert result["pred_min_a_ego_mps2"] >= -1.10
