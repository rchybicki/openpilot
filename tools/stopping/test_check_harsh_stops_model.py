from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.controls.lib.stopping_controller import DEFAULT_STOPPING_CONTROLLER_STRATEGY
from openpilot.tools.stopping.stopping_model import FittedStoppingModel
from openpilot.tools.stopping.check_harsh_stops_model import (
  jerk_window_metrics,
  rank_controller_strategies,
  simulate_event_with_controller,
  score_event_metrics,
)


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


def build_regression_seed_samples_cf_signal_event1() -> list[FakeSample]:
  # Seeded from route 000006cf--551c9ecf95 signal-event 1 (engaged, harsh feel regression report).
  dt_s = 0.09997653450000143
  initial = [
    FakeSample(t=10.417827360, v_ego=0.789999127, a_ego=0.704612732, accel_cmd=-0.1271934658),
    FakeSample(t=10.518689071, v_ego=0.854692936, a_ego=0.666201830, accel_cmd=-0.1621117890),
    FakeSample(t=10.618366272, v_ego=0.915958345, a_ego=0.629397452, accel_cmd=-0.1699545383),
    FakeSample(t=10.718508673, v_ego=0.973092616, a_ego=0.601208866, accel_cmd=-0.2292619050),
    FakeSample(t=10.819824595, v_ego=1.030643106, a_ego=0.582585037, accel_cmd=-0.2292619050),
    FakeSample(t=10.919740698, v_ego=1.085875154, a_ego=0.565537930, accel_cmd=-0.2292619050),
    FakeSample(t=11.018375209, v_ego=1.137278318, a_ego=0.534513474, accel_cmd=-0.2292619050),
    FakeSample(t=11.120053833, v_ego=1.184468031, a_ego=0.486601591, accel_cmd=-0.2292619050),
    FakeSample(t=11.218064344, v_ego=1.231197119, a_ego=0.476730049, accel_cmd=-0.2991747558),
    FakeSample(t=11.319064230, v_ego=1.298817992, a_ego=0.637863874, accel_cmd=-0.3214490414),
    FakeSample(t=11.418977104, v_ego=1.354197025, a_ego=0.508641303, accel_cmd=-0.3280791044),
    FakeSample(t=11.519170858, v_ego=1.372763753, a_ego=0.341142386, accel_cmd=-0.3329808116),
    FakeSample(t=11.618986234, v_ego=1.426249504, a_ego=0.475553840, accel_cmd=-0.3375347853),
    FakeSample(t=11.719352225, v_ego=1.456263304, a_ego=0.330976903, accel_cmd=-0.3426951170),
    FakeSample(t=11.819056977, v_ego=1.453584552, a_ego=0.097895093, accel_cmd=-0.3464952707),
    FakeSample(t=11.918206323, v_ego=1.444904327, a_ego=-0.021272024, accel_cmd=-0.3489612937),
    FakeSample(t=12.018509346, v_ego=1.438217402, a_ego=-0.057672903, accel_cmd=-0.3508340418),
    FakeSample(t=12.118428053, v_ego=1.439142704, a_ego=-0.014256682, accel_cmd=-0.3525655270),
    FakeSample(t=12.219153517, v_ego=1.432251215, a_ego=-0.063209459, accel_cmd=-0.3544983268),
    FakeSample(t=12.319451852, v_ego=1.400010109, a_ego=-0.239324406, accel_cmd=-0.3558231592),
    FakeSample(t=12.419160980, v_ego=1.343721747, a_ego=-0.451968402, accel_cmd=-0.3560229838),
    FakeSample(t=12.518706777, v_ego=1.292041898, a_ego=-0.491229951, accel_cmd=-0.3560229838),
  ]
  tail = [
    FakeSample(
      t=initial[-1].t + ((idx + 1) * dt_s),
      v_ego=initial[-1].v_ego,
      a_ego=initial[-1].a_ego,
      accel_cmd=initial[-1].accel_cmd,
    )
    for idx in range(35)
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
    controller_strategy="v3",
  )

  assert len(result["times"]) == 61
  assert len(result["predicted_a_ego"]) == 61
  assert len(result["predicted_v_ego"]) == 61
  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_min_a_ego_mps2"] <= min(result["predicted_a_ego"][-30:])
  assert result["pred_rollout_distance_m"] > 0.0


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
    controller_strategy="v3",
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
    controller_strategy="v3",
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
    controller_strategy="v3",
  )

  assert result["pred_min_a_ego_mps2"] >= -1.10


def test_default_strategy_matches_baseline_comfort_on_cf_signal_regression_seed() -> None:
  samples = build_regression_seed_samples_cf_signal_event1()
  model = regression_model_20260208()

  baseline = simulate_event_with_controller(
    samples=samples,
    start_idx=0,
    hold_idx=len(samples) - 1,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_strategy="baseline",
  )
  default = simulate_event_with_controller(
    samples=samples,
    start_idx=0,
    hold_idx=len(samples) - 1,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_strategy=DEFAULT_STOPPING_CONTROLLER_STRATEGY,
  )
  v2 = simulate_event_with_controller(
    samples=samples,
    start_idx=0,
    hold_idx=len(samples) - 1,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_strategy="v2",
  )

  assert DEFAULT_STOPPING_CONTROLLER_STRATEGY == "baseline"
  assert default["pred_end_stop_jerk_mps3"] <= baseline["pred_end_stop_jerk_mps3"] + 1e-6
  assert default["pred_rollout_distance_m"] <= baseline["pred_rollout_distance_m"] + 1e-6
  # Guard against re-introducing the recent comfort regression observed with v2 default.
  assert baseline["pred_end_stop_jerk_mps3"] < v2["pred_end_stop_jerk_mps3"] - 0.02


def test_score_event_metrics_penalizes_rollout_and_harsh_decel() -> None:
  smooth_short = score_event_metrics(pred_jerk=0.42, pred_min_a=-0.95, pred_rollout_m=1.2, max_rollout_m=2.0)
  smooth_long = score_event_metrics(pred_jerk=0.42, pred_min_a=-0.95, pred_rollout_m=2.8, max_rollout_m=2.0)
  harsh_short = score_event_metrics(pred_jerk=0.92, pred_min_a=-1.30, pred_rollout_m=1.2, max_rollout_m=2.0)

  assert smooth_short < smooth_long
  assert smooth_short < harsh_short


def test_rank_controller_strategies_prefers_rollout_feasible_smooth_strategy() -> None:
  rows = [
    {
      "controller_strategy": "baseline",
      "is_harsh": True,
      "pred_end_stop_jerk_mps3": 0.82,
      "pred_min_a_ego_mps2": -1.22,
      "pred_rollout_distance_m": 1.7,
      "event_score": 1.15,
    },
    {
      "controller_strategy": "baseline",
      "is_harsh": False,
      "pred_end_stop_jerk_mps3": 0.70,
      "pred_min_a_ego_mps2": -1.08,
      "pred_rollout_distance_m": 1.5,
      "event_score": 0.76,
    },
    {
      "controller_strategy": "v2",
      "is_harsh": False,
      "pred_end_stop_jerk_mps3": 0.46,
      "pred_min_a_ego_mps2": -0.96,
      "pred_rollout_distance_m": 2.6,
      "event_score": 1.95,
    },
    {
      "controller_strategy": "v2",
      "is_harsh": False,
      "pred_end_stop_jerk_mps3": 0.44,
      "pred_min_a_ego_mps2": -0.94,
      "pred_rollout_distance_m": 2.4,
      "event_score": 1.55,
    },
    {
      "controller_strategy": "v3",
      "is_harsh": False,
      "pred_end_stop_jerk_mps3": 0.52,
      "pred_min_a_ego_mps2": -1.01,
      "pred_rollout_distance_m": 1.8,
      "event_score": 0.55,
    },
    {
      "controller_strategy": "v3",
      "is_harsh": False,
      "pred_end_stop_jerk_mps3": 0.50,
      "pred_min_a_ego_mps2": -0.99,
      "pred_rollout_distance_m": 1.7,
      "event_score": 0.52,
    },
  ]

  ranking = rank_controller_strategies(rows, max_rollout_m=2.0)
  assert ranking
  assert ranking[0]["strategy"] == "v3"
  assert ranking[0]["feasible_rollout"]
  assert ranking[-1]["strategy"] == "v2"
