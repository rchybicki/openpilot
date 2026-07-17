#!/usr/bin/env python3
"""Closed-loop stopping replay: drives the `StoppingControllerV2` through a `PlantModel` on
event-store scenarios and stop_scenarios fixtures (spec 1.2 / 7.6).

Wiring is the executable recipe from test_stopping_v2_replay.simulate (WP5): ONE harness-owned
StopTargetArbiter -> minimal long_control_state_trans port -> post-transition dropout-hold pin ->
controller seam -> PlantModel step, i.e. the INTEGRATED LongControl wiring (state machine + single
arbiter + controller exactly as on-car, spec 7.6 integrated-path requirement). The V2 facade
receives the arbiter's StopDecision via the seam's one trailing kwarg (spec section 2). Includes the
standstill clamp the proven replay used (check_harsh_stops_model.py:802-812 pattern,
braking threshold -0.08): the archived AR(1) plants drift positive at standstill (documented
spec-5.1 gain collapse) and would otherwise synthesize creep. Runs may stall in the 0.08-0.2 m/s
authority-collapse band -- a plant artifact; the similarity_gate.py comparison is unaffected.

Sim interface (spec section 2): `simulate_stop(controller, plant, scenario, dt) -> StopTrace`
where `controller` is any object with the facade update/reset/seed_command_history seam.

Output event rows carry stable keys (spec 7.1) plus legacy (route, event_id) so
check_leapfrog_alignment.py can join either way.
"""

from __future__ import annotations

import argparse
import inspect
import json
import math
import os
import sys
from dataclasses import dataclass, field, replace
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(os.environ.get("OPENPILOT_REPO_ROOT", Path(__file__).resolve().parents[2])).resolve()
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.controls.lib import stop_target_arbiter as sta
from openpilot.selfdrive.controls.lib.stop_target_arbiter import StopTargetArbiter
from openpilot.selfdrive.controls.lib.stop_context import StopContext
from openpilot.selfdrive.controls.lib.stopping_controller_v2 import StoppingControllerV2, StoppingResult
from openpilot.selfdrive.controls.lib import stopping_service as stopping_service_module
from openpilot.selfdrive.controls.lib.stopping_service import Phase as ServicePhase, StoppingService
from openpilot.selfdrive.controls.lib.stopping_params import STOPPING_PARAMS
from openpilot.selfdrive.controls.lib.stopping_plant import (
  PLANT_PARAMS_REF,
  FrictionResidual,
  PlantModel,
  PlantParams,
  friction_params_from_json,
  load_legacy_model_json,
)
from openpilot.selfdrive.controls.lib.tests.stop_scenarios import SCENARIOS, FakeSample
from openpilot.tools.stopping import scoring_config as sc

interp = np.interp
P = STOPPING_PARAMS

OFF, PID, STOPPING, STARTING = (sta.LONG_CTRL_STATE_OFF, sta.LONG_CTRL_STATE_PID,
                                sta.LONG_CTRL_STATE_STOPPING, sta.LONG_CTRL_STATE_STARTING)
DEFAULT_DT = 0.1
DEFAULT_EXTEND_S = 20.0          # keep the final intent asserted until the sim settles
STANDSTILL_CLAMP_DELAY_S = 0.6   # check_harsh_stops_model.py:723 clamp delay
STANDSTILL_CLAMP_CMD = -0.08     # WP5-adapted braking threshold (V2 hold band is -0.10..-0.16)
V_EGO_STARTING = 0.4
STOP_ACCEL = -2.0
STANDSTILL_V = 0.05
SERVICE_FULL_BAND_V = 2.5
DEFAULT_EVENT_STORE = Path.home() / ".comma" / "stopping_behavior" / "event_store"
ARCHIVED_REFIT_JSON = REPO_ROOT / "docs" / "stopping" / "archive" / "plant_model_20260531T075153Z_all.json"
CONTROLLERS = ("v2", "service")

# DEVELOPMENT-ONLY friction-augmented plant (NOT a gate). The default/gated paths never construct one;
# the on-road IMU settle metric (settle_peak_imu_jerk) remains the promoter. See fit_friction_residual.py
# and stopping_plant.FrictionPlant for the discipline. When a friction fit is supplied the sim ALSO
# rolls a predicted-IMU accel channel (wheel/linear response + velocity-dependent friction residual) so
# the terminal disc-grab the wheel plant is blind to can be SCORED offline -- a model prediction to be
# confirmed on-road, never a pass/fail decision.
DEFAULT_FRICTION_JSON = REPO_ROOT / "docs" / "stopping" / "archive" / "friction_residual_20260614.json"


class _CP:
  def __init__(self):
    self.carFingerprint = "HYUNDAI_SANTA_FE_HEV_2022"
    self.startingState = True
    self.enableGasInterceptor = False


class ServiceControllerAdapter:
  """--controller service: drives the Stopping Service V3 (stop_context + stopping_service) through the
  facade update/reset/seed_command_history seam so the plan's stage-0 sim adapter is one CLI flag away.
  Default behavior is untouched: nothing constructs this unless --controller service is passed. The
  seam carries no planner aTarget, so the service's a_plan lane is inert here (a_target=None) and the
  RELEASE go-trigger relies on gap growth/state exit -- documented adapter limitation."""

  def __init__(self):
    self.full_band = True
    self.ctx = StopContext()
    self.svc = StoppingService()
    self._seed: float | None = None
    # legacy telemetry seam attributes (harness getattr reads)
    self.phase = 0
    self.active = False
    self.low_speed_rollout_m = 0.0

  def reset(self) -> None:
    self.ctx.reset()
    self.svc.reset()
    self.phase = 0
    self.active = False

  def seed_command_history(self, commands: list[float]) -> None:
    if commands:
      self._seed = float(commands[-1])

  def holds_stopping_state(self, decision) -> bool:
    helper = getattr(stopping_service_module, "service_holds_stopping_state", None)
    if helper is not None:
      return bool(helper(self.svc.phase))
    # Pre-helper baseline behavior: only the two named legacy release predicates were blocked.
    return bool(self.svc.phase in (ServicePhase.RAMP_TO_HOLD, ServicePhase.HOLD, ServicePhase.RELEASE)
                and (decision.far_stopped_lead_release or decision.departing_lead_release))

  def update(self, output_accel, last_output_accel, should_stop, v_ego, a_ego,
             max_expected_accel, min_expected_accel, stop_accel, dt,
             distance_to_stop_target_m=None, raw_should_stop=None,
             lead_status=False, lead_v=0.0, lead_d_rel=None, debug=None, decision=None,
             planner_a_target=None) -> StoppingResult:
    del max_expected_accel, min_expected_accel, decision  # service-owned laws; seam compat only
    signals = self.ctx.update(v_ego=float(v_ego), a_ego=float(a_ego), a_cmd=float(last_output_accel),
                              lead_status=bool(lead_status), lead_v=float(lead_v),
                              lead_d_rel=None if lead_d_rel is None else float(lead_d_rel),
                              standstill=float(v_ego) < 0.01, dt=float(dt))
    dts = (float(distance_to_stop_target_m)
           if distance_to_stop_target_m is not None and distance_to_stop_target_m >= 0.0 else None)
    stop = bool(raw_should_stop) if raw_should_stop is not None else bool(should_stop)
    seed = self._seed if self._seed is not None else float(last_output_accel)
    a_target = float(planner_a_target) if planner_a_target is not None and math.isfinite(planner_a_target) else None
    result = self.svc.update(engaged=True, v_ego=float(v_ego), a_ego=float(a_ego), a_target=a_target,
                             should_stop=stop, dts_planner=dts, planner_min_limit=float(stop_accel),
                             signals=signals, lead_status=bool(lead_status), lead_v=float(lead_v),
                             dt=float(dt), wire_accel=seed)
    self._seed = None
    self.phase = int(result.phase)
    self.active = bool(result.active)
    if debug is not None:
      debug.update(result.debug)
      debug["version"] = "service_v3"
      debug["source"] = int(result.phase)
    u = result.accel if result.active else float(output_accel)
    return StoppingResult(output_accel=u, release_lock_active=False)


def make_controller(name: str = "v2"):
  if name == "v2":
    return StoppingControllerV2(_CP())
  if name == "service":
    return ServiceControllerAdapter()
  raise ValueError(f"unknown controller: {name}")


def resolve_plants(plant_arg: str) -> dict[str, PlantParams]:
  """'ref' = frozen 20260514 fit, 'refit' = archived 20260531 fit, 'both', or a JSON path."""
  if plant_arg == "ref":
    return {"ref_20260514": PLANT_PARAMS_REF}
  if plant_arg == "refit":
    return {"refit_20260531": load_legacy_model_json(str(ARCHIVED_REFIT_JSON))}
  if plant_arg == "both":
    return {"ref_20260514": PLANT_PARAMS_REF,
            "refit_20260531": load_legacy_model_json(str(ARCHIVED_REFIT_JSON))}
  return {Path(plant_arg).stem: load_legacy_model_json(plant_arg)}


def load_friction(friction_arg: str | None) -> FrictionResidual | None:
  """Resolve the OPT-IN friction residual (DEVELOPMENT-ONLY): None, 'default' (the archived
  2026-06-14 coarse-provisional fit), or a fit-archive JSON path. Returns None when no friction is
  requested -- the linear/gated paths then run completely unchanged."""
  if friction_arg is None:
    return None
  path = DEFAULT_FRICTION_JSON if friction_arg == "default" else Path(friction_arg).expanduser()
  with open(path) as f:
    return FrictionResidual(friction_params_from_json(json.load(f)))


@dataclass(frozen=True)
class Scenario:
  name: str
  # FakeSample-shaped rows: t, v_ego, a_ego, accel_cmd, should_stop, distance_to_stop_target_m,
  # raw_should_stop, lead_status, lead_v, lead_d_rel_m
  samples: list
  key: dict[str, Any] | None = None        # spec 7.1 stable key when event-store sourced
  event_id: int | None = None              # legacy positional id (analyzer summaries)
  signals_version: int = 1
  telemetry_version: int = 1
  stratum: str = ""
  planner_a_targets: list[float | None] | None = None


@dataclass
class StopTrace:
  name: str
  controller: str
  plant: str
  t: list = field(default_factory=list)
  v: list = field(default_factory=list)
  a: list = field(default_factory=list)
  u: list = field(default_factory=list)
  # OPTIONAL predicted-IMU accel channel (DEVELOPMENT-ONLY). Only populated when simulate_stop runs
  # with a friction residual: a_imu[k] = wheel/linear plant accel a[k] + friction_residual(v[k]).
  # Empty otherwise; no metric or gate reads it unless a friction fit was supplied.
  a_imu: list = field(default_factory=list)
  state: list = field(default_factory=list)
  stop_request: list = field(default_factory=list)
  # per-frame lead ground truth (closing-aware gap propagated by the sim) for the cranked metrics
  lead_status: list = field(default_factory=list)
  lead_v: list = field(default_factory=list)
  lead_gap: list = field(default_factory=list)
  debug_frames: list = field(default_factory=list)
  first_stop_idx: int | None = None
  ends_stopped: bool = False
  final_lead_gap_m: float | None = None


def _extended_rows(samples: list, dt: float, extend_s: float) -> list:
  rows = list(samples)
  if not rows or extend_s <= 0.0:
    return rows
  last = rows[-1]
  t = float(last.t)
  for _ in range(int(round(extend_s / dt))):
    t += dt
    rows.append(replace(last, t=t))
  return rows


def _recorded_ego_positions(samples: list) -> list[float]:
  """Integrate the recorded ego path so lead/stop-target positions become exogenous world paths.

  The simulated ego must never inherit the recorded ego's future distance. Replaying raw dRel on
  every frame would otherwise erase candidate-vs-baseline travel differences until the trace tail.
  """
  positions = [0.0]
  for prev, cur in zip(samples, samples[1:], strict=False):
    dt = max(float(cur.t) - float(prev.t), 0.0)
    positions.append(positions[-1] + max((float(prev.v_ego) + float(cur.v_ego)) * 0.5 * dt, 0.0))
  return positions


def simulate_stop(controller, plant: PlantModel, scenario: Scenario, dt: float = DEFAULT_DT,
                  extend_s: float = DEFAULT_EXTEND_S, collect_debug: bool = False,
                  controller_name: str = "", plant_name: str = "",
                  friction: FrictionResidual | None = None) -> StopTrace:
  """Closed-loop integrated replay (see module docstring). Deterministic by construction.

  `friction` is OPT-IN and DEVELOPMENT-ONLY (NOT a gate). When given, the controller still drives off
  the WHEEL/linear plant exactly as before (the closed loop is byte-identical), but the trace ALSO
  records a predicted-IMU channel a_imu[k] = a[k] + friction_residual(v[k]) so the terminal disc-grab
  the wheel plant is blind to can be scored offline. With friction=None the predicted-IMU channel is
  empty and nothing downstream changes."""
  samples = scenario.samples
  CP = _CP()
  arbiter = StopTargetArbiter(CP)
  controller.reset()
  wants_decision = "decision" in inspect.signature(controller.update).parameters
  wants_planner_target = "planner_a_target" in inspect.signature(controller.update).parameters
  standstill_clamp_frames = max(int(round(STANDSTILL_CLAMP_DELAY_S / dt)), 1)

  rows = _extended_rows(samples, dt, extend_s)
  n_fixture = len(samples)
  v = max(float(samples[0].v_ego), 0.0)
  a = float(samples[0].a_ego)
  last_u = float(samples[0].accel_cmd) if samples[0].accel_cmd is not None else -0.1
  a_target = last_u
  state = PID
  standstill_frames = 0
  explicit_target: float | None = None
  target_position: float | None = None
  lead_gap: float | None = None
  lead_position: float | None = None
  sim_position = 0.0
  recorded_positions = _recorded_ego_positions(samples)
  sent = [last_u] * max(plant.delay_frames, 1)  # in-flight command pipeline seed (WP5 recipe)
  trace = StopTrace(name=scenario.name, controller=controller_name, plant=plant_name)

  for k, row in enumerate(rows):
    # Recorded-state warm-up: the plant does not own the approach before stopping authority starts.
    # Seed the real state/position/command pipeline, then free-run from the first STOPPING frame.
    if k < n_fixture and trace.first_stop_idx is None:
      v = max(float(row.v_ego), 0.0)
      a = float(row.a_ego)
      sim_position = recorded_positions[k]
      if row.accel_cmd is not None:
        last_u = float(row.accel_cmd)
    if k < n_fixture:
      if row.distance_to_stop_target_m is not None and row.distance_to_stop_target_m > 0.0:
        target_position = recorded_positions[k] + float(row.distance_to_stop_target_m)
        explicit_target = max(target_position - sim_position, 0.05)
      else:
        target_position = None
        explicit_target = None
      if row.lead_status and row.lead_d_rel_m is not None:
        lead_position = recorded_positions[k] + float(row.lead_d_rel_m)
        lead_gap = max(lead_position - sim_position, 0.0)
      else:
        lead_gap = None
      if row.accel_cmd is not None:
        a_target = float(row.accel_cmd)
    else:
      explicit_target = max(target_position - sim_position, 0.05) if target_position is not None else None
      lead_gap = max(lead_position - sim_position, 0.0) if row.lead_status and lead_position is not None else None
    raw_should_stop = bool(row.raw_should_stop) if row.raw_should_stop is not None else bool(row.should_stop)
    planner_target = explicit_target if explicit_target is not None else -1.0

    decision = arbiter.update(
      v_ego=v, a_ego=a, a_target=a_target,
      raw_should_stop=raw_should_stop, planner_target_m=planner_target,
      lead_status=bool(row.lead_status), lead_v=float(row.lead_v),
      lead_d_rel=lead_gap if lead_gap is not None else 0.0,
      increased_stopped_distance_m=0.0,
      brake_pressed=False, cruise_standstill=False, standstill=v < 0.01,
      force_coast=False, long_control_state=state,
      last_output_accel=last_u, dt=dt,
      human_acceleration=True, v_ego_starting=V_EGO_STARTING)

    new_state = _long_control_state_trans(CP, state, v, decision.state_should_stop, a_target, decision.target_distance_m)
    if (state == STOPPING and new_state != STOPPING
        and hasattr(controller, "holds_stopping_state") and controller.holds_stopping_state(decision)):
      new_state = STOPPING
    if state == STOPPING and new_state != STOPPING and decision.state_dropout_hold:
      new_state = STOPPING  # post-transition dropout-hold pin (longcontrol.py:843-868 port)
    state = new_state

    controller_full_band = bool(getattr(controller, "full_band", False))
    run_controller = state == STOPPING or (controller_full_band and v < SERVICE_FULL_BAND_V)
    if state == STOPPING:
      if trace.first_stop_idx is None:
        trace.first_stop_idx = k
    if run_controller:
      debug = {} if collect_debug else None
      max_exp = float(interp(v, P.EXPECTED_ACCEL_V_BP, P.EXPECTED_ACCEL_MAX))
      min_exp = float(interp(v, P.EXPECTED_ACCEL_V_BP, P.EXPECTED_ACCEL_MIN))
      kwargs: dict[str, Any] = dict(
        distance_to_stop_target_m=(explicit_target if controller_full_band else decision.target_distance_m),
        raw_should_stop=raw_should_stop,
        lead_status=bool(row.lead_status), lead_v=float(row.lead_v),
        lead_d_rel=lead_gap, debug=debug)
      if wants_decision:
        kwargs["decision"] = decision  # the seam's single trailing kwarg, V2 branch only (spec section 2)
      if wants_planner_target:
        planner_target = scenario.planner_a_targets[k] if scenario.planner_a_targets is not None and k < n_fixture else None
        kwargs["planner_a_target"] = planner_target
      legacy_proxy = min(last_u, -0.1) if state == STOPPING else a_target
      result = controller.update(legacy_proxy, last_u, decision.stop_request_active, v, a,
                                 max_exp, min_exp, STOP_ACCEL, dt, **kwargs)
      u = result.output_accel
      if controller_full_band and controller.active and trace.first_stop_idx is None:
        trace.first_stop_idx = k
      if debug is not None:
        trace.debug_frames.append(debug)
    else:
      u = a_target  # PID passthrough proxy (pid output is a_target clipped on this platform)
    stop_intent_active = (decision.stop_request_active or decision.approach_cap_active
                          or decision.carry_floor_active or state == STOPPING)
    if controller_full_band and not run_controller:
      controller.reset()  # LongControl _run_stopping_service(run=False) resets outside the full band
    elif not stop_intent_active and not (controller_full_band and controller.active):
      controller.reset()  # longcontrol.py:902-903 reset discipline

    trace.t.append(float(row.t))
    trace.v.append(v)
    trace.a.append(a)
    if friction is not None:
      # Predicted IMU accel = wheel/linear plant accel + velocity-dependent friction residual. This is
      # exactly FrictionPlant.predict_next_imu evaluated on this frame's (a, v): the wheel trajectory
      # supplies the stop-to-stop shape, the residual adds back the wheel-blind terminal grab.
      trace.a_imu.append(a + friction.residual(v))
    trace.u.append(u)
    trace.state.append(state)
    trace.stop_request.append(bool(decision.stop_request_active))
    trace.lead_status.append(bool(row.lead_status))
    trace.lead_v.append(float(row.lead_v))
    trace.lead_gap.append(float(lead_gap) if lead_gap is not None else None)

    last_u = u
    sent.append(u)
    a_next = plant.predict_next(a, _delayed_command(sent, plant.delay_frames), v)
    a = float(np.clip(a_next, -4.0, 3.0))
    if standstill_frames >= standstill_clamp_frames and v < 0.01 and u <= STANDSTILL_CLAMP_CMD and a > 0.0:
      a = 0.0  # AR(1) standstill drift clamp (module docstring)
    standstill_frames = standstill_frames + 1 if v < STANDSTILL_V else 0
    v_prev = v
    v = max(v + a * dt, 0.0)
    travel = max((v_prev + v) / 2.0 * dt, 0.0)
    sim_position += travel
    # Past the recorded horizon the final exogenous lead continues at its last measured speed.
    # Within it, the next row reconstructs lead_position from recorded ego + dRel.
    if k >= n_fixture - 1 and lead_position is not None and row.lead_status:
      lead_position += float(row.lead_v) * dt

  trace.ends_stopped = bool(trace.state and trace.state[-1] == STOPPING and trace.stop_request[-1] and trace.v[-1] < STANDSTILL_V)
  trace.final_lead_gap_m = (max(lead_position - sim_position, 0.0)
                            if rows[-1].lead_status and lead_position is not None else None)
  return trace


def _delayed_command(sent: list[float], delay_frames: int) -> float:
  return sent[max(len(sent) - 1 - delay_frames, 0)]


def _long_control_state_trans(CP, state: int, v_ego: float, should_stop: bool,
                              a_target: float, distance_to_stop_target_m: float) -> int:
  # minimal port of longcontrol.py:509-546 (same recipe as the WP4/WP5 test oracles); the replay
  # runs always-active with no driver brake/cruise-standstill inputs
  stopping_condition = should_stop or sta.should_enter_stop_target_mode(v_ego, a_target, distance_to_stop_target_m)
  if state == STOPPING and not should_stop:
    stopping_condition = stopping_condition or sta.should_hold_stop_target_mode(v_ego, a_target, distance_to_stop_target_m)
  starting_condition = not stopping_condition
  if state == OFF:
    return STOPPING if not starting_condition else (STARTING if CP.startingState else PID)
  if state == STOPPING:
    return (STARTING if CP.startingState else PID) if starting_condition else STOPPING
  if state in (STARTING, PID):
    if stopping_condition:
      return STOPPING
    if v_ego > V_EGO_STARTING:
      return PID
    return state
  return state


# --- trace metrics (definitions mirror analyze_stopping_behavior on the sim trace) ---------------

def _window_max_jerk(t: list, x: list, lo_t: float, hi_t: float) -> float | None:
  best = None
  for i in range(1, len(t)):
    if lo_t <= t[i] <= hi_t:
      dt_i = t[i] - t[i - 1]
      if dt_i <= 0.0:
        continue
      jerk = abs((x[i] - x[i - 1]) / dt_i)
      best = jerk if best is None else max(best, jerk)
  return best


def _window_mean(t: list, x: list, lo_t: float, hi_t: float) -> float | None:
  vals = [x[i] for i in range(len(t)) if lo_t <= t[i] <= hi_t]
  return float(np.mean(vals)) if vals else None


def trace_metrics(trace: StopTrace, scenario: Scenario) -> dict[str, Any]:
  """Per-event predicted metrics on the closed-loop trace; field names match the analyzer/event
  store so the frozen scoring config classifies them unchanged."""
  t, v, a, u = trace.t, trace.v, trace.a, trace.u
  n = len(t)
  hold_idx = None
  for i in range((trace.first_stop_idx or 0), n):
    if v[i] < STANDSTILL_V:
      hold_idx = i
      break

  entry_speed = max(v[: (trace.first_stop_idx or n)] or [0.0])
  metrics: dict[str, Any] = {
    "entry_speed_mps": float(entry_speed),
    "min_a_ego_mps2": float(min(a)) if a else None,
    "min_accel_cmd_mps2": float(min(u)) if u else None,
    "entered_stopping": trace.first_stop_idx is not None,
    "settled": hold_idx is not None,
    "ends_stopped": trace.ends_stopped,
  }

  # hard decel duration (analyze_stopping_behavior.py:38-39 definition)
  hard = 0.0
  for i in range(1, n):
    if a[i - 1] <= -1.50 and v[i - 1] >= 1.00:
      hard += max(t[i] - t[i - 1], 0.0)
  metrics["hard_decel_duration_s"] = hard

  # rollout from 2 m/s to hold
  rollout = None
  if hold_idx is not None:
    start = hold_idx
    for i in range(hold_idx, -1, -1):
      if v[i] >= 2.0:
        break
      start = i
    rollout = sum(max((v[i] + v[i + 1]) / 2.0, 0.0) * max(t[i + 1] - t[i], 0.0) for i in range(start, hold_idx))
  metrics["rollout_distance_from_2mps_m"] = rollout

  if hold_idx is not None:
    hold_t = t[hold_idx]
    metrics["end_stop_jerk_mps3"] = _window_max_jerk(t, a, hold_t - 0.45, hold_t + 0.20)
    metrics["end_stop_cmd_jerk_mps3"] = _window_max_jerk(t, u, hold_t - 0.45, hold_t + 0.20)
    after = _window_mean(t, a, hold_t, hold_t + 0.15)
    before = _window_mean(t, a, hold_t - 0.50, hold_t - 0.10)
    metrics["end_stop_accel_step_mps2"] = abs(after - before) if after is not None and before is not None else None
    if trace.first_stop_idx is not None:
      metrics["time_to_standstill_s"] = float(hold_t - t[trace.first_stop_idx])
    # Rebound uses the same bounded +1.0 s post-hold window as the on-road analyzer. The replay's
    # longer settle extension must never turn a later legitimate departure into a leapfrog.
    baseline_speeds = [v[i] for i in range(n) if hold_t - 0.20 <= t[i] <= hold_t + 0.10]
    post_stop_speeds = [v[i] for i in range(hold_idx, n)
                        if hold_t <= t[i] <= hold_t + 1.00 and trace.stop_request[i]]
    rebound = (max(0.0, max(post_stop_speeds) - min(baseline_speeds))
               if baseline_speeds and post_stop_speeds else 0.0)
    unexpected = 0.0
    for i in range(hold_idx, n):
      if t[i] > hold_t + 1.00:
        break
      if trace.stop_request[i] and v[i] <= 1.2 and u[i] <= -0.1:
        envelope = float(interp(v[i], P.EXPECTED_ACCEL_V_BP, P.EXPECTED_ACCEL_MAX))
        unexpected = max(unexpected, a[i] - envelope)
    metrics["speed_rebound_while_should_stop_mps"] = rebound
    metrics["speed_rebound_while_stop_signal_mps"] = rebound
    metrics["should_stop_unexpected_accel_mps2"] = unexpected
  else:
    metrics["time_to_standstill_s"] = None
    metrics["end_stop_jerk_mps3"] = None
    metrics["end_stop_cmd_jerk_mps3"] = None
    metrics["end_stop_accel_step_mps2"] = None
    metrics["speed_rebound_while_should_stop_mps"] = None
    metrics["speed_rebound_while_stop_signal_mps"] = None
    metrics["should_stop_unexpected_accel_mps2"] = None

  if trace.first_stop_idx is not None:
    entry_t = t[trace.first_stop_idx]
    metrics["entry_stop_jerk_mps3"] = _window_max_jerk(t, a, entry_t - 0.45, entry_t + 0.20)
    metrics["entry_stop_cmd_jerk_mps3"] = _window_max_jerk(t, u, entry_t - 0.45, entry_t + 0.20)
    after = _window_mean(t, a, entry_t, entry_t + 0.15)
    before = _window_mean(t, a, entry_t - 0.50, entry_t - 0.10)
    metrics["entry_stop_accel_step_mps2"] = abs(after - before) if after is not None and before is not None else None
    metrics["stop_signal_dropped_before_hold"] = bool(hold_idx is not None
                                                      and any(not trace.stop_request[i] for i in range(trace.first_stop_idx, hold_idx)))
    metrics["left_stopping_state_before_hold"] = bool(hold_idx is not None
                                                      and any(trace.state[i] != STOPPING for i in range(trace.first_stop_idx, hold_idx)))

  lead_entry = next((float(row.lead_d_rel_m) for row in scenario.samples
                     if row.lead_status and row.lead_d_rel_m is not None), None)
  metrics["lead_distance_stop_entry_m"] = lead_entry
  hold_gap = trace.lead_gap[hold_idx] if hold_idx is not None and trace.lead_status[hold_idx] else None
  metrics["lead_distance_hold_m"] = float(hold_gap) if lead_entry is not None and hold_gap is not None else None
  stop_start = trace.first_stop_idx or 0
  stop_end = hold_idx if hold_idx is not None else len(trace.lead_gap) - 1
  lead_gaps = [float(gap) for i, gap in enumerate(trace.lead_gap)
               if stop_start <= i <= stop_end and trace.lead_status[i] and gap is not None]
  metrics["minimum_lead_gap_m"] = min(lead_gaps) if lead_gaps else None
  metrics["lead_contact"] = bool(lead_gaps and min(lead_gaps) <= 0.05)
  confirmed_departure = False
  if hold_idx is not None and hold_gap is not None:
    departure_t = 0.0
    for i in range(hold_idx + 1, n):
      dt_i = max(t[i] - t[i - 1], 0.0)
      gap_grew = trace.lead_gap[i] is not None and trace.lead_gap[i] > hold_gap + 0.3
      lead_receding = trace.lead_status[i] and trace.lead_v[i] - v[i] > 0.5
      departure_t = departure_t + dt_i if gap_grew and lead_receding else 0.0
      if departure_t >= 0.5:
        confirmed_departure = True
        break
  metrics["confirmed_lead_departure"] = confirmed_departure

  # Cranked-requirement metrics (2026-06-13): the two user-felt forces, computed on the sim trace
  # with the SAME definitions as build_event_store so the offline-sim verdict and the on-road eval
  # agree. The sim is always engaged + long-control-active, so the engaged mask maps to "in a
  # command-producing state" (state != OFF). Without these the sim cannot measure the cranked
  # forces and the cranked flags can never fire (the gate would be blind to exactly the iteration
  # this cycle targets).
  app = _approach_decel_over_gap2m_trace(trace, sc.SCORING_CONFIG.cranked)
  metrics["approach_peak_decel_over_gap2m"] = app["peak_decel"] if app else None
  metrics["approach_required_decel_to_2m"] = app["required_decel"] if app else None
  metrics["approach_necessary"] = app["necessary"] if app else None
  metrics["approach_worst_gap_m"] = app["worst_gap_m"] if app else None
  metrics["approach_worst_v_ego_mps"] = app["worst_v_ego_mps"] if app else None
  metrics["approach_worst_closing_mps"] = app["worst_closing_mps"] if app else None
  settle = _settle_meas_jerk_trace(trace)
  metrics["settle_peak_meas_jerk"] = settle["peak_meas_jerk"] if settle else None
  metrics["settle_peak_sent_jerk"] = settle["peak_sent_jerk"] if settle else None
  metrics["settle_meas_minus_sent_jerk"] = settle["meas_minus_sent_jerk"] if settle else None
  # DEVELOPMENT-ONLY predicted-IMU settle metric -- only present when the sim ran with a friction
  # residual (trace.a_imu populated). NOT gated: scoring_config's IMU gate reads the ON-ROAD store
  # value, never this predicted one. None when friction was not supplied.
  imu = _settle_imu_jerk_trace(trace)
  if imu is not None:
    metrics["settle_peak_imu_jerk_pred"] = imu["peak_imu_jerk"]
    metrics["settle_peak_imu_decel_pred"] = imu["peak_imu_decel"]
  return metrics


# Sim-trace ports of the cranked-requirement metric builders (build_event_store.approach_decel_over_gap2m
# / settle_meas_jerk). Definitions are kept byte-for-byte equivalent in physics so the sim verdict
# and the on-road eval classify the same event the same way; the only adaptation is the
# engaged + long-control-active mask, which for the always-active sim is `state != OFF`.
SIM_SETTLE_STANDSTILL_SPEED = 0.06  # build_event_store.SETTLE_STANDSTILL_SPEED
SIM_SETTLE_PRE_S = 0.6              # build_event_store.settle_meas_jerk pre_settle_s
APPROACH_STANDSTILL_SPEED = 0.12   # build_event_store.approach_decel_over_gap2m standstill_speed default


def _approach_decel_over_gap2m_trace(trace: StopTrace, cranked) -> dict[str, Any] | None:
  """Peak commanded decel during the stopping phase while the lead gap is still > gap_floor, with
  the same kinematic necessity test as the eval (closing^2 / (2*max(gap-floor, eps)))."""
  if trace.first_stop_idx is None:
    return None
  t, v, u = trace.t, trace.v, trace.u
  n = len(t)
  hold_idx = None
  for i in range(trace.first_stop_idx, n):
    if v[i] < STANDSTILL_V:
      hold_idx = i
      break
  if hold_idx is None or hold_idx <= trace.first_stop_idx:
    return None
  gap_floor = float(cranked.approach_gap_floor_m)
  worst = None  # (peak_decel, gap, v_ego, closing)
  for idx in range(trace.first_stop_idx, min(hold_idx, n - 1) + 1):
    if trace.state[idx] == OFF:
      continue
    gap = trace.lead_gap[idx]
    if not (trace.lead_status[idx] and gap is not None and gap > gap_floor):
      continue
    if v[idx] <= APPROACH_STANDSTILL_SPEED:
      continue
    decel = max(-float(u[idx]), 0.0)
    if worst is None or decel > worst[0]:
      closing = max(float(v[idx]) - float(trace.lead_v[idx]), 0.0)
      worst = (decel, float(gap), float(v[idx]), closing)
  if worst is None:
    return None
  peak_decel, gap, v_ego, closing = worst
  required_decel = (closing * closing) / (2.0 * max(gap - gap_floor, 0.1))
  necessary = peak_decel <= required_decel + float(cranked.approach_necessary_margin)
  return {"peak_decel": peak_decel, "required_decel": required_decel, "necessary": bool(necessary),
          "worst_gap_m": gap, "worst_v_ego_mps": v_ego, "worst_closing_mps": closing}


def _settle_meas_jerk_trace(trace: StopTrace) -> dict[str, Any] | None:
  """Peak MEASURED jerk in the terminal-settle window terminating at the first genuine standstill,
  with the sent-command companion -- build_event_store.settle_meas_jerk on the sim trace."""
  if trace.first_stop_idx is None:
    return None
  t, v, a, u = trace.t, trace.v, trace.a, trace.u
  n = len(t)
  first_standstill_idx = None
  for idx in range(trace.first_stop_idx, n):
    if v[idx] <= SIM_SETTLE_STANDSTILL_SPEED:
      first_standstill_idx = idx
      break
  if first_standstill_idx is None:
    return None
  settle_t0 = t[first_standstill_idx] - SIM_SETTLE_PRE_S
  window: list[int] = []
  active_seen = False
  for idx in range(trace.first_stop_idx, first_standstill_idx + 1):
    if t[idx] < settle_t0:
      continue
    if trace.state[idx] == OFF:
      if active_seen:
        break
      continue
    active_seen = True
    window.append(idx)
  if len(window) < 2:
    return None
  peak_meas: float | None = None
  peak_sent: float | None = None
  for prev, cur in zip(window, window[1:], strict=False):
    dt_i = t[cur] - t[prev]
    if dt_i <= 1e-6:
      continue
    meas = abs((a[cur] - a[prev]) / dt_i)
    peak_meas = meas if peak_meas is None else max(peak_meas, meas)
    sent = abs((u[cur] - u[prev]) / dt_i)
    peak_sent = sent if peak_sent is None else max(peak_sent, sent)
  if peak_meas is None:
    return None
  meas_minus_sent = None if peak_sent is None else (peak_meas - peak_sent)
  return {"peak_meas_jerk": peak_meas, "peak_sent_jerk": peak_sent, "meas_minus_sent_jerk": meas_minus_sent}


def _settle_imu_jerk_trace(trace: StopTrace) -> dict[str, Any] | None:
  """DEVELOPMENT-ONLY predicted-IMU settle metric (NOT a gate). Peak |d(a_imu)/dt| and peak |a_imu|
  over the SAME terminal-settle window as _settle_meas_jerk_trace, but read off the predicted-IMU
  channel (wheel/linear plant accel + friction residual) instead of the wheel accel. This mirrors
  build_event_store.settle_imu_jerk()'s definition on the sim trace so the predicted settle_peak_imu_jerk
  / settle_peak_imu_decel are comparable in *form* to the on-road numbers. They remain a MODEL
  PREDICTION: the on-road IMU promotes, the sim only develops. Returns None when the trace carries no
  predicted-IMU channel (friction was not supplied)."""
  if trace.first_stop_idx is None or not trace.a_imu:
    return None
  t, v, a_imu = trace.t, trace.v, trace.a_imu
  n = len(t)
  first_standstill_idx = None
  for idx in range(trace.first_stop_idx, n):
    if v[idx] <= SIM_SETTLE_STANDSTILL_SPEED:
      first_standstill_idx = idx
      break
  if first_standstill_idx is None:
    return None
  settle_t0 = t[first_standstill_idx] - SIM_SETTLE_PRE_S
  window: list[int] = []
  active_seen = False
  for idx in range(trace.first_stop_idx, first_standstill_idx + 1):
    if t[idx] < settle_t0:
      continue
    if trace.state[idx] == OFF:
      if active_seen:
        break
      continue
    active_seen = True
    window.append(idx)
  if len(window) < 2:
    return None
  peak_jerk: float | None = None
  peak_decel = max(abs(float(a_imu[idx])) for idx in window)
  for prev, cur in zip(window, window[1:], strict=False):
    dt_i = t[cur] - t[prev]
    if dt_i <= 1e-6:
      continue
    jerk = abs((a_imu[cur] - a_imu[prev]) / dt_i)
    peak_jerk = jerk if peak_jerk is None else max(peak_jerk, jerk)
  if peak_jerk is None:
    return None
  return {"peak_imu_jerk": peak_jerk, "peak_imu_decel": peak_decel}


def classify_metrics(metrics: dict[str, Any]) -> tuple[list[str], list[str]]:
  """Frozen-config classification of a sim event (spec 7.3: scoring_config is the only source)."""
  return sc.classify_event(metrics)


def event_row(scenario: Scenario, trace: StopTrace, metrics: dict[str, Any]) -> dict[str, Any]:
  harsh_flags, leapfrog_flags = classify_metrics(metrics)
  return {
    "name": scenario.name,
    "route": scenario.key["route"] if scenario.key else scenario.name,
    "event_id": scenario.event_id if scenario.event_id is not None else 0,
    "key": scenario.key,
    "controller": trace.controller,
    "plant": trace.plant,
    "stratum": scenario.stratum,
    "signals_version": scenario.signals_version,
    "telemetry_version": scenario.telemetry_version,
    **metrics,
    "harsh_flags": harsh_flags,
    "leapfrog_flags": leapfrog_flags,
    "is_harsh": sc.is_harsh(harsh_flags),
    "is_leapfrog": sc.is_leapfrog(leapfrog_flags),
  }


# --- scenario sources ------------------------------------------------------------------------------

def fixture_scenarios() -> list[Scenario]:
  return [Scenario(name=name, samples=list(samples)) for name, samples in SCENARIOS.items()]


def _resample_step(t_src: np.ndarray, values: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
  idx = np.clip(np.searchsorted(t_src, t_grid, side="right") - 1, 0, len(t_src) - 1)
  return values[idx]


def scenario_from_store_record(store_dir: Path, record: dict[str, Any], dt: float) -> Scenario | None:
  trace_path = store_dir / record["trace_ref"]
  if not trace_path.is_file():
    return None
  with np.load(trace_path) as npz:
    if not all(name in npz for name in ("t", "v_ego", "a_ego", "accel_cmd")):
      return None
    t_src = np.asarray(npz["t"], dtype=float)
    if len(t_src) < 3:
      return None
    n = int(math.floor((t_src[-1] - t_src[0]) / dt)) + 1
    t_grid = t_src[0] + np.arange(n) * dt
    v_ego = np.interp(t_grid, t_src, np.asarray(npz["v_ego"], dtype=float))
    a_ego = np.interp(t_grid, t_src, np.asarray(npz["a_ego"], dtype=float))
    accel_cmd = _resample_step(t_src, np.asarray(npz["accel_cmd"], dtype=float), t_grid)
    planner_a_target = (_resample_step(t_src, np.asarray(npz["a_target"], dtype=float), t_grid)
                        if "a_target" in npz else np.full(n, np.nan))
    should_stop = (_resample_step(t_src, np.asarray(npz["should_stop"], dtype=float), t_grid) > 0.5
                   if "should_stop" in npz else np.ones(n, dtype=bool))
    target = (_resample_step(t_src, np.asarray(npz["distance_to_stop_target_m"], dtype=float), t_grid)
              if "distance_to_stop_target_m" in npz else np.full(n, -1.0))
    lead_status = (_resample_step(t_src, np.asarray(npz["lead_status"], dtype=float), t_grid) > 0.5
                   if "lead_status" in npz else np.zeros(n, dtype=bool))
    lead_v = (_resample_step(t_src, np.asarray(npz["lead_v"], dtype=float), t_grid)
              if "lead_v" in npz else np.zeros(n))
    lead_d_rel = (_resample_step(t_src, np.asarray(npz["lead_d_rel_m"], dtype=float), t_grid)
                  if "lead_d_rel_m" in npz else np.full(n, np.nan))

  samples = []
  for i in range(n):
    target_i = float(target[i])
    gap_i = float(lead_d_rel[i])
    samples.append(FakeSample(
      t=float(t_grid[i]), v_ego=float(v_ego[i]), a_ego=float(a_ego[i]),
      accel_cmd=float(accel_cmd[i]) if math.isfinite(accel_cmd[i]) else None,
      should_stop=bool(should_stop[i]),
      distance_to_stop_target_m=target_i if target_i > 0.0 else None,
      raw_should_stop=None,
      lead_status=bool(lead_status[i]),
      lead_v=float(lead_v[i]) if math.isfinite(lead_v[i]) else 0.0,
      lead_d_rel_m=gap_i if (bool(lead_status[i]) and math.isfinite(gap_i)) else None,
    ))
  key = dict(record["key"])
  entry = record.get("entry", {}) if isinstance(record.get("entry"), dict) else {}
  return Scenario(
    name=f"{key.get('route')}--{key.get('seg')}--{key.get('hold_mono_ns')}",
    samples=samples,
    key=key,
    event_id=record.get("event_id"),
    signals_version=int(record.get("signals_version", 1)),
    telemetry_version=int(record.get("telemetry_version", 1)),
    stratum=stratum_for_entry(entry),
    planner_a_targets=[float(value) if math.isfinite(value) else None for value in planner_a_target],
  )


def stratum_for_entry(entry: dict[str, Any]) -> str:
  """Spec 7.6 strata: approach speed {<1, 1-2, >2} x {explicit target, stopped lead, no target}."""
  v = float(entry.get("v_approach") or 0.0)
  speed_bin = "<1" if v < 1.0 else "1-2" if v <= 2.0 else ">2"
  if entry.get("explicit_target"):
    target = "explicit"
  elif entry.get("lead_entry_gap_m") is not None:
    target = "lead"
  else:
    target = "no_target"
  return f"v{speed_bin}|{target}"


def load_store_scenarios(store_dir: Path, dt: float, max_events: int = 0,
                         routes: set[str] | None = None) -> list[Scenario]:
  events_path = store_dir / "events.jsonl"
  if not events_path.is_file():
    return []
  scenarios: list[Scenario] = []
  for line in events_path.read_text().splitlines():
    line = line.strip()
    if not line:
      continue
    record = json.loads(line)
    if routes is not None and str(record.get("key", {}).get("route")) not in routes:
      continue
    scenario = scenario_from_store_record(store_dir, record, dt)
    if scenario is not None:
      scenarios.append(scenario)
    if max_events > 0 and len(scenarios) >= max_events:
      break
  return scenarios


# --- CLI -------------------------------------------------------------------------------------------

def run_replay(scenarios: list[Scenario], controllers: list[str], plants: dict[str, PlantParams],
               dt: float, extend_s: float = DEFAULT_EXTEND_S,
               friction: FrictionResidual | None = None) -> dict[str, Any]:
  rows: list[dict[str, Any]] = []
  for plant_name, plant_params in plants.items():
    plant = PlantModel(plant_params, dt)
    for controller_name in controllers:
      for scenario in scenarios:
        controller = make_controller(controller_name)
        trace = simulate_stop(controller, plant, scenario, dt, extend_s=extend_s,
                              controller_name=controller_name, plant_name=plant_name, friction=friction)
        rows.append(event_row(scenario, trace, trace_metrics(trace, scenario)))
  return {
    "replay_schema_version": 2,
    "generated_utc": datetime.now(UTC).replace(microsecond=0).isoformat(),
    "dt": dt,
    "controllers": controllers,
    "plants": sorted(plants),
    "scenario_count": len(scenarios),
    # DEVELOPMENT-ONLY annotation; the friction plant never participates in gating.
    "friction_residual": friction.as_dict() if friction is not None else None,
    "scoring_config_version": sc.SCORING_CONFIG.version,
    "scoring_config": json.loads(sc.canonical_json()),
    "event_rows": rows,
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Closed-loop stopping replay through the identified plant (spec 7.6)")
  parser.add_argument("--event-store", default=None, help=f"Event store dir (default: {DEFAULT_EVENT_STORE} when present)")
  parser.add_argument("--include-fixtures", action="store_true", help="Also replay every stop_scenarios.py fixture")
  parser.add_argument("--controller", default="v2", choices=list(CONTROLLERS))
  parser.add_argument("--plant", default="ref", help="'ref' (frozen 20260514), 'refit' (archived 20260531), 'both', or a model JSON path")
  parser.add_argument("--friction", default=None,
                      help="DEVELOPMENT-ONLY (NOT a gate): also predict an IMU channel through the friction-augmented "
                           + "plant. 'default' = archived 2026-06-14 coarse-provisional fit, or a fit-archive JSON path. "
                           + "Off by default; the on-road IMU settle metric remains the promoter.")
  parser.add_argument("--dt", type=float, default=DEFAULT_DT)
  parser.add_argument("--extend-s", type=float, default=DEFAULT_EXTEND_S)
  parser.add_argument("--max-events", type=int, default=0, help="Cap event-store scenarios (0 = all)")
  parser.add_argument("--route", action="append", default=[], help="Restrict event-store scenarios to these routes (repeatable)")
  parser.add_argument("--output-json", default=None)
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  scenarios: list[Scenario] = []
  store_dir = Path(args.event_store).expanduser() if args.event_store else DEFAULT_EVENT_STORE
  if store_dir.is_dir():
    routes = set(args.route) if args.route else None
    scenarios.extend(load_store_scenarios(store_dir, args.dt, max_events=args.max_events, routes=routes))
  elif args.event_store:
    print(f"[sim-replay] event store not found: {store_dir}", file=sys.stderr)
    return 2
  if args.include_fixtures or not scenarios:
    scenarios.extend(fixture_scenarios())
  if not scenarios:
    print("[sim-replay] no scenarios to replay", file=sys.stderr)
    return 2

  controllers = [args.controller]
  friction = load_friction(args.friction)
  if friction is not None:
    c = friction.params
    print("[sim-replay] DEVELOPMENT-ONLY friction plant active (NOT a gate): "
          + f"resid(v)={c.c0:+.4f}{c.c1:+.4f}*exp(-v/{c.v0:.4f}); predicted-IMU channel scored, on-road IMU still promotes")
  report = run_replay(scenarios, controllers, resolve_plants(args.plant), args.dt, extend_s=args.extend_s, friction=friction)

  settled = sum(1 for row in report["event_rows"] if row.get("settled"))
  print(f"[sim-replay] scenarios={report['scenario_count']} rows={len(report['event_rows'])} settled_rows={settled}")
  for controller_name in controllers:
    rows = [r for r in report["event_rows"] if r["controller"] == controller_name]
    n_harsh = sum(1 for r in rows if r["is_harsh"])
    n_leap = sum(1 for r in rows if r["is_leapfrog"])
    print(f"[sim-replay] {controller_name}: rows={len(rows)} harsh={n_harsh} leapfrog={n_leap}")
    if friction is not None:
      preds = [r["settle_peak_imu_jerk_pred"] for r in rows if r.get("settle_peak_imu_jerk_pred") is not None]
      if preds:
        print(f"[sim-replay] {controller_name}: predicted-IMU settles={len(preds)} "
              + f"median_jerk_pred={float(np.median(preds)):.2f} max_jerk_pred={float(max(preds)):.2f} m/s^3 (MODEL prediction)")

  if args.output_json:
    out = Path(args.output_json).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(f"[sim-replay] output_json={out}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
