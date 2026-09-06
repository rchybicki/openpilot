"""WP5 acceptance tests: closed-loop StoppingControllerV2 replay (FINAL_SPEC sections 5.4, 7.6, 8).

The harness owns ONE StopTargetArbiter and wires it to the facade exactly as longcontrol does
(spec 5.4 / F2: the facade itself never instantiates an arbiter -- an AST guard below and in
test_stop_target_arbiter.py enforce it). Each stop_scenarios.py fixture seeds a closed-loop sim
through the frozen archived plants (spec 7.6 dual-plant flavor); the full Tier-1 similarity gate
(legacy-vs-V2 paired bounds) is WP6's tools/stopping/similarity_gate.py -- this file asserts the
facade-side outcome envelope: finite bounded commands, no harsh per-frame steps, no rollback,
settle into the hold band, bounded creep distance.

Pure python + numpy only (spec section 8 import-clean rule).
"""

import ast
import math
import pathlib
from dataclasses import dataclass, replace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import stop_target_arbiter as sta
from openpilot.selfdrive.controls.lib.stop_target_arbiter import StopDecision, StopSource, StopTargetArbiter
from openpilot.selfdrive.controls.lib.stopping_controller_v2 import DEBUG_VERSION, StoppingControllerV2
from openpilot.selfdrive.controls.lib.stopping_params import STOPPING_PARAMS
from openpilot.selfdrive.controls.lib.stopping_plant import PLANT_PARAMS_REF, PlantModel, load_legacy_model_json
from openpilot.selfdrive.controls.lib.stopping_trajectory import TrajPhase
from openpilot.selfdrive.controls.lib.tests.stop_scenarios import SCENARIOS, FakeSample

interp = np.interp
P = STOPPING_PARAMS
REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
FACADE_PATH = REPO_ROOT / "selfdrive" / "controls" / "lib" / "stopping_controller_v2.py"
ARCHIVE = REPO_ROOT / "docs" / "stopping" / "archive"

OFF, PID, STOPPING, STARTING = (sta.LONG_CTRL_STATE_OFF, sta.LONG_CTRL_STATE_PID,
                                sta.LONG_CTRL_STATE_STOPPING, sta.LONG_CTRL_STATE_STARTING)
DT = 0.1            # the fixtures are 10 Hz logged seeds; PlantModel re-discretizes to any dt
V_EGO_STARTING = 0.4
STOP_ACCEL = -2.0
EXTEND_S = 20.0     # the seeds are short; keep the last intent asserted until the sim settles
STANDSTILL_CLAMP_FRAMES = int(round(0.6 / DT))  # check_harsh_stops_model.py:723 clamp delay

# the frozen 20260514 fit (== PLANT_PARAMS_REF) + the archived 20260531 refit (spec 7.6 dual plant)
PLANTS = {
  "ref_20260514": PLANT_PARAMS_REF,
  "refit_20260531": load_legacy_model_json(str(ARCHIVE / "plant_model_20260531T075153Z_all.json")),
}

REQUIRED_DEBUG_KEYS = {"version", "phase", "a_ref", "disturbance", "rollout_m", "remaining_m",
                       "release_inhibit_active", "recovery_i", "settled_time_s", "source", "triggers"}


class _CP:
  def __init__(self):
    self.carFingerprint = "HYUNDAI_SANTA_FE_HEV_2022"
    self.startingState = True
    self.enableGasInterceptor = False


def long_control_state_trans(CP, active, state, v_ego, should_stop, brake_pressed, cruise_standstill,
                             a_target, distance_to_stop_target_m):
  # minimal port of longcontrol.py:509-546 (same recipe as test_stop_target_arbiter.py's oracle)
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptor
  stopping_condition = should_stop or sta.should_enter_stop_target_mode(v_ego, a_target, distance_to_stop_target_m)
  if state == STOPPING and not should_stop:
    stopping_condition = stopping_condition or sta.should_hold_stop_target_mode(v_ego, a_target, distance_to_stop_target_m)
  starting_condition = not stopping_condition and not cruise_standstill and not brake_pressed
  if not active:
    return OFF
  if state == OFF:
    if not starting_condition:
      return STOPPING
    return STARTING if CP.startingState else PID
  if state == STOPPING:
    if starting_condition:
      return STARTING if CP.startingState else PID
    return STOPPING
  if state in (STARTING, PID):
    if stopping_condition:
      return STOPPING
    if v_ego > V_EGO_STARTING:
      return PID
    return state
  return state


@dataclass
class StopTrace:
  name: str
  t: list
  v: list
  a: list
  u: list
  state: list
  stop_request: list
  debug_frames: list
  first_stop_idx: int | None
  ends_stopped: bool


def _signal_rows(samples: list[FakeSample]) -> list[FakeSample]:
  """Fixture frames + an extension that keeps the final intent asserted so the sim settles.
  Explicit targets shrink with simulated ego travel inside the loop (closed-loop semantics)."""
  rows = list(samples)
  last = samples[-1]
  n_extend = int(round(EXTEND_S / DT))
  t = last.t
  for _ in range(n_extend):
    t += DT
    rows.append(replace(last, t=t))
  return rows


def simulate(name: str, samples: list[FakeSample], plant_params, collect_debug=False) -> StopTrace:
  plant = PlantModel(plant_params, DT)
  CP = _CP()
  arbiter = StopTargetArbiter(CP)
  facade = StoppingControllerV2(CP)

  rows = _signal_rows(samples)
  n_fixture = len(samples)
  v = max(float(samples[0].v_ego), 0.0)
  a = float(samples[0].a_ego)
  last_u = float(samples[0].accel_cmd) if samples[0].accel_cmd is not None else -0.1
  a_target = last_u
  sent = [last_u] * max(plant.delay_frames, 1)
  state = PID
  standstill_frames = 0
  explicit_target = None   # closed-loop remaining distance for the extension frames
  lead_gap = None
  trace = StopTrace(name=name, t=[], v=[], a=[], u=[], state=[], stop_request=[],
                    debug_frames=[], first_stop_idx=None, ends_stopped=False)

  for k, row in enumerate(rows):
    in_fixture = k < n_fixture
    if in_fixture:
      explicit_target = (float(row.distance_to_stop_target_m)
                         if row.distance_to_stop_target_m is not None and row.distance_to_stop_target_m > 0.0 else None)
      lead_gap = float(row.lead_d_rel_m) if row.lead_d_rel_m is not None else None
      if row.accel_cmd is not None:
        a_target = float(row.accel_cmd)
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
      last_output_accel=last_u, dt=DT,
      human_acceleration=True, v_ego_starting=V_EGO_STARTING)

    new_state = long_control_state_trans(CP, True, state, v, decision.state_should_stop,
                                         False, False, a_target, decision.target_distance_m)
    if state == STOPPING and new_state != STOPPING and decision.state_dropout_hold:
      new_state = STOPPING
    state = new_state

    if state == STOPPING:
      if trace.first_stop_idx is None:
        trace.first_stop_idx = k
      debug = {} if collect_debug else None
      max_exp = float(interp(v, P.EXPECTED_ACCEL_V_BP, P.EXPECTED_ACCEL_MAX))
      min_exp = float(interp(v, P.EXPECTED_ACCEL_V_BP, P.EXPECTED_ACCEL_MIN))
      result = facade.update(min(last_u, -0.1), last_u, decision.stop_request_active, v, a,
                             max_exp, min_exp, STOP_ACCEL, DT,
                             distance_to_stop_target_m=decision.target_distance_m,
                             raw_should_stop=raw_should_stop,
                             lead_status=bool(row.lead_status), lead_v=float(row.lead_v),
                             lead_d_rel=lead_gap, debug=debug, decision=decision)
      u = result.output_accel
      assert result.release_lock_active is False  # seam contract: always False from the facade
      if debug is not None:
        trace.debug_frames.append(debug)
    else:
      u = a_target  # PID passthrough proxy (pid output is a_target clipped on this platform)
    stop_intent_active = (decision.stop_request_active or decision.approach_cap_active
                          or decision.carry_floor_active or state == STOPPING)
    if not stop_intent_active:
      facade.reset()  # longcontrol.py:902-903 discipline (spec 5.5.6 reset bullet)

    trace.t.append(row.t)
    trace.v.append(v)
    trace.a.append(a)
    trace.u.append(u)
    trace.state.append(state)
    trace.stop_request.append(decision.stop_request_active)

    # plant step (one code path for runtime predictor and offline sim, spec 5.1)
    last_u = u
    sent.append(u)
    u_delayed = sent[max(len(sent) - 1 - plant.delay_frames, 0)]
    a = plant.predict_next(a, u_delayed, v)
    a = float(np.clip(a, -4.0, 3.0))
    if standstill_frames >= STANDSTILL_CLAMP_FRAMES and v < 0.01 and u <= -0.08 and a > 0.0:
      # The fitted first-order model drifts into positive accel at standstill (the documented
      # gain collapse/inversion, spec 5.1); clamp this narrow regime exactly as the proven
      # legacy replay does (check_harsh_stops_model.py:802-812) to avoid synthetic creep.
      a = 0.0
    standstill_frames = standstill_frames + 1 if v < 0.05 else 0
    v_prev = v
    v = max(v + a * DT, 0.0)
    travel = max((v_prev + v) / 2.0 * DT, 0.0)
    if explicit_target is not None:
      explicit_target = max(explicit_target - travel, 0.05)
    if lead_gap is not None:
      lead_gap = max(lead_gap - travel, 0.0)

  trace.ends_stopped = trace.state[-1] == STOPPING and trace.stop_request[-1] and trace.v[-1] < 0.05
  return trace


def _stopping_steps(trace: StopTrace) -> list[float]:
  steps = []
  for k in range(1, len(trace.u)):
    if trace.state[k] == STOPPING and trace.state[k - 1] == STOPPING:
      steps.append(trace.u[k] - trace.u[k - 1])
  return steps


@pytest.mark.parametrize("plant_name", list(PLANTS))
class TestClosedLoopReplay:
  def test_comfort_envelope_on_all_fixtures(self, plant_name):
    plant_params = PLANTS[plant_name]
    settled_runs = 0
    for name, samples in SCENARIOS.items():
      trace = simulate(name, samples, plant_params)
      ctx = (plant_name, name)
      # facade authority bounds (spec 5.7): finite, never outside [stop_accel, 0.0] in stopping
      for k, u in enumerate(trace.u):
        assert math.isfinite(u), ctx
        if trace.state[k] == STOPPING and trace.stop_request[k]:
          assert STOP_ACCEL - 1e-9 <= u <= 0.0 + 1e-9, (ctx, k, u)
      # no harsh per-frame spike: the deepest budget in the stack is J_ARREST(0) = 4.0 m/s^3
      for step in _stopping_steps(trace):
        assert abs(step) <= 4.0 * DT + 1e-9, (ctx, step)
      if trace.first_stop_idx is None or not trace.stop_request[-1]:
        continue  # fixture never entered stopping or intent was released (departing/far lead)
      if trace.ends_stopped:
        settled_runs += 1
        # settle: command rests inside the hold band (A_HOLD_RELAXED .. arrest bound margin)
        assert -0.45 <= trace.u[-1] <= -0.05, (ctx, trace.u[-1])
        # time-to-standstill guard (spec 7.6 flavor): settled well inside the extension window
        idx_stop = next(i for i in range(trace.first_stop_idx, len(trace.v)) if trace.v[i] < 0.05)
        assert (trace.t[idx_stop] - trace.t[trace.first_stop_idx]) <= 15.0, ctx
        # no rollback: once stopped with intent asserted, the car stays stopped (no leapfrog)
        assert max(trace.v[idx_stop:]) < 0.15, ctx
        # bounded creep: low-speed travel after stopping entry stays within the rollout budget zone
        creep_m = sum(trace.v[i] * DT for i in range(trace.first_stop_idx, len(trace.v)) if trace.v[i] < 1.2)
        assert creep_m <= 3.0, (ctx, creep_m)
      else:
        # A run may stall in the < 0.2 m/s band: the archived AR(1) plant's authority collapses
        # and inverts sign near v ~ 0.21 m/s (spec 5.1 -- a_ss at the quiescent terminal ceiling
        # is POSITIVE at 0.1 m/s, so no spec-legal command can stop this plant there). The
        # controller must still be doing the right thing: pinned brake, crawl never re-grows.
        assert trace.v[-1] < 0.30, (ctx, trace.v[-1])
        assert trace.u[-1] <= -0.15, (ctx, trace.u[-1])
        crawl = trace.v[trace.first_stop_idx:]
        assert max(crawl[len(crawl) // 2:]) < 0.30, ctx  # the tail never re-accelerates
    # the plant artifact above caps what can settle; both archived plants settle >= 12 fixtures
    assert settled_runs >= 12, f"only {settled_runs} fixtures settled on {plant_name}"

  def test_determinism(self, plant_name):
    name, samples = next(iter(SCENARIOS.items()))
    t1 = simulate(name, samples, PLANTS[plant_name])
    t2 = simulate(name, samples, PLANTS[plant_name])
    assert t1.u == t2.u
    assert t1.v == t2.v


class TestDebugDictContract:
  def test_schema_keys_and_retired_oracle_keys(self):
    # spec section-2 telemetry contract (F36): the v2 key set, shadow_* intentionally retired
    name, samples = "explicit_target_clean_entry_83_event9", SCENARIOS["explicit_target_clean_entry_83_event9"]
    trace = simulate(name, samples, PLANT_PARAMS_REF, collect_debug=True)
    assert trace.debug_frames, "no stopping frames collected"
    for debug in trace.debug_frames:
      assert REQUIRED_DEBUG_KEYS <= set(debug), set(debug)
      assert debug["version"] == DEBUG_VERSION
      assert debug["triggers"] == ()
      assert not any(key.startswith("shadow_") for key in debug)
      assert debug["phase"] in {int(p) for p in TrajPhase}
      assert isinstance(debug["source"], int)
      for key in ("a_ref", "disturbance", "rollout_m", "remaining_m", "recovery_i", "settled_time_s"):
        assert math.isfinite(debug[key]), key

  def test_output_identical_with_and_without_debug(self):
    for name in ("explicit_target_clean_entry_83_event9", "entry_seed_7df_event1", "terminal_unwind_9cc_event1"):
      with_debug = simulate(name, SCENARIOS[name], PLANT_PARAMS_REF, collect_debug=True)
      without = simulate(name, SCENARIOS[name], PLANT_PARAMS_REF, collect_debug=False)
      assert with_debug.u == without.u


def make_decision(**kwargs) -> StopDecision:
  base = dict(stop_request_active=True, state_should_stop=True, target_distance_m=2.0,
              source=StopSource.PLANNER, approach_cap_active=False, carry_floor_active=False,
              departing_lead_release=False, departing_lead_ready=False, far_stopped_lead_release=False,
              legacy_forced=False, release_reason="", state_dropout_hold=False)
  base.update(kwargs)
  return StopDecision(**base)


class TestNonFiniteRobustness:
  """Spec 5.5.5 step 5 / F8: the facade never emits a non-finite command; a poisoned command
  path falls back to last_output_accel with the `nonfinite_fallback` debug flag."""

  def _update(self, facade, *, output_accel=-0.3, last=-0.30, v=0.4, a=-0.3, max_exp=-0.1,
              min_exp=-0.5, stop_accel=STOP_ACCEL, dt=0.01, target=2.0, decision=None, debug=None):
    decision = decision or make_decision(target_distance_m=target)
    return facade.update(output_accel, last, True, v, a, max_exp, min_exp, stop_accel, dt,
                         distance_to_stop_target_m=target, raw_should_stop=True,
                         debug=debug, decision=decision)

  @pytest.mark.parametrize("field", ["v", "a", "max_exp", "min_exp", "stop_accel", "dt", "target", "last", "output_accel"])
  @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
  def test_any_poisoned_input_yields_finite_output(self, field, bad):
    facade = StoppingControllerV2()
    facade.seed_command_history([-0.3] * 10)
    debug = {}
    result = self._update(facade, **{field: bad}, debug=debug)
    assert math.isfinite(result.output_accel), (field, bad)
    # and the next clean frame recovers (no NaN latched into tracker state)
    clean = self._update(facade)
    assert math.isfinite(clean.output_accel), (field, bad)

  def test_poisoned_v_ego_falls_back_to_last_output_accel(self):
    facade = StoppingControllerV2()
    debug = {}
    result = self._update(facade, v=float("nan"), last=-0.37, debug=debug)
    assert result.output_accel == -0.37
    assert debug["nonfinite_fallback"] is True

  def test_poisoned_last_output_accel_falls_back_to_legacy_seed(self):
    facade = StoppingControllerV2()
    debug = {}
    result = self._update(facade, v=float("nan"), last=float("nan"), debug=debug)
    assert result.output_accel == -0.1  # longcontrol.py:925 stopping seed, the degenerate resort
    assert debug["nonfinite_fallback"] is True

  def test_poisoned_tracker_intermediate_state_recovers(self):
    facade = StoppingControllerV2()
    self._update(facade)
    facade.tracker.d_hat = float("nan")
    facade.tracker.rollout_m = float("inf")
    facade.tracker.recovery_i = float("nan")
    result = self._update(facade)
    assert math.isfinite(result.output_accel)
    assert math.isfinite(facade.tracker.d_hat)
    assert math.isfinite(facade.tracker.rollout_m)
    assert math.isfinite(facade.tracker.recovery_i)

  def test_passthrough_and_dropout_branches_are_guarded(self):
    facade = StoppingControllerV2()
    inactive = make_decision(stop_request_active=False, state_should_stop=False,
                             target_distance_m=-1.0, source=StopSource.NONE)
    result = facade.update(float("nan"), -0.30, False, 0.5, -0.3, -0.1, -0.5, STOP_ACCEL, 0.01,
                           decision=inactive)
    assert math.isfinite(result.output_accel)
    facade2 = StoppingControllerV2()
    result2 = facade2.update(float("nan"), -0.30, False, 0.10, float("nan"), -0.1, -0.5, STOP_ACCEL, 0.01,
                             decision=inactive)
    assert math.isfinite(result2.output_accel)


class TestFacadeSeamAndGuards:
  def test_facade_requires_decision(self):
    # spec section 2 / F2: V2 asserts the longcontrol-computed decision is present
    facade = StoppingControllerV2()
    with pytest.raises(AssertionError):
      facade.update(-0.3, -0.3, True, 0.5, -0.3, -0.1, -0.5, STOP_ACCEL, 0.01)

  def test_facade_never_imports_the_arbiter_class(self):
    # spec section 2 / F2 AST guard: only the StopDecision/StopSource data types may be imported
    tree = ast.parse(FACADE_PATH.read_text())
    for node in ast.walk(tree):
      if isinstance(node, ast.ImportFrom) and node.module and "stop_target_arbiter" in node.module:
        names = {alias.name for alias in node.names}
        assert names <= {"StopDecision", "StopSource"}, names
      if isinstance(node, ast.Import):
        assert not any("stop_target_arbiter" in alias.name for alias in node.names)
      if isinstance(node, ast.Name):
        assert node.id != "StopTargetArbiter"

  def test_legacy_getattr_telemetry_attributes(self):
    # longcontrol getattr reads (spec section 2): .low_speed_rollout_m and .phase must exist
    facade = StoppingControllerV2()
    assert isinstance(facade.low_speed_rollout_m, float)
    assert isinstance(facade.phase, int)

  def test_reset_on_disengage_clears_facade_and_tracker(self):
    # spec 5.5.6 / F5: brake tap => USER_DISABLE => reset(); clean re-engage, no stale state
    facade = StoppingControllerV2()
    decision = make_decision()
    u = -0.3
    for k in range(50):
      u = facade.update(-0.4, u, True, 0.4 - 0.005 * k, 0.2, -0.1, -0.1, STOP_ACCEL, 0.01,
                        raw_should_stop=True, decision=decision).output_accel
    assert facade.tracker.rollout_m > 0.0
    facade.reset()
    assert facade.tracker.d_hat == 0.0
    assert facade.tracker.recovery_i == 0.0
    assert facade.tracker.rollout_m == 0.0
    assert facade.low_speed_rollout_m == 0.0
    fresh = StoppingControllerV2()
    u_re, u_fresh = -0.30, -0.30
    for k in range(25):
      args = (-0.35, True, 0.40 - 0.01 * k, -0.25, -0.1, -0.5, STOP_ACCEL, 0.01)
      u_re = facade.update(-0.35, u_re, *args[1:], raw_should_stop=True, decision=decision).output_accel
      u_fresh = fresh.update(-0.35, u_fresh, *args[1:], raw_should_stop=True, decision=decision).output_accel
      assert u_re == u_fresh
