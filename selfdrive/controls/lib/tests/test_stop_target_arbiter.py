"""WP4 acceptance tests for the stop-target arbiter (FINAL_SPEC sections 5.2, 8, 11; red-team F2/F3/F4/F14/F28/F32).

Pure python + numpy only (spec section 8 import-clean rule): no cereal, no Params, no msgq.

Contents:
  * verbatim-port AST equality of every ported predicate vs the longcontrol.py definition
    (skips per-function once Commit B moves the definition out of longcontrol);
  * frame-level equivalence of StopTargetArbiter.update against a legacy oracle that replicates
    the longcontrol.py:729-868/:888-897/:1124-1128 wiring verbatim -- the harness evolves the
    state machine exactly the way WP7's Commit B will, so it doubles as the wiring recipe;
  * consolidated dual-window dropout holds (0.4 s rolling / 0.8 s release / 1.4 s standstill with
    the legacy escapes) + unbounded STOPPED_LEAD_LATCH, shadow-only, with divergence counters;
  * independent release booleans (F14), legacy_forced intent tiering (F32), source enum + the
    -1.0 target sentinel, ISD invariance (the arbiter performs no internal compensation, 4.2.4);
  * AST guards: lead_d_rel_eff single-producer/allowlisted-consumers (F4) and the
    stopping_controller_v2 no-arbiter-import rule (F2).
"""

import ast
import pathlib
import random
from dataclasses import dataclass

import pytest

from openpilot.selfdrive.controls.lib import stop_target_arbiter as sta
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import (
  LEAD_STOP_DISTANCE_TARGET,
  STOP_TARGET_CLOSE_HOLD_REMAINING_M,
  STOP_TARGET_LATCH_DURATION_S,
  get_distance_to_stopped_lead_target,
  get_stopped_lead_control_target,
  update_distance_to_stop_target_with_latch,
)
from openpilot.selfdrive.controls.lib.stopping_params import STOPPING_PARAMS

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
LONGCONTROL_PATH = REPO_ROOT / "selfdrive" / "controls" / "lib" / "longcontrol.py"
ARBITER_PATH = REPO_ROOT / "selfdrive" / "controls" / "lib" / "stop_target_arbiter.py"
FACADE_PATH = REPO_ROOT / "selfdrive" / "controls" / "lib" / "stopping_controller_v2.py"

OFF = sta.LONG_CTRL_STATE_OFF
PID = sta.LONG_CTRL_STATE_PID
STOPPING = sta.LONG_CTRL_STATE_STOPPING
STARTING = sta.LONG_CTRL_STATE_STARTING

V_EGO_STARTING = 0.4
DT = 0.01


class _CP:
  def __init__(self, fingerprint="HYUNDAI_SANTA_FE_HEV_2022"):
    self.carFingerprint = fingerprint
    self.startingState = True
    self.enableGasInterceptor = False


@dataclass
class Frame:
  v_ego: float = 0.0
  a_ego: float = 0.0
  a_target: float = 0.0
  should_stop: bool = False
  planner_target_m: float = -1.0
  lead_status: bool = False
  lead_v: float = 0.0
  lead_d_rel: float = 0.0
  brake_pressed: bool = False
  cruise_standstill: bool = False
  standstill: bool = False
  force_coast: bool = False
  active: bool = True
  last_output_accel: float = -0.3
  dt: float = DT
  human_acceleration: bool = True


def step(arb: sta.StopTargetArbiter, state: int, f: Frame) -> sta.StopDecision:
  return arb.update(
    v_ego=f.v_ego, a_ego=f.a_ego, a_target=f.a_target,
    raw_should_stop=f.should_stop, planner_target_m=f.planner_target_m,
    lead_status=f.lead_status, lead_v=f.lead_v, lead_d_rel=f.lead_d_rel,
    increased_stopped_distance_m=0.0,
    brake_pressed=f.brake_pressed, cruise_standstill=f.cruise_standstill, standstill=f.standstill,
    force_coast=f.force_coast, long_control_state=state,
    last_output_accel=f.last_output_accel, dt=f.dt,
    human_acceleration=f.human_acceleration, v_ego_starting=V_EGO_STARTING,
  )


# --- legacy oracle: verbatim replica of the longcontrol.py wiring ---------------------------------
# Predicate helpers come from the arbiter module; their AST equality with longcontrol.py is proven
# separately below, so the oracle independently exercises only the WIRING (mutation order, timers,
# state machine), which is what the arbiter class re-implements.


def long_control_state_trans(CP, active, long_control_state, v_ego, should_stop, brake_pressed,
                             cruise_standstill, v_ego_starting, a_target=0.0, distance_to_stop_target_m=None):
  # verbatim port of longcontrol.py:509-546 (ints instead of the capnp enum)
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptor
  stopping_condition = should_stop or sta.should_enter_stop_target_mode(v_ego, a_target, distance_to_stop_target_m)
  if long_control_state == STOPPING and not should_stop:
    stopping_condition = stopping_condition or sta.should_hold_stop_target_mode(v_ego, a_target, distance_to_stop_target_m)
  starting_condition = (not stopping_condition and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > v_ego_starting

  if not active:
    return OFF
  if long_control_state == OFF:
    if not starting_condition:
      return STOPPING
    return STARTING if CP.startingState else PID
  if long_control_state == STOPPING:
    if starting_condition and CP.startingState:
      return STARTING
    if starting_condition:
      return PID
    return STOPPING
  if long_control_state in (STARTING, PID):
    if stopping_condition:
      return STOPPING
    if started_condition:
      return PID
    return long_control_state
  return long_control_state


class LegacyArbitrationOracle:
  """longcontrol.py:729-868 + :888-897 + :907-908/:972 resets + :1124-1128, replicated verbatim."""

  def __init__(self, CP, human_acceleration=True, v_ego_starting=V_EGO_STARTING):
    self.CP = CP
    self.human_acceleration = human_acceleration
    self.v_ego_starting = v_ego_starting
    self.long_control_state = OFF
    self.time_since_standstill_s = 10.0
    self.time_since_stop_intent_s = 10.0
    self.last_distance_to_stop_target_m = None
    self._quirk = sta.should_apply_low_speed_stopped_lead_glide_accel_cap(CP)

  def reset(self):  # longcontrol.py:610-616 (arbitration-relevant part)
    self.time_since_standstill_s = 10.0
    self.time_since_stop_intent_s = 10.0
    self.last_distance_to_stop_target_m = None

  def update(self, f: Frame) -> dict:
    prev_distance_to_stop_target_m = self.last_distance_to_stop_target_m  # :730
    stopped_lead_control_target_m = (  # :731-739
      get_stopped_lead_control_target(v_ego=f.v_ego, lead_v=float(f.lead_v), lead_d_rel=float(f.lead_d_rel))
      if bool(f.lead_status) and self._quirk else None
    )
    control = f.planner_target_m  # :740-746
    if stopped_lead_control_target_m is not None and (control is None or control <= 0.0 or stopped_lead_control_target_m < control):
      control = stopped_lead_control_target_m

    stopped_lead_control_stop_active = stopped_lead_control_target_m is not None  # :750
    stop_target_request = sta.should_enter_stop_target_mode(f.v_ego, f.a_target, control)  # :751
    stop_request_active = f.should_stop or stop_target_request or stopped_lead_control_stop_active  # :752
    stop_target_approach_active = (  # :753-756
      not stop_request_active and sta.should_apply_stop_target_approach_mode(f.v_ego, f.a_target, control))
    stop_target_carry_active = (  # :757-761
      not stop_request_active and not stop_target_approach_active
      and sta.should_apply_stop_target_carry_mode(f.v_ego, f.a_target, control))
    standstill = bool(f.standstill) or bool(f.cruise_standstill)  # :762
    departing_lead_ready = sta.should_release_stop_hold_for_departing_lead(  # :763-773
      human_acceleration=bool(self.human_acceleration), output_should_stop=True,
      force_coast=bool(f.force_coast), standstill=standstill,
      v_ego=float(f.v_ego), v_ego_starting=float(self.v_ego_starting),
      lead_status=bool(f.lead_status), lead_v=float(f.lead_v), lead_d_rel=float(f.lead_d_rel))
    departing_lead_release = bool(f.should_stop) and departing_lead_ready  # :774
    if departing_lead_release:  # :775-777
      stop_request_active = False
      stop_target_approach_active = False
    far_stopped_lead_gap_release = (  # :778-788
      self._quirk and not f.force_coast
      and sta.should_release_far_stopped_lead_gap(
        v_ego=f.v_ego, lead_status=bool(f.lead_status), lead_v=float(f.lead_v),
        lead_d_rel=float(f.lead_d_rel), distance_to_stop_target_m=control))
    if far_stopped_lead_gap_release:  # :789-792
      stop_request_active = False
      stop_target_approach_active = False
      stop_target_carry_active = False
    close_stopped_lead_dropout_hold_active = (  # :793-808
      self._quirk and not departing_lead_release and not far_stopped_lead_gap_release
      and sta.should_hold_recent_close_stopped_lead_dropout(
        v_ego=f.v_ego, v_ego_starting=float(self.v_ego_starting), standstill=standstill,
        time_since_standstill_s=self.time_since_standstill_s,
        lead_status=bool(f.lead_status), lead_v=float(f.lead_v), lead_d_rel=float(f.lead_d_rel),
        distance_to_stop_target_m=control, force_coast=bool(f.force_coast)))
    if close_stopped_lead_dropout_hold_active:  # :809-812
      stop_request_active = True
      stop_target_approach_active = False
      stop_target_carry_active = False
    stop_target_release_hold_active = (  # :813-825
      not departing_lead_release and not far_stopped_lead_gap_release and not close_stopped_lead_dropout_hold_active
      and sta.should_hold_low_speed_stop_target_release(
        v_ego=f.v_ego, a_target=f.a_target, distance_to_stop_target_m=control,
        last_distance_to_stop_target_m=prev_distance_to_stop_target_m,
        last_output_accel=f.last_output_accel, time_since_stop_intent_s=self.time_since_stop_intent_s))
    if stop_target_release_hold_active:  # :826-829
      stop_request_active = True
      stop_target_approach_active = False
      stop_target_carry_active = False
    force_coast_standstill_hold = bool(f.force_coast) and standstill  # :830
    state_should_stop = (  # :831-837
      f.should_stop or stopped_lead_control_stop_active or close_stopped_lead_dropout_hold_active
      or stop_target_release_hold_active or force_coast_standstill_hold
    ) and not departing_lead_release and not far_stopped_lead_gap_release

    new_control_state = long_control_state_trans(  # :838-842
      self.CP, f.active, self.long_control_state, f.v_ego, state_should_stop,
      f.brake_pressed, f.cruise_standstill, self.v_ego_starting,
      a_target=f.a_target, distance_to_stop_target_m=control)
    state_dropout_hold_pred = False
    if self.long_control_state == STOPPING and not departing_lead_release and not far_stopped_lead_gap_release:
      state_dropout_hold_pred = (  # predicates of :843-868
        sta.should_hold_stop_target_dropout(
          v_ego=f.v_ego, a_target=f.a_target, distance_to_stop_target_m=control,
          last_distance_to_stop_target_m=prev_distance_to_stop_target_m,
          last_output_accel=f.last_output_accel, time_since_stop_intent_s=self.time_since_stop_intent_s)
        or sta.should_hold_no_target_standstill_dropout(
          v_ego=f.v_ego, standstill=standstill, force_coast=bool(f.force_coast), a_target=f.a_target,
          distance_to_stop_target_m=control, last_output_accel=f.last_output_accel,
          time_since_stop_intent_s=self.time_since_stop_intent_s))
    if self.long_control_state == STOPPING and new_control_state != STOPPING and state_dropout_hold_pred:
      new_control_state = STOPPING  # :843-868
    self.long_control_state = new_control_state  # :886 (prep_stopping is dead code, always False)

    if standstill:  # :888-891
      self.time_since_standstill_s = 0.0
    else:
      self.time_since_standstill_s = min(self.time_since_standstill_s + f.dt, 10.0)
    stop_intent_active = (stop_request_active or stop_target_approach_active or stop_target_carry_active
                          or (self.long_control_state == STOPPING))  # :893
    if stop_intent_active:  # :894-897
      self.time_since_stop_intent_s = 0.0
    else:
      self.time_since_stop_intent_s = min(self.time_since_stop_intent_s + f.dt, 10.0)
    standstill_recent = self.time_since_standstill_s < 0.5  # :899
    stop_intent_recent = self.time_since_stop_intent_s < 1.0  # :900
    # the same-frame consumers (:899-900) read PRE-reset values; the resets below only
    # affect what the next frame sees
    tss_frame = self.time_since_standstill_s
    tsi_frame = self.time_since_stop_intent_s

    if self.long_control_state == OFF:  # :907-908
      self.reset()
    elif self.long_control_state == STARTING:  # :972
      self.reset()

    # :988-1001 (allow_fast_release consumers of the F14 booleans)
    allow_fast_release = None
    if self.long_control_state != OFF:
      allow_fast_release = (
        not f.force_coast and not stop_request_active and not stop_target_approach_active
        and self.long_control_state in (PID, STARTING) and f.a_target > 0.2 and f.v_ego > 0.12)
      if departing_lead_release and not f.force_coast:
        allow_fast_release = True
      if departing_lead_ready and self.long_control_state == STARTING and not f.force_coast:
        allow_fast_release = True
      if stop_intent_recent and not standstill_recent:
        allow_fast_release = False

    self.last_distance_to_stop_target_m = (  # :1124-1128
      float(control) if control is not None and control > 0.0 else None)

    return {
      "stop_request_active": stop_request_active,
      "state_should_stop": state_should_stop,
      "approach": stop_target_approach_active,
      "carry": stop_target_carry_active,
      "departing_lead_release": departing_lead_release,
      "departing_lead_ready": departing_lead_ready,
      "far_stopped_lead_release": far_stopped_lead_gap_release,
      "legacy_forced": close_stopped_lead_dropout_hold_active or stop_target_release_hold_active or force_coast_standstill_hold,
      "target_distance_m": float(control) if control is not None and control > 0.0 else -1.0,
      "state_dropout_hold": state_dropout_hold_pred,
      "long_control_state": self.long_control_state,
      "time_since_stop_intent_s": tsi_frame,
      "time_since_standstill_s": tss_frame,
      "allow_fast_release": allow_fast_release,
    }


def run_equivalence(frames, CP=None, human_acceleration=True):
  """Drive arbiter + oracle in lockstep, evolving the state machine exactly as WP7's Commit B will."""
  CP = CP or _CP()
  oracle = LegacyArbitrationOracle(CP, human_acceleration=human_acceleration)
  arb = sta.StopTargetArbiter(CP)
  state = OFF
  for i, f in enumerate(frames):
    exp = oracle.update(f)
    dec = arb.update(
      v_ego=f.v_ego, a_ego=f.a_ego, a_target=f.a_target,
      raw_should_stop=f.should_stop, planner_target_m=f.planner_target_m,
      lead_status=f.lead_status, lead_v=f.lead_v, lead_d_rel=f.lead_d_rel,
      increased_stopped_distance_m=0.0,
      brake_pressed=f.brake_pressed, cruise_standstill=f.cruise_standstill, standstill=f.standstill,
      force_coast=f.force_coast, long_control_state=state,
      last_output_accel=f.last_output_accel, dt=f.dt,
      human_acceleration=human_acceleration, v_ego_starting=V_EGO_STARTING,
    )
    ctx = f"frame {i}: {f}"
    assert dec.stop_request_active == exp["stop_request_active"], ctx
    assert dec.state_should_stop == exp["state_should_stop"], ctx
    assert dec.approach_cap_active == exp["approach"], ctx
    assert dec.carry_floor_active == exp["carry"], ctx
    assert dec.departing_lead_release == exp["departing_lead_release"], ctx
    assert dec.departing_lead_ready == exp["departing_lead_ready"], ctx
    assert dec.far_stopped_lead_release == exp["far_stopped_lead_release"], ctx
    assert dec.legacy_forced == exp["legacy_forced"], ctx
    assert dec.target_distance_m == exp["target_distance_m"], ctx
    assert dec.state_dropout_hold == exp["state_dropout_hold"], ctx

    # WP7 Commit B wiring recipe: transition, then apply the post-transition hold
    new_state = long_control_state_trans(
      CP, f.active, state, f.v_ego, dec.state_should_stop, f.brake_pressed, f.cruise_standstill,
      V_EGO_STARTING, a_target=f.a_target, distance_to_stop_target_m=dec.target_distance_m)
    if state == STOPPING and new_state != STOPPING and dec.state_dropout_hold:
      new_state = STOPPING
    assert new_state == exp["long_control_state"], ctx

    # timer consumers (:899-900): standstill timer is read directly, intent timer via the helper
    tsi_k = arb.projected_time_since_stop_intent_s(dec, new_state, f.dt)
    assert tsi_k == exp["time_since_stop_intent_s"], ctx
    assert arb.time_since_standstill_s == exp["time_since_standstill_s"], ctx

    # allow_fast_release replica from the three independent F14 booleans (:988-1001)
    if exp["allow_fast_release"] is not None:
      afr = (not f.force_coast and not dec.stop_request_active and not dec.approach_cap_active
             and new_state in (PID, STARTING) and f.a_target > 0.2 and f.v_ego > 0.12)
      if dec.departing_lead_release and not f.force_coast:
        afr = True
      if dec.departing_lead_ready and new_state == STARTING and not f.force_coast:
        afr = True
      if tsi_k < 1.0 and not arb.time_since_standstill_s < 0.5:
        afr = False
      assert afr == exp["allow_fast_release"], ctx

    state = new_state
    if state in (OFF, STARTING):
      arb.reset()
  return arb


def random_frames(seed: int, n: int, dt: float = DT) -> list[Frame]:
  rng = random.Random(seed)
  v = rng.uniform(0.0, 2.0)
  a_target = -0.3
  target = -1.0
  lead = False
  lead_v = 0.0
  lead_d = 6.0
  should = False
  force_coast = False
  active = True
  last_out = -0.2
  frames = []
  for _ in range(n):
    v = max(0.0, min(3.0, v + rng.uniform(-0.05, 0.045)))
    if rng.random() < 0.03:
      v = 0.0
    standstill = v < 0.01
    a_target = max(-1.5, min(1.0, a_target + rng.uniform(-0.06, 0.06)))
    if rng.random() < 0.06:
      should = not should
    if rng.random() < 0.05:
      target = rng.choice([-1.0, -1.0, rng.uniform(0.05, 4.5)])
    elif target > 0.0:
      target = max(0.05, target - v * dt + rng.uniform(-0.02, 0.02))
    if rng.random() < 0.04:
      lead = not lead
    if lead:
      lead_v = max(0.0, min(2.0, lead_v + rng.uniform(-0.08, 0.09)))
      lead_d = max(0.2, min(12.0, lead_d + (lead_v - v) * dt + rng.uniform(-0.06, 0.06)))
      if rng.random() < 0.02:
        lead_d = rng.uniform(0.5, 9.0)
        lead_v = rng.uniform(0.0, 1.2)
    if rng.random() < 0.02:
      force_coast = not force_coast
    if rng.random() < 0.008:
      active = not active
    last_out = max(-1.5, min(0.5, last_out + rng.uniform(-0.05, 0.05)))
    frames.append(Frame(
      v_ego=v, a_ego=rng.uniform(-1.0, 0.5), a_target=a_target, should_stop=should,
      planner_target_m=target, lead_status=lead, lead_v=lead_v, lead_d_rel=lead_d if lead else 0.0,
      brake_pressed=rng.random() < 0.01, cruise_standstill=standstill and rng.random() < 0.3,
      standstill=standstill, force_coast=force_coast, active=active,
      last_output_accel=last_out, dt=dt,
    ))
  return frames


# --- verbatim-port AST equality --------------------------------------------------------------------

PORTED_FUNCTIONS = [
  "has_explicit_stop_target",
  "should_enter_stop_target_mode",
  "should_hold_stop_target_mode",
  "should_apply_stop_target_approach_mode",
  "should_apply_stop_target_carry_mode",
  "should_release_far_stopped_lead_gap",
  "should_hold_recent_close_stopped_lead_dropout",
  "should_apply_low_speed_stopped_lead_glide_accel_cap",
  "should_apply_stop_entry_handoff_soften",
  "stop_entry_handoff_accel_cap",
  "should_hold_stop_target_dropout",
  "should_hold_no_target_standstill_dropout",
  "should_hold_low_speed_stop_target_release",
]

PORTED_CONSTANTS = [
  "MIN_STOP_TARGET_MODE_DISTANCE_M",
  "MAX_STOP_TARGET_MODE_DISTANCE_M",
  "LEAD_FOLLOW_TARGET_HOLD_GAP_M",
  "FAR_STOPPED_LEAD_CRAWL_GAP_M",
  "FAR_STOPPED_LEAD_CLOSE_TARGET_HOLD_M",
]


def _module_functions(path: pathlib.Path) -> dict[str, ast.FunctionDef]:
  tree = ast.parse(path.read_text())
  return {node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)}


def _module_constants(path: pathlib.Path) -> dict[str, object]:
  tree = ast.parse(path.read_text())
  out = {}
  for node in tree.body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
      try:
        out[node.targets[0].id] = ast.literal_eval(node.value)
      except ValueError:
        pass
  return out


class TestVerbatimPort:
  @pytest.mark.parametrize("name", PORTED_FUNCTIONS)
  def test_function_ast_identical_to_longcontrol(self, name):
    legacy = _module_functions(LONGCONTROL_PATH)
    if name not in legacy:
      pytest.skip(f"{name} no longer defined in longcontrol.py (moved to the arbiter at Commit B)")
    ported = _module_functions(ARBITER_PATH)
    assert name in ported, f"arbiter module must port {name}"
    assert ast.unparse(ported[name]) == ast.unparse(legacy[name]), f"{name} diverged from the longcontrol.py source"

  @pytest.mark.parametrize("name", PORTED_CONSTANTS)
  def test_constant_matches_longcontrol(self, name):
    legacy = _module_constants(LONGCONTROL_PATH)
    if name not in legacy:
      pytest.skip(f"{name} no longer defined in longcontrol.py (moved to the arbiter at Commit B)")
    assert getattr(sta, name) == legacy[name]

  def test_constants_consistent_with_stopping_params(self):
    # one definition site (spec section 3): the verbatim literals must agree with the registry
    assert sta.FAR_STOPPED_LEAD_CRAWL_GAP_M == STOPPING_PARAMS.FAR_CRAWL_GAP_M
    assert sta.LEAD_FOLLOW_TARGET_HOLD_GAP_M == STOPPING_PARAMS.TARGET_HOLD_GAP_M
    assert STOPPING_PARAMS.HOLD_ESCAPE_A_TARGET == 0.12     # longcontrol.py:475
    assert STOPPING_PARAMS.HOLD_ESCAPE_LAST_OUTPUT == -0.08  # longcontrol.py:471
    assert STOPPING_PARAMS.T_STOP_INTENT_HOLD_S == 0.4
    assert STOPPING_PARAMS.T_STOP_INTENT_HOLD_STANDSTILL_S == 1.4


# --- frame-level equivalence vs the legacy oracle --------------------------------------------------


class TestLegacyEquivalence:
  def test_randomized_traces_100hz(self):
    run_equivalence(random_frames(seed=1, n=4000, dt=0.01))
    run_equivalence(random_frames(seed=2, n=4000, dt=0.01))

  def test_randomized_traces_10hz(self):
    run_equivalence(random_frames(seed=3, n=1500, dt=0.1))

  def test_randomized_traces_no_human_acceleration(self):
    run_equivalence(random_frames(seed=4, n=2000, dt=0.01), human_acceleration=False)

  def test_randomized_traces_non_santa_fe(self):
    # quirk layer off: no synthetic target, no far release, no close-lead hold
    run_equivalence(random_frames(seed=5, n=2000, dt=0.01), CP=_CP("TOYOTA_COROLLA"))

  def test_scripted_stop_dropout_reacquire_departure(self):
    frames = []
    # approach a stop: rolling, planner shouldStop
    v = 1.4
    for _ in range(40):
      v = max(0.0, v - 0.03)
      frames.append(Frame(v_ego=v, a_ego=-0.4, a_target=-0.5, should_stop=True, last_output_accel=-0.45, dt=0.05))
    # standstill with intent
    for _ in range(20):
      frames.append(Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True, last_output_accel=-0.15, dt=0.05))
    # dropout: intent lost at standstill (radar/planner flicker)
    for _ in range(20):
      frames.append(Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False, last_output_accel=-0.15, dt=0.05))
    # reacquire
    for _ in range(10):
      frames.append(Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True, last_output_accel=-0.15, dt=0.05))
    # green light: a_target rises, output releases
    for _ in range(30):
      frames.append(Frame(v_ego=0.0, standstill=True, a_target=0.5, should_stop=False, last_output_accel=-0.05, dt=0.05))
    run_equivalence(frames)

  def test_scripted_stopped_lead_glide(self):
    frames = []
    v, gap = 1.2, 7.0
    for _ in range(120):
      v = max(0.0, v - 0.012)
      gap = max(2.8, gap - v * 0.05)
      frames.append(Frame(v_ego=v, a_ego=-0.3, a_target=-0.35, should_stop=v < 0.25,
                          lead_status=True, lead_v=0.0, lead_d_rel=gap, standstill=v < 0.01,
                          last_output_accel=-0.3, dt=0.05))
    # lead departs from standstill
    lead_v = 0.0
    for _ in range(40):
      lead_v = min(2.0, lead_v + 0.08)
      gap = gap + lead_v * 0.05
      frames.append(Frame(v_ego=0.0, standstill=True, a_target=0.3, should_stop=True,
                          lead_status=True, lead_v=lead_v, lead_d_rel=gap, last_output_accel=-0.12, dt=0.05))
    run_equivalence(frames)


def test_stop_scenarios_fixture_equivalence():
  scenarios = pytest.importorskip(
    "openpilot.selfdrive.controls.lib.tests.stop_scenarios",
    reason="stop_scenarios.py is WP5's first deliverable; equivalence runs on it once it lands")
  fixture_lists = []
  for value in vars(scenarios).values():
    if isinstance(value, (list, tuple)) and value and all(hasattr(s, "v_ego") and hasattr(s, "a_ego") for s in value):
      fixture_lists.append(value)
  if not fixture_lists:
    pytest.skip("no FakeSample-style fixture lists found in stop_scenarios")
  for samples in fixture_lists:
    frames = []
    last_out = -0.12
    for idx, s in enumerate(samples):
      if idx + 1 < len(samples):
        dt = max(float(samples[idx + 1].t - s.t), 0.01)
      else:
        dt = 0.1
      raw = getattr(s, "raw_should_stop", None)
      should = bool(raw) if raw is not None else bool(getattr(s, "should_stop", False))
      target = getattr(s, "distance_to_stop_target_m", None)
      lead_d = getattr(s, "lead_d_rel_m", None)
      a_cmd = getattr(s, "accel_cmd", None)
      frames.append(Frame(
        v_ego=float(s.v_ego), a_ego=float(s.a_ego), a_target=float(a_cmd) if a_cmd is not None else -0.3,
        should_stop=should, planner_target_m=float(target) if target is not None else -1.0,
        lead_status=bool(getattr(s, "lead_status", False)), lead_v=float(getattr(s, "lead_v", 0.0)),
        lead_d_rel=float(lead_d) if lead_d is not None else 0.0,
        standstill=float(s.v_ego) < 0.01, last_output_accel=last_out, dt=dt,
      ))
      last_out = float(a_cmd) if a_cmd is not None else last_out
    run_equivalence(frames)


# --- consolidated dropout holds (shadow-only) -------------------------------------------------------


def _arm_rolling(arb, v=0.5, n=10, dt=0.05):
  state = STOPPING
  for _ in range(n):
    step(arb, state, Frame(v_ego=v, a_target=-0.4, should_stop=True, last_output_accel=-0.4, dt=dt))
  return state


class TestConsolidatedDropoutHolds:
  def test_rolling_dropout_holds_0p4s_then_releases(self):
    arb = sta.StopTargetArbiter(_CP())
    _arm_rolling(arb, v=0.5)
    # drop intent while rolling; feed PID state so nothing self-sustains the legacy timer.
    # time since the drop is i*dt (0 on the falling-edge frame, legacy timer semantics)
    dt = 0.05
    for i in range(20):
      dec = step(arb, PID, Frame(v_ego=0.5, a_target=-0.3, should_stop=False, last_output_accel=-0.3, dt=dt))
      # shadow-only: the authoritative decision must NOT be held by the consolidated mechanism
      assert not dec.stop_request_active
      if i * dt <= STOPPING_PARAMS.T_STOP_INTENT_HOLD_S:
        assert arb.consolidated_hold_active, f"i={i}"
        assert arb.consolidated_hold_source == sta.StopSource.DROPOUT_HOLD
      else:
        assert not arb.consolidated_hold_active, f"i={i}"

  def test_release_hold_extends_to_0p8s_with_target(self):
    arb = sta.StopTargetArbiter(_CP())
    state = STOPPING
    dt = 0.05
    for _ in range(10):
      step(arb, state, Frame(v_ego=0.1, a_target=-0.3, should_stop=True, planner_target_m=0.6, last_output_accel=-0.4, dt=dt))
    # intent drops; explicit target stays present briefly, then disappears at 0.5 s
    for i in range(20):
      target = 0.6 if i < 10 else -1.0
      dec = step(arb, PID, Frame(v_ego=0.1, a_target=0.05, should_stop=False, planner_target_m=target, last_output_accel=-0.4, dt=dt))
      t = i * dt
      if t <= sta.RELEASE_HOLD_WINDOW_S - 1e-9:
        assert arb.consolidated_hold_active, f"i={i}"
        assert arb.consolidated_hold_source == sta.StopSource.DROPOUT_HOLD
      elif t >= sta.RELEASE_HOLD_WINDOW_S + 1e-9:
        assert not arb.consolidated_hold_active, f"i={i}"
      # the exact boundary frame is unspecified (float accumulation of dt)
      assert not dec.departing_lead_release

  def test_release_hold_unbounded_while_target_present(self):
    arb = sta.StopTargetArbiter(_CP())
    dt = 0.05
    for _ in range(10):
      step(arb, STOPPING, Frame(v_ego=0.1, a_target=-0.3, should_stop=True, planner_target_m=0.6, last_output_accel=-0.4, dt=dt))
    for _ in range(40):  # 2.0 s >> 0.8 s, target still present
      step(arb, PID, Frame(v_ego=0.1, a_target=0.05, should_stop=False, planner_target_m=0.6, last_output_accel=-0.4, dt=dt))
      assert arb.consolidated_hold_active

  def test_standstill_dropout_window_binds_outside_stopping_state(self):
    # with the state machine NOT pinning the legacy intent timer, the 1.4 s window is real time
    arb = sta.StopTargetArbiter(_CP())
    dt = 0.05
    for _ in range(10):
      step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True, last_output_accel=-0.15, dt=dt))
    for i in range(40):
      step(arb, PID, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False, last_output_accel=-0.15, dt=dt))
      # the standstill window compares the legacy intent timer, which is i*dt here (PID state)
      if i * dt <= STOPPING_PARAMS.T_STOP_INTENT_HOLD_STANDSTILL_S:
        assert arb.consolidated_hold_active, f"i={i}"
      else:
        assert not arb.consolidated_hold_active, f"i={i}"

  def test_standstill_dropout_self_sustains_while_state_pinned(self):
    # wired as on-car: the legacy hold keeps the state in stopping, the intent timer stays 0,
    # and the hold persists until an escape -- the consolidated hold must cover every frame
    arb = sta.StopTargetArbiter(_CP())
    dt = 0.05
    for _ in range(10):
      step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True, last_output_accel=-0.15, dt=dt))
    for _ in range(60):  # 3.0 s >> 1.4 s
      dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False, last_output_accel=-0.15, dt=dt))
      assert dec.state_dropout_hold     # legacy authoritative hold
      assert arb.consolidated_hold_active  # shadow covers it
    assert arb.hold_divergence == 0

  @pytest.mark.parametrize("escape", ["a_target", "last_output"])
  def test_standstill_dropout_escape_terms(self, escape):
    arb = sta.StopTargetArbiter(_CP())
    dt = 0.05
    for _ in range(10):
      step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True, last_output_accel=-0.15, dt=dt))
    for _ in range(5):
      dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False, last_output_accel=-0.15, dt=dt))
      assert dec.state_dropout_hold and arb.consolidated_hold_active
    a_target = 0.3 if escape == "a_target" else 0.0
    last_out = -0.15 if escape == "a_target" else -0.05
    dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=a_target, should_stop=False, last_output_accel=last_out, dt=dt))
    assert not dec.state_dropout_hold   # legacy escape (a_target > 0.12 / last_output_accel > -0.08)
    assert not arb.consolidated_hold_active  # consolidated escape mirrors it
    assert arb.hold_divergence == 0

  def test_early_release_on_rising_v_and_a_target(self):
    arb = sta.StopTargetArbiter(_CP())
    _arm_rolling(arb, v=0.5)
    step(arb, PID, Frame(v_ego=0.5, a_target=-0.3, should_stop=False, last_output_accel=-0.3, dt=0.05))
    assert arb.consolidated_hold_active
    step(arb, PID, Frame(v_ego=0.5, a_target=0.3, should_stop=False, last_output_accel=-0.3, dt=0.05))
    assert not arb.consolidated_hold_active  # v >= 0.30 with a_target > 0.2


class TestStoppedLeadLatch:
  def _arm_latch(self, arb, n=10, dt=0.05):
    for _ in range(n):
      step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True,
                                lead_status=True, lead_v=0.0, lead_d_rel=3.0, last_output_accel=-0.15, dt=dt))

  def test_latch_unbounded_while_stopped_lead_remains(self):
    arb = sta.StopTargetArbiter(_CP())
    self._arm_latch(arb)
    for _ in range(80):  # 4.0 s >> any timed window
      dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False,
                                      lead_status=True, lead_v=0.0, lead_d_rel=3.0, last_output_accel=-0.15, dt=0.05))
      assert dec.stop_request_active           # legacy close-stopped-lead hold forces stop intent
      assert dec.source == sta.StopSource.STOPPED_LEAD_LATCH
      assert dec.legacy_forced
      assert arb.consolidated_hold_active
      assert arb.consolidated_hold_source == sta.StopSource.STOPPED_LEAD_LATCH
    assert arb.hold_divergence == 0
    assert arb.legacy_hold_fired == 1

  def test_latch_releases_on_departing_lead(self):
    arb = sta.StopTargetArbiter(_CP())
    self._arm_latch(arb)
    for _ in range(10):
      step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False,
                                lead_status=True, lead_v=0.0, lead_d_rel=3.0, last_output_accel=-0.15, dt=0.05))
    # lead drives away: the latch (close hold) releases, but the legacy no-target standstill
    # hold takes over until the planner reacts (a_target rise) -- the consolidated mechanism
    # must arm its standstill window on the latch falling edge and keep covering
    for _ in range(5):
      dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False,
                                      lead_status=True, lead_v=1.5, lead_d_rel=6.0, last_output_accel=-0.15, dt=0.05))
      assert not dec.stop_request_active   # the close hold no longer forces stop intent
      assert dec.state_dropout_hold        # but the standstill dropout hold pins the state
      assert dec.source == sta.StopSource.DROPOUT_HOLD
      assert arb.consolidated_hold_active
    # green light: planner accelerates -> escape releases both legacy and consolidated holds
    dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.5, should_stop=False,
                                    lead_status=True, lead_v=1.8, lead_d_rel=7.0, last_output_accel=-0.15, dt=0.05))
    assert not dec.stop_request_active
    assert not dec.state_should_stop
    assert not dec.state_dropout_hold
    assert dec.source == sta.StopSource.NONE
    assert not arb.consolidated_hold_active
    assert arb.hold_divergence == 0
    assert arb.single_hold_covered == 1

  def test_latch_releases_on_brake_press_without_divergence(self):
    arb = sta.StopTargetArbiter(_CP())
    self._arm_latch(arb)
    dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False, brake_pressed=True,
                                    lead_status=True, lead_v=0.0, lead_d_rel=3.0, last_output_accel=-0.15, dt=0.05))
    assert dec.stop_request_active  # legacy predicate ignores brake (disengage handles it, spec 5.5.6)
    assert not arb.consolidated_hold_active  # latch releases on brake press (spec 5.2.5)
    assert arb.hold_divergence == 0  # brake frames excluded from divergence counting

  def test_latch_covers_creep_push_with_close_stopped_lead(self):
    # creep-push to 0.4 m/s with a stopped lead 3.5 m ahead (outside the synthetic-target
    # trigger gap) and the planner wanting to go: the legacy close hold still fires (it has
    # no a_target term) and the latch must cover it despite the v/a_target early-release pair
    arb = sta.StopTargetArbiter(_CP())
    self._arm_latch(arb)
    for _ in range(10):
      dec = step(arb, STOPPING, Frame(v_ego=0.4, standstill=False, a_target=0.3, should_stop=False,
                                      lead_status=True, lead_v=0.0, lead_d_rel=3.5, last_output_accel=-0.15, dt=0.05))
      assert dec.stop_request_active and dec.source == sta.StopSource.STOPPED_LEAD_LATCH
      assert arb.consolidated_hold_active
    assert arb.hold_divergence == 0


class TestDivergenceCounters:
  def test_covered_episode_counts(self):
    arb = sta.StopTargetArbiter(_CP())
    dt = 0.05
    for _ in range(10):
      step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True, last_output_accel=-0.15, dt=dt))
    for _ in range(10):
      step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False, last_output_accel=-0.15, dt=dt))
    # escape ends the episode
    step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.5, should_stop=False, last_output_accel=-0.15, dt=dt))
    assert arb.legacy_hold_fired == 1
    assert arb.single_hold_covered == 1
    assert arb.hold_divergence == 0

  def test_uncovered_frames_count_as_divergence(self):
    arb = sta.StopTargetArbiter(_CP())
    dt = 0.05
    for _ in range(10):
      step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True, last_output_accel=-0.15, dt=dt))
    # white-box: sabotage the consolidated mechanism so the legacy hold runs uncovered
    for i in range(5):
      arb._drop_kind = None
      arb._had_primary_intent = False
      dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False, last_output_accel=-0.15, dt=dt))
      assert dec.state_dropout_hold
      assert arb.hold_divergence == i + 1
    step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.5, should_stop=False, last_output_accel=-0.15, dt=dt))
    assert arb.legacy_hold_fired == 1
    assert arb.single_hold_covered == 0
    assert arb.hold_divergence == 5

  def test_counters_survive_reset(self):
    arb = sta.StopTargetArbiter(_CP())
    arb.legacy_hold_fired = 3
    arb.single_hold_covered = 2
    arb.hold_divergence = 7
    arb.reset()
    assert (arb.legacy_hold_fired, arb.single_hold_covered, arb.hold_divergence) == (3, 2, 7)


# --- F14 release booleans, F32 tiering, sources, sentinel, ISD --------------------------------------


class TestReleaseBooleans:
  def test_departing_and_far_release_simultaneously_true(self):
    arb = sta.StopTargetArbiter(_CP())
    # slow-departing lead at gap 6.0 m from standstill: lead_v=0.5 satisfies BOTH the departing
    # predicate (gap > ~5.44 m at that departure speed) and the far-stopped predicate (<= 0.65)
    dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.1, should_stop=True,
                                    lead_status=True, lead_v=0.5, lead_d_rel=6.0, last_output_accel=-0.15))
    assert dec.departing_lead_ready
    assert dec.departing_lead_release
    assert dec.far_stopped_lead_release
    assert not dec.stop_request_active
    assert not dec.state_should_stop
    assert dec.source == sta.StopSource.NONE
    assert dec.release_reason == "departing_lead"  # derived telemetry, priority order documented

  def test_far_release_alone(self):
    arb = sta.StopTargetArbiter(_CP())
    # nearly stopped far lead, ego crawling: far release fires, departing does not (lead too slow)
    dec = step(arb, STOPPING, Frame(v_ego=0.1, a_target=-0.1, should_stop=True,
                                    lead_status=True, lead_v=0.1, lead_d_rel=6.0, last_output_accel=-0.2))
    assert dec.far_stopped_lead_release
    assert not dec.departing_lead_ready
    assert not dec.departing_lead_release
    assert dec.release_reason == "far_stopped_lead"

  def test_departing_ready_without_release(self):
    arb = sta.StopTargetArbiter(_CP())
    # ready is the raw predicate; release additionally requires raw shouldStop (legacy :774)
    dec = step(arb, STARTING, Frame(v_ego=0.0, standstill=True, a_target=0.3, should_stop=False,
                                    lead_status=True, lead_v=1.5, lead_d_rel=6.5, last_output_accel=0.0))
    assert dec.departing_lead_ready
    assert not dec.departing_lead_release


class TestIntentTiering:
  @staticmethod
  def _tiering(dec):
    # spec 5.2.5 intent tiering, consumed by the WP5 tracker
    return dec.stop_request_active and (dec.source != sta.StopSource.DROPOUT_HOLD or dec.legacy_forced)

  def test_stopped_lead_latch_is_full_intent(self):
    arb = sta.StopTargetArbiter(_CP())
    for _ in range(10):
      dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True,
                                      lead_status=True, lead_v=0.0, lead_d_rel=3.0, last_output_accel=-0.15))
    dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False,
                                    lead_status=True, lead_v=0.0, lead_d_rel=3.0, last_output_accel=-0.15))
    assert dec.source == sta.StopSource.STOPPED_LEAD_LATCH and dec.legacy_forced
    assert self._tiering(dec)

  def test_stop_target_release_hold_is_full_intent(self):
    arb = sta.StopTargetArbiter(_CP())
    for _ in range(10):
      step(arb, STOPPING, Frame(v_ego=0.1, a_target=-0.3, should_stop=True, planner_target_m=0.6, last_output_accel=-0.4))
    dec = step(arb, STOPPING, Frame(v_ego=0.1, a_target=0.05, should_stop=False, planner_target_m=0.6, last_output_accel=-0.4))
    assert dec.stop_request_active and dec.source == sta.StopSource.DROPOUT_HOLD and dec.legacy_forced
    assert self._tiering(dec)

  def test_controller_internal_dropout_hold_is_excluded(self):
    # the controller tail_commit latch is never longcontrol-forced: legacy_forced stays False
    dec = sta.StopDecision(
      stop_request_active=True, state_should_stop=True, target_distance_m=-1.0,
      source=sta.StopSource.DROPOUT_HOLD, approach_cap_active=False, carry_floor_active=False,
      departing_lead_release=False, departing_lead_ready=False, far_stopped_lead_release=False,
      legacy_forced=False, release_reason="")
    assert not self._tiering(dec)


class TestSourcesAndSentinel:
  def test_no_target_sentinel(self):
    arb = sta.StopTargetArbiter(_CP())
    dec = step(arb, STOPPING, Frame(v_ego=0.5, a_target=-0.3, should_stop=True, planner_target_m=-1.0))
    assert dec.target_distance_m == -1.0
    assert dec.source == sta.StopSource.PLANNER

  def test_explicit_target_source(self):
    arb = sta.StopTargetArbiter(_CP())
    dec = step(arb, STOPPING, Frame(v_ego=0.5, a_target=-0.3, should_stop=False, planner_target_m=0.4))
    assert dec.source == sta.StopSource.EXPLICIT_TARGET
    assert dec.target_distance_m == 0.4
    assert dec.stop_request_active

  def test_stopped_lead_source_and_min_merge(self):
    arb = sta.StopTargetArbiter(_CP())
    # synthetic target = lead_d_rel - 2.75 = 0.25; planner target farther at 2.0 -> min() wins
    dec = step(arb, STOPPING, Frame(v_ego=0.5, a_target=-0.04, should_stop=False, planner_target_m=2.0,
                                    lead_status=True, lead_v=0.0, lead_d_rel=3.0))
    assert dec.source == sta.StopSource.STOPPED_LEAD
    assert dec.target_distance_m == pytest.approx(0.25)
    assert dec.stop_request_active

  def test_force_coast_standstill_source(self):
    arb = sta.StopTargetArbiter(_CP())
    dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False, force_coast=True))
    assert dec.source == sta.StopSource.FORCE_COAST_STANDSTILL
    assert dec.state_should_stop
    assert not dec.stop_request_active
    assert dec.legacy_forced

  def test_planner_priority_over_holds(self):
    arb = sta.StopTargetArbiter(_CP())
    dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True,
                                    lead_status=True, lead_v=0.0, lead_d_rel=3.0, last_output_accel=-0.15))
    assert dec.source == sta.StopSource.PLANNER


class TestIsdInvariance:
  def test_decision_independent_of_isd_kwarg(self):
    # spec 4.2.4: compensation happens upstream in LongControl.update (lead_d_rel arrives
    # already-effective); the arbiter must not consume the ISD kwarg in v1
    frames = random_frames(seed=7, n=800, dt=0.01)
    arb_a = sta.StopTargetArbiter(_CP())
    arb_b = sta.StopTargetArbiter(_CP())
    state = STOPPING
    for f in frames:
      dec_a = arb_a.update(
        v_ego=f.v_ego, a_ego=f.a_ego, a_target=f.a_target, raw_should_stop=f.should_stop,
        planner_target_m=f.planner_target_m, lead_status=f.lead_status, lead_v=f.lead_v,
        lead_d_rel=f.lead_d_rel, increased_stopped_distance_m=0.0, brake_pressed=f.brake_pressed,
        cruise_standstill=f.cruise_standstill, standstill=f.standstill, force_coast=f.force_coast,
        long_control_state=state, last_output_accel=f.last_output_accel, dt=f.dt,
        human_acceleration=True, v_ego_starting=V_EGO_STARTING)
      dec_b = arb_b.update(
        v_ego=f.v_ego, a_ego=f.a_ego, a_target=f.a_target, raw_should_stop=f.should_stop,
        planner_target_m=f.planner_target_m, lead_status=f.lead_status, lead_v=f.lead_v,
        lead_d_rel=f.lead_d_rel, increased_stopped_distance_m=3.0, brake_pressed=f.brake_pressed,
        cruise_standstill=f.cruise_standstill, standstill=f.standstill, force_coast=f.force_coast,
        long_control_state=state, last_output_accel=f.last_output_accel, dt=f.dt,
        human_acceleration=True, v_ego_starting=V_EGO_STARTING)
      assert dec_a == dec_b


class TestResetSemantics:
  def test_reset_clears_holds_and_timers(self):
    arb = sta.StopTargetArbiter(_CP())
    _arm_rolling(arb, v=0.5)
    step(arb, PID, Frame(v_ego=0.5, a_target=-0.3, should_stop=False, last_output_accel=-0.3, dt=0.05))
    assert arb.consolidated_hold_active
    arb.reset()
    assert not arb.consolidated_hold_active
    assert arb.time_since_standstill_s == 10.0
    assert arb.time_since_stop_intent_s == 10.0
    dec = step(arb, PID, Frame(v_ego=0.5, a_target=-0.3, should_stop=False, last_output_accel=-0.3, dt=0.05))
    assert not arb.consolidated_hold_active
    assert dec.source == sta.StopSource.NONE

  def test_last_output_accel_is_previous_frame_semantics(self):
    # the standstill dropout hold reads the PREVIOUS frame's output: passing a released value
    # (> -0.08) must drop the hold even though everything else is unchanged
    arb = sta.StopTargetArbiter(_CP())
    for _ in range(10):
      step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True, last_output_accel=-0.15))
    dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False, last_output_accel=-0.15))
    assert dec.state_dropout_hold
    dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False, last_output_accel=-0.05))
    assert not dec.state_dropout_hold


class TestPromotedConsolidatedHolds:
  def test_promoted_latch_forces_full_intent(self, monkeypatch):
    monkeypatch.setattr(sta, "ARBITER_LEGACY_DROPOUT_HOLDS", False)
    arb = sta.StopTargetArbiter(_CP())
    for _ in range(10):
      step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True,
                                lead_status=True, lead_v=0.0, lead_d_rel=3.0, last_output_accel=-0.15))
    for _ in range(60):
      dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=0.0, should_stop=False,
                                      lead_status=True, lead_v=0.0, lead_d_rel=3.0, last_output_accel=-0.15))
      assert dec.stop_request_active and dec.state_should_stop and dec.state_dropout_hold
      assert dec.source == sta.StopSource.STOPPED_LEAD_LATCH
      assert dec.legacy_forced

  def test_promoted_rolling_hold_bounded(self, monkeypatch):
    monkeypatch.setattr(sta, "ARBITER_LEGACY_DROPOUT_HOLDS", False)
    arb = sta.StopTargetArbiter(_CP())
    _arm_rolling(arb, v=0.5)
    dt = 0.05
    for i in range(20):
      dec = step(arb, PID, Frame(v_ego=0.5, a_target=-0.3, should_stop=False, last_output_accel=-0.3, dt=dt))
      if i * dt <= STOPPING_PARAMS.T_STOP_INTENT_HOLD_S:
        assert dec.stop_request_active and dec.source == sta.StopSource.DROPOUT_HOLD, f"i={i}"
      else:
        assert not dec.stop_request_active, f"i={i}"


# --- AST guards (F2, F4) ----------------------------------------------------------------------------

# spec 4.2.4 named consumer allowlist: the arbiter call site, the stopping-controller update call
# (the legacy controller's in-layer far_stopped_lead_release predicate is distance-tuned,
# stopping_controller.py:685-692, G11 row 32) + the kept-verbatim Santa Fe quirk-layer call sites
# tuned against the mutated dRel. Moving-lead consumers (e.g. the experimental close-lead cap)
# see the TRUE gap and are deliberately NOT listed.
LEAD_D_REL_EFF_ALLOWED_CALLEES = {
  "self.arbiter.update",
  "self.stopping_controller.update",
  "low_speed_close_lead_accel_cap",
  "low_speed_close_lead_brake_step",
  "low_speed_stopped_lead_glide_accel_cap",
  "far_stopped_lead_crawl_accel_cap",
  "far_stopped_lead_brake_floor",
  "far_stopped_lead_settle_accel_cap",
  "should_release_far_stopped_lead_gap",
  "should_hold_recent_close_stopped_lead_dropout",
  "get_stopped_lead_control_target",
  "should_release_stop_hold_for_departing_lead",
  # stopping-phase planner-aTarget floor (incident 0000173c seg24): its small-gap gate is a
  # Santa-Fe stopping-layer consumer tuned against the same effective (ISD-compensated) gap.
  "stopping_planner_floor_active",
  # close-the-gap forward creep behind a confirmed stopped lead (route 00001764 seg27): Santa-Fe
  # stopping-layer consumers gated against the same effective (ISD-compensated) gap.
  "stopping_close_gap_creep_should_arm",
  "stopping_close_gap_creep_should_disarm",
  "stopping_close_gap_creep_accel_target",
}


def _callee_repr(func) -> str:
  parts = []
  node = func
  while isinstance(node, ast.Attribute):
    parts.append(node.attr)
    node = node.value
  if isinstance(node, ast.Name):
    parts.append(node.id)
  return ".".join(reversed(parts))


def _annotate_parents(tree):
  for node in ast.walk(tree):
    for child in ast.iter_child_nodes(node):
      child._wp4_parent = node


class TestLeadDRelEffAstGuard:
  def test_name_confined_to_longcontrol(self):
    # identifier-level scan (docstrings/comments may explain the contract; code may not use it)
    offenders = []
    for path in (REPO_ROOT / "selfdrive").rglob("*.py"):
      rel = path.relative_to(REPO_ROOT).as_posix()
      if "/tests/" in rel or path.name.startswith("test_"):
        continue
      if path == LONGCONTROL_PATH:
        continue
      source = path.read_text(errors="ignore")
      if "lead_d_rel_eff" not in source:
        continue
      try:
        tree = ast.parse(source)
      except SyntaxError:
        continue
      for node in ast.walk(tree):
        if (isinstance(node, ast.Name) and node.id == "lead_d_rel_eff") or (isinstance(node, ast.arg) and node.arg == "lead_d_rel_eff"):
          offenders.append(f"{rel}:{node.lineno}")
    assert not offenders, f"the lead_d_rel_eff identifier may exist only in longcontrol.py (spec 4.2.4): {offenders}"

  def test_single_producer_and_allowlisted_consumers(self):
    source = LONGCONTROL_PATH.read_text()
    if "lead_d_rel_eff" not in source:
      pytest.skip("lead_d_rel_eff not yet introduced (lands with longcontrol Commit B); guard activates then")
    tree = ast.parse(source)
    _annotate_parents(tree)

    # producers: exactly one assignment, inside LongControl.update
    producers = []
    for node in ast.walk(tree):
      targets = []
      if isinstance(node, ast.Assign):
        targets = node.targets
      elif isinstance(node, (ast.AnnAssign, ast.AugAssign, ast.NamedExpr)):
        targets = [node.target]
      for t in targets:
        for name in ast.walk(t):
          if isinstance(name, ast.Name) and name.id == "lead_d_rel_eff":
            producers.append(node)
    assert len(producers) == 1, f"lead_d_rel_eff must have exactly one producer, found {len(producers)}"

    def enclosing(node, kind):
      while node is not None:
        if isinstance(node, kind):
          return node
        node = getattr(node, "_wp4_parent", None)
      return None

    func = enclosing(producers[0], (ast.FunctionDef, ast.AsyncFunctionDef))
    cls = enclosing(func, ast.ClassDef) if func is not None else None
    assert func is not None and func.name == "update" and cls is not None and cls.name == "LongControl", \
      "the single lead_d_rel_eff producer must be LongControl.update (spec 4.2.4)"

    # consumers: every load must sit inside a call to an allowlisted callee
    for node in ast.walk(tree):
      if isinstance(node, ast.Name) and node.id == "lead_d_rel_eff" and isinstance(node.ctx, ast.Load):
        call = enclosing(node, ast.Call)
        callee = _callee_repr(call.func) if call is not None else None
        assert call is not None and callee in LEAD_D_REL_EFF_ALLOWED_CALLEES, \
          f"lead_d_rel_eff consumed outside the named allowlist (callee={callee}, line {node.lineno})"


class TestFacadeNeverImportsArbiter:
  def test_no_arbiter_import_in_facade(self):
    # spec section 2 / F2: a second, facade-internal arbiter is FORBIDDEN; the facade consumes the
    # StopDecision longcontrol computed. Importing the StopDecision/StopSource DATA types is allowed.
    if not FACADE_PATH.exists():
      pytest.skip("stopping_controller_v2.py not yet created (WP5); guard activates then")
    source = FACADE_PATH.read_text()
    assert "StopTargetArbiter" not in source, "the facade must never reference StopTargetArbiter (F2)"
    tree = ast.parse(source)
    for node in ast.walk(tree):
      if isinstance(node, ast.Import):
        for alias in node.names:
          assert "stop_target_arbiter" not in alias.name, \
            "module import of stop_target_arbiter gives the facade access to the arbiter class (F2)"
      elif isinstance(node, ast.ImportFrom):
        if node.module and "stop_target_arbiter" in node.module:
          imported = {alias.name for alias in node.names}
          assert imported <= {"StopDecision", "StopSource"}, \
            f"facade may import only the StopDecision/StopSource data types, got {imported}"


class TestSpecInterface:
  def test_stop_source_values(self):
    assert [s.value for s in sta.StopSource] == [0, 1, 2, 3, 4, 5, 6]
    assert sta.StopSource.STOPPED_LEAD_LATCH == 6

  def test_stop_decision_fields_match_spec(self):
    spec_fields = [
      "stop_request_active", "state_should_stop", "target_distance_m", "source",
      "approach_cap_active", "carry_floor_active", "departing_lead_release",
      "departing_lead_ready", "far_stopped_lead_release", "legacy_forced", "release_reason",
    ]
    actual = list(sta.StopDecision.__dataclass_fields__)
    assert actual[:len(spec_fields)] == spec_fields
    assert actual[len(spec_fields):] == ["state_dropout_hold"]  # documented WP4 extension

  def test_legacy_dropout_holds_default_true(self):
    assert sta.ARBITER_LEGACY_DROPOUT_HOLDS is True


# --- stop hold vs planner go-signal precedence (deliberate semantics, no code change) ---------------


class TestHoldVsPlannerGoSemantics:
  """Regression pin for the driveway conflict (route 00001702--dcdc5c3eea--0, engagement 1).

  Observed on-vehicle: planner e2e go-signal (shouldStop flipped False at engage+0.26 s, aTarget
  ramping to +1.45 m/s2) against a stationary radar lead at 2.19 m (inside the 2.75 m
  STOPPED_LEAD_MIN_CONTROL_GAP_M close-hold gap; planner distanceToStopTarget pinned at the
  0.05 m STOP_TARGET_CLOSE_HOLD_REMAINING_M floor). The close-stopped-lead hold
  (should_hold_recent_close_stopped_lead_dropout, surfaced as STOPPED_LEAD_LATCH) kept full stop
  intent and the brake stayed held until the driver brake-pressed. DELIBERATE defensive
  semantics: the hold has no a_target escape by design -- a planner go must never launch into a
  radar-confirmed stationary obstacle closer than the rest gap; driver gas/brake override is the
  escape. These tests pin BOTH directions: the hold must keep holding (obstacle inside the hold
  gap), and every release path must stay bounded so the defensive case can never regress into
  "release never fires" for a genuinely departing or lost lead.
  """

  def _driveway_frame(self, should_stop, a_target, lead_status=True, last_output_accel=-0.5):
    # rlog facts at engage t=20.29 s: vEgo ~0.05 creep with standstill=1, lead 2.19 m / vLead ~0.03,
    # planner target pinned at the 0.05 m floor, ISD 0.
    return Frame(v_ego=0.05, a_ego=0.0, a_target=a_target, should_stop=should_stop,
                 planner_target_m=STOP_TARGET_CLOSE_HOLD_REMAINING_M,
                 lead_status=lead_status, lead_v=0.03, lead_d_rel=2.19 if lead_status else 0.0,
                 standstill=True, cruise_standstill=True, last_output_accel=last_output_accel, dt=DT)

  def _arm_driveway_hold(self, arb):
    # engage to +0.26 s: planner still says stop (source PLANNER)
    for i in range(26):
      dec = step(arb, STOPPING, self._driveway_frame(should_stop=True, a_target=min(0.20, 0.05 * (i // 5))))
      assert dec.stop_request_active and dec.source == sta.StopSource.PLANNER

  def test_driveway_obstacle_inside_hold_gap_holds_against_planner_go(self):
    arb = sta.StopTargetArbiter(_CP())
    self._arm_driveway_hold(arb)
    # planner flips to go and ramps aTarget to +1.45 while the obstacle sits at 2.19 m: the hold
    # must persist on EVERY frame, beyond any timed window (3 s >> 0.4/0.8/1.4 s)
    for i in range(300):
      a_target = min(1.45, 0.25 + 0.01 * i)
      dec = step(arb, STOPPING, self._driveway_frame(should_stop=False, a_target=a_target, last_output_accel=-1.06))
      assert dec.stop_request_active, f"i={i}"
      assert dec.state_should_stop, f"i={i}"
      assert dec.source == sta.StopSource.STOPPED_LEAD_LATCH, f"i={i}"
      assert dec.legacy_forced                      # full stop intent for the tiering consumers
      assert not dec.departing_lead_release and not dec.far_stopped_lead_release
      assert dec.target_distance_m == pytest.approx(STOP_TARGET_CLOSE_HOLD_REMAINING_M)

  def test_lead_lost_with_latched_planner_floor_releases_same_frame(self):
    # radar drops the track while the planner floor target (0.05 m) is still latched: every leg
    # of the close hold needs current-frame lead_status, and the remaining dropout holds carry
    # a_target ceilings (0.275 at v=0.05) -- with the go-ramp already at +0.45 nothing may hold.
    arb = sta.StopTargetArbiter(_CP())
    self._arm_driveway_hold(arb)
    dec = step(arb, STOPPING, self._driveway_frame(should_stop=False, a_target=0.45,
                                                   lead_status=False, last_output_accel=-1.06))
    assert not dec.stop_request_active
    assert not dec.state_should_stop
    assert not dec.state_dropout_hold

  def test_no_lead_no_target_standstill_dropout_releases_on_a_target_ramp(self):
    # NO lead, NO target: the no-target standstill dropout hold pins the state machine only while
    # a_target <= 0.12 (and is a state pin, not full stop intent); the planner go-ramp escapes it
    # within 3 frames at +0.05 per frame.
    arb = sta.StopTargetArbiter(_CP())
    for _ in range(20):
      step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True, last_output_accel=-0.15, dt=0.05))
    for i in range(10):
      a_target = 0.05 * i
      dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=a_target, should_stop=False, last_output_accel=-0.15, dt=0.05))
      assert not dec.stop_request_active and not dec.state_should_stop
      if a_target <= 0.12:
        assert dec.state_dropout_hold and dec.source == sta.StopSource.DROPOUT_HOLD, f"i={i}"
      else:
        assert not dec.state_dropout_hold, f"i={i}"

  def _run_departure(self, start_gap, lead_speed, max_t=8.0, dt=DT):
    """Lead departs while ego is held at standstill under a planner go-signal.

    The planner target follows the real planner helpers (stopped-lead target with the
    LEAD_STOP_DISTANCE_TARGET rest gap + the 0.6 s latch, exactly the long_mpc.py pipeline);
    a_target ramps to +1.5 at 1.0 m/s3. The state input stays pinned at STOPPING -- the worst
    case for release (the intent timer self-sustains at 0). Returns (release_t, gap_at_release);
    release_t is None when the hold never releases ("never fires" guard).
    """
    arb = sta.StopTargetArbiter(_CP())
    gap = start_gap
    ptgt, latch_s = STOP_TARGET_CLOSE_HOLD_REMAINING_M, STOP_TARGET_LATCH_DURATION_S
    for _ in range(20):
      step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=-0.2, should_stop=True,
                                planner_target_m=ptgt, lead_status=True, lead_v=0.0,
                                lead_d_rel=gap, last_output_accel=-0.5, dt=dt))
    t = 0.0
    while t < max_t:
      lead_v = lead_speed(t)
      gap += lead_v * dt
      candidate = get_distance_to_stopped_lead_target([lead_v], [gap], 0.0, LEAD_STOP_DISTANCE_TARGET)
      ptgt, latch_s = update_distance_to_stop_target_with_latch(ptgt, latch_s, dt, (candidate,))
      dec = step(arb, STOPPING, Frame(v_ego=0.0, standstill=True, a_target=min(1.5, 0.05 + 1.0 * t),
                                      should_stop=False, planner_target_m=ptgt, lead_status=True,
                                      lead_v=lead_v, lead_d_rel=gap, last_output_accel=-1.06, dt=dt))
      if not (dec.stop_request_active or dec.state_should_stop or dec.state_dropout_hold):
        return t, gap
      t += dt
    return None, gap

  def test_stop_and_go_departure_from_rest_gap_releases_promptly(self):
    # stop-and-go: lead accelerates away at 1.0 m/s2 from the 4.0 m rest gap; the planner floor
    # target leaves the < 0.2 m close-hold band at gap ~4.2 m -> release well under 1 s of motion
    release_t, gap = self._run_departure(start_gap=4.0, lead_speed=lambda t: max(0.0, min(2.0, 1.0 * t)))
    assert release_t is not None
    assert release_t <= 1.0
    assert gap <= 4.4

  def test_creeping_departure_releases_inside_rest_gap_band(self):
    # crawling traffic: lead creeps away at 0.25 m/s; release is bounded by the same ~4.2 m gap
    # (== the planner's own re-stop gap, so no drivable distance is ever withheld)
    release_t, gap = self._run_departure(start_gap=4.0, lead_speed=lambda t: 0.25)
    assert release_t is not None
    assert release_t <= 1.5
    assert gap <= 4.4

  def test_close_obstacle_departure_releases_via_departing_predicate(self):
    # driveway-like gap (2.2 m): a lead driving off at 1.8 m/s2 releases through the embedded
    # departing-lead predicate (gap > 5.80->3.80 m by departure speed) while the planner floor
    # target is still latched at 0.05 m -- guards the "never fires" direction from close range
    release_t, gap = self._run_departure(start_gap=2.2, lead_speed=lambda t: max(0.0, 1.8 * t))
    assert release_t is not None
    assert release_t <= 2.0
    assert gap <= 4.0
