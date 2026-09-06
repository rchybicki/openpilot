"""Cycle-32: Santa Fe HEV approach accel-tracking TRIM (longcontrol.py SANTA_FE_TRIM_* block).

Pinned acceptance tests from the design review (adopted with two harness-driven changes: the wind
deadband is 0.05 -- a 0.10 deadband leaves a 0.10 steady residual by construction, which cannot meet
the review's own +-0.08 convergence pin -- and KI_WIND 1.0 instead of 0.50; the convergence window is
4.0 s rather than 3.0 s because every gain that meets 3.0 s limit-cycles at 0.7 s delay).
The plant harness: first-order lag + pure delay + depth-independent gain, deterministic noise.
"""
import itertools
import math
from collections import deque

import pytest

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import longcontrol as lc_mod
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import (
  LongControl,
  LongCtrlState,
  SANTA_FE_TRIM_A_ARM,
  SANTA_FE_TRIM_DECAY,
  SANTA_FE_TRIM_MAX,
  SANTA_FE_TRIM_SLEW,
  SANTA_FE_TRIM_V_MAX,
  SANTA_FE_TRIM_V_MIN,
  update_santa_fe_tracking_trim,
)
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import (
  DummyCarParams,
  DummyCarState,
  DummyFrogPilotToggles,
)
from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR

DT = DT_CTRL
LIMITS = (-3.5, 2.0)
EPS = 1e-9


def _cp(fingerprint=HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022):
  cp = DummyCarParams(fingerprint)
  cp.longitudinalTuning.kpV = [0.0]  # the live Hyundai path: pure feedforward
  return cp


class Plant:
  """gain * (command delayed by `delay_s`) through a first-order lag `lag_s`."""

  def __init__(self, gain, delay_s, lag_s=0.5):
    self.gain, self.lag = gain, lag_s
    self.buf = deque([0.0] * max(int(round(delay_s / DT)), 1))
    self.a = 0.0

  def step(self, cmd):
    self.buf.append(float(cmd))
    delayed = self.buf.popleft()
    self.a += (self.gain * delayed - self.a) * DT / self.lag
    return self.a


def run(lc, *, n, demand, gain=0.87, delay_s=0.45, lag_s=0.5, v=8.0, noise=None, freeze=None,
        active=True, a_meas_override=None, limits=LIMITS):
  """demand/v/freeze/active may be scalars or callables of t. Returns a dict of per-frame lists."""
  toggles = DummyFrogPilotToggles()
  plant = Plant(gain, delay_s, lag_s)
  rec = {"t": [], "wire": [], "trim": [], "a": [], "demand": []}
  a_meas = 0.0
  for i in range(n):
    t = i * DT
    d = demand(t) if callable(demand) else demand
    vv = v(t) if callable(v) else v
    fr = freeze(t) if callable(freeze) else freeze
    ac = active(t) if callable(active) else active
    a_in = a_meas + (noise(i) if noise else 0.0)
    if a_meas_override is not None:
      ov = a_meas_override(t)
      if ov is not None:
        a_in = ov
    cs = DummyCarState(v_ego=vv, a_ego=a_in)
    wire = float(lc.update(active=ac, CS=cs, a_target=d, should_stop=False, distance_to_stop_target_m=-1.0,
                           accel_limits=limits, frogpilot_toggles=toggles, freeze_integrator=bool(fr)))
    a_meas = plant.step(wire)
    rec["t"].append(t)
    rec["wire"].append(wire)
    rec["trim"].append(float(lc._trim_i))
    rec["a"].append(a_meas)
    rec["demand"].append(float(d))
  return rec


def _mean(xs):
  return sum(xs) / len(xs)


def test_tracking_trim_kill_switch_is_live():
  assert stopping_flags.SANTA_FE_ACCEL_TRACKING_TRIM is True


def test_tracking_trim_cycle33_constants_pinned():
  # cycle-33: on the road the shipped set left the trim active on ~20% of braking frames (median -0.09).
  # The review's re-tune (deadband 0.03, guard 0.10 s) was swept against the review's OWN gates: it winds
  # -0.20 on a step onset at 0.6 s delay and limit-cycles 0.079 p2p at 0.7 s; no set with guard <= 0.20
  # meets the onset gate, for +4-7 pts of activity. KEPT the shipped constants (honest record).
  assert lc_mod.SANTA_FE_TRIM_WIND_DEADBAND == pytest.approx(0.05)
  assert lc_mod.SANTA_FE_TRIM_RATE_GUARD_S == pytest.approx(0.25)
  assert lc_mod.SANTA_FE_TRIM_KI_WIND == pytest.approx(1.00)
  assert SANTA_FE_TRIM_V_MIN == pytest.approx(2.50)


def test_no_false_onset_wind_before_the_plant_responds():
  # a step onset on a plant that tracks PERFECTLY (gain 1.0) must not wind the trim: the model
  # reference (seeded at aEgo) + the rate guard read the ramp-in as motion, not shortfall
  for delay_s in (0.30, 0.45, 0.60, 0.70):
    lc = LongControl(_cp())
    rec = run(lc, n=int(3.0 / DT), demand=-1.60, gain=1.0, delay_s=delay_s)
    assert min(rec["trim"]) >= -0.05, f"false onset wind {min(rec['trim']):.3f} at delay {delay_s}"


def test_trim_decays_when_speed_drops_below_v_min_into_the_service_band():
  # 2.5 -> 1.5 m/s: the trim decays at V_MIN; it must not become the short-rest aim lever
  lc = LongControl(_cp())
  t0 = _wound(lc)
  rec = run(lc, n=int(1.0 / DT), demand=-1.80, gain=0.80, delay_s=0.45, v=lambda t: 2.6 - 1.1 * t)
  k_cross = next(k for k, t in enumerate(rec["t"]) if 2.6 - 1.1 * t < SANTA_FE_TRIM_V_MIN)
  assert min(rec["trim"][:k_cross]) <= t0 + 1e-6 or rec["trim"][0] <= t0 + 0.02  # still winding/holding above
  prev = rec["trim"][k_cross]
  for x in rec["trim"][k_cross + 1:]:
    assert x >= prev - EPS
    prev = x
  assert rec["trim"][-1] == 0.0


def test_pure_update_bounds_and_direction():
  # winds on under-delivery beyond the deadband, never below -MAX, never above 0, slew-bounded
  tr = 0.0
  for _ in range(2000):
    tr = update_santa_fe_tracking_trim(tr, -1.8, -1.0, True)
  assert -SANTA_FE_TRIM_MAX - EPS <= tr <= 0.0
  assert tr == pytest.approx(-SANTA_FE_TRIM_MAX)
  # one-frame change is bounded by the slew
  assert update_santa_fe_tracking_trim(0.0, -1.8, 0.0, True) >= -SANTA_FE_TRIM_SLEW * DT - EPS
  # over-delivery unwinds toward 0, never positive
  tr2 = update_santa_fe_tracking_trim(-0.30, -1.8, -2.5, True)
  assert -0.30 < tr2 <= 0.0
  assert update_santa_fe_tracking_trim(-0.004, -1.8, -2.5, True) == 0.0
  # not learning -> decay at DECAY
  assert update_santa_fe_tracking_trim(-0.30, -1.8, -1.0, False) == pytest.approx(-0.30 + SANTA_FE_TRIM_DECAY * DT)
  # NaN aEgo is a decay frame, never NaN state
  out = update_santa_fe_tracking_trim(-0.30, -1.8, float("nan"), True)
  assert math.isfinite(out) and out > -0.30


@pytest.mark.parametrize("delay_s", [0.35, 0.45, 0.60])
def test_s19_steady_shortfall_converges(delay_s):
  # 013 s19: -1.80 demanded, plant ratio 0.87 -> realized -1.57 open-loop. By 3.0 s the realized
  # accel (mean over the next 0.5 s) must be within +-0.08 of the demand; no wire step > 0.03/frame;
  # no trim oscillation > 0.04 p2p after convergence.
  # Convergence window 4.0-4.5 s (the review proposed 3.0 s: KI >= 1.0 meets it in the nominal cell
  # but limit-cycles at 0.7 s delay, p2p 0.08-0.15; KI 0.75 converges by ~3.6 s and stays clean).
  lc = LongControl(_cp())
  rec = run(lc, n=int(7.0 / DT), demand=-1.80, gain=0.87, delay_s=delay_s)
  k4, k45 = int(4.0 / DT), int(4.5 / DT)
  realized = _mean(rec["a"][k4:k45])
  assert abs(realized - (-1.80)) <= 0.08, f"delay {delay_s}: realized {realized:.3f}"
  assert all(abs(rec["wire"][k] - rec["wire"][k - 1]) <= 0.03 + EPS for k in range(2, len(rec["wire"])))
  tail = rec["trim"][int(5.0 / DT):]
  assert max(tail) - min(tail) <= 0.04
  assert all(math.isfinite(x) for x in rec["trim"])
  # open-loop reference for the record: without the trim the same plant realizes ~-1.57
  lc0 = LongControl(_cp())
  lc0._trim_scope = False
  rec0 = run(lc0, n=int(7.0 / DT), demand=-1.80, gain=0.87, delay_s=delay_s)
  assert _mean(rec0["a"][k4:k45]) == pytest.approx(-1.566, abs=0.02)


def test_good_tracking_leaves_trim_near_zero_under_noise():
  # 012 s2 class: ratio 0.98 with +-0.10 deterministic noise -- no drift toward the bound
  for demand in (-0.75, -0.60, -0.50):
    lc = LongControl(_cp())
    rec = run(lc, n=int(20.0 / DT), demand=demand, gain=0.98, delay_s=0.45,
              noise=lambda i: 0.10 * (1.0 if (i // 7) % 2 == 0 else -1.0))
    assert abs(rec["trim"][-1]) <= 0.03, f"demand {demand}: trim {rec['trim'][-1]:.3f}"
    assert min(rec["trim"]) > -0.06


def test_unwind_is_faster_than_wind():
  # wind under a deep shortfall, then the recorded regen->friction transition (demand -1.84,
  # aEgo -2.05): trim shallower than -0.05 within 0.9 s; unwinding 0.25 takes < half the wind time
  lc = LongControl(_cp())
  rec = run(lc, n=int(8.0 / DT), demand=-1.80, gain=0.78, delay_s=0.45)
  assert min(rec["trim"]) <= -0.30
  t_wind = next(rec["t"][k] for k in range(len(rec["t"])) if rec["trim"][k] <= -0.25)
  # continue from the wound state with the over-delivering plant measurement
  rec2 = run(lc, n=int(1.5 / DT), demand=-1.84, gain=0.78, delay_s=0.45, a_meas_override=lambda t: -2.05)
  k_shallow = next(k for k in range(len(rec2["t"])) if rec2["trim"][k] > -0.05)
  assert rec2["t"][k_shallow] <= 0.9
  trim0 = rec2["trim"][0]
  k_unw = next(k for k in range(len(rec2["t"])) if rec2["trim"][k] >= trim0 + 0.25)
  assert rec2["t"][k_unw] < 0.5 * t_wind
  assert max(rec2["trim"]) <= 0.0


def test_hard_bounds_and_planner_limit():
  lc = LongControl(_cp())
  rec = run(lc, n=int(20.0 / DT), demand=-1.80, gain=0.45, delay_s=0.45)
  assert all(-SANTA_FE_TRIM_MAX - 1e-7 <= x <= 0.0 for x in rec["trim"])
  assert min(rec["trim"]) == pytest.approx(-SANTA_FE_TRIM_MAX, abs=1e-6)
  assert all(LIMITS[0] - EPS <= w <= LIMITS[1] + EPS for w in rec["wire"])
  # at the planner limit the trim cannot hold what the clip would not send
  lc2 = LongControl(_cp())
  rec2 = run(lc2, n=int(5.0 / DT), demand=-1.80, gain=0.45, delay_s=0.45, limits=(-1.90, 2.0))
  assert all(w >= -1.90 - EPS for w in rec2["wire"])
  assert all(x >= -0.10 - 1e-6 for x in rec2["trim"])


def _wound(lc):
  rec = run(lc, n=int(8.0 / DT), demand=-1.80, gain=0.80, delay_s=0.45)
  assert rec["trim"][-1] <= -0.25
  return rec["trim"][-1]


@pytest.mark.parametrize("gate", ["demand_shallows", "below_v_min", "above_v_max", "gas_override", "kill_switch"])
def test_disarm_decays_never_steps(gate, monkeypatch):
  lc = LongControl(_cp())
  t0 = _wound(lc)
  kw = {"demand": -1.80, "v": 8.0, "freeze": False}
  if gate == "demand_shallows":
    kw["demand"] = SANTA_FE_TRIM_A_ARM + 0.10
  elif gate == "below_v_min":
    kw["v"] = SANTA_FE_TRIM_V_MIN - 0.3
  elif gate == "above_v_max":
    kw["v"] = SANTA_FE_TRIM_V_MAX + 1.0
  elif gate == "gas_override":
    kw["freeze"] = True
  elif gate == "kill_switch":
    monkeypatch.setattr(stopping_flags, "SANTA_FE_ACCEL_TRACKING_TRIM", False)
  rec = run(lc, n=int(1.0 / DT), gain=0.80, delay_s=0.45, **kw)
  if gate == "kill_switch":
    # the switch off is a hard gate: the trim state is zero and the wire is the untrimmed chain
    assert all(x == 0.0 for x in rec["trim"])
    return
  prev = t0
  for x in rec["trim"]:
    assert x >= prev - EPS, "trim deepened after disarm"
    assert x - prev <= SANTA_FE_TRIM_DECAY * DT + EPS, "decay faster than DECAY (a step)"
    prev = x
  assert rec["trim"][int(0.5 / DT)] == 0.0  # 0.40 at 1.0 m/s^3 -> gone in 0.4 s


def test_disengagement_resets_trim():
  lc = LongControl(_cp())
  _wound(lc)
  rec = run(lc, n=20, demand=-1.80, gain=0.80, delay_s=0.45, active=False)
  assert rec["trim"][0] == 0.0 and lc.long_control_state == LongCtrlState.off


def test_kill_switch_and_scope_are_inert(monkeypatch):
  # switch off on the Santa Fe: trim exactly 0 through a scenario that winds with the switch on
  monkeypatch.setattr(stopping_flags, "SANTA_FE_ACCEL_TRACKING_TRIM", False)
  lc = LongControl(_cp())
  rec_off = run(lc, n=int(6.0 / DT), demand=-1.80, gain=0.87, delay_s=0.45)
  assert all(x == 0.0 for x in rec_off["trim"])
  monkeypatch.setattr(stopping_flags, "SANTA_FE_ACCEL_TRACKING_TRIM", True)
  lc_on = LongControl(_cp())
  rec_on = run(lc_on, n=int(6.0 / DT), demand=-1.80, gain=0.87, delay_s=0.45)
  assert min(rec_on["trim"]) < -0.15  # the delta IS the switch
  # a non-Santa-Fe fingerprint with the switch on: byte-identical to the switch-off wire
  lc_other = LongControl(_cp(HYUNDAI_CAR.HYUNDAI_SONATA))
  rec_other = run(lc_other, n=int(6.0 / DT), demand=-1.80, gain=0.87, delay_s=0.45)
  assert all(x == 0.0 for x in rec_other["trim"])
  assert rec_other["wire"] == rec_off["wire"]


def test_downstream_writer_frame_is_not_a_learning_frame(monkeypatch):
  # a cap that rewrites the untrimmed pid demand (here: the low-speed slew patched to deepen by 0.2
  # inside a window) freezes the wind and the trim decays for the window's duration
  lc = LongControl(_cp())
  t0 = _wound(lc)
  real = lc_mod.apply_low_speed_output_slew
  window = {"on": False}

  def patched(**kwargs):
    out = real(**kwargs)
    return out - 0.2 if window["on"] else out
  monkeypatch.setattr(lc_mod, "apply_low_speed_output_slew", patched)
  window["on"] = True
  rec = run(lc, n=int(0.6 / DT), demand=-1.80, gain=0.80, delay_s=0.45)
  prev = t0
  for x in rec["trim"]:
    assert x >= prev - EPS
    prev = x
  assert rec["trim"][-1] == 0.0
  window["on"] = False


def _takeover_scenario(monkeypatch, trim_on):
  """Phase 1: settled pid wire at 5 m/s with the car under-delivering (a_ego -1.0 vs demand -1.2) so the
  trim winds NATURALLY (through its own slew). Phase 2: approach a stopped lead from 3.2 m/s with
  should_stop -> the LIVE service takes the wire below 2.5 m/s."""
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  lc = LongControl(_cp())
  lc._trim_scope = trim_on
  toggles = DummyFrogPilotToggles()
  wires, owned, trims = [], [], []
  for _ in range(int(4.0 / DT)):
    cs = DummyCarState(v_ego=5.0, a_ego=-1.0)
    wire = float(lc.update(active=True, CS=cs, a_target=-1.2, should_stop=False, distance_to_stop_target_m=-1.0,
                           accel_limits=LIMITS, frogpilot_toggles=toggles))
    wires.append(wire)
    owned.append(False)
    trims.append(float(lc._trim_i))
  v, gap = 3.2, 14.0
  for _ in range(int(4.0 / DT)):
    cs = DummyCarState(v_ego=v, a_ego=-1.1)
    wire = float(lc.update(active=True, CS=cs, a_target=-1.2, should_stop=True, distance_to_stop_target_m=max(gap - 4.3, 0.05),
                           accel_limits=LIMITS, frogpilot_toggles=toggles, lead_status=True, lead_v=0.0, lead_d_rel=gap,
                           lead_model_prob=0.9, lead_track_id=5))
    wires.append(wire)
    owned.append(bool(lc._service_live_owning))
    trims.append(float(lc._trim_i))
    v = max(v - 1.1 * DT, 0.0)
    gap = max(gap - v * DT, 0.5)
  return wires, owned, trims


def test_service_takeover_continuity_with_wound_trim(monkeypatch):
  # LIVE: the trim is wound when the approach reaches the service band. Through the takeover the trim
  # may not worsen the worst wire step of the trim-off run by more than its own slew, must not deepen
  # while owned, and ends at 0. (The service's own entry ramp is pre-existing: identical trim on/off.)
  w_on, own_on, trims = _takeover_scenario(monkeypatch, True)
  w_off, own_off, _ = _takeover_scenario(monkeypatch, False)
  k_p2 = int(4.0 / DT)
  assert min(trims[:k_p2]) <= -0.12, f"trim did not wind in phase 1: {min(trims[:k_p2]):.3f}"
  assert any(own_on) and own_on.index(True) == own_off.index(True), "the trim changed the takeover frame"
  k_own = own_on.index(True)
  step_on = max(abs(w_on[k] - w_on[k - 1]) for k in range(2, len(w_on)))
  step_off = max(abs(w_off[k] - w_off[k - 1]) for k in range(2, len(w_off)))
  assert step_on <= step_off + SANTA_FE_TRIM_SLEW * DT + 1e-6, f"trim worsened the worst step: {step_on:.4f} vs {step_off:.4f}"
  # phase 1 (pid is the wire): the trim-induced per-frame delta is bounded by its slew
  for k in range(2, k_p2):
    assert abs((w_on[k] - w_on[k - 1]) - (w_off[k] - w_off[k - 1])) <= SANTA_FE_TRIM_SLEW * DT + 1e-6
  for k in range(k_own + 1, len(trims)):
    assert trims[k] >= trims[k - 1] - EPS, "trim deepened while service-owned"
  assert trims[-1] == 0.0


def test_service_owned_previous_frame_is_not_a_learning_frame():
  # The trim block runs BEFORE the service takeover, so a service-written wire is invisible to its
  # cap detection; the ownership flag (previous frame) is the gate. A band-exit RELEASE can keep the
  # service active above V_MIN with a braking demand, so this gate is not redundant with V_MIN.
  lc = LongControl(_cp())
  t0 = _wound(lc)
  toggles = DummyFrogPilotToggles()
  lc._service_live_owning = True      # "the service wrote the wire on the previous frame"
  cs = DummyCarState(v_ego=8.0, a_ego=-1.0)   # under-delivering: a learning frame would WIND
  lc.update(active=True, CS=cs, a_target=-1.80, should_stop=False, distance_to_stop_target_m=-1.0,
            accel_limits=LIMITS, frogpilot_toggles=toggles)
  assert t0 < -0.2
  assert lc._trim_i == 0.0, "a service-owned frame must zero the trim state (R1 HIGH: no residual)"


def test_service_exception_after_takeover_cannot_reapply_residual_trim(monkeypatch):
  # R1 HIGH: takeover through the stopped-lead LATCH (no planner stop request, so the pid is the wire
  # right up to the service entry and the residual trim is in the seed), then _run_stopping_service
  # throws on the frame after the first owned frame. The fault fallback (min(legacy, last)) must not
  # put a residual trim on the wire as a DEEPEN step; the wire may only return toward the untrimmed
  # legacy value (a release bounded by the residual), and the state is zero on every owned frame.
  # (The review's geometry -- service relaxed above legacy, then a fault -- is not reproducible in
  # this harness; the fix removes it by construction: nothing is left to reapply.)
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  lc = LongControl(_cp())
  toggles = DummyFrogPilotToggles()
  for _ in range(int(4.0 / DT)):  # wind at 5 m/s behind a far stopped lead (latch warms)
    lc.update(active=True, CS=DummyCarState(v_ego=5.0, a_ego=-1.0), a_target=-1.2, should_stop=False,
              distance_to_stop_target_m=-1.0, accel_limits=LIMITS, frogpilot_toggles=toggles,
              lead_status=True, lead_v=0.0, lead_d_rel=30.0, lead_model_prob=0.9, lead_track_id=5)
  assert lc._trim_i <= -0.30
  real_run = lc._run_stopping_service
  state = {"armed": False, "threw": False}

  def throwing(**kwargs):
    if state["armed"] and not state["threw"]:
      state["threw"] = True
      raise RuntimeError("injected service fault")
    return real_run(**kwargs)
  monkeypatch.setattr(lc, "_run_stopping_service", throwing)
  v, gap = 2.7, 12.0
  wires, owned, trims = [], [], []
  for _ in range(int(3.0 / DT)):
    if owned and owned[-1] and not state["armed"]:
      state["armed"] = True       # throw on the frame AFTER the first owned frame
    wire = float(lc.update(active=True, CS=DummyCarState(v_ego=v, a_ego=-1.1), a_target=-1.2, should_stop=False,
                           distance_to_stop_target_m=-1.0, accel_limits=LIMITS, frogpilot_toggles=toggles,
                           lead_status=True, lead_v=0.0, lead_d_rel=gap, lead_model_prob=0.9, lead_track_id=5))
    wires.append(wire)
    owned.append(bool(lc._service_live_owning))
    trims.append(float(lc._trim_i))
    v = max(v - 1.1 * DT, 0.0)
    gap = max(gap - v * DT, 0.5)
  assert state["threw"], "the fault was never injected"
  k_own = owned.index(True)
  residual = -trims[k_own]          # what was in the seed
  assert residual > 0.05, "the residual never reached the takeover (harness lost the geometry)"
  assert all(x == 0.0 for x in trims[k_own + 1:k_own + 40]), "owned frames kept a residual trim"
  k_fault = k_own + 1
  for k in range(k_fault, k_fault + 5):
    step = wires[k] - wires[k - 1]
    assert step >= -0.03 - EPS, f"DEEPEN step {step:+.3f} on the fault path at {k}"
    assert step <= residual + 0.03 + EPS, f"release step {step:+.3f} beyond the residual at {k}"
  assert lc._service_live_disabled


def test_learning_waits_tau_after_a_cap_window(monkeypatch):
  # After a downstream writer releases the wire, learning must wait TAU of clean frames (the plant
  # state then reflects the cap's history, not the untrimmed demand): the trim keeps decaying.
  lc = LongControl(_cp())
  _wound(lc)
  real = lc_mod.apply_low_speed_output_slew
  window = {"on": True}

  def patched(**kwargs):
    out = real(**kwargs)
    return out - 0.2 if window["on"] else out
  monkeypatch.setattr(lc_mod, "apply_low_speed_output_slew", patched)
  rec_cap = run(lc, n=int(0.2 / DT), demand=-1.80, gain=0.80, delay_s=0.45)
  window["on"] = False
  t_end = rec_cap["trim"][-1]
  assert t_end < -0.05
  rec = run(lc, n=int(0.40 / DT), demand=-1.80, gain=0.80, delay_s=0.45)  # < TAU (0.45 s) of clean frames
  prev = t_end
  for x in rec["trim"]:
    assert x >= prev - EPS, "the trim wound inside the TAU clean-frame window after a cap"
    prev = x


# GAUNTLET NOTE: the cap-frame gate (`not trim_cap_written`) and the clean-frames gate
# (`_trim_clean >= TAU_FRAMES`) mask each other on the cap frame itself -- dropping either alone is
# caught only by dropping both (paired mutation, killed by test_downstream_writer_frame_is_not_a_
# learning_frame); test_learning_waits_tau_after_a_cap_window pins the clean-frames gate's OWN effect.


def test_robustness_sweep_no_limit_cycle_no_sustained_over_delivery():
  for gain, delay_s, na in itertools.product((0.78, 0.87, 0.98, 1.05), (0.30, 0.45, 0.60, 0.70), (0.05, 0.12)):
    lc = LongControl(_cp())
    rec = run(lc, n=int(12.0 / DT), demand=-1.60, gain=gain, delay_s=delay_s,
              noise=lambda i, na=na: na * math.sin(i * 0.37) )
    assert all(math.isfinite(x) for x in rec["trim"]) and all(math.isfinite(w) for w in rec["wire"])
    tail = rec["trim"][int(7.0 / DT):]
    assert max(tail) - min(tail) <= 0.06, f"limit cycle {max(tail)-min(tail):.3f} at gain {gain} delay {delay_s} noise {na}"
    over = [a - d < -0.20 for a, d in zip(rec["a"], rec["demand"], strict=True)]
    run_len = max((len(list(g)) for k, g in itertools.groupby(over) if k), default=0)
    assert run_len <= int(0.5 / DT), f"over-delivery > 0.2 for {run_len*DT:.2f} s at gain {gain} delay {delay_s}"
