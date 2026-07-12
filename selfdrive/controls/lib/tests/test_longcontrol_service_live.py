"""LongControl stage-3 LIVE wiring tests -- Stopping Service V3 stage 3
(docs/stopping/stopping_service_v3_plan.md §6 stage 3).

Proves the full-band ownership semantics at the LongControl.update seam:
  - in LIVE the service owns EVERY frame it reports active, in BOTH the pid and stopping states --
    the stage-2 0.85 seam is gone (route 00001b72: on the first stage-2 live drive the stopping
    state only engaged at v = 0.15, so the planner's one-frame aTarget slam -0.32 -> -0.81 at
    v = 0.92 reached the wire through the pid state = the felt harsh stop, IMU jerk 8.3);
  - THE SLAM FIXTURE: a closed-loop replay of that stop. The scripted planner steps to -0.81 in one
    frame at the traced geometry and holds the slam while the wire has not caught its demand (the
    MPC re-solves against the tracked state; it relaxes once the command answers). Under LIVE the
    service owns the pid frames: when conditioned lead geometry is trustworthy, cycle 8 preserves
    the constraint-resolved trajectory demand while bounding only redundant direct-model depth to
    the conservative 2.5 m floor; the 4 m phase law lands continuously and arrives at wheel-stop
    shallow. Under LIVE_TERMINAL the
    same frames run the legacy pid chain: the raw pid demand slams past -1.0 and the asymmetric C1
    slew ratchets the wire deep and holds it there into the stop -- the wire shows the slam. The
    delta IS the stage-3 flag;
  - PID HANDBACK CONTINUITY: while the service owns pid-state frames the integrator is frozen and
    reseeded to the service command, so a RELEASE handback resumes the legacy pid exactly from the
    service trajectory -- no wire step beyond the C1 low-speed slew, no windup jump in pid.i;
  - C4 (brake-model alignment) and C5 (stopped/slowing-lead approach caps) are not consulted -- and
    so cannot mutate pid.i -- on service-owned pid frames;
  - LIVE_TERMINAL keeps today's semantics: it never owns pid-band frames (byte-identical to SHADOW
    there) and its stopping-band behavior is pinned by test_longcontrol_live_terminal.py.
"""

import math

import pytest

from openpilot.selfdrive.controls.lib import longcontrol as longcontrol_module
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.stopping_service import Phase, ServiceParams
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import (
  DummyCarParams,
  DummyCarState,
  DummyFrogPilotToggles,
)

P = ServiceParams()
DT = 0.01
EPS = 1e-9
LIMITS = (-3.0, 2.0)


# --- closed-loop plant harness ----------------------------------------------------------------------
# LongControl does not integrate a plant; the slam replay needs one so the planner model can react to
# the ACTUAL braking state (the author-diagnosed causality: the MPC slammed because the pid-state wire
# had not answered the stop-commitment demand; a tracked wire relaxes it).

def run_plant(lc: LongControl, *, v0: float, gap0: float, n: int, a_target_fn,
              a_target_trajectory_fn=None, lead_v_fn=None, should_stop_fn=None,
              dts_offset: float = 4.3, dts_enabled: bool = True, tau: float = 0.15):
  toggles = DummyFrogPilotToggles()
  v, a_act, a_meas, gap, wire = float(v0), 0.0, 0.0, float(gap0), 0.0
  rec = {"t": [], "v": [], "wire": [], "own": [], "state": [], "gap": [], "a_tgt": [],
         "pid_raw": [], "pid_i": [], "phase": []}
  for i in range(n):
    t = i * DT
    lead_v = lead_v_fn(t) if lead_v_fn is not None else 0.0
    dts = max(gap - dts_offset, 0.05) if dts_enabled else -1.0
    a_tgt = a_target_fn(t, v, dts, wire)
    a_tgt_trajectory = (a_target_trajectory_fn(t, v, dts, wire)
                        if a_target_trajectory_fn is not None else a_tgt)
    should_stop = bool(should_stop_fn(t, v)) if should_stop_fn is not None else False
    cs = DummyCarState(v_ego=v, a_ego=a_meas, standstill=(v <= 0.005))
    wire = float(lc.update(active=True, CS=cs, a_target=a_tgt, should_stop=should_stop,
                           distance_to_stop_target_m=dts, accel_limits=LIMITS,
                           frogpilot_toggles=toggles, lead_status=True, lead_v=lead_v,
                           lead_d_rel=gap, a_target_trajectory=a_tgt_trajectory))
    rec["t"].append(t)
    rec["v"].append(v)
    rec["wire"].append(wire)
    rec["own"].append(bool(lc._service_live_owning))
    rec["state"].append(lc.long_control_state)
    rec["gap"].append(gap)
    rec["a_tgt"].append(a_tgt)
    rec["pid_raw"].append(float(lc.pid.control))
    rec["pid_i"].append(float(lc.pid.i))
    rec["phase"].append(lc._service_shadow_svc.phase)
    # plant: first-order actuation lag
    a_act += (wire - a_act) * DT / (tau + DT)
    if v <= 0.0 and a_act < 0.0:
      v_new, a_meas = 0.0, 0.0
    else:
      v_new = max(v + a_act * DT, 0.0)
      a_meas = (v_new - v) / DT
    gap = gap - max((v + v_new) / 2.0 * DT, 0.0) + lead_v * DT
    v = v_new
  return rec


def slam_planner_fn(slam_v_min: float = 0.85, slam_dts: float = 1.3, release_wire: float = -0.60):
  """The traced planner (route 00001b72): -0.32 through the approach; a ONE-FRAME step to -0.81 at
  the stop-commitment geometry (dts <= 1.3 still rolling ~0.9 m/s); the slam HOLDS while the wire
  has not answered it (the MPC keeps re-demanding against an under-braking state) and relaxes to
  the kinematic-consistent -0.36 once the command reaches release_wire."""
  state = {"slam": False, "done": False}

  def fn(t, v, dts, wire_prev):
    if state["done"]:
      return -0.36
    if not state["slam"]:
      if dts <= slam_dts and v > slam_v_min:
        state["slam"] = True
        return -0.81
      return -0.32
    if wire_prev <= release_wire:
      state["done"] = True
      return -0.36
    return -0.81

  return fn


def slam_indices(rec) -> tuple[int, int]:
  k_slam = next(k for k, a in enumerate(rec["a_tgt"]) if a <= -0.80)
  k_relax = next(k for k in range(k_slam, len(rec["a_tgt"])) if rec["a_tgt"][k] > -0.80)
  return k_slam, k_relax


def last_rolling_idx(rec) -> int:
  idx = None
  for k, v in enumerate(rec["v"]):
    if v > 0.05:
      idx = k
  assert idx is not None and idx < len(rec["v"]) - 50, "car never stopped inside the trace"
  return idx


# --- THE SLAM FIXTURE (route 00001b72) --------------------------------------------------------------

def test_slam_fixture_live_owns_pid_frames_and_lands_shallow(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  rec = run_plant(LongControl(DummyCarParams()), v0=1.4, gap0=7.35, n=900,
                  a_target_fn=slam_planner_fn(), a_target_trajectory_fn=lambda t, v, d, w: -0.32,
                  should_stop_fn=lambda t, v: v <= 0.15)
  k_slam, k_relax = slam_indices(rec)
  k_roll = last_rolling_idx(rec)

  # the fixture reproduces the traced geometry: slam fires near v 0.92 / dts 1.3 / gap 5.6
  assert 0.85 < rec["v"][k_slam] <= 1.05, f"slam at v {rec['v'][k_slam]:.2f}"
  assert 5.3 <= rec["gap"][k_slam] <= 5.9, f"slam at gap {rec['gap'][k_slam]:.2f}"
  # STAGE-3 OWNERSHIP: the service owns the slam frame IN THE PID STATE, above the stage-2 0.85 seam
  assert rec["state"][k_slam] == LongCtrlState.pid
  assert rec["own"][k_slam], "stage 3 must own the pid-state slam frame"
  assert any(rec["own"][k] and rec["v"][k] > 1.0 for k in range(k_slam)), "no ownership above 1.0 m/s"
  # the -0.49 one-frame planner step NEVER appears on the wire: every deepen is service-jerk-limited
  for k in range(1, k_roll + 1):
    step = rec["wire"][k] - rec["wire"][k - 1]
    assert step >= -P.J_SAFE * DT - EPS, f"wire slam {step:.4f} at frame {k}"
  # comfort frames before the raw slam move at no more than J_DOWN
  for k in range(1, k_roll + 1):
    if k < k_slam:
      step = rec["wire"][k] - rec["wire"][k - 1]
      assert step >= -P.J_DOWN * DT - EPS, f"comfort-frame deepen {step:.4f} at frame {k}"
  # The synthetic planner keeps asking -0.81 until the post-stop secure hold reaches -0.60.
  # With resolved geometry that redundant depth must not ratchet the moving approach: the phase
  # law lands at 4 m while a_kin remains live against the 2 m hard margin.
  assert k_relax > k_roll
  assert min(rec["wire"][: k_roll + 1]) >= -0.45
  # The felt fix: the car arrives at wheel-stop shallow, then pressure builds silently to the
  # secure service hold after motion has resolved.
  assert -0.38 <= rec["wire"][k_roll] <= -0.03, f"wheel-stop wire {rec['wire'][k_roll]:.3f}"
  assert rec["wire"][-1] == pytest.approx(P.A_HOLD_SECURE, abs=0.03)
  assert rec["phase"][-1] in (Phase.RAMP_TO_HOLD, Phase.HOLD)
  # ... in the user rest band despite persistent raw model depth
  assert 2.5 <= rec["gap"][-1] <= 5.0, f"rest gap {rec['gap'][-1]:.2f}"


def test_slam_fixture_live_terminal_wire_shows_the_slam(monkeypatch) -> None:
  # Same closed-loop fixture under LIVE_TERMINAL: the pid state keeps the wire until the stopping
  # state engages late (the traced v=0.15 seam), so the planner slam reaches the legacy chain --
  # the raw pid demand slams past -1.0 and the asymmetric C1 slew walks the wire deep and HOLDS it
  # there through the terminal approach. This is tonight's harsh stop; the delta vs the test above
  # is exactly the stage-3 flag.
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE_TERMINAL")
  rec = run_plant(LongControl(DummyCarParams()), v0=1.4, gap0=7.35, n=900,
                  a_target_fn=slam_planner_fn(), a_target_trajectory_fn=lambda t, v, d, w: -0.32,
                  should_stop_fn=lambda t, v: v <= 0.15)
  k_slam, k_relax = slam_indices(rec)
  k_roll = last_rolling_idx(rec)

  assert 0.85 < rec["v"][k_slam] <= 1.05  # same traced geometry
  # LIVE_TERMINAL never owns a pid-state frame; ownership (if any) begins in stopping state <= 0.85
  for k in range(len(rec["own"])):
    if rec["own"][k]:
      assert rec["state"][k] == LongCtrlState.stopping
      assert rec["v"][k] <= 0.95 + EPS
  assert not rec["own"][k_slam]
  # the slam reaches the legacy pid demand unfiltered ...
  assert min(rec["pid_raw"][k_slam:k_slam + 5]) <= -1.0, "raw pid demand never slammed"
  # ... and the wire shows it: the asymmetric C1 slew walks the wire deep at its full brake-step
  # rate, frame after frame, until the demand is answered -- the planner only relaxes ~0.2 s later
  # (vs ~0.05 s under LIVE, where the service answers at J_SAFE)
  assert (k_relax - k_slam) * DT >= 0.15, "slam released too fast to be the traced pid-state walk"
  assert min(rec["wire"][k_slam:k_roll + 1]) <= -0.55, "the slam never reached the wire"
  assert rec["wire"][k_relax - 1] - rec["wire"][k_slam - 1] <= -0.20, "no sustained C1 walk on the wire"
  # ... and the snap-back violates the service release law (C4 alignment yanks the wire up much
  # faster than J_UP): the slam AND its recovery both show on the wire, unfiltered
  max_release = max(rec["wire"][k] - rec["wire"][k - 1] for k in range(k_relax, k_relax + 10))
  assert max_release >= 0.03, f"release snap {max_release:.4f} (expected the legacy C4 yank > J_UP*dt)"
  # everything stays finite/braking; the late stage-2 catch (stopping state, v <= 0.85) is allowed
  assert all(math.isfinite(w) and w < 0.0 for w in rec["wire"][k_slam:])


def test_slam_fixture_delta_is_the_stage3_flag(monkeypatch) -> None:
  # Direct A/B on identical fixture code: under LIVE the resolved-geometry bound keeps the raw
  # planner slam out of the moving wire; under LIVE_TERMINAL the same demand plays out through the
  # legacy chain, walking much deeper and snapping back through C4.
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  rec_live = run_plant(LongControl(DummyCarParams()), v0=1.4, gap0=7.35, n=900,
                       a_target_fn=slam_planner_fn(), a_target_trajectory_fn=lambda t, v, d, w: -0.32,
                       should_stop_fn=lambda t, v: v <= 0.15)
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE_TERMINAL")
  rec_lt = run_plant(LongControl(DummyCarParams()), v0=1.4, gap0=7.35, n=900,
                     a_target_fn=slam_planner_fn(), a_target_trajectory_fn=lambda t, v, d, w: -0.32,
                     should_stop_fn=lambda t, v: v <= 0.15)
  k_slam_live, _k_relax_live = slam_indices(rec_live)
  _k_slam_lt, k_relax_lt = slam_indices(rec_lt)
  # LIVE: every rolling frame from first ownership obeys the SERVICE jerk law end to end
  # (deepen <= J_SAFE, release <= J_UP)
  k_own = rec_live["own"].index(True)
  k_roll_live = last_rolling_idx(rec_live)
  for k in range(k_own + 1, k_roll_live + 1):
    step = rec_live["wire"][k] - rec_live["wire"][k - 1]
    assert -P.J_SAFE * DT - EPS <= step <= P.J_UP * DT + EPS, f"LIVE step {step:.4f} at {k}"
  k_roll_lt = last_rolling_idx(rec_lt)
  assert min(rec_live["wire"][k_slam_live:k_roll_live + 1]) >= -0.45
  assert min(rec_lt["wire"][:k_roll_lt + 1]) <= -0.55
  # LIVE_TERMINAL: the legacy chain's release snap exceeds the service law (the C4 alignment yank)
  max_release_lt = max(rec_lt["wire"][k] - rec_lt["wire"][k - 1] for k in range(k_relax_lt, k_relax_lt + 10))
  assert max_release_lt > 2.0 * P.J_UP * DT, f"LT release snap only {max_release_lt:.4f}"
  # and the terminal arrival stays shallow under LIVE (the felt fix)
  assert rec_live["wire"][k_roll_live] >= -0.38


# --- PID handback continuity (freeze + owned-frame reseed) ------------------------------------------

def _ki_car_params() -> DummyCarParams:
  cp = DummyCarParams()
  cp.longitudinalTuning.kiV = [0.3]  # a real integrator so windup/handback are meaningful
  return cp


def _queue_release_scenario(lc: LongControl, n: int = 800):
  """Service owns pid-state frames behind a stopped lead at v ~1.05 (above the stage-2 seam), then
  the lead accelerates away and the planner goes -> the service RELEASEs -> legacy pid resumes."""
  toggles = DummyFrogPilotToggles()
  gap = 6.0
  rec = {"wire": [], "own": [], "pid_i": [], "pid_p": [], "pid_f": [], "state": []}
  for i in range(n):
    t = i * DT
    if t < 4.0:
      lead_v, a_tgt = 0.0, -0.30
    else:
      lead_v, a_tgt = 1.8, 0.35
      gap += (lead_v - 1.05) * DT + 1.05 * DT  # lead pulling away
    cs = DummyCarState(v_ego=1.05, a_ego=0.0, standstill=False)
    wire = float(lc.update(active=True, CS=cs, a_target=a_tgt, should_stop=False,
                           distance_to_stop_target_m=-1.0, accel_limits=LIMITS,
                           frogpilot_toggles=toggles, lead_status=True, lead_v=lead_v,
                           lead_d_rel=gap))
    rec["wire"].append(wire)
    rec["own"].append(bool(lc._service_live_owning))
    rec["pid_i"].append(float(lc.pid.i))
    rec["pid_p"].append(float(lc.pid.p))
    rec["pid_f"].append(float(lc.pid.f))
    rec["state"].append(lc.long_control_state)
  return rec


def test_pid_handback_continuity_and_no_windup(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  lc = LongControl(_ki_car_params())
  rec = _queue_release_scenario(lc)

  k_own = rec["own"].index(True)
  assert rec["state"][k_own] == LongCtrlState.pid  # stage-3: owns in pid state at v 1.05
  k_hb = next(k for k in range(k_own, len(rec["own"])) if not rec["own"][k])
  assert k_hb > int(4.0 / DT), "service released before the lead departed"
  # owned frames keep pid.i reseeded to the service command: pid output == wire identically, so the
  # legacy pid resumes exactly from the service trajectory (d == 0 with no error_rate)
  for k in range(k_own + 1, k_hb):
    if rec["own"][k]:
      recon = rec["pid_p"][k] + rec["pid_i"][k] + rec["pid_f"][k]
      assert recon == pytest.approx(rec["wire"][k], abs=1e-9), f"pid state diverged from wire at {k}"
  # no windup: while owned pre-release, pid.i tracks the service command against the p/f terms
  # (|i| small); during the RELEASE ramp the reseed keeps |i| = |wire - (p+f)| <= ~1.5 with the go
  # a_target -- bounded either way, never an unbounded 4 s integration of the brake error
  k_go = int(4.0 / DT)
  assert all(abs(i) <= 0.8 for i in rec["pid_i"][k_own:k_go])
  assert all(abs(i) <= 2.0 for i in rec["pid_i"][k_go:k_hb])
  # handback continuity: no wire step beyond the C1 low-speed slew authority (v 1.05: release step
  # <= ~0.02/frame even with fast-release; deepen <= ~0.014)
  for k in range(k_hb, min(k_hb + 30, len(rec["wire"]))):
    step = rec["wire"][k] - rec["wire"][k - 1]
    assert -0.02 - EPS <= step <= 0.035 + EPS, f"handback step {step:.4f} at frame {k}"
  # ... and no integrator jump across the handback frame
  assert abs(rec["pid_i"][k_hb] - rec["pid_i"][k_hb - 1]) <= 0.01


def test_live_hold_blocks_far_stopped_lead_starting_escape(monkeypatch) -> None:
  # Route 00001b82 seg40: the service was already in HOLD at a ~6 m stopped-lead gap, then the
  # legacy far-stopped-lead close-gap release suppressed state_should_stop and the Hyundai starting
  # state escaped underneath it. That produced a small go pulse followed by a harsh legacy re-stop.
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  cp = DummyCarParams()
  cp.startingState = True
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = P.A_HOLD
  lc._service_live_owning = True
  lc._service_shadow_svc.phase = Phase.HOLD
  lc._service_shadow_svc._last_cmd = P.A_HOLD
  lc._service_shadow_svc._hold_entry_gap = 6.0

  out = float(lc.update(active=True, CS=DummyCarState(v_ego=0.06, a_ego=-0.05, standstill=True),
                        a_target=-0.17, should_stop=True, distance_to_stop_target_m=1.66,
                        accel_limits=LIMITS, frogpilot_toggles=DummyFrogPilotToggles(),
                        lead_status=True, lead_v=0.16, lead_d_rel=6.10))

  assert lc.long_control_state == LongCtrlState.stopping
  assert lc._service_live_owning
  assert lc._service_shadow_svc.phase == Phase.HOLD
  assert out == pytest.approx(P.A_HOLD, abs=0.02)  # one frame of J_HOLD build toward A_HOLD_SECURE


def test_live_hold_waits_for_service_release_before_departing_lead_start(monkeypatch) -> None:
  # Route 00001c90 seg142: radar briefly reported the same stopped track creeping at ~0.57 m/s,
  # enough for the legacy departing-lead helper to enter `starting` while shouldStop and aTarget
  # still requested braking. The service remained in HOLD, but lost the wire; ego launched 0.43 m
  # and was then harshly re-stopped. HOLD must retain state authority until the service sees the
  # planner's positive go demand and enters its own jerk-limited RELEASE phase.
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  toggles.human_acceleration = True
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = P.A_HOLD_SECURE
  lc._service_live_owning = True
  lc._service_shadow_svc.phase = Phase.HOLD
  lc._service_shadow_svc._last_cmd = P.A_HOLD_SECURE
  lc._service_shadow_svc._hold_entry_gap = 5.60

  def step(a_target: float) -> float:
    return float(lc.update(active=True, CS=DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True),
                           a_target=a_target, should_stop=True, distance_to_stop_target_m=2.05,
                           accel_limits=LIMITS, frogpilot_toggles=toggles,
                           lead_status=True, lead_v=0.57, lead_d_rel=6.35))

  held = step(-0.07)
  assert lc.long_control_state == LongCtrlState.stopping
  assert lc._service_live_owning
  assert lc._service_shadow_svc.phase == Phase.HOLD
  assert held == pytest.approx(P.A_HOLD_SECURE, abs=0.02)

  # A real planner go moves the service to RELEASE. It keeps state/wire authority until the
  # jerk-limited ramp reaches zero and resets the service; only the following frame may enter
  # Hyundai's starting state.
  step(0.35)
  assert lc.long_control_state == LongCtrlState.stopping
  assert lc._service_shadow_svc.phase == Phase.RELEASE
  release_wire = []
  for _ in range(100):
    release_wire.append(step(0.35))
    assert lc.long_control_state == LongCtrlState.stopping
    if lc._service_shadow_svc.phase == Phase.INACTIVE:
      break
  assert lc._service_shadow_svc.phase == Phase.INACTIVE
  assert release_wire[-1] > -0.05
  assert all(cur >= prev for prev, cur in zip(release_wire, release_wire[1:], strict=False))

  step(0.35)
  assert lc.long_control_state == LongCtrlState.starting


def test_live_terminal_never_owns_the_pid_band_scenario(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE_TERMINAL")
  lc = LongControl(_ki_car_params())
  rec = _queue_release_scenario(lc)
  assert not any(rec["own"])  # stage-2 semantics untouched: pid-state frames are never owned


# --- C4/C5 pid-cap non-interference on owned frames --------------------------------------------------
# Honesty note (Codex review 2026-07-02): C5 (pid_stopped/slowing_lead_approach_accel_cap) only
# BINDS in its 6-22 m/s band, structurally outside the service's <2.5 m/s ownership, so its gating
# here is belt-and-suspenders and the spy proves consultation gating only. The load-bearing pid.i
# protection in the service band is C4 (brake-model alignment), which these fixtures do exercise.

def _cap_spy_scenario(lc: LongControl, monkeypatch):
  calls = {"align": [], "approach": []}
  frame = {"i": 0}
  orig_align = longcontrol_module.apply_pid_brake_model_alignment
  orig_approach = longcontrol_module.pid_stopped_lead_approach_accel_cap

  def spy_align(*args, **kwargs):
    calls["align"].append(frame["i"])
    return orig_align(*args, **kwargs)

  def spy_approach(*args, **kwargs):
    calls["approach"].append(frame["i"])
    return orig_approach(*args, **kwargs)

  monkeypatch.setattr(longcontrol_module, "apply_pid_brake_model_alignment", spy_align)
  monkeypatch.setattr(longcontrol_module, "pid_stopped_lead_approach_accel_cap", spy_approach)

  toggles = DummyFrogPilotToggles()
  ownings = []
  for i in range(300):  # stopped lead at gap 8, v 1.0: pid state, service enters via the lead latch
    frame["i"] = i
    cs = DummyCarState(v_ego=1.0, a_ego=-0.2, standstill=False)
    lc.update(active=True, CS=cs, a_target=-0.40, should_stop=False,
              distance_to_stop_target_m=-1.0, accel_limits=LIMITS,
              frogpilot_toggles=toggles, lead_status=True, lead_v=0.0, lead_d_rel=8.0)
    ownings.append(bool(lc._service_live_owning))
  return calls, ownings


def test_pid_caps_not_consulted_on_owned_frames(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  calls, ownings = _cap_spy_scenario(LongControl(DummyCarParams()), monkeypatch)
  k_own = ownings.index(True)
  assert all(ownings[k_own:])
  # legacy-capped up to and INCLUDING the first owned frame (prev-frame bypass keying) ...
  assert any(k <= k_own for k in calls["align"]) and any(k <= k_own for k in calls["approach"])
  # ... then never again while owned: C4/C5 cannot touch pid.i on service-owned frames
  assert not [k for k in calls["align"] if k > k_own], "C4 consulted on an owned frame"
  assert not [k for k in calls["approach"] if k > k_own], "C5 consulted on an owned frame"


def test_pid_caps_still_consulted_under_shadow_and_live_terminal(monkeypatch) -> None:
  for mode in ("SHADOW", "LIVE_TERMINAL"):
    monkeypatch.setattr(stopping_flags, "SERVICE_MODE", mode)
    calls, ownings = _cap_spy_scenario(LongControl(DummyCarParams()), monkeypatch)
    assert not any(ownings), f"{mode} must not own pid-band frames"
    assert len(calls["align"]) == 300, f"{mode} dropped C4 calls"
    assert len(calls["approach"]) == 300, f"{mode} dropped C5 calls"


# --- LIVE_TERMINAL byte-identity on the pid band -----------------------------------------------------

def test_live_terminal_pid_band_wire_byte_identical_to_shadow(monkeypatch) -> None:
  # the stage-3 wiring (bypass extension, freeze term, reseed) must be inert under LIVE_TERMINAL:
  # on a pure pid-band script the LT wire equals the SHADOW (legacy) wire frame by frame
  def run(mode: str):
    monkeypatch.setattr(stopping_flags, "SERVICE_MODE", mode)
    lc = LongControl(_ki_car_params())
    toggles = DummyFrogPilotToggles()
    wires = []
    v, gap = 1.6, 9.0
    for _ in range(500):
      cs = DummyCarState(v_ego=v, a_ego=-0.15, standstill=False)
      wires.append(float(lc.update(active=True, CS=cs, a_target=-0.35, should_stop=False,
                                   distance_to_stop_target_m=-1.0, accel_limits=LIMITS,
                                   frogpilot_toggles=toggles, lead_status=True, lead_v=0.0,
                                   lead_d_rel=gap)))
      v = max(v - 0.15 * DT, 1.0)
      gap = max(gap - v * DT, 6.0)
    return wires, lc

  wires_shadow, _ = run("SHADOW")
  wires_lt, lc_lt = run("LIVE_TERMINAL")
  assert not lc_lt._service_live_owning
  assert wires_lt == wires_shadow  # byte-identical, frame by frame
