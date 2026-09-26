"""TEMPORARY brake-response test program (docs/stopping/brake_response_session_2026-09-26.md, plans KCS1, KCS2).

Scripted open-loop braking to a held stop, cycled through a fixed table of maneuvers, so the car's command-to-motion
response can be identified from 20 km/h to standstill (closed-loop stop logs cannot identify it: cycles 34/45/47).
Command computation only: no Params, no I/O. LongControl owns one instance and applies its FLOOR at the final writer
(while a rep runs or holds the floor IS the wire: with no lead, stop or fault the normal chain only lags behind the
scripted braking (the planner ramps out of it) or holds its own stop for the hook's intent, and on the car both
overwrote the soft segments and the hold build; an abort hands back or finishes under wire = min(normal chain, floor),
so any deeper demand passes from then on); controlsd builds
the inputs, loads/saves the progress counts and publishes the banner. Constructed only when
stopping_flags.IDENTIFICATION_HOOK is True on the Santa Fe HEV (FrogPilot identification_mode: all three distance
mappings act as NOTHING, physical wheel button only, Standard personality, Traffic off; saved Params never written).

Physical distance button (PressTimer; a press ends only after MIN_PRESS_S of release). State lives in memory only,
so every controlsd start is OFF; only the per-maneuver completed counts persist (controlsd).
- a fresh LONG press (0.5 s) turns test mode on (ARMED) or off; card sets the set speed to SET_SPEED_KPH.
- READY = every start condition held PRECONDITION_S at a steady cruise. After AUTO_START_S more of READY (0: at once)
  the next maneuver (fewest completed reps first) starts by itself; a SHORT press that begins in READY starts it at once
  on its end. Any failed condition restarts the wait. Other presses are discarded, never queued.
- ACTIVE walks the maneuver's segments. Below the block's INTENT_V the stop intent puts LongControl in the stopping
  state (StopReq at rest). At the wheel stop: HELD, the floor deepens at J_HOLD to A_HOLD and stays until the driver's brake, which
  ends the rep (counted after HOLD_MIN_S). Fast cycle (user request): HOLD_AUTO_S after the hold reaches A_HOLD the rep
  counts; if every launch condition holds (launch_failure) LAUNCH releases the hold to zero at J_GO in the stopping state,
  as the stopping service does when a lead departs, then hands the stopped car to the normal chain, which launches it
  back to the set speed; a blocker during the release re-holds at J_REHOLD (never a second launch in that rep); after
  the block's last rep the car holds. The driver's brake still ends a hold (RESUME continues the cycle).
- a press or an ordinary abort before the intent hands back (bounded release, cruise resumes); after the intent the
  current floor is kept to standstill and held (not counted). Gas, brake while moving or disengagement end authority
  on the same frame and turn test mode off. Faults lock test mode until controlsd restarts (a hold stays until the
  brake). DELETE this module, its wiring and tests in the step that consumes the response data (or rejects it)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import NamedTuple

from openpilot.selfdrive.car.cruise import CRUISE_LONG_PRESS

DT = 0.01
MIN_PRESS_S = 0.05          # shortest press, and the release a press needs before it ends
LONG_PRESS_S = CRUISE_LONG_PRESS * DT   # arms/disarms; a start press must be shorter
PRECONDITION_S = 0.5        # every start condition must hold continuously this long before a start is accepted (fast cycle)
AUTO_START_S = 0.0          # READY this long starts the shown maneuver by itself (user request: at once, no countdown)
V_ARM_MIN, V_ARM_MAX = 3.5, 9.0         # start band (the 20 km/h cruise; SET-/SET+ within about 13-32 km/h)
V_STEADY, A_STEADY, PLAN_STEADY = 0.5, 0.2, 0.15   # steady cruise at the start: |v - set speed|, |aEgo|, |plan aTarget|
V_OVER = 0.5                # a rep aborts if the car gets this much faster than at its current segment's start
STEER_MAX_DEG = 5.0
YAW_MAX = 0.03              # rad/s
LEAD_PROB_MAX = 0.10
RELEASE_JERK = 0.8          # m/s^3: handback release bound (a deeper normal demand passes immediately)
# stop intent (stopping state, StopReq at rest) from here down, in speed/standstill segments. KCS2 keeps the pid state
# (SCC14 release limit 3.0) down to 0.5 m/s, where the normal chain enters the stopping state (planner shouldStop).
INTENT_V = {"KCS1": 2.0, "KCS2": 0.5}
A_HOLD, J_HOLD = -0.70, 0.6            # secure hold built after the wheel stop (StoppingService A_HOLD_SECURE, J_HOLD)
STALL_V, STALL_DV, STALL_T = 2.5, 0.30, 2.0   # below STALL_V, less than STALL_DV of slowing in STALL_T (0.15 m/s^2): deepen to A_HOLD
CAP_S = 30.0                # no standstill this long after the press: lock
HOLD_MIN_S = 1.0            # a hold ended by the brake sooner does not count
HOLD_AUTO_S = 0.3           # fast cycle: the rep counts this long after the hold reaches A_HOLD, then LAUNCH (before StopReq, ~2.2 s)
J_GO = 1.2                  # LAUNCH release rate (StoppingService J_GO, its release-to-go when a lead departs)
J_REHOLD = 5.0              # a blocker during LAUNCH re-holds at this rate (the SCC14 lower jerk limit)
PLAN_GO = 0.2               # LAUNCH needs the planner to want to go (aTarget, m/s^2)
LAUNCH_CLEAN_S = 0.2        # after the count, every launch condition must hold this long (a one-frame model lead flicker waits)
LAUNCH_WAIT_S = 2.0         # ... within this long after the count; then the last blocker holds the car (a fault or a roll at once)
V_SET_TOL = 0.3             # m/s: a rep starts only at the test set speed (a SET engagement can set another one)
ROLL_V = 0.1                # m/s: a hold rolls only with measured speed (a one-frame standstill flicker at v 0 is not a roll)
NOTICE_S = 3.0              # an OFF or LOCKED notice stays on screen this long
SET_SPEED_KPH = 15          # test build: card sets it on every fresh long press (PressTimer): now if engaged, else at the next engagement
N_REPS = 6
DRIVER_ENDS = ("pedal", "disengaged")    # a rep ended by the driver turns test mode off
FAULT_ENDS = ("inputs", "car", "mapping", "fcw", "vehicle", "fault", "exception", "banner")   # these lock it


class Seg(NamedTuple):
  accel: float                # constant command (never positive)
  v_end: float | None = None  # ends at v <= v_end
  t_s: float | None = None    # ends after t_s; neither = ends at the wheel stop
  jerk: float | None = None   # m/s^3: move from the previous command to accel at this rate (None = a step; not the first segment)


# id, banner text, segments; table order breaks ties. Ids are unique across blocks (the log analysis reads both).
BLOCKS = {
  # plant identification from 20 km/h (B first: the shortest, firmest run checks the site and the hold)
  "KCS1": (
    ("B", "-1.0 to stop", (Seg(-1.0),)),
    ("A", "-0.5 to stop", (Seg(-0.5),)),
    ("C", "-1.0 to 9 km/h, -0.3 to stop", (Seg(-1.0, v_end=2.5), Seg(-0.3))),
    ("D", "-1.0 to 9 km/h, 0 for 2 s, -0.8 to stop", (Seg(-1.0, v_end=2.5), Seg(0.0, t_s=2.0), Seg(-0.8))),
    ("E", "-0.8 to 5 km/h, -0.3 for 2 s, -0.8 to stop", (Seg(-0.8, v_end=1.5), Seg(-0.3, t_s=2.0), Seg(-0.8))),
  ),
  # KCS1 showed a release below ~2.6 m/s loses 0.1-0.4 m/s^2 (-0.3 never finished a stop). KCS2 (the first G/F/H table
  # never reached the car) pairs held and released commands at one level in the pid state, as the normal chain runs them:
  # is a ramped release to -0.5 held at 9, 5 and 3 km/h (I, M, N) as it is when built from cruise (L); is -0.6 held where
  # -0.5 is not (J pairs with M); does route 2129's fade under -0.7 after a partial release come back (K), against an
  # uninterrupted -0.7 (P, first: the firmest run checks the site and the hold)?
  "KCS2": (
    ("P", "-0.7 to stop", (Seg(-0.7),)),
    ("L", "-0.5 to stop", (Seg(-0.5),)),
    ("I", "-1.0 to 9 km/h, ease to -0.5 to stop", (Seg(-1.0, v_end=2.5), Seg(-0.5, jerk=1.5))),
    ("M", "-1.0 to 5 km/h, ease to -0.5 to stop", (Seg(-1.0, v_end=1.5), Seg(-0.5, jerk=1.5))),
    ("J", "-1.0 to 5 km/h, ease to -0.6 to stop", (Seg(-1.0, v_end=1.5), Seg(-0.6, jerk=1.5))),
    ("N", "-1.0 to 3 km/h, ease to -0.5 to stop", (Seg(-1.0, v_end=0.8), Seg(-0.5, jerk=1.5))),
    ("K", "-0.7 to 7 km/h, -0.45 for 0.5 s, -0.7 to stop", (Seg(-0.7, v_end=1.9), Seg(-0.45, t_s=0.5), Seg(-0.7))),
  ),
}
PLAN_ID = "KCS2"
MANEUVERS = BLOCKS[PLAN_ID]


@dataclass
class HookInputs:
  """Validated envelope inputs, built by controlsd every frame. Any non-finite/missing value = unusable."""
  valid: bool                 # carState/radarState/modelV2/longitudinalPlan/livePose/frogpilotCarState/selfdriveState valid AND alive
  santa_fe: bool
  long_active: bool           # CC.longActive and openpilot longitudinal
  enabled: bool               # selfdriveState.enabled
  pid_state: bool             # LongCtrlState.pid (not stopping/starting/off)
  v_ego: float
  a_ego: float
  v_cruise: float             # set speed, m/s
  gas: bool
  brake: bool
  force_coast: bool
  pause_long: bool
  standstill: bool
  steer_deg: float
  yaw_rate: float
  blinker: bool
  steer_fault: bool
  esp_active: bool
  acc_faulted: bool
  can_valid: bool
  gear_drive: bool
  stock_aeb: bool
  stock_fcw: bool
  lead_status: bool           # leadOne.status or leadTwo.status
  radar_error: bool
  lead_prob: float            # max model lead probability (leadsV3[0], [1])
  plan_has_lead: bool
  plan_should_stop: bool
  plan_fcw: bool
  stop_target_m: float        # distanceToStopTarget (-1 none)
  plan_accel: float           # longitudinalPlan.aTarget: the planner's own demand, independent of the wire
  distance_pressed: bool      # physical wheel distance button (identification mode)
  distance_long: bool         # card's long/very-long classification of the current press
  mapping_ok: bool            # identification mode active and every distance mapping is NOTHING
  experimental: bool          # selfdriveState.experimentalMode: the fast cycle needs the ACC planner (e2e says stop at standstill)


@dataclass
class HookOutput:
  floor: float | None = None  # wire = min(normal chain, floor); None = no effect
  own: bool = False           # wire = floor (a running or held rep); False = wire = min(normal chain, floor)
  stop_intent: bool = False   # LongControl ORs it into should_stop on the next frame
  rep_done: str = ""          # maneuver id whose rep just completed (controlsd saves the counts)
  state: str = "OFF"
  maneuver: str = ""
  rep: int = 0
  seg: int = 0
  reason: str = ""            # why the last rep or test mode ended, or the lock reason
  text1: str = ""
  text2: str = ""
  changed: bool = False       # state or segment transition this frame (log it)


def _finite(*xs) -> bool:
  return all(isinstance(x, (int, float)) and math.isfinite(float(x)) for x in xs)


def precondition_failure(i: HookInputs, normal_accel: float, start_accel: float | None = None, intent: bool = False) -> str | None:
  """First failing condition or None; a fault (FAULT_ENDS) always reports before an ordinary reason. start_accel given
  = the start gate (band, steady cruise, no deeper demand than the first command); otherwise a running rep, where the
  stop intent makes the stopping state, standstill and a planner stop expected."""
  if not i.valid or not _finite(i.v_ego, i.a_ego, i.v_cruise, i.steer_deg, i.yaw_rate, i.lead_prob, i.stop_target_m, i.plan_accel,
                                normal_accel):
    return "inputs"
  if not i.santa_fe:
    return "car"
  if not i.mapping_ok:
    return "mapping"
  if i.radar_error or i.stock_aeb or i.stock_fcw or i.plan_fcw:
    return "fcw"
  if i.esp_active or i.acc_faulted or not i.can_valid or not i.gear_drive:
    return "vehicle"
  if not (i.long_active and i.enabled):
    return "disengaged"
  if not intent and (not i.pid_state or i.standstill):
    return "state"
  if i.gas or i.brake or i.force_coast or i.pause_long:
    return "pedal"
  if start_accel is not None and not (V_ARM_MIN <= i.v_ego <= V_ARM_MAX):
    return "speed"
  if start_accel is not None and abs(i.v_cruise - SET_SPEED_KPH / 3.6) > V_SET_TOL:
    return "set speed"
  if start_accel is not None and i.experimental:
    return "experimental"
  if i.lead_status or i.lead_prob >= LEAD_PROB_MAX or i.plan_has_lead:
    return "lead"
  if not intent and (i.plan_should_stop or (0.0 <= i.stop_target_m < 200.0)):
    return "stop"
  if abs(i.steer_deg) > STEER_MAX_DEG or abs(i.yaw_rate) > YAW_MAX or i.blinker or i.steer_fault:
    return "steer"
  if start_accel is not None:
    if min(normal_accel, i.plan_accel) < start_accel:
      return "demand"
    if abs(i.v_ego - i.v_cruise) > V_STEADY or abs(i.a_ego) > A_STEADY or abs(i.plan_accel) > PLAN_STEADY:
      return "settling"
  return None


def launch_failure(i: HookInputs, normal_accel: float, standstill: bool = True) -> str | None:
  """First reason the car must not drive off by itself, or None: the start gate's checks without the speed band, steady
  cruise and state (the stop intent holds the stopping state), plus no stop ahead and a planner that wants to go.
  standstill=False during the release itself, where the car may begin to roll."""
  fail = precondition_failure(i, normal_accel, None, intent=True)
  if fail is not None:
    return fail
  if standstill and not i.standstill:
    return "rolling"
  if i.plan_should_stop or 0.0 <= i.stop_target_m < 200.0:
    return "stop"
  if i.experimental:
    return "experimental"
  if i.plan_accel < PLAN_GO:
    return "planner"
  return None


@dataclass
class PressTimer:
  """The debounced physical distance button, shared by the hook and card so both see the same long press. A press
  ends only after the button reads released for MIN_PRESS_S; a shorter dropout is part of the press and counts toward
  its length. A press already held when the timer starts, or across a gap, is never fresh."""
  pressed: bool = True
  fresh: bool = False         # began from a debounced release and not used yet
  t: float = 0.0              # length of the current press, dropouts included
  release_t: float = 0.0

  def gap(self):
    self.pressed, self.fresh, self.release_t = True, False, 0.0

  def update(self, raw: bool, dt: float) -> tuple[bool, bool]:
    """One frame of the raw button -> (a press began, the press ended)."""
    if raw:
      began = not self.pressed
      if began:
        self.pressed, self.fresh, self.t = True, True, 0.0
      else:
        self.t += self.release_t
      self.release_t = 0.0
      self.t += dt
      return began, False
    self.release_t += dt
    ended = self.pressed and self.release_t >= MIN_PRESS_S - 1e-9
    if ended:
      self.pressed = False
    return False, ended

  def long(self, raw: bool) -> bool:
    return raw and self.fresh and self.t >= LONG_PRESS_S


@dataclass
class IdentificationHook:
  state: str = "OFF"          # OFF | ARMED | READY | ACTIVE | HELD | LAUNCH | HANDBACK | LOCKED
  done: dict[str, int] = field(default_factory=lambda: {m[0]: 0 for m in MANEUVERS})
  _locked: str = ""           # fault reason: LOCKED for the rest of this instance
  _pre_t: float = 0.0
  _press: PressTimer = field(default_factory=PressTimer)
  _press_long: bool = False   # the card classified the current press long
  _ready_at_press: bool = False
  _man: int = 0               # MANEUVERS index of the current (or next) rep
  _seg: int = 0
  _seg_t: float = 0.0
  _rep_t: float = 0.0
  _v_seg: float = 0.0         # speed at the start of the current segment (over-speed guard)
  _v: float = math.inf        # the last speed seen in a rep (a lock at or below INTENT_V finishes the stop)
  _intent: bool = False
  _finish: bool = False       # aborted after the intent: keep the floor to standstill, hold, do not count
  _stalled: bool = False      # the stall rule fired this rep: the floor only deepens from here
  _stall: list[tuple[float, float]] = field(default_factory=list)   # (rep time, v) over the last STALL_T
  _hold_t: float = 0.0
  _full_t: float = 0.0        # time at A_HOLD in this hold
  _counted: str = ""          # the tag of this hold's counted rep ("" = not counted yet); a rep counts once
  _hold_block: str = ""       # why this hold does not (or no longer) launch; latched for the rep
  _rehold: bool = False       # a blocked LAUNCH: the hold re-deepens at J_REHOLD
  _clean_t: float = 0.0       # after the count: how long every launch condition has held
  _last_cmd: float = 0.0
  _reason: str = ""
  _last: str = ""             # result of the last rep, for the banner
  _notice: tuple[str, str] = ("", "")
  _notice_t: float = 0.0

  # -- progress (plain dicts; controlsd owns the storage) -------------------------------------------
  def load(self, record) -> None:
    """Counts saved for PLAN_ID; anything else (another plan, malformed) starts from zero, never ahead."""
    self.done = {m[0]: 0 for m in MANEUVERS}
    if isinstance(record, dict) and record.get("plan") == PLAN_ID and isinstance(record.get("done"), dict):
      for k, n in record["done"].items():
        if k in self.done and isinstance(n, int) and not isinstance(n, bool):
          self.done[k] = min(max(n, 0), N_REPS)

  def progress(self) -> dict:
    return {"plan": PLAN_ID, "done": dict(self.done)}

  def _next(self) -> int | None:
    k = min(range(len(MANEUVERS)), key=lambda j: (self.done[MANEUVERS[j][0]], j))
    return None if self.done[MANEUVERS[k][0]] >= N_REPS else k

  # -- transitions ----------------------------------------------------------------------------------
  def _rest(self, state: str, text1: str = "", text2: str = ""):
    self.state = "LOCKED" if self._locked else state
    self._pre_t = 0.0
    self._ready_at_press = False
    self._intent = self._finish = False
    if self.state == "LOCKED":
      text1, text2 = f"TEST MODE LOCKED - {self._locked}", "restart the car to use test mode again"
    self._notice, self._notice_t = (text1, text2), NOTICE_S if text1 else 0.0

  def _tag(self) -> str:
    return f"{MANEUVERS[self._man][0]} {self.done[MANEUVERS[self._man][0]] + 1}/{N_REPS}"

  def _abort(self, reason: str, out: HookOutput, v: float = math.inf) -> HookOutput:
    """End a moving rep: above INTENT_V without the intent release to cruise (HANDBACK); else keep the floor to the stop."""
    self._reason = reason
    if reason in FAULT_ENDS:
      self._locked = self._locked or reason
    self._last = f"last: {self._tag()} aborted - {reason}"
    if self._intent or v <= INTENT_V[PLAN_ID]:
      self._intent = self._finish = True
      out.floor, out.stop_intent, out.changed = self._last_cmd, True, True
      out.text1, out.text2 = f"TEST {self._tag()} ABORTED - {reason}", "finishing the stop; brake to end"
      return out
    self.state = "HANDBACK"
    out.floor = self._last_cmd
    return self._handback_text(out)

  def _handback_text(self, out: HookOutput) -> HookOutput:
    out.text1 = f"TEST {self._tag()} ABORTED - {self._reason}"
    out.text2 = "releasing; test mode LOCKED" if self._locked else "releasing; cruise resumes and can accelerate"
    return out

  def lock(self, reason: str) -> HookOutput:
    """Fault latch for the rest of this instance. A moving rep hands back or finishes its stop; a hold stays until the
    brake."""
    self._locked = self._locked or reason
    self._last_cmd = min(self._last_cmd, 0.0) if _finite(self._last_cmd) else 0.0
    if self.state == "ACTIVE" and not self._finish:
      return self._finish_out(self._abort(reason, HookOutput(), self._v), "")
    if self.state == "LAUNCH":           # a fault during the release re-holds (never launches), until the brake
      self.state, self._intent, self._rehold, self._hold_block = "HELD", True, True, self._locked
    if self.state in ("ACTIVE", "HELD"):
      out = HookOutput(floor=self._last_cmd, stop_intent=True)
      if self.state == "HELD":
        out.text1, out.text2 = f"TEST LOCKED - {self._locked} - HELD", "brake to end; restart the car"
      else:
        out.text1, out.text2 = f"TEST {self._tag()} ABORTED - {self._reason or self._locked}", "finishing the stop; brake to end"
      return self._finish_out(out, "")
    if self.state == "HANDBACK":
      return self._finish_out(self._handback_text(HookOutput(floor=self._last_cmd)), "")
    self._rest("LOCKED")
    return self._finish_out(self._notice_out(HookOutput(), 0.0), "")

  def interrupt(self, dt: float = DT, driver: bool = False) -> HookOutput:
    """A LongControl input-fault frame (the hook is not updated): a gap for the button. A rep, hold or release locks;
    a driver pedal or disengagement seen on the fault frame also ends its authority at once (no floor survives to a
    later re-engagement); ARMED/READY only lose their qualification (a one-frame planner lag must not end the session)."""
    self._press.gap()
    if self.state in ("ACTIVE", "HELD", "LAUNCH", "HANDBACK") and driver:
      prev, self._locked = self.state, self._locked or "fault"
      self._rest("LOCKED")
      return self._finish_out(self._notice_out(HookOutput(), 0.0), prev)
    if self.state in ("ACTIVE", "HELD", "LAUNCH", "HANDBACK"):
      return self.lock("fault")
    prev = self.state
    out = HookOutput()
    if self.state in ("ARMED", "READY"):
      self.state, self._pre_t, self._ready_at_press = "ARMED", 0.0, False
      out.text1, out.text2 = "TEST ARMED - waiting: fault", self._last or "long press = off"
    else:
      self._notice_out(out, dt)
    return self._finish_out(out, prev)

  def reset(self):
    """LongControl.reset(), on every frame without longitudinal control: ARMED/READY lose their qualification but stay
    armed, so test mode can be armed while parked. A rep, hold or release ends on the same frame's update (long_active
    is False there); an input fault already locked the hook through interrupt()."""
    if self.state in ("ARMED", "READY"):
      self._pre_t = 0.0
      self._ready_at_press = False

  def update(self, i: HookInputs, normal_accel: float, dt: float = DT) -> HookOutput:
    """Advance one control frame. normal_accel is the normal chain's final command this frame."""
    prev = self.state
    try:
      out = self._update(i, float(normal_accel), float(dt), HookOutput())
    except Exception:
      # any defect locks test mode; a moving rep hands back (or keeps its floor to the stop) and a release still runs
      # out even if every frame raises, but a driver pedal or disengagement still ends hook authority at once
      self._locked = self._locked or "exception"
      driver = not (getattr(i, "long_active", False) and getattr(i, "enabled", False)) or getattr(i, "gas", True) or getattr(i, "brake", True)
      if self.state in ("ACTIVE", "HELD", "LAUNCH", "HANDBACK") and driver:
        self._rest("LOCKED")
        out = self._notice_out(HookOutput(), 0.0)
      elif self.state == "HANDBACK":
        self._last_cmd = min(self._last_cmd, 0.0) if _finite(self._last_cmd) else 0.0
        out = self._release(normal_accel if _finite(normal_accel) else math.nan, DT, HookOutput())
      else:
        out = self.lock("exception")
    return self._finish_out(out, prev)

  def _finish_out(self, out: HookOutput, prev: str) -> HookOutput:
    out.state = self.state
    if not out.maneuver:                  # the rep-end frame labels the finished rep itself
      out.maneuver, out.seg, out.rep = MANEUVERS[self._man][0], self._seg + 1, self.done[MANEUVERS[self._man][0]] + 1
    out.reason = self._locked if self.state == "LOCKED" else self._reason
    out.changed = out.changed or self.state != prev
    return out

  def _update(self, i: HookInputs, normal_accel: float, dt: float, out: HookOutput) -> HookOutput:
    released = False
    if not i.valid:
      self._press.gap()                     # nothing observed: the press in progress never counts
    else:
      began, released = self._press.update(bool(i.distance_pressed), dt)
      if began:
        self._press_long = False
        self._ready_at_press = self._pre_t >= PRECONDITION_S   # readiness BEFORE this frame is credited
      if self._press.pressed or released:   # the press and its release frames until it ends
        self._press_long = self._press_long or bool(i.distance_long)
    # own timer only: the card keeps counting a press begun one frame after a long one, so its flag can be stale
    long_press = self._press.long(bool(i.distance_pressed))
    pressed_now = bool(i.distance_pressed) and i.valid
    if self.state in ("ACTIVE", "HELD", "LAUNCH", "HANDBACK") and pressed_now:
      self._press.fresh = False           # a press during a rep, hold or release only cancels (or is ignored)

    moving = self.state in ("ACTIVE", "HELD", "LAUNCH", "HANDBACK")
    fail = precondition_failure(i, normal_accel, None if moving else self._first_accel(), self._intent)
    if moving and fail in FAULT_ENDS:
      self._locked = self._locked or fail   # a fault always locks, whatever ends the rep or its release

    # disengagement (the panda then accepts only a zero request) or a driver pedal ends hook authority on this frame:
    # no floor and no stop intent; the normal chain and the driver own it. The brake in a hold is the normal end.
    if moving and (not (i.long_active and i.enabled) or i.gas or i.brake):
      if self.state in ("HELD", "LAUNCH") and i.brake and not i.gas:
        man = MANEUVERS[self._man][0]
        out.maneuver, out.rep, out.seg = man, self.done[man] + (0 if self._counted else 1), self._seg + 1
        if not self._counted and not self._finish and not self._locked and self._hold_t >= HOLD_MIN_S - 1e-9:
          self._count(out)
        elif not self._counted:
          if not self._finish and not self._locked:
            self._reason = "short-hold"
          self._last = f"last: {self._tag()} not counted - {self._reason}"
        tag = self._counted or self._tag()
        nxt = self._next()
        if nxt is not None:
          self._man = nxt
        self._rest("ARMED", f"{tag} DONE" if self._counted else f"{tag} NOT COUNTED - {self._reason}",
                   f"next {self._tag()}: {MANEUVERS[self._man][1]}" if nxt is not None else "block complete")
        return self._notice_out(out, 0.0)
      if self.state == "HANDBACK" and self._reason not in DRIVER_ENDS:
        self._rest("ARMED")
        return self._notice_out(out, 0.0)
      self._reason = "pedal" if i.gas or i.brake else "disengaged"
      if self.state != "HANDBACK":
        self._last = f"last: {self._tag()} aborted - {self._reason}"
      self._rest("OFF", f"TEST {self._tag()} ABORTED - {self._reason}", "test mode off; long press distance to arm")
      return self._notice_out(out, 0.0)

    if self.state == "HANDBACK":
      return self._release(normal_accel, dt, out)
    if self.state == "LAUNCH":
      return self._launch(i, normal_accel, dt, out)
    if self.state == "HELD":
      return self._held(i, fail, normal_accel, dt, out)
    if self.state == "ACTIVE":
      return self._active(i, fail, normal_accel, pressed_now, dt, out)

    if self.state in ("OFF", "LOCKED"):
      if long_press and i.santa_fe and i.mapping_ok:
        self._press.fresh = False         # the rest of the press does nothing
        self._rest("ARMED")               # stays LOCKED (and shows why) after a fault
        if self.state == "ARMED":
          nxt = self._next()
          self._man = self._man if nxt is None else nxt
          return self._armed_out(fail, dt, out, False)
      return self._notice_out(out, dt)

    # ARMED / READY
    if long_press:
      self._press.fresh = False
      self._rest("OFF", "TEST MODE OFF", "long press distance to arm")
      return self._notice_out(out, 0.0)
    return self._armed_out(fail, dt, out, released)

  def _first_accel(self) -> float:
    return MANEUVERS[self._man][2][0].accel

  def _armed_out(self, fail: str | None, dt: float, out: HookOutput, released: bool) -> HookOutput:
    nxt = self._next()
    if nxt is None:
      self.state = "ARMED"
      out.text1, out.text2 = "TEST BLOCK COMPLETE - long press = off", f"plan {PLAN_ID}: {N_REPS} reps of every maneuver"
      return out
    self._man = nxt
    self._pre_t = self._pre_t + dt if fail is None else 0.0
    ready = self._pre_t >= PRECONDITION_S
    # a failed condition or a long classification during the press (or its release) discards it
    self._ready_at_press = self._ready_at_press and fail is None and not self._press_long
    pressed_start = (released and self._press.fresh and self._ready_at_press and ready
                     and MIN_PRESS_S - 1e-9 <= self._press.t < LONG_PRESS_S)
    # the countdown waits while the button is held (a long press may be coming: it disarms instead)
    auto_start = self._pre_t >= PRECONDITION_S + AUTO_START_S - 1e-9 and not self._press.pressed
    if pressed_start or auto_start:
      self._press.fresh = self._press.fresh and not pressed_start
      self.state = "ACTIVE"
      self._seg, self._seg_t, self._rep_t, self._v_seg, self._v = 0, 0.0, 0.0, 0.0, math.inf
      self._intent = self._finish = self._stalled = False
      self._stall, self._hold_t, self._reason = [], 0.0, ""
      self._last_cmd = self._first_accel()
      out.floor, out.own, out.changed = self._last_cmd, True, True
      out.text1, out.text2 = self._active_text(), "press = cancel (releases, cruise resumes)"
      return out
    if released:
      self._press.fresh = False
    self.state = "READY" if ready else "ARMED"
    if not ready and self._notice_t > 0.0:   # the last rep's result stays on screen for NOTICE_S
      return self._notice_out(out, dt)
    if ready:
      left = max(PRECONDITION_S + AUTO_START_S - self._pre_t, 0.0)
      out.text1 = f"TEST {self._tag()} STARTS IN {left:.1f} s"
      out.text2 = f"{MANEUVERS[self._man][0]}: {MANEUVERS[self._man][1]}; brake = not now"
    else:
      out.text1 = f"TEST ARMED - waiting: {fail or 'settling'}"
      out.text2 = f"next {self._tag()}: {MANEUVERS[self._man][1]}; long press = off"
    return out

  def _active_text(self) -> str:
    return f"TEST {self._tag()} s{self._seg + 1} {self._last_cmd:+.2f}"

  def _active(self, i: HookInputs, fail: str | None, normal_accel: float, pressed: bool, dt: float, out: HookOutput) -> HookOutput:
    v = self._v = float(i.v_ego)
    if self._seg_t == 0.0:
      self._v_seg = v
    self._rep_t += dt
    self._seg_t += dt
    if not self._finish:
      if fail is None and v > self._v_seg + V_OVER:
        fail = "speed"
      if fail is not None:
        return self._abort(fail, out, v)
      if pressed:
        return self._abort("press", out, v)   # the start needed a release, so this is a new press
    if self._rep_t >= CAP_S:
      self._locked = self._locked or "vehicle"
      return self._abort("vehicle", out, v) if not self._finish else self._hold_start(out)
    if i.standstill:
      return self._hold_start(out)
    segs = MANEUVERS[self._man][2]
    if not self._finish:
      seg = segs[self._seg]
      if self._seg + 1 < len(segs) and ((seg.v_end is not None and v <= seg.v_end) or (seg.t_s is not None and self._seg_t >= seg.t_s - 1e-9)):
        self._seg, self._seg_t, self._stall = self._seg + 1, 0.0, []
        out.changed = True
      seg = segs[self._seg]
      cmd = seg.accel
      if seg.jerk is not None:
        cmd = min(max(cmd, self._last_cmd - seg.jerk * dt), self._last_cmd + seg.jerk * dt)
      if seg.t_s is None and v <= INTENT_V[PLAN_ID]:
        self._intent = True
    else:
      cmd = self._last_cmd
    # stall (within one speed/standstill-ended segment): slowing less than STALL_DV over STALL_T below STALL_V ->
    # deepen to the hold at J_HOLD and never back (sticky for the rep)
    self._stall = [(t, s) for t, s in self._stall if t >= self._rep_t - STALL_T - 1e-9] + [(self._rep_t, v)]
    if (not self._stalled and segs[self._seg].t_s is None and v < STALL_V and self._stall[0][0] <= self._rep_t - STALL_T + 1e-9
            and self._stall[0][1] - v < STALL_DV):
      self._stalled = self._intent = True
      out.changed = True
    if self._stalled:
      cmd = min(cmd, max(self._last_cmd - J_HOLD * dt, A_HOLD) if self._last_cmd > A_HOLD else self._last_cmd)
    self._last_cmd = min(cmd, 0.0)
    out.floor, out.own, out.stop_intent = self._last_cmd, not self._finish, self._intent
    out.text1 = self._active_text() + f" - {v:.1f} m/s"
    if self._finish:
      out.text2 = "finishing the stop; brake to end"
    else:
      out.text2 = "press = finish and hold" if self._intent else "press = cancel (releases, cruise resumes)"
    return out

  def _hold_start(self, out: HookOutput) -> HookOutput:
    self.state, self._hold_t, self._full_t, self._intent = "HELD", 0.0, 0.0, True
    self._counted, self._hold_block, self._rehold, self._clean_t = "", "", False, 0.0
    out.changed = True
    return self._held_frame(0.0, out)

  def _count(self, out: HookOutput) -> None:
    """The hold's rep counts (once): the automatic count at HOLD_AUTO_S or the driver's brake before it."""
    man = MANEUVERS[self._man][0]
    self._counted = self._tag()
    self.done[man] += 1
    out.rep_done, out.changed = man, True
    self._reason = "stalled" if self._stalled else "complete"
    self._last = f"last: {self._counted} done" + (" (stalled)" if self._stalled else "")

  def _held(self, i: HookInputs, fail: str | None, normal_accel: float, dt: float, out: HookOutput) -> HookOutput:
    self._hold_t += dt
    rolling = not i.standstill and i.v_ego > ROLL_V
    if not self._finish and (rolling or (fail == "lead" and not self._counted)):
      # the car rolls, or a lead appears before the count: stop owning the wire (a deeper normal demand passes); the rep
      # does not count and the car never launches from this hold. After the count a lead only blocks the launch.
      self._finish, self._reason, out.changed = True, "rolling" if rolling else "lead", True
      self._hold_block = self._hold_block or self._reason
      if not self._counted:
        self._last = f"last: {self._tag()} not counted - {self._reason}"
    if self._last_cmd > A_HOLD:
      self._last_cmd = max(self._last_cmd - (J_REHOLD if self._rehold else J_HOLD) * dt, A_HOLD)
    if self._last_cmd <= A_HOLD + 1e-9:
      self._full_t += dt
    if not self._counted and not self._finish and not self._locked and self._full_t >= HOLD_AUTO_S - 1e-9:
      self._count(out)
      if self._next() is None:
        self._hold_block = "block complete"
    if self._counted and not self._hold_block and not self._finish and not self._locked:
      block = launch_failure(i, normal_accel)
      self._clean_t = self._clean_t + dt if block is None else 0.0
      if self._clean_t >= LAUNCH_CLEAN_S - 1e-9:
        self.state, out.changed = "LAUNCH", True
        return self._launch_frame(out)
      if block is not None and (block in FAULT_ENDS or self._full_t >= HOLD_AUTO_S + LAUNCH_WAIT_S - 1e-9):
        self._hold_block, out.changed = block, True
    return self._held_frame(self._hold_t, out)

  def _held_frame(self, hold_t: float, out: HookOutput) -> HookOutput:
    out.floor, out.own, out.stop_intent = self._last_cmd, not (self._finish or self._locked), True
    if self._locked:
      out.text1, out.text2 = f"TEST LOCKED - {self._locked} - HELD", "brake to end; restart the car"
    elif self._counted:
      out.text1 = f"TEST {self._counted} DONE - " + (f"holding: {self._hold_block}" if self._hold_block else "driving off")
      out.text2 = "brake to continue" if self._hold_block else "brake = stay stopped"
    else:
      out.text1 = f"TEST {self._tag()} STOPPED - hold {hold_t:.1f} s"
      out.text2 = "brake to finish" + (" (not counted)" if self._finish else "")
    return out

  def _launch(self, i: HookInputs, normal_accel: float, dt: float, out: HookOutput) -> HookOutput:
    """LAUNCH: release the hold to zero at J_GO in the stopping state (the stop intent stays, as the stopping service's
    own release when a lead departs), then drop the intent with the wire at zero, then hand the car to the normal chain
    (next frame, now leaving the stopping state), which launches it. Any blocker re-holds for the rest of the hold."""
    block = launch_failure(i, normal_accel, standstill=False)
    if block is not None:
      self.state, self._intent, self._rehold, self._hold_block, out.changed = "HELD", True, True, block, True
      return self._held_frame(self._hold_t, out)
    if self._last_cmd >= 0.0 and not self._intent:   # the intent was dropped last frame: hand over
      nxt = self._next()
      self._man = self._man if nxt is None else nxt
      self._rest("ARMED", f"{self._counted} DONE", f"next {self._tag()}: {MANEUVERS[self._man][1]}")
      return self._notice_out(out, 0.0)
    self._last_cmd = min(self._last_cmd + J_GO * dt, 0.0)
    if self._last_cmd >= 0.0:
      self._intent = False                           # zero on the wire; LongControl leaves the stopping state next frame
    return self._launch_frame(out)

  def _launch_frame(self, out: HookOutput) -> HookOutput:
    out.floor, out.own, out.stop_intent = self._last_cmd, True, self._intent
    out.text1, out.text2 = f"TEST {self._counted} DONE - driving off", "brake = stay stopped"
    return out

  def _release(self, normal_accel: float, dt: float, out: HookOutput) -> HookOutput:
    """One HANDBACK frame: release toward the normal chain at RELEASE_JERK; a deeper normal demand wins immediately."""
    cap = min(self._last_cmd + RELEASE_JERK * dt, 0.0)   # a release bound is never a positive command
    self._last_cmd = min(normal_accel, cap) if _finite(normal_accel) else cap
    out.floor = cap
    self._handback_text(out)
    if normal_accel <= cap + 1e-6 or self._last_cmd >= 0.0:
      if self._reason in DRIVER_ENDS:
        self._rest("OFF", "TEST MODE OFF", "long press distance to arm")
      else:
        self._rest("ARMED")
    return out

  def _notice_out(self, out: HookOutput, dt: float) -> HookOutput:
    if self._notice_t > 0.0:
      out.text1, out.text2 = self._notice
      self._notice_t -= dt
    return out
