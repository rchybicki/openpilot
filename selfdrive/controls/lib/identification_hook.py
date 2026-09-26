"""TEMPORARY brake-response test mode (docs/stopping/brake_response_session_2026-09-26.md).

Injects one fixed open-loop braking step as the FINAL wire owner so the car's command-to-motion response can be
measured (closed-loop stop logs cannot identify it: cycles 34/45/47). Command computation only: no Params, no I/O.
LongControl owns one instance and applies the output at its final writer; controlsd builds the inputs from validated
messages and publishes the banner. Constructed only when stopping_flags.IDENTIFICATION_HOOK is True on the Santa Fe HEV
(FrogPilot identification_mode: master flag + Santa Fe HEV + openpilot longitudinal -> all three distance mappings act
as NOTHING, physical wheel button only, canonical Standard personality, Traffic off; saved Params are never written).

Physical distance button. The state lives in memory only, so every controlsd start is OFF. A press ends only after
the button reads released for MIN_PRESS_S, so a short dropout inside a hold is not a release.
- a fresh LONG press (CRUISE_LONG_PRESS, the car's own 0.5 s) turns test mode on (ARMED) or off at that moment; the
  rest of that press does nothing. A press held through startup or an input fault never counts.
- READY = every start condition held continuously for PRECONDITION_S. A SHORT press (MIN_PRESS_S to under 0.5 s) that
  begins in READY, with every condition held through its end, starts ONE trial. Other presses are discarded.
- any press during a trial cancels it at once (that press does nothing else); the braking releases at RELEASE_JERK,
  then normal cruise resumes and can accelerate. Test mode stays armed.
Every trial is the same step (STEP_ACCEL for STEP_S, ending early at V_END); the hook never commands positive
acceleration. Any normal or planner demand deeper than the step blocks a start and aborts a trial, and passes at once.
A pedal or disengagement during the step turns test mode off. A fault (FAULT_ENDS) locks it until controlsd restarts.
DELETE this module, its wiring and tests in the program step that consumes the response data (or rejects it)."""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.selfdrive.car.cruise import CRUISE_LONG_PRESS

DT = 0.01
MIN_PRESS_S = 0.05          # shortest press, and the release a press needs before it ends
LONG_PRESS_S = CRUISE_LONG_PRESS * DT   # arms/disarms; a start press must be shorter
PRECONDITION_S = 2.0        # every precondition must hold continuously this long before a start is accepted
V_ARM_MIN, V_ARM_MAX = 7.0, 11.0
V_END = 4.5                 # the step ends early here
V_ABORT_MIN, V_ABORT_MAX = 4.0, 12.0
STEER_MAX_DEG = 5.0
YAW_MAX = 0.03              # rad/s
LEAD_PROB_MAX = 0.10
RELEASE_JERK = 0.8          # m/s^3: handback release bound (a deeper normal demand passes immediately)
STEP_ACCEL, STEP_S = -0.5, 3.0   # the only profile, repeated by every trial; deeper steps need a separate reviewed protocol
NOTICE_S = 3.0              # an OFF or LOCKED notice stays on screen this long
DRIVER_ENDS = ("pedal", "disengaged")    # a step ended by the driver turns test mode off
FAULT_ENDS = ("inputs", "car", "mapping", "fcw", "vehicle", "fault", "exception", "banner")   # these lock it


@dataclass
class HookInputs:
  """Validated envelope inputs, built by controlsd every frame. Any non-finite/missing value = unusable."""
  valid: bool                 # carState/radarState/modelV2/longitudinalPlan/livePose/frogpilotCarState/selfdriveState valid AND alive
  santa_fe: bool
  long_active: bool           # CC.longActive and openpilot longitudinal
  enabled: bool               # selfdriveState.enabled
  pid_state: bool             # LongCtrlState.pid (not stopping/starting/off)
  v_ego: float
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


@dataclass
class HookOutput:
  active: bool = False        # the hook owns the wire this frame
  handback: bool = False      # releasing toward the normal chain (min(normal, cap))
  accel: float = 0.0          # scripted command (active) or the release cap (handback)
  state: str = "OFF"
  trial: int = 0
  reason: str = ""            # why the last trial ended, or the lock reason
  text1: str = ""
  text2: str = ""
  changed: bool = False       # state transition this frame (log it)


def _finite(*xs) -> bool:
  return all(isinstance(x, (int, float)) and math.isfinite(float(x)) for x in xs)


def precondition_failure(i: HookInputs, for_start: bool, normal_accel: float) -> str | None:
  """First failing precondition or None; a fault (FAULT_ENDS) always reports before an ordinary reason. for_start
  applies the arming speed band; during a trial the wider abort band and V_END apply instead. normal_accel is the
  normal chain's final command this frame."""
  if not i.valid or not _finite(i.v_ego, i.steer_deg, i.yaw_rate, i.lead_prob, i.stop_target_m, i.plan_accel, normal_accel):
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
  if not i.pid_state or i.standstill:
    return "state"
  if i.gas or i.brake or i.force_coast or i.pause_long:
    return "pedal"
  if for_start:
    if not (V_ARM_MIN <= i.v_ego <= V_ARM_MAX):
      return "speed"
  elif not (V_ABORT_MIN <= i.v_ego <= V_ABORT_MAX):
    return "speed"
  if i.lead_status or i.lead_prob >= LEAD_PROB_MAX or i.plan_has_lead:
    return "lead"
  if i.plan_should_stop or (0.0 <= i.stop_target_m < 200.0):
    return "stop"
  if min(normal_accel, i.plan_accel) < STEP_ACCEL:
    return "demand"
  if abs(i.steer_deg) > STEER_MAX_DEG or abs(i.yaw_rate) > YAW_MAX or i.blinker or i.steer_fault:
    return "steer"
  return None


@dataclass
class IdentificationHook:
  state: str = "OFF"          # OFF | ARMED | READY | ACTIVE | HANDBACK | LOCKED
  trial: int = 0              # trials started by this instance (labels only)
  _locked: str = ""           # fault reason: LOCKED for the rest of this instance
  _pre_t: float = 0.0
  _pressed: bool = True       # debounced button; True at start so a press held through a restart never counts
  _release_t: float = 0.0     # how long the button has read released
  _press_fresh: bool = False  # the current press began from a debounced release and has not been used
  _press_t: float = 0.0       # observed duration of the current press
  _press_long: bool = False   # the card classified the current press long
  _ready_at_press: bool = False
  _t: float = 0.0
  _last_cmd: float = 0.0
  _reason: str = ""
  _last: str = ""             # result of the last trial, for the banner
  _notice: tuple[str, str] = ("", "")
  _notice_t: float = 0.0

  def _rest(self, state: str, text1: str = "", text2: str = ""):
    self.state = "LOCKED" if self._locked else state
    self._pre_t = 0.0
    self._ready_at_press = False
    if self.state == "LOCKED":
      text1, text2 = f"TEST MODE LOCKED - {self._locked}", "restart the car to use test mode again"
    self._notice, self._notice_t = (text1, text2), NOTICE_S if text1 else 0.0

  def _end(self, reason: str, out: HookOutput) -> HookOutput:
    """End the trial: bounded release from the last scripted command toward the normal chain."""
    self._reason = reason
    if reason in FAULT_ENDS:
      self._locked = self._locked or reason
    self._last = f"last: trial {self.trial} " + ("complete" if reason == "complete" else f"aborted - {reason}")
    self.state = "HANDBACK"
    out.handback, out.accel = True, self._last_cmd
    return self._handback_text(out)

  def _handback_text(self, out: HookOutput) -> HookOutput:
    out.text1 = f"TEST {self.trial} " + ("COMPLETE" if self._reason == "complete" else f"ABORTED - {self._reason}")
    out.text2 = "braking releases; test mode LOCKED" if self._locked else "braking releases; normal cruise resumes and can accelerate"
    return out

  def _gap(self):
    # nothing observed: the press in progress never counts and a new press needs a fresh observed release
    self._pressed, self._press_fresh, self._release_t = True, False, 0.0

  def lock(self, reason: str) -> HookOutput:
    """Fault latch for the rest of this instance. An active trial or release hands back through the release bound."""
    self._locked = self._locked or reason
    if self.state in ("ACTIVE", "HANDBACK"):
      self._last_cmd = min(self._last_cmd, 0.0) if _finite(self._last_cmd) else 0.0
      return self._finish(self._end(reason, HookOutput()), "")
    self._rest("LOCKED")
    return self._finish(self._notice_out(HookOutput(), 0.0), "")

  def interrupt(self, dt: float = DT) -> HookOutput:
    """A LongControl input-fault frame (the hook is not updated): a gap for the button. A trial or its release locks;
    ARMED/READY only lose their qualification (a one-frame planner lag must not end the session)."""
    self._gap()
    if self.state in ("ACTIVE", "HANDBACK"):
      return self.lock("fault")
    prev = self.state
    out = HookOutput()
    if self.state in ("ARMED", "READY"):
      self.state, self._pre_t, self._ready_at_press = "ARMED", 0.0, False
      out.text1, out.text2 = "TEST MODE ARMED - waiting: fault", self._last or "long press distance to turn test mode off"
    else:
      self._notice_out(out, dt)
    return self._finish(out, prev)

  def reset(self):
    """LongControl.reset(), on every frame without longitudinal control: ARMED/READY lose their qualification but stay
    armed, so test mode can be armed while parked. An ACTIVE step or its release ends on the same frame's update
    (long_active is False there); an input fault already locked the hook through interrupt()."""
    if self.state in ("ARMED", "READY"):
      self._pre_t = 0.0
      self._ready_at_press = False

  def update(self, i: HookInputs, normal_accel: float, dt: float = DT) -> HookOutput:
    """Advance one control frame. normal_accel is the normal chain's final command this frame."""
    prev = self.state
    try:
      out = self._update(i, float(normal_accel), float(dt), HookOutput())
    except Exception:
      # any defect locks test mode; a trial hands back and its release still runs out even if every frame raises
      self._locked = self._locked or "exception"
      if self.state == "HANDBACK":
        self._last_cmd = min(self._last_cmd, 0.0) if _finite(self._last_cmd) else 0.0
        out = self._release(normal_accel if _finite(normal_accel) else math.nan, DT, HookOutput())
      else:
        out = self.lock("exception")
    return self._finish(out, prev)

  def _finish(self, out: HookOutput, prev: str) -> HookOutput:
    out.state, out.trial = self.state, self.trial
    out.reason = self._locked if self.state == "LOCKED" else self._reason
    out.changed = out.changed or self.state != prev
    return out

  def _update(self, i: HookInputs, normal_accel: float, dt: float, out: HookOutput) -> HookOutput:
    released = False
    if not i.valid:
      self._gap()
    elif i.distance_pressed:
      self._release_t = 0.0
      if not self._pressed:
        self._pressed, self._press_fresh, self._press_t, self._press_long = True, True, 0.0, False
        self._ready_at_press = self._pre_t >= PRECONDITION_S   # readiness BEFORE this frame is credited
      self._press_t += dt
    else:
      self._release_t += dt
      released = self._pressed and self._release_t >= MIN_PRESS_S - 1e-9
    if i.valid and self._pressed:           # the press and its release frames until it ends
      self._press_long = self._press_long or bool(i.distance_long)
    if released:
      self._pressed = False
    # own timer only: the card keeps counting a press begun one frame after a long one, so its flag can be stale
    long_press = i.distance_pressed and self._press_fresh and self._press_t >= LONG_PRESS_S
    if self.state in ("ACTIVE", "HANDBACK") and i.distance_pressed:
      self._press_fresh = False           # a press during a trial or its release only cancels

    fail = precondition_failure(i, self.state not in ("ACTIVE", "HANDBACK"), normal_accel)
    if self.state in ("ACTIVE", "HANDBACK") and fail in FAULT_ENDS:
      self._locked = self._locked or fail   # a fault always locks, whatever ends the trial or its release

    # disengagement (the panda then accepts only a zero request) or a driver pedal ends hook authority on this
    # frame, trial or handback: no scripted command and no release bound; the normal chain and the driver own it.
    # Interrupting the step turns test mode off; taking over its release does not.
    if self.state in ("ACTIVE", "HANDBACK") and (not (i.long_active and i.enabled) or i.gas or i.brake):
      if self.state == "ACTIVE":
        self._reason = "pedal" if i.gas or i.brake else "disengaged"
        self._last = f"last: trial {self.trial} aborted - {self._reason}"
        self._rest("OFF", f"TEST {self.trial} ABORTED - {self._reason}", "test mode off; long press distance to arm")
      elif self._reason in DRIVER_ENDS:
        self._rest("OFF", "TEST MODE OFF", "long press distance to arm")
      else:
        self._rest("ARMED")
      return self._notice_out(out, 0.0)

    if self.state == "HANDBACK":
      return self._release(normal_accel, dt, out)

    if self.state in ("OFF", "LOCKED"):
      if long_press and i.santa_fe and i.mapping_ok:
        self._press_fresh = False         # the rest of the press does nothing
        self._rest("ARMED")               # stays LOCKED (and shows why) after a fault
        if self.state == "ARMED":
          return self._armed_out(fail, dt, out, False)
      return self._notice_out(out, dt)

    if self.state == "ACTIVE":
      if fail is not None:
        return self._end(fail, out)       # a deeper demand passes through the handback min on this frame
      if i.distance_pressed:
        return self._end("press", out)    # the start needed a release, so this is a new press: cancel at once
      self._t += dt
      if self._t >= STEP_S - 1e-9 or i.v_ego <= V_END:
        return self._end("complete", out)
      out.active, out.accel = True, self._last_cmd
      out.text1 = f"TEST ACTIVE {self._last_cmd:+.2f} m/s^2 - {self._t:.1f} s"
      out.text2 = f"trial {self.trial} - press distance to cancel"
      return out

    # ARMED / READY
    if long_press:
      self._press_fresh = False
      self._rest("OFF", "TEST MODE OFF", "long press distance to arm")
      return self._notice_out(out, 0.0)
    return self._armed_out(fail, dt, out, released)

  def _release(self, normal_accel: float, dt: float, out: HookOutput) -> HookOutput:
    """One HANDBACK frame: release toward the normal chain at RELEASE_JERK; a deeper normal demand wins immediately."""
    cap = min(self._last_cmd + RELEASE_JERK * dt, 0.0)   # a release bound is never a positive command
    self._last_cmd = min(normal_accel, cap) if _finite(normal_accel) else cap
    out.handback, out.accel = True, cap
    self._handback_text(out)
    if normal_accel <= cap + 1e-6 or self._last_cmd >= 0.0:
      if self._reason in DRIVER_ENDS:
        self._rest("OFF", "TEST MODE OFF", "long press distance to arm")
      else:
        self._rest("ARMED")
    return out

  def _armed_out(self, fail: str | None, dt: float, out: HookOutput, released: bool) -> HookOutput:
    self._pre_t = self._pre_t + dt if fail is None else 0.0
    ready = self._pre_t >= PRECONDITION_S
    # a failed precondition or a long classification during the press (or its release) discards it
    self._ready_at_press = self._ready_at_press and fail is None and not self._press_long
    if (released and self._press_fresh and self._ready_at_press and ready
            and MIN_PRESS_S - 1e-9 <= self._press_t < LONG_PRESS_S):
      self._press_fresh = False
      self.trial += 1
      self._t, self._last_cmd, self._reason = 0.0, STEP_ACCEL, ""
      self.state = "ACTIVE"
      out.active, out.accel = True, STEP_ACCEL
      out.text1 = f"TEST ACTIVE {STEP_ACCEL:+.2f} m/s^2 - 0.0 s"
      out.text2 = f"trial {self.trial} - press distance to cancel"
      return out
    if released:
      self._press_fresh = False
    self.state = "READY" if ready else "ARMED"
    if ready:
      out.text1 = "TEST READY - short press distance to start"
      out.text2 = (self._last or f"step {STEP_ACCEL:+.1f} m/s^2 for {STEP_S:.0f} s") + "; long press = off"
    else:
      out.text1 = f"TEST MODE ARMED - waiting: {fail or 'settling'}"
      out.text2 = self._last or "long press distance to turn test mode off"
    return out

  def _notice_out(self, out: HookOutput, dt: float) -> HookOutput:
    if self._notice_t > 0.0:
      out.text1, out.text2 = self._notice
      self._notice_t -= dt
    return out
