"""TEMPORARY identification-drive step hook (program doc 2026-09-05, protocol v2; red-team 20260905-152855).

Injects scripted longitudinal acceleration commands as the FINAL wire owner so the actuator/plant can be identified
from open-loop steps (closed-loop stop logs cannot identify it: cycles 34/45/47). Pure module: no cereal, no
Params, no I/O. LongControl owns one instance and applies the output at its final writer; controlsd builds the
inputs from validated messages and shows the banner. Everything is off unless stopping_flags.IDENTIFICATION_HOOK is
True AND the arm file existed at process start AND the distance-button long/very-long mappings are NOTHING.

Trigger: a deliberate >= 1.5 s hold of the distance button, then release, starts ONE trial when the preconditions
have held continuously for 2.0 s. Any abort condition wins on the same frame; a trial never resumes; the driver
recovers speed manually (no positive commands). DELETE this module, its wiring and tests in the program step that
consumes the fitted plant (or rejects the collection)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

ARM_FILE = "/data/identification_hook.arm"     # created by SSH before the drive; latched at process start
DT = 0.01
HOLD_S = 1.5                # distance-button hold that starts a trial (release fires it)
PRECONDITION_S = 2.0        # every precondition must hold continuously this long before a start is accepted
V_ARM_MIN, V_ARM_MAX = 7.0, 11.0
V_END = 4.5                 # every command ends early here
V_ABORT_MIN, V_ABORT_MAX = 4.0, 12.0
STEER_MAX_DEG = 5.0
YAW_MAX = 0.03              # rad/s
LEAD_PROB_MAX = 0.10
RELEASE_JERK = 0.8          # m/s^3: handback release bound (safety may deepen immediately)
MAX_TRIALS = 24             # 8 trials x 3 repetitions per arm; a new arm file + restart is needed for more
TRIAL_DEADLINE_S = 5.0

# (name, accel_bp, time_bp): held steps and ramps; the crossing is -0.8 for 1 s then -2.2 for <= 2 s
TRIALS = (
  ("step -0.5", (-0.5, -0.5), (0.0, 3.0)),
  ("step -1.0", (-1.0, -1.0), (0.0, 3.0)),
  ("step -1.5", (-1.5, -1.5), (0.0, 3.0)),
  ("step -2.0", (-2.0, -2.0), (0.0, 2.5)),
  ("step -2.5", (-2.5, -2.5), (0.0, 2.0)),
  ("ramp -0.5>-2.0", (-0.5, -2.0), (0.0, 3.0)),
  ("ramp -2.0>-0.5", (-2.0, -0.5), (0.0, 3.0)),
  ("cross -0.8|-2.2", (-0.8, -0.8, -2.2, -2.2), (0.0, 1.0, 1.0, 3.0)),
)
# counterbalanced order across the three repetitions (depth vs battery/brake temperature/road drift)
ORDER = ((0, 1, 2, 3, 4, 5, 6, 7), (4, 3, 2, 1, 0, 7, 6, 5), (2, 0, 4, 1, 3, 6, 5, 7))


@dataclass
class HookInputs:
  """Validated envelope inputs, built by controlsd every frame. Any non-finite/missing value = unusable."""
  valid: bool                 # carState/radarState/modelV2/longitudinalPlan/livePose valid AND alive this frame
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
  distance_pressed: bool      # raw distance button
  mapping_ok: bool            # long AND very-long distance mappings are NOTHING


@dataclass
class HookOutput:
  active: bool = False        # the hook owns the wire this frame
  handback: bool = False      # releasing toward the normal chain (min(normal, cap))
  accel: float = 0.0          # scripted command (active) or the release cap (handback)
  state: str = "DISARMED"
  trial: int = 0
  name: str = ""
  reason: str = ""            # abort reason (latched until the next start)
  text1: str = ""
  text2: str = ""
  changed: bool = False       # state/trial transition this frame (log it)


def _finite(*xs) -> bool:
  return all(isinstance(x, (int, float)) and math.isfinite(float(x)) for x in xs)


def precondition_failure(i: HookInputs, for_start: bool) -> str | None:
  """First failing precondition (a string reason) or None. for_start applies the arming speed band;
  during a trial the wider abort band and V_END apply instead."""
  if not i.valid or not _finite(i.v_ego, i.steer_deg, i.yaw_rate, i.lead_prob, i.stop_target_m):
    return "inputs"
  if not i.santa_fe:
    return "car"
  if not i.mapping_ok:
    return "mapping"
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
  if i.radar_error or i.stock_aeb or i.stock_fcw or i.plan_fcw:
    return "fcw"
  if i.plan_should_stop or (0.0 <= i.stop_target_m < 200.0):
    return "stop"
  if abs(i.steer_deg) > STEER_MAX_DEG or abs(i.yaw_rate) > YAW_MAX or i.blinker or i.steer_fault:
    return "steer"
  if i.esp_active or i.acc_faulted or not i.can_valid or not i.gear_drive:
    return "vehicle"
  return None


def _interp(t, xs, ys):
  if t <= xs[0]:
    return ys[0]
  for k in range(1, len(xs)):
    if t <= xs[k]:
      x0, x1, y0, y1 = xs[k - 1], xs[k], ys[k - 1], ys[k]
      return y0 if x1 <= x0 else y0 + (y1 - y0) * (t - x0) / (x1 - x0)
  return ys[-1]


@dataclass
class IdentificationHook:
  armed: bool                                   # arm file present at process start
  state: str = "DISARMED"
  trial: int = 0                                # trials started this arm
  _pre_t: float = 0.0
  _hold_t: float = 0.0
  _t: float = 0.0
  _last_cmd: float = 0.0
  _reason: str = ""
  _current: tuple = field(default_factory=tuple)
  _latched_off: bool = False
  _ready_at_press: bool = False

  def __post_init__(self):
    self.state = "ARMED" if self.armed else "DISARMED"

  def _trial_spec(self, idx: int):
    rep, k = divmod(idx, len(TRIALS))
    return TRIALS[ORDER[rep % len(ORDER)][k]]

  def _handback(self, reason: str, out: HookOutput) -> HookOutput:
    self._reason = reason
    self.state = "HANDBACK"
    out.state, out.reason, out.changed = self.state, reason, True
    out.handback, out.accel = True, self._last_cmd
    out.text1, out.text2 = (f"STEP {'COMPLETE' if reason == 'complete' else 'ABORTED'} - {reason}",
                            "recover to 10-11 m/s, hold the distance button 1.5 s for the next trial")
    return out

  def abort(self, reason: str) -> None:
    """External abort (input fault, controller reset): an ACTIVE trial hands back (bounded release from the
    last scripted command); ARMED/READY lose their qualification. Never resumes."""
    if self.state == "ACTIVE":
      self._handback(reason, HookOutput())
    elif self.state in ("ARMED", "READY"):
      self._pre_t = self._hold_t = 0.0
      self._ready_at_press = False

  def update(self, i: HookInputs, normal_accel: float, dt: float = DT) -> HookOutput:
    """Advance one control frame. normal_accel is the normal chain's final command this frame (finite)."""
    out = HookOutput(state=self.state, trial=self.trial, reason=self._reason)
    try:
      return self._update(i, float(normal_accel), float(dt), out)
    except Exception:
      # any defect latches future trials OFF for the drive; the release stays bounded through HANDBACK
      self._latched_off = True
      if not _finite(self._last_cmd):
        self._last_cmd = 0.0
      self._last_cmd = min(self._last_cmd, 0.0)
      return self._handback("exception", out)

  def _update(self, i: HookInputs, normal_accel: float, dt: float, out: HookOutput) -> HookOutput:
    if self.state == "DISARMED":
      return out
    fail = precondition_failure(i, for_start=self.state != "ACTIVE")
    if self.state == "ACTIVE":
      spec = self._current
      if fail is not None:
        return self._handback(fail, out)
      if i.distance_pressed:
        return self._handback("press", out)    # any press during a trial aborts (the start required a release)
      self._t += dt
      cmd = float(_interp(self._t, spec[2], spec[1]))
      if self._t >= spec[2][-1] - 1e-9 or self._t >= TRIAL_DEADLINE_S or i.v_ego <= V_END:
        self._last_cmd = min(cmd, 0.0)
        return self._handback("complete", out)
      self._last_cmd = min(cmd, 0.0)
      out.active, out.accel = True, self._last_cmd
      out.text1 = f"STEP TEST ACTIVE - {self._last_cmd:+.2f} m/s^2 - {self._t:.1f} s"
      out.text2 = f"trial {self.trial}/{MAX_TRIALS}: {spec[0]}"
      return out
    if self.state == "HANDBACK":
      # release toward the normal chain at RELEASE_JERK; safety (a deeper normal demand) wins immediately
      cap = min(self._last_cmd + RELEASE_JERK * dt, 0.0)   # a release bound is never a positive command
      self._last_cmd = min(normal_accel, cap)
      out.handback, out.accel = True, cap
      out.text1, out.text2 = (f"STEP {'COMPLETE' if self._reason == 'complete' else 'ABORTED'} - {self._reason}",
                              "recover to 10-11 m/s, hold the distance button 1.5 s for the next trial")
      if normal_accel <= cap + 1e-6 or self._last_cmd >= 0.0:
        self.state = "DISARMED" if self._latched_off else "ARMED"
        self._pre_t = self._hold_t = 0.0
        self._ready_at_press = False
        out.state, out.changed = self.state, True
      return out
    # ARMED / READY
    if self.trial >= MAX_TRIALS:
      out.text1, out.text2 = "STEP TEST DONE - all trials used", "remove the arm file"
      return out
    self._pre_t = self._pre_t + dt if fail is None else 0.0
    ready = self._pre_t >= PRECONDITION_S
    if i.distance_pressed:
      if self._hold_t <= 0.0:
        self._ready_at_press = ready        # the 2.0 s must already hold when the press STARTS
      self._hold_t += dt
      released = False
    else:
      released = self._hold_t >= HOLD_S and self._ready_at_press
      self._hold_t = 0.0
    if released and ready:
      self.trial += 1
      self._current = self._trial_spec(self.trial - 1)
      self._t = 0.0
      self._last_cmd = 0.0
      self.state = "ACTIVE"
      out.state, out.trial, out.changed = self.state, self.trial, True
      out.active, out.accel = True, min(float(self._current[1][0]), 0.0)
      self._last_cmd = out.accel
      out.text1 = f"STEP TEST ACTIVE - {out.accel:+.2f} m/s^2 - 0.0 s"
      out.text2 = f"trial {self.trial}/{MAX_TRIALS}: {self._current[0]}"
      return out
    self.state = "READY" if ready else "ARMED"
    out.state = self.state
    if ready:
      out.text1 = f"STEP TEST READY - hold distance {HOLD_S:.1f} s then release"
    else:
      out.text1 = f"STEP TEST ARMED - waiting: {fail or 'settling'}"
    out.text2 = f"next trial {self.trial + 1}/{MAX_TRIALS}: {self._trial_spec(self.trial)[0]}"
    return out
