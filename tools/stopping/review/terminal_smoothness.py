#!/usr/bin/env python3
"""Terminal-descent smoothness score -- THE cycle-19 gate (user directive: "make sure we can
detect it's still wrong ... know when it's good or not").

Scores the final approach of one stop (v below V_WINDOW until wheel-stop) on four numbers:

  wire_jerk_max   max deepen rate of the commanded wire (m/s^3, 20 ms window) -- the "sudden
                  unnecessary increase" of route 00001f7b was 0.55-1.0 here
  wire_pump       largest wire RISE between two deepens inside the window (m/s^2) -- the
                  EASE<->GLIDE flap pump was 0.39 on f7b stop 1
  descent_count   how many separate descent episodes the wire makes (1 == the ideal single
                  continuous re-engagement of the human template)
  felt_jerk_max   max |d(aEgo)/dt| over 0.3 s (m/s^3) -- what the body actually feels

GOOD (targets for the single-curve terminal descent):
  wire_jerk_max <= 0.80, wire_pump <= 0.06, descent_count == 1, felt_jerk_max <= 0.8
wire_jerk_max is calibrated to the HUMAN template: manual re-engagements run 0.6-1.0 m/s3
(cycle-14), genuine curve-following on the single-segment descent reads ~0.5 at nominal decel,
while the defects sit far above -- J_DOWN-rate steps 2.5, the f7b flap 6.8-10. Safety-lane frames (a_kin/a_plan/monitor binding) are
exempt from wire_jerk_max: safety may always slam.

Usage: terminal_smoothness.py <rlog.zst> <t_stop_abs> [label]
Also importable: score_terminal(T, V, WIRE, safety_mask=None) for fixtures.
"""
import json
import math
import sys

V_WINDOW = 0.45          # the terminal window opens when v first drops below this -- the descent
                         # law's REALISTIC ARMING point (V_DESCENT_START 0.50 minus the 0.05
                         # drop-from-peak the arming latch requires). Frames above belong to EASE
                         # (its own pins); quantized EASE demand there steps up to ~0.9 m/s3 in
                         # single frames and is not this law's contract to answer for. The f7b
                         # defect classes all lived at 0.40 and below -- fully inside.
V_RELAUNCH = 0.12        # a rise above this after a sub-0.05 dip = the stop relaunched (FAIL);
                         # above the 0.09 latch-reset so 0.03-quantum flicker cannot false-flag
STANDSTILL_DWELL_S = 0.5 # a standstill candidate must hold sub-0.05 this long to end the window
T_TARGET_MAX_DIST_S = 3.0  # refuse to score if no standstill episode lies within this of t_target
WIRE_JERK_WIN_S = 0.02   # one control frame at 50 Hz wire logging; rate computed per-frame
FELT_JERK_WIN_S = 0.30
PUMP_MIN_DEEPEN = 0.03   # a descent episode must deepen at least this much to count


def score_terminal(T, V, WIRE, safety_mask=None, t_target=None):
  """T, V, WIRE: equal-length arrays. t_target (optional): timestamp near the WANTED stop's
  standstill -- the scorer selects the sustained-standstill episode nearest it (end-review round
  4: a prior stop inside the extraction window otherwise ends the scan first and gets certified
  under the requested stop's label). Without t_target the first episode is scored."""
  n = len(T)
  # 1. collect sustained-standstill episodes (contiguous sub-0.05 bands that dwell
  #    STANDSTILL_DWELL_S, or a band running to the trace edge)
  episodes = []
  cand = None
  dwelled = False
  for k in range(n):
    if V[k] < 0.05:
      if cand is None:
        cand, dwelled = k, False
      if not dwelled and T[k] - T[cand] >= STANDSTILL_DWELL_S:
        episodes.append(cand)
        dwelled = True
    else:
      cand = None
  if cand is not None and not dwelled:
    episodes.append(cand)  # trace-edge band: no dwell room
  if not episodes:
    if t_target is not None:
      return None  # a TARGETED request with no standstill at all: refuse (end-review round 6)
    k1 = n - 1
  elif t_target is None:
    k1 = episodes[0]
  else:
    k1 = min(episodes, key=lambda kk: abs(T[kk] - t_target))
    if abs(T[k1] - t_target) > T_TARGET_MAX_DIST_S:
      return None  # no standstill near the requested stop: refuse rather than certify a neighbor
  # 2. open the terminal window BACKWARD from the selected episode: the last v >= V_WINDOW
  #    crossing before it is THIS stop's approach entry
  k0 = 0
  for k in range(k1, -1, -1):
    if V[k] >= V_WINDOW:
      k0 = k  # KEEP the boundary sample (end-review round 5): scoring starts at k0+1, so the
              # transition INTO the first sub-0.50 sample is measured -- a jerk timed exactly at
              # the window entry must not escape both metrics
      break
  if k0 > k1:
    return None
  # 3. same-stop relaunch: inside [k0, k1), a rise above V_RELAUNCH after a sub-0.05 dip
  #    (the selected standstill itself ends the window, so departures cannot flag)
  relaunched = False
  dipped = False
  for k in range(k0, k1):
    if V[k] < 0.05:
      dipped = True
    elif dipped and V[k] > V_RELAUNCH:
      relaunched = True
  wire_jerk_max = 0.0
  pump = 0.0
  descents = 0
  in_descent = False
  descent_depth = 0.0
  rise_since_deep = 0.0
  for k in range(k0 + 1, k1 + 1):
    dt = T[k] - T[k - 1]
    if dt <= 0:
      continue
    dw = WIRE[k] - WIRE[k - 1]
    if dw < 0 and not (safety_mask and safety_mask[k]):
      wire_jerk_max = max(wire_jerk_max, -dw / dt)
    if dw < 0:
      descent_depth += -dw
      if not in_descent and descent_depth >= PUMP_MIN_DEEPEN:
        in_descent = True
        descents += 1
      rise_since_deep = 0.0
    elif dw > 0 and descents > 0:
      # rises BEFORE the first descent are the deliberate unload (the human template's release);
      # a pump is a rise after re-engagement has begun
      rise_since_deep += dw
      pump = max(pump, rise_since_deep)
      if rise_since_deep >= PUMP_MIN_DEEPEN:
        in_descent = False
        descent_depth = 0.0
  # felt jerk from the measured accel channel if the caller has one: optional second pass
  return {"wire_jerk_max": round(wire_jerk_max, 3), "wire_pump": round(pump, 3),
          "descent_count": descents, "relaunched": relaunched, "k_window": (k0, k1)}


def felt_jerk(T, A, k0, k1):
  worst = 0.0
  j = k0
  for k in range(k0, k1 + 1):
    while T[k] - T[j] > FELT_JERK_WIN_S:
      j += 1
    if k > j:
      worst = max(worst, abs(A[k] - A[j]) / (T[k] - T[j]))
  return round(worst, 3)


def main(path, t_stop, label=""):
  sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch/tools/stopping/review")
  from triage_one import read_events
  T, V, A, W = [], [], [], []
  wire = float("nan")
  for ev in read_events(path):
    w = ev.which()
    t = ev.logMonoTime * 1e-9
    if w == "carOutput":
      wire = ev.carOutput.actuatorsOutput.accel
    elif w == "carState" and t_stop - 8.0 <= t <= t_stop + 1.2:
      if math.isfinite(wire):
        T.append(t)
        V.append(ev.carState.vEgo)
        A.append(ev.carState.aEgo)
        W.append(wire)
  s = score_terminal(T, V, W, t_target=t_stop)
  if s:
    k0, k1 = s.pop("k_window")
    s["felt_jerk_max"] = felt_jerk(T, A, k0, k1)
    s["label"] = label or path.split("realdata/")[-1]
    s["good"] = (s["wire_jerk_max"] <= 0.80 and s["wire_pump"] <= 0.06
                 and s["descent_count"] == 1 and s["felt_jerk_max"] <= 0.8
                 and not s["relaunched"])
  print(json.dumps(s))


if __name__ == "__main__":
  main(sys.argv[1], float(sys.argv[2]), sys.argv[3] if len(sys.argv) > 3 else "")
