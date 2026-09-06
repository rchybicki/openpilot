#!/usr/bin/env python3
import math
"""Offline replay of the planner's cycle-31 REST-CLOSE gates from an rlog (20 Hz at radarState cadence):
prints, around a settle, which rc_ok gate blocks arming and the state-machine outcome. Re-implements the
small state machine (planner import pulls the MPC libs)
the classifier/authority are the real classes.
usage: rest_close_replay.py <rlog.zst> <t0 route-s> <t1 route-s>"""
import sys
import glob
sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch")
sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch/tools/stopping/review")
from triage_one import read_events
from openpilot.selfdrive.controls.lib.stop_context import StopContext
from openpilot.selfdrive.controls.lib.lead_provenance import StoppingLeadAuthority

ARM_V_MIN, ARM_V_MAX, CANCEL_V, V_CAP, DECEL, LAG_S, D_MIN, D_MAX, LEAD_V_MIN, EPOCH_V = 0.06, 1.50, 1.60, 0.80, 0.50, 0.25, 0.50, 2.50, -0.10, 3.0
REST = 4.0
def sm(armed, spent, vcap, tid, rc_ok, v, d_eff_arm, rc_tid, standstill, d_eff_epoch=None, drive_frames=0):
  # mirrors update_santa_fe_rest_close_state after the cycle-33 R1 fixes (identity + completed-rest latch)
  cancelled_now = False
  if armed:
    if (not rc_ok or v > CANCEL_V or rc_tid != tid):
      armed, spent, cancelled_now = False, True, True
  elif (not spent and rc_ok and rc_tid >= 0 and ARM_V_MIN < v <= ARM_V_MAX and d_eff_arm is not None and D_MIN <= d_eff_arm <= D_MAX):
    armed, spent = True, False
    vcap = min(v, V_CAP)
    tid = rc_tid
  if not armed and not cancelled_now and (v > EPOCH_V or drive_frames >= 10 or (d_eff_epoch is not None and d_eff_epoch > D_MAX + 0.5)):
    spent = False
  return armed, spent, vcap, tid


path, t0, t1 = glob.glob(sys.argv[1])[0], float(sys.argv[2]), float(sys.argv[3])
ctx, auth = StopContext(), StoppingLeadAuthority()
cs = cc = fcs = fp = ss = None
first = None
armed = spent = False
vcap = 0.0
tid = None
last_print = -9
out_frames = 0
drive_frames = 0
rested = False
for ev in read_events(path):
  t = ev.logMonoTime * 1e-9
  if first is None:
    first = t
  tt = t - first
  w = ev.which()
  if w == 'carState':
    cs = ev.carState
  elif w == 'carControl':
    cc = ev.carControl
  elif w == 'frogpilotCarState':
    fcs = ev.frogpilotCarState
  elif w == 'frogpilotPlan':
    fp = ev.frogpilotPlan
  elif w == 'selfdriveState':
    ss = ev.selfdriveState
  elif w == 'radarState' and cs and cc and ss:
    lead = ev.radarState.leadOne
    v = float(cs.vEgo)
    reset_state = not ss.enabled
    rc_tid = int(getattr(lead, 'radarTrackId', -1))
    authorized = auth.update(v_ego=v, lead_status=bool(lead.status and not reset_state), lead_d_rel=float(lead.dRel),
                             lead_track_id=rc_tid, model_prob=float(getattr(lead, 'modelProb', 0.0)))
    lead_ok = bool(lead.status and not reset_state and authorized)
    sig = ctx.update(v_ego=v, a_ego=float(cs.aEgo), a_cmd=float(cc.actuators.accel), lead_status=lead_ok, lead_v=float(lead.vLead),
                     lead_d_rel=float(lead.dRel) if lead_ok else None, lead_track_id=rc_tid if lead_ok else None,
                     standstill=bool(cs.standstill), dt=0.05)
    gap_live = (not sig.dropout_active and (sig.gap_source == "measured" or (sig.gap_source == "held" and sig.gap_hold_outward)))
    isd = float(getattr(fp, 'increasedStoppedDistance', 0.0)) if fp else 0.0
    force_coast = bool(getattr(fcs, 'forceCoast', False)) if fcs else False
    blended = bool(ss.experimentalMode)
    # cycle-33 gate: a trusted CRAWLING lead qualifies (no stopped confirmation); wheel-stop latch pins E3
    lv = float(lead.vLead)
    in_band = math.isfinite(lv) and LEAD_V_MIN <= lv <= 0.90
    out_frames = 0 if in_band else out_frames + 1
    band_ok = in_band if not armed else (in_band or out_frames < 6)   # cycle-33: 6-frame (0.30 s) cancel debounce
    gates = {"blended": blended, "engaged": not reset_state, "auth": authorized, "noFC": not force_coast,
             "earned": sig.lead_motion_earned, "gap_live": gap_live, "gap": sig.d_gap is not None,
             "lv band(debounced)": band_ok, "notSS": not cs.standstill,
             "noWheelStop": not sig.wheel_stop_latched}
    rc_ok = all(gates.values())
    d_eff_arm = (sig.d_gap - (REST + isd) - LAG_S * v) if sig.d_gap is not None else None
    trusted = (authorized and sig.lead_motion_earned and not sig.dropout_active and sig.gap_source == "measured"
               and rc_tid >= 0 and (tid is None or rc_tid == tid))
    d_eff_epoch = d_eff_arm if trusted else None
    # R4: driving-again evidence only after a completed rest (rested), consumed on any re-open
    rested = rested or (bool(cs.standstill) and spent)
    drive_frames = (drive_frames + 1 if v >= 1.0 else 0) if rested else 0
    spent_before = spent
    armed, spent, vcap, tid = sm(armed, spent, vcap, tid, rc_ok, v, d_eff_arm, rc_tid, bool(cs.standstill), d_eff_epoch,
                                 drive_frames if rested else 0)
    if spent_before and not spent:
      rested, drive_frames = False, 0
    if t0 <= tt <= t1 and tt - last_print >= 0.25:
      last_print = tt
      blocked = [k for k, ok in gates.items() if not ok]
      gap_s = None if sig.d_gap is None else round(sig.d_gap, 2)
      deff_s = None if d_eff_arm is None else round(d_eff_arm, 2)
      head = f"t={tt:7.2f} v={v:4.2f} dRel={float(lead.dRel):5.1f} vL={float(lead.vLead):5.2f} gap={gap_s} d_eff={deff_s}"
      print(head + f" armed={armed} spent={spent} vcap={vcap:.2f} | blocked: {blocked} src={sig.gap_source}")
