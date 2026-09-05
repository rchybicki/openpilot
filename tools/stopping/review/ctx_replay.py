#!/usr/bin/env python3
"""Offline replay of StopContext over one rlog segment: the per-frame attributed-safety eligibility
reasons (gap / identity / fcw / lead_braking) exactly as stopping_service derives them from the signals.

Usage: python tools/stopping/review/ctx_replay.py <segment-dir-name> <t_lo> <t_hi>   (segment-local seconds)

carState drives the 100 Hz frames; radarState (20 Hz) is sample-and-held like controlsd's SubMaster; carControl
accel is a_cmd. Prints the reason runs and, for the governor band (v <= 2.5 m/s), the ineligible-frame and
dwell-reset counts under the deployed rule and under the cycle-49 gap_live rule (an OUTWARD persistence hold =
min(prediction, raw) = a lower bound on the true gap stays eligible)."""
import sys
from collections import Counter
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.controls.lib.stop_context import StopContext
from openpilot.selfdrive.controls.lib.stopping_service import ATTR_TRUST_IN_S, ATTR_LEAD_BRAKING

DATA = Path.home() / ".route_sync/data/media/0/realdata"
BAND_V = 2.5


def gap_trusted_deployed(sig):
  return not sig.dropout_active and sig.gap_source == "measured"


def gap_trusted_gap_live(sig):
  # the service's gap_live predicate: measured, OR the filter's OUTWARD hold (min(prediction, raw) = a lower bound)
  return not sig.dropout_active and (sig.gap_source == "measured" or (sig.gap_source == "held" and sig.gap_hold_outward))


def reason(sig, fcw, lead_a, gap_trusted):
  """The service's ordered eligibility chain (stopping_service.update): gap trust first, then the identity,
  FCW and braking-lead vetoes -- every veto is evaluated on every frame class (review R1: an outward hold must not
  hide an FCW / immature-identity / braking-lead veto)."""
  if not gap_trusted(sig):
    return "gap"
  if not sig.lead_motion_earned or sig.track_age_s < ATTR_TRUST_IN_S:
    return "identity"
  if fcw:
    return "fcw"
  if lead_a < ATTR_LEAD_BRAKING:
    return "lead_braking"
  return None


def reason_deployed(sig, fcw, lead_a):
  return reason(sig, fcw, lead_a, gap_trusted_deployed)


def reason_gap_live(sig, fcw, lead_a):
  return reason(sig, fcw, lead_a, gap_trusted_gap_live)


def replay(seg, t_lo, t_hi):
  seg_idx = int(seg.split("--")[-1])
  ctx = StopContext()
  lead = None
  a_cmd = 0.0
  fcw = False
  t0 = t_prev = None
  rows = []
  for m in LogReader(str(DATA / seg / "rlog.zst")):
    t = m.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    rel = t - t0 - 60.0 * seg_idx
    w = m.which()
    if w == "radarState":
      lead_one = m.radarState.leadOne
      lead = (lead_one.status, lead_one.vLead, lead_one.dRel, lead_one.radarTrackId, lead_one.aLeadK)
    elif w == "carControl":
      a_cmd = m.carControl.actuators.accel
    elif w == "longitudinalPlan":
      fcw = bool(m.longitudinalPlan.fcw)
    elif w == "carState":
      cs = m.carState
      dt = 0.01 if t_prev is None else max(min(t - t_prev, 0.05), 0.001)
      t_prev = t
      if lead is None:
        continue
      status, lead_v, d_rel, track_id, lead_a = lead
      sig = ctx.update(v_ego=cs.vEgo, a_ego=cs.aEgo, a_cmd=a_cmd, lead_status=status, lead_v=lead_v, lead_d_rel=d_rel,
                       lead_track_id=track_id, standstill=cs.standstill, dt=dt)
      if t_lo <= rel <= t_hi and status:
        rows.append((rel, cs.vEgo, reason_deployed(sig, fcw, lead_a), reason_gap_live(sig, fcw, lead_a), sig.gap_source,
                     sig.gap_hold_outward, d_rel, sig.d_gap, track_id, sig.track_age_s))
  return rows


def dwell_resets(rows, idx):
  n = 0
  prev_ok = None
  for r in rows:
    ok = r[idx] is None
    if prev_ok and not ok:
      n += 1
    prev_ok = ok
  return n


def main():
  seg, t_lo, t_hi = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
  rows = replay(seg, t_lo, t_hi)
  band = [r for r in rows if r[1] <= BAND_V]
  reasons = dict(Counter(r[2] for r in rows if r[2]))
  print(f"{seg} window {t_lo}-{t_hi}: frames {len(rows)} ineligible(deployed) {sum(1 for r in rows if r[2])} reasons {reasons}")
  inel_now, inel_new = sum(1 for r in band if r[2]), sum(1 for r in band if r[3])
  print(f"  IN-BAND (v<={BAND_V}): frames {len(band)} ineligible deployed {inel_now} / gap_live {inel_new}; "
        + f"dwell resets deployed {dwell_resets(band, 2)} / gap_live {dwell_resets(band, 3)}")
  print("  ineligible runs (reason, t_start, t_end, frames, gap_source, hold_outward, raw_dRel, d_gap, track, age, v):")
  cur = None
  runs = []
  for r in rows:
    if cur is None or cur[0] != r[2]:
      if cur is not None:
        runs.append(cur)
      cur = [r[2], round(r[0], 2), round(r[0], 2), 1, r[4], r[5], round(r[6], 3), round(r[7], 3) if r[7] is not None else None,
             r[8], round(r[9], 2), round(r[1], 2)]
    else:
      cur[2] = round(r[0], 2)
      cur[3] += 1
  if cur is not None:
    runs.append(cur)
  for run in runs:
    if run[0] is not None:
      print("   ", run)


if __name__ == "__main__":
  main()
