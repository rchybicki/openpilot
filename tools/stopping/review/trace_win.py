#!/usr/bin/env python3
"""Window trace: t0..t1 of one rlog, incl. longControlState + stopping_service telemetry."""
import sys

import capnp
import zstandard

REPO = "/Users/radoslawchybicki/Repos/openpilot-rch"
capnp.remove_import_hook()
LOG = capnp.load(f"{REPO}/cereal/log.capnp")

path, a, b = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
raw = zstandard.ZstdDecompressor().decompress(open(path, "rb").read(), max_output_size=int(9e8))
cur = {}
rows = []
svc_events = []
t0 = None
for ev in LOG.Event.read_multiple_bytes(raw):
  w = ev.which()
  t = ev.logMonoTime * 1e-9
  if t0 is None:
    t0 = t
  t -= t0
  if w == "carState":
    c = ev.carState
    cur.update(v=c.vEgo, gas=c.gasPressed, brake=c.brakePressed)
    if a <= t <= b:
      rows.append((t, dict(cur)))
  elif w == "carOutput":
    cur["cmd"] = ev.carOutput.actuatorsOutput.accel
  elif w == "longitudinalPlan":
    lp = ev.longitudinalPlan
    cur["aTgt"] = lp.aTarget
    cur["ss"] = bool(lp.shouldStop)
    cur["dts"] = getattr(lp, "distanceToStopTarget", -1.0)
  elif w == "radarState":
    l1 = ev.radarState.leadOne
    cur["dRel"] = round(l1.dRel, 1) if l1.status else None
    cur["vLead"] = round(l1.vLead, 2) if l1.status else None
  elif w == "selfdriveState":
    cur["en"] = ev.selfdriveState.enabled
  elif w == "controlsState":
    cur["lcs"] = str(ev.controlsState.longControlState)
  elif w == "logMessage" and a - 5 <= t <= b + 2:
    m = str(ev.logMessage)
    if "stopping_service" in m:
      svc_events.append((round(t, 1), m[:260]))

prev = None
for (t, r) in rows:
  if prev is not None and t - prev < 0.15:
    continue
  prev = t
  def f(x):
    return f"{x:+.2f}" if isinstance(x, float) else str(x)
  line1 = f"t={t:7.2f} v={r.get('v',0):5.2f} cmd={f(r.get('cmd'))} aTgt={f(r.get('aTgt'))} ss={int(bool(r.get('ss')))} "
  line2 = f"dts={f(r.get('dts'))} dRel={r.get('dRel')} vLead={r.get('vLead')} {r.get('lcs','?'):9s} en={int(bool(r.get('en')))}"
  line3 = f"{' GAS' if r.get('gas') else ''}{' BRK' if r.get('brake') else ''}"
  print(line1 + line2 + line3)
print("--- stopping_service telemetry ---")
for (t, m) in svc_events[-25:]:
  print(t, m)
