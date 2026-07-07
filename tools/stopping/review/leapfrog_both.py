#!/usr/bin/env python3
"""Both leapfrog variants over ONE qlog/rlog, while ENGAGED with an essentially stationary lead:
A) FULL_SETTLE: v<=0.08 sustained >=0.4s -> travel 0.03-3.0m -> re-settle.
B) NEAR_STOP:   local min v<=0.35 (no full settle) -> v rises >=0.15 within 4s -> falls again;
                lead vmax < 0.5 during the cycle; gap not growing > 1.0m."""
import json
import sys

import capnp
import zstandard

REPO = "/Users/radoslawchybicki/Repos/openpilot-rch"
capnp.remove_import_hook()
LOG = capnp.load(f"{REPO}/cereal/log.capnp")


def main(path):
  raw = open(path, "rb").read()
  raw = zstandard.ZstdDecompressor().decompress(raw, max_output_size=int(9e8))
  vs = []
  cur = {"en": False, "dRel": None, "vLead": None, "cmd": None}
  t0 = None
  git = None
  for ev in LOG.Event.read_multiple_bytes(raw):
    w = ev.which()
    t = ev.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    t -= t0
    if w == "initData":
      git = ev.initData.gitCommit[:8]
    elif w == "selfdriveState":
      cur["en"] = ev.selfdriveState.enabled
    elif w == "radarState":
      l1 = ev.radarState.leadOne
      cur["dRel"] = l1.dRel if l1.status else None
      cur["vLead"] = l1.vLead if l1.status else None
    elif w == "carOutput":
      cur["cmd"] = ev.carOutput.actuatorsOutput.accel
    elif w == "carState":
      vs.append((t, ev.carState.vEgo, cur["en"], cur["dRel"], cur["vLead"], cur["cmd"]))

  out = []
  seg = path.split("/realdata/")[-1].rsplit("/", 1)[0]
  n = len(vs)

  i = 0
  while i < n:
    t, v, en = vs[i][:3]
    if en and v <= 0.08:
      j = i
      while j < n and vs[j][1] <= 0.08 and vs[j][2]:
        j += 1
      dur = vs[j - 1][0] - t if j > i else 0.0
      if dur >= 0.4 and j < n:
        k = j
        travel = 0.0
        lead_vmax = 0.0
        lead_seen = False
        resettle = None
        while k < n and vs[k][0] - vs[j][0] < 20.0 and vs[k][2]:
          if k > j and vs[k][0] - vs[k - 1][0] < 0.3:
            travel += vs[k][1] * (vs[k][0] - vs[k - 1][0])
          if vs[k][4] is not None:
            lead_seen = True
            lead_vmax = max(lead_vmax, vs[k][4])
          if vs[k][1] <= 0.05 and travel > 0.03:
            resettle = vs[k][0]
            break
          k += 1
        if resettle is not None and 0.03 < travel < 3.0 and (not lead_seen or lead_vmax < 0.5):
          out.append({"type": "FULL_SETTLE", "seg": seg, "git": git, "t": round(t, 1),
                      "dur": round(dur, 1), "travel": round(travel, 2),
                      "gap": round(vs[j - 1][3], 2) if vs[j - 1][3] else None})
        i = k + 1
        continue
      i = j + 1
    else:
      i += 1

  i = 1
  while i < n - 1:
    t, v, en, dRel, vLead, cmd = vs[i]
    if en and 0.02 < v <= 0.35 and vs[i - 1][1] >= v and vs[i + 1][1] >= v:
      k = i + 1
      vpeak = v
      kpeak = i
      while k < n and vs[k][0] - t < 4.0 and vs[k][2]:
        if vs[k][1] > vpeak:
          vpeak, kpeak = vs[k][1], k
        if vs[k][1] <= 0.02:
          break
        k += 1
      rise = vpeak - v
      if rise >= 0.15:
        fell = any(x[1] < vpeak - 0.12 for x in vs[kpeak:min(kpeak + 300, n)])
        lv = [x[4] for x in vs[i:kpeak] if x[4] is not None]
        gaps = [x[3] for x in vs[i:kpeak] if x[3] is not None]
        gap_grew = gaps and (max(gaps) - gaps[0]) > 1.0
        if fell and (not lv or max(lv) < 0.5) and not gap_grew:
          out.append({"type": "NEAR_STOP", "seg": seg, "git": git, "t": round(t, 1),
                      "v_min": round(v, 2), "v_peak": round(vpeak, 2),
                      "gap": round(dRel, 1) if dRel is not None else None,
                      "cmd": round(cmd, 2) if cmd is not None else None})
          i = kpeak + 150
          continue
    i += 1

  for o in out:
    print(json.dumps(o))


if __name__ == "__main__":
  main(sys.argv[1])
