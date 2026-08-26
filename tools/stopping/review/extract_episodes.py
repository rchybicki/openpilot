#!/usr/bin/env python3
"""Extract genuine lead-stop approaches from one rlog into one JSONL file.

usage: extract_episodes.py <rlog.zst> <output.jsonl>
"""
import json
import math
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from triage_one import read_events


def extract(path):
  segment = Path(path).parent.name
  try:
    route = int(segment.split("--", 1)[0], 16)
  except (ValueError, IndexError) as exc:
    raise ValueError(f"invalid segment path: {path}") from exc
  if route < 0x1F00:
    raise ValueError(f"route is before 00001f00: {segment}")

  first = None
  cs = co = plan = model = frog_plan = frog_cs = state = pose = None
  commit = None
  frames = []
  for ev in read_events(path):
    t = ev.logMonoTime * 1e-9
    if first is None:
      first = t  # initData is stamped at route start, including in later segments.
    which = ev.which()
    if which == "initData":
      commit = ev.initData.gitCommit[:10]
    elif which == "carState":
      cs = ev.carState
    elif which == "carOutput":
      co = ev.carOutput
    elif which == "longitudinalPlan":
      plan = ev.longitudinalPlan
    elif which == "modelV2":
      model = ev.modelV2
    elif which == "frogpilotPlan":
      frog_plan = ev.frogpilotPlan
    elif which == "frogpilotCarState":
      frog_cs = ev.frogpilotCarState
    elif which == "selfdriveState":
      state = ev.selfdriveState
    elif which == "livePose":
      pose = ev.livePose
    elif which == "radarState" and cs is not None and co is not None and plan is not None and state is not None:
      lead = ev.radarState.leadOne
      vplan = [float(model.velocity.x[i]) for i in range(6)] if model is not None and len(model.velocity.x) >= 6 else None
      frames.append({
        "t": round(t - first, 3), "v": round(float(cs.vEgo), 3), "a": round(float(cs.aEgo), 3),
        "sent": round(float(co.actuatorsOutput.accel), 3), "aTgt": round(float(plan.aTarget), 3),
        "vplan": [round(x, 3) for x in vplan] if vplan else None,
        "ld": round(float(lead.dRel), 2), "lv": round(float(lead.vLead), 3), "la": round(float(lead.aLeadK), 3),
        "ls": bool(lead.status), "lp": round(float(getattr(lead, "modelProb", 0.0)), 2),
        "lt": int(getattr(lead, "radarTrackId", -1)), "ss": bool(cs.standstill),
        "isd": round(float(getattr(frog_plan, "increasedStoppedDistance", 0.0)), 2) if frog_plan else 0.0,
        "fc": bool(getattr(frog_cs, "forceCoast", False)) if frog_cs else False, "exp": bool(state.experimentalMode),
        "en": bool(state.enabled), "brk": bool(cs.brakePressed), "gas": bool(cs.gasPressed),
        "pitch": round(float(pose.orientationNED.y), 4) if pose is not None else None,
      })

  episodes = []
  candidate = None
  i = 0
  while i < len(frames):
    f = frames[i]
    starts = (f["en"] and f["ls"] and 1.5 <= f["v"] < 14.0 and math.isfinite(f["ld"]) and 0.0 < f["ld"] <= 60.0
              and (f["lv"] < 3.0 or f["la"] < -1.0))
    candidate = i if starts and candidate is None else candidate if starts else None
    if candidate is None or f["t"] - frames[candidate]["t"] < 0.5:
      i += 1
      continue

    start = candidate
    j = start
    rest_k = None
    pull_start = None
    end_reason = None
    while j < len(frames):
      g = frames[j]
      if not g["ls"] or not math.isfinite(g["ld"]) or g["ld"] <= 0.0:
        break
      if g["v"] < 0.05 and rest_k is None:
        rest_k = j
      if g["lv"] > g["v"] + 1.0:
        pull_start = j if pull_start is None else pull_start
        if g["t"] - frames[pull_start]["t"] >= 1.0 and g["ld"] > frames[pull_start]["ld"]:
          end_reason = "lead_pullaway"
          j += 1
          break
      else:
        pull_start = None
      if rest_k is not None and g["t"] - frames[rest_k]["t"] >= 2.0:
        end_reason = "rest_dwell"
        j += 1
        break
      j += 1

    ep = frames[start:j]
    if rest_k is not None and end_reason is not None and frames[rest_k]["t"] - frames[start]["t"] >= 1.0 and rest_k < j:
      gaps = [x["ld"] for x in frames[max(start, rest_k - 2):min(j, rest_k + 3)] if x["ls"] and x["ld"] > 0.0]
      rest = frames[rest_k]
      episodes.append({
        "seg": segment, "commit": commit, "t0": ep[0]["t"], "t_rest": rest["t"],
        "rest_gap": round(statistics.median(gaps), 3), "lead_v_at_rest": rest["lv"],
        "v_entry": ep[0]["v"], "gap_entry": ep[0]["ld"],
        "driver": any(x["brk"] or x["gas"] for x in ep), "end": end_reason, "frames": ep,
        # radar target switches inside the episode (harness gate: the lead trajectory must be ONE object)
        "switches": sum(1 for a, b in zip(ep, ep[1:], strict=False)
                        if a["ls"] and b["ls"] and (a["lt"] != b["lt"] or abs(b["ld"] - a["ld"]) > 1.5)),
      })
    i = max(j, i + 1)
    candidate = None
  return episodes


if __name__ == "__main__":
  if len(sys.argv) != 3:
    raise SystemExit(__doc__)
  output = Path(sys.argv[2])
  output.write_text("".join(json.dumps(ep, separators=(",", ":")) + "\n" for ep in extract(sys.argv[1])))
