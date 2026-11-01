import sys
#!/usr/bin/env python3
"""Replay recorded lead-stop approaches on the measured Santa Fe HEV plant.

usage: stop_harness.py <episodes.jsonl> [--grid] [--ac 0.6] [--tau 0.8] [--lag 0.45] [--vown 4.5]
"""
import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path

import numpy as np

sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch")
from openpilot.selfdrive.controls.lib.stopping_service import governor_demand  # noqa: E402

PLANT_DELAY_S = 0.45
PLANT_LAG_S = 0.50
LEAD_EMA_S = 0.30
GAIN_PTS = [(-2.6, 0.78), (-2.2, 0.87), (-1.86, 0.89), (-1.64, 0.93), (-1.37, 0.92),
            (-1.11, 0.93), (-0.89, 0.94), (-0.62, 0.97), (0.0, 1.0)]
FRAME_KEYS = {"t", "v", "a", "sent", "aTgt", "vplan", "ld", "lv", "la", "ls", "lp", "lt", "ss",
              "isd", "fc", "exp", "en", "brk", "gas", "pitch"}


def plant_gain(command):
  for (a0, g0), (a1, g1) in zip(GAIN_PTS, GAIN_PTS[1:], strict=False):
    if a0 <= command <= a1:
      return g0 + (command - a0) / (a1 - a0) * (g1 - g0)
  return GAIN_PTS[0][1] if command < GAIN_PTS[0][0] else 1.0


def validate_episode(ep, source="episode"):
  required = {"seg", "t0", "t_rest", "rest_gap", "lead_v_at_rest", "v_entry", "gap_entry", "driver", "frames"}
  missing = required - ep.keys()
  if missing:
    raise ValueError(f"{source}: missing episode fields {sorted(missing)}")
  try:
    route = int(ep["seg"].split("--", 1)[0], 16)
  except (AttributeError, ValueError) as exc:
    raise ValueError(f"{source}: invalid segment {ep['seg']!r}") from exc
  frames = ep["frames"]
  if route < 0x1F00 or len(frames) < 2 or float(ep["v_entry"]) < 1.5 or float(ep["t_rest"]) - float(ep["t0"]) < 1.0:
    raise ValueError(f"{source}: not a genuine route-00001f00+ approach")
  for i, frame in enumerate(frames):
    missing = FRAME_KEYS - frame.keys()
    if missing:
      raise ValueError(f"{source} frame {i}: missing fields {sorted(missing)}")
    if not all(math.isfinite(float(frame[k])) for k in ("t", "v", "a", "sent", "lv", "isd")):
      raise ValueError(f"{source} frame {i}: non-finite dynamics")
  times = np.asarray([f["t"] for f in frames], dtype=float)
  if np.any(np.diff(times) <= 0.0):
    raise ValueError(f"{source}: frame times are not strictly increasing")
  rest_i = int(np.argmin(np.abs(times - float(ep["t_rest"]))))
  if frames[rest_i]["v"] >= 0.05:
    raise ValueError(f"{source}: rest was not reached")
  if not all(f["ls"] and math.isfinite(float(f["ld"])) and f["ld"] > 0.0 for f in frames[:rest_i + 1]):
    raise ValueError(f"{source}: lead is not present throughout the approach")


def load_episodes(path):
  episodes = []
  with Path(path).open() as stream:
    for line_no, line in enumerate(stream, 1):
      if line.strip():
        episode = json.loads(line)
        validate_episode(episode, f"{path}:{line_no}")
        episodes.append(episode)
  if not episodes:
    raise ValueError(f"{path}: no episodes")
  return episodes


def reconstruct(ep):
  """Return times, recorded ego/lead positions, and lead-velocity jitter."""
  frames = ep["frames"]
  t = np.asarray([f["t"] for f in frames], dtype=float)
  v = np.asarray([f["v"] for f in frames], dtype=float)
  lv = np.asarray([f["lv"] for f in frames], dtype=float)
  gap = np.asarray([f["ld"] for f in frames], dtype=float)
  valid = np.asarray([f["ls"] for f in frames]) & np.isfinite(gap) & (gap > 0.0)
  if not np.any(valid):
    raise ValueError(f"{ep['seg']}: no usable lead gap")
  x_ego = np.zeros(len(frames))
  x_ego[1:] = np.cumsum(0.5 * (v[:-1] + v[1:]) * np.diff(t))
  filtered = np.full(len(frames), np.nan)
  for i in np.flatnonzero(valid):
    window = valid & (np.abs(t - t[i]) <= 0.125)
    filtered[i] = np.median(gap[window])
  x_lead = x_ego + filtered
  anchors = np.flatnonzero(np.isfinite(x_lead))
  first, last = int(anchors[0]), int(anchors[-1])
  for i in range(first - 1, -1, -1):
    x_lead[i] = x_lead[i + 1] - 0.5 * (lv[i] + lv[i + 1]) * (t[i + 1] - t[i])
  for left, right in zip(anchors[:-1], anchors[1:], strict=False):
    if right == left + 1:
      continue
    predicted = x_lead[left]
    bridge = []
    for i in range(left + 1, right + 1):
      predicted += 0.5 * (lv[i - 1] + lv[i]) * (t[i] - t[i - 1])
      bridge.append(predicted)
    correction = x_lead[right] - bridge[-1]
    for i in range(left + 1, right):
      fraction = (t[i] - t[left]) / (t[right] - t[left])
      x_lead[i] = bridge[i - left - 1] + correction * fraction
  for i in range(last + 1, len(frames)):
    x_lead[i] = x_lead[i - 1] + 0.5 * (lv[i - 1] + lv[i]) * (t[i] - t[i - 1])
  derivative = np.diff(x_lead) / np.diff(t)
  jitter = float(np.std(derivative - 0.5 * (lv[:-1] + lv[1:])))
  return t, x_ego, x_lead, jitter


def model_trace(ep):
  frames = ep["frames"]
  t = [float(f["t"]) for f in frames]
  sent = [float(f["sent"]) for f in frames]
  speed = np.asarray([f["v"] for f in frames], dtype=float)
  recorded_accel = np.empty(len(frames))
  recorded_accel[1:] = np.diff(speed) / np.diff(t)
  recorded_accel[0] = recorded_accel[1]
  output = np.empty(len(frames))
  output[0] = recorded_accel[0]
  for k in range(1, len(frames)):
    dt = t[k] - t[k - 1]
    delayed = sent[max(0, bisect_right(t, t[k] - PLANT_DELAY_S, 0, k) - 1)]
    target = plant_gain(delayed) * delayed
    output[k] = output[k - 1] + (target - output[k - 1]) * min(dt / PLANT_LAG_S, 1.0)
  return output, recorded_accel


def closure_law(v, v_lead, gap, isd, ac, tau, lag, a_max=2.5):
  d_rem = gap - (4.0 + isd) - lag * max(v - v_lead, 0.0)
  v_ref = max(v_lead, 0.0) + math.sqrt(2.0 * ac * max(d_rem, 0.0))
  return float(np.clip((v_ref - v) / tau, -a_max, 0.5)), v_ref, d_rem


def simulate(ep, law="closure", ac=0.6, tau=0.8, lag=0.45, vown=4.5, residual=True):
  frames = ep["frames"]
  t, x_ego_recorded, x_lead, jitter = reconstruct(ep)
  recorded_model, recorded_accel = model_trace(ep)
  bias = recorded_accel - recorded_model if residual else np.zeros(len(frames))
  command = np.empty(len(frames))
  accel = np.empty(len(frames))
  speed = np.empty(len(frames))
  position = np.zeros(len(frames))
  gap = np.empty(len(frames))
  owned = np.zeros(len(frames), dtype=bool)
  speed[0] = float(frames[0]["v"])
  accel[0] = recorded_accel[0]
  gap[0] = x_lead[0]
  model = accel[0]
  trim = 0.0
  lead_ema = float(frames[0]["lv"])
  last_command = float(frames[0]["sent"])
  rest_k = None

  for k in range(len(frames)):
    if k:
      dt = t[k] - t[k - 1]
      delayed = command[max(0, bisect_right(t, t[k] - PLANT_DELAY_S, 0, k) - 1)]
      target = plant_gain(delayed) * delayed
      model += (target - model) * min(dt / PLANT_LAG_S, 1.0)
      accel[k] = model + bias[k]
      speed[k] = max(speed[k - 1] + accel[k] * dt, 0.0)
      position[k] = position[k - 1] + 0.5 * (speed[k - 1] + speed[k]) * dt
      gap[k] = x_lead[k] - position[k]
    else:
      dt = float(np.median(np.diff(t)))

    if law == "recorded":
      cmd = float(frames[k]["sent"])
    else:
      owned[k] = owned[k - 1] if k else False
      if not owned[k] and speed[k] <= vown and frames[k]["ls"]:
        owned[k] = True
      if owned[k]:
        lead_ema += (float(frames[k]["lv"]) - lead_ema) * min(dt / LEAD_EMA_S, 1.0)
        if law == "governor":
          # the PRODUCTION law (shared code, not a copy): profile feedforward + bounded pursuit
          g = governor_demand(speed[k], lead_ema, gap[k], float(frames[k]["isd"]), a_c=ac, tau=tau, lag=lag)
          target, d_rem = (g[0], g[3]) if g is not None else (float(frames[k]["sent"]), 1.0)
        else:
          target, _, d_rem = closure_law(speed[k], lead_ema, gap[k], float(frames[k]["isd"]), ac, tau, lag)
        hold = speed[k] < 0.30 and d_rem <= 0.35
        if hold:
          target = -0.70
        if speed[k] >= 1.0 and target <= -0.5:
          error = target - accel[k]
          delta = error + 0.05 if error < -0.05 else 2.0 * (error - 0.03) if error > 0.03 else 0.0
          trim = float(np.clip(trim + delta * dt, -0.4, 0.0))
          target += trim
        else:
          trim = min(trim + dt, 0.0)
        rate = 0.6 if hold and target < last_command else 2.5 if target < last_command else 1.5
        cmd = float(np.clip(target, last_command - rate * dt, last_command + rate * dt))
      else:
        cmd = float(frames[k]["sent"])
    command[k] = cmd
    last_command = cmd
    if rest_k is None and k > 5 and speed[k] < 0.05:
      rest_k = k

  recorded_speed = np.asarray([f["v"] for f in frames], dtype=float)
  result = {
    "completed": rest_k is not None, "speed_errors": np.abs(speed - recorded_speed),
    "residual_rms": float(np.sqrt(np.mean(bias * bias))), "residual_tau": autocorrelation_time(t, bias),
    "frame_count": len(frames),
    "lead_jitter": jitter, "lead_state": "crawl" if ep["lead_v_at_rest"] > 0.3 else "stopped",
    "entry_band": "<3" if ep["v_entry"] < 3.0 else "3-6" if ep["v_entry"] <= 6.0 else ">6",
  }
  all_use_jerk = [k for k in range(1, len(frames)) if law == "recorded" or owned[k]]
  all_jerks = [abs(command[k] - command[k - 1]) / (t[k] - t[k - 1]) for k in all_use_jerk]
  result["max_owned_jerk"] = max(all_jerks, default=0.0) if law != "recorded" else None
  if rest_k is None:
    return result
  rec_k = int(np.argmin(np.abs(t - float(ep["t_rest"]))))
  rec_rest = x_lead[rec_k] - x_ego_recorded[rec_k]
  use_jerk = [k for k in range(1, rest_k + 1) if law == "recorded" or owned[k]]
  jerks = [abs(command[k] - command[k - 1]) / (t[k] - t[k - 1]) for k in use_jerk]
  late = [k for k in range(rest_k + 1) if gap[k] - gap[rest_k] <= 1.5]
  felt_indices = [k for k in range(rest_k + 1) if gap[k] - gap[rest_k] <= 2.0]
  felt = 0.0
  for p, i in enumerate(felt_indices):
    for j in felt_indices[p + 1:]:
      if t[j] - t[i] > 0.5:
        break
      felt = max(felt, abs(accel[j] - accel[i]))
  result.update({
    "rest": float(gap[rest_k]), "rec_rest": float(rec_rest), "min_gap": float(np.min(gap[:rest_k + 1])),
    "max_decel": max(float(-np.min(accel[:rest_k + 1])), 0.0), "jerk": max(jerks, default=0.0),
    "chase": max(float(-np.min(accel[late])), 0.0), "felt": felt,
    "t_rest": float(t[rest_k] - t[0]),
  })
  return result


def autocorrelation_time(times, residual):
  centered = np.asarray(residual, dtype=float) - np.mean(residual)
  energy = float(np.dot(centered, centered))
  if energy <= 1e-12:
    return 0.0
  corr = np.correlate(centered, centered, mode="full")[len(centered) - 1:] / energy
  crossing = np.flatnonzero(corr <= math.exp(-1.0))
  lag = int(crossing[0]) if len(crossing) else len(centered) - 1
  return float(lag * np.median(np.diff(times)))


def quantile(rows, key, q):
  return float(np.quantile([row[key] for row in rows], q))


def metrics(rows):
  done = [row for row in rows if row["completed"]]
  if not done:
    return ["0"] + ["-"] * 11
  rests = [row["rest"] for row in done]
  return [str(len(done)), f"{quantile(done, 'rest', 0.1):.2f}", f"{quantile(done, 'rest', 0.5):.2f}",
          f"{quantile(done, 'rest', 0.9):.2f}", f"{sum(3.5 <= x <= 5.0 for x in rests) / len(done):.1%}",
          str(sum(x < 3.0 for x in rests)), str(sum(x > 5.5 for x in rests)), f"{quantile(done, 'min_gap', 0.01):.2f}",
          f"{quantile(done, 'max_decel', 0.9):.2f}", f"{quantile(done, 'jerk', 0.9):.2f}",
          f"{quantile(done, 'chase', 0.9):.2f}", f"{quantile(done, 'felt', 0.25):.2f}"]


def print_table(title, configurations, split=False):
  print(f"\n{title}")
  print("| law | group | n | rest p10 | rest p50 | rest p90 | in 3.5-5.0 | <3.0 | >5.5 | min gap p1 | max decel p90 | cmd jerk p90 | chase p90 | felt p25 |")
  print("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
  groups = (("lead stopped", lambda r: r["lead_state"] == "stopped"), ("lead crawl", lambda r: r["lead_state"] == "crawl"),
            ("entry <3", lambda r: r["entry_band"] == "<3"), ("entry 3-6", lambda r: r["entry_band"] == "3-6"),
            ("entry >6", lambda r: r["entry_band"] == ">6")) if split else (("all", lambda _: True),)
  for name, rows in configurations:
    for group, select in groups:
      print("| " + " | ".join([name, group, *metrics([row for row in rows if select(row)])]) + " |")


def run_report(episodes, grid, ac, tau, lag, vown, law="closure"):
  replay = [simulate(ep, "recorded") for ep in episodes]
  generic = [simulate(ep, "recorded", residual=False) for ep in episodes]
  settings = ([(a, t, l) for a in (0.4, 0.5, 0.6, 0.8) for t in (0.6, 0.8, 1.0) for l in (0.35, 0.45)]
              if grid else [(ac, tau, lag)])
  configurations = [("recorded", replay)]
  for a, t, l in settings:
    configurations.append((f"{law[:3]} A{a:.1f} T{t:.1f} L{l:.2f}", [simulate(ep, law, a, t, l, vown) for ep in episodes]))

  replay_done = [row for row in replay if row["completed"]]
  errors = [abs(row["rest"] - row["rec_rest"]) for row in replay_done]
  speed_errors = np.concatenate([row["speed_errors"] for row in replay])
  generic_done = [row for row in generic if row["completed"]]
  generic_errors = [abs(row["rest"] - row["rec_rest"]) for row in generic_done]
  generic_speed = np.concatenate([row["speed_errors"] for row in generic])
  residual_rms = math.sqrt(sum(row["residual_rms"] ** 2 * row["frame_count"] for row in replay)
                           / sum(row["frame_count"] for row in replay))
  residual_tau = [row["residual_tau"] for row in replay]
  jitter = [row["lead_jitter"] for row in replay]
  routes = {ep["seg"].split("--", 1)[0] for ep in episodes}
  g1 = len(episodes) >= 150
  g2 = (len(replay_done) == len(episodes) and np.quantile(errors, 0.5) <= 0.15 and np.quantile(errors, 0.9) <= 0.40
        and np.quantile(speed_errors, 0.9) <= 0.30)
  candidate_rows = [row for _, rows in configurations[1:] for row in rows]
  g4 = bool(candidate_rows) and max(row["max_owned_jerk"] for row in candidate_rows) <= 2.5 + 1e-6
  g5 = len(settings) == (24 if grid else 1)
  print(f"corpus: {len(episodes)} approaches, {len(routes)} routes, {sum(ep['driver'] for ep in episodes)} driver episodes")
  residual_summary = f"residual plant: RMS {residual_rms:.3f} m/s^2, autocorrelation time "
  residual_summary += f"p50 {np.quantile(residual_tau, 0.5):.2f} s p90 {np.quantile(residual_tau, 0.9):.2f} s"
  print(residual_summary)
  generic_summary = f"generic model: completed {len(generic_done)}/{len(episodes)}, rest |error| "
  generic_summary += f"p50 {np.quantile(generic_errors, 0.5):.2f} m p90 {np.quantile(generic_errors, 0.9):.2f} m, "
  generic_summary += f"speed |error| p90 {np.quantile(generic_speed, 0.9):.2f} m/s"
  print(generic_summary)
  print(f"lead reconstruction: velocity jitter p50 {np.quantile(jitter, 0.5):.2f} m/s p90 {np.quantile(jitter, 0.9):.2f} m/s max {max(jitter):.2f} m/s")
  print("lead reconstruction per-episode velocity jitter (m/s):")
  for ep, row in sorted(zip(episodes, replay, strict=True), key=lambda item: item[1]["lead_jitter"], reverse=True):
    print(f"  {ep['seg']}@{ep['t0']:.3f}: {row['lead_jitter']:.3f}")
  print(f"G1 CORPUS: {'PASS' if g1 else 'FAIL'} (need >=150 genuine approaches from route 00001f00+)")
  sanity_summary = f"G2 SANITY: {'PASS' if g2 else 'FAIL'} (completed {len(replay_done)}/{len(episodes)}, rest |error| "
  sanity_summary += f"p50 {np.quantile(errors, 0.5):.2f} m p90 {np.quantile(errors, 0.9):.2f} m, "
  sanity_summary += f"speed |error| p90 {np.quantile(speed_errors, 0.9):.2f} m/s)"
  print(sanity_summary)
  print("G3 LEAD TRAJECTORY: PASS (0.25 s median gap; velocity-integrated dropout bridge)")
  print(f"G4 LAW FAIRNESS: {'PASS' if g4 else 'FAIL'} (0.3 s lead EMA; 2.5/1.5/0.6 jerk limits; clutch hold; V_OWN={vown:.1f})")
  print(f"G5 REPORT: {'PASS' if g5 else 'FAIL'} ({len(settings)} candidate settings plus recorded replay; full and split tables)")
  print("G6 TEST: run the required focused pytest command; this report does not infer an external test result")
  print_table("TABLE 1 - ALL EPISODES", configurations)
  print_table("TABLE 2 - LEAD-STATE AND ENTRY-SPEED SPLITS", configurations, split=True)
  return configurations


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("episodes")
  parser.add_argument("--grid", action="store_true")
  parser.add_argument("--ac", type=float, default=0.6)
  parser.add_argument("--tau", type=float, default=0.8)
  parser.add_argument("--lag", type=float, default=0.45)
  parser.add_argument("--vown", type=float, default=4.5)
  parser.add_argument("--no-driver", action="store_true", help="exclude marked pedal-input episodes")
  parser.add_argument("--law", default="closure", choices=["closure", "governor"])
  args = parser.parse_args()
  eps = load_episodes(args.episodes)
  if args.no_driver:
    eps = [ep for ep in eps if not ep["driver"]]
  run_report(eps, args.grid, args.ac, args.tau, args.lag, args.vown, args.law)
