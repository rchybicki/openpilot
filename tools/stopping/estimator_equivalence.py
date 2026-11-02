#!/usr/bin/env python3
"""Estimator-equivalence gate artifact (spec 5.5.2 / F21) -- a mandatory row in the similarity
report, run BEFORE the similarity gate.

Replays the V2 disturbance estimator two ways over every recorded event-store trace
(~/.comma/stopping_behavior/event_store by default; machine-local data, which is why this is a
tool artifact and not a pytest -- the fixture-based unit test lives in test_stopping_tracker.py):

  legacy single-frame semantics:  d_hat[k] = a_ego[k] - a_exp[k]            (DIST_LPF_TAU_S = 0.0 bypass)
  V2 LPF semantics:               d_hat[k] += (dt/tau) * ((a_ego - a_exp) - d_hat)

with a_exp from the same re-discretized PlantModel the tracker runs (PLANT_MODEL_REF), and the
verbatim G4 push gates (thresh 0.04/0.03 split at 0.08 m/s; 0.002 < v < 1.2; braking command).

Pass criteria (spec 5.5.2):
  (i)  the LPF threshold-crossing onset falls within +/-0.2 s of the legacy single-frame trigger
       on >= 90% of push-disturbance events (first onset per event), and
  (ii) the per-event release-inhibit window overlap (Jaccard, windows from T_RELEASE_INHIBIT(v)
       refreshed while the push trigger holds) is >= 80% -- enforced as >= 90% of push events
       meeting the 80% bound, mirroring criterion (i)'s event-fraction form.

Exit codes: 0 pass, 1 fail, 2 insufficient data / store absent. `DIST_LPF_TAU_S` is tuned on
recorded events to pass this artifact BEFORE the similarity gate runs (R9; the 0.0 bypass is the
kill switch).
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.controls.lib.stopping_params import STOPPING_PARAMS
from openpilot.selfdrive.controls.lib.stopping_plant import PLANT_PARAMS_REF, PlantModel

P = STOPPING_PARAMS
DEFAULT_EVENT_STORE = Path.home() / ".comma" / "stopping_behavior" / "event_store"
ONSET_TOLERANCE_S = 0.2
EVENT_FRACTION_FLOOR = 0.90
WINDOW_OVERLAP_FLOOR = 0.80
MIN_PUSH_EVENTS = 5


def push_threshold(v: float) -> float:
  return P.DIST_PUSH_THRESH_LOW if v < P.DIST_PUSH_THRESH_V_SPLIT else P.DIST_PUSH_THRESH_HIGH


def push_gate(v: float, last_cmd: float) -> bool:
  return P.DIST_PUSH_V_MIN < v < P.DIST_PUSH_V_MAX and last_cmd < P.DIST_PUSH_MIN_BRAKE


def estimator_triggers(t: np.ndarray, v_ego: np.ndarray, a_ego: np.ndarray, accel_cmd: np.ndarray,
                       dt: float, tau_s: float, active: np.ndarray | None = None) -> np.ndarray:
  """Boolean push-trigger series for one estimator variant (tau_s = 0.0 => legacy single-frame).

  `active` (enabled AND should_stop when recorded) bounds where the estimator runs -- on-car the
  disturbance machinery only sees stopping-authority frames; outside a span the recorded command
  was never actuated and the innovation is meaningless, so d_hat resets at span entry.
  """
  plant = PlantModel(PLANT_PARAMS_REF, dt)
  delay = plant.delay_frames
  n = len(t)
  triggers = np.zeros(n, dtype=bool)
  d_hat = 0.0
  for k in range(1, n):
    if active is not None and not bool(active[k]):
      d_hat = 0.0
      continue
    cmd_delayed = float(accel_cmd[max(k - 1 - delay, 0)])
    a_exp = plant.predict_next(float(a_ego[k - 1]), cmd_delayed, float(v_ego[k - 1]))
    innovation = float(a_ego[k]) - a_exp
    if tau_s <= 0.0:
      d_hat = innovation
    else:
      d_hat += (dt / tau_s) * (innovation - d_hat)
    v = float(v_ego[k])
    last_cmd = float(accel_cmd[k - 1])
    triggers[k] = push_gate(v, last_cmd) and d_hat > push_threshold(v)
  return triggers


def inhibit_windows(t: np.ndarray, v_ego: np.ndarray, triggers: np.ndarray, dt: float) -> np.ndarray:
  """Release-inhibit activity: timer set/refreshed to T_RELEASE_INHIBIT(v) while the trigger holds."""
  active = np.zeros(len(t), dtype=bool)
  timer = 0.0
  for k in range(len(t)):
    if triggers[k]:
      timer = float(np.interp(float(v_ego[k]), P.T_RELEASE_INHIBIT_TABLE[0], P.T_RELEASE_INHIBIT_TABLE[1]))
    active[k] = timer > 0.0
    timer = max(timer - dt, 0.0)
  return active


def jaccard(a: np.ndarray, b: np.ndarray) -> float:
  union = int(np.count_nonzero(a | b))
  if union == 0:
    return 1.0
  return int(np.count_nonzero(a & b)) / union


def evaluate_trace(t: np.ndarray, v_ego: np.ndarray, a_ego: np.ndarray, accel_cmd: np.ndarray,
                   tau_s: float, active: np.ndarray | None = None) -> dict[str, Any] | None:
  """Per-event comparison; None when the legacy variant never triggers (not a push event)."""
  if len(t) < 5:
    return None
  dt = float(np.median(np.diff(t)))
  if not (0.001 <= dt <= 0.5):
    return None
  legacy = estimator_triggers(t, v_ego, a_ego, accel_cmd, dt, 0.0, active=active)
  lpf = estimator_triggers(t, v_ego, a_ego, accel_cmd, dt, tau_s, active=active)
  legacy_onsets = np.flatnonzero(legacy & ~np.concatenate(([False], legacy[:-1])))
  if len(legacy_onsets) == 0:
    return None
  lpf_onsets = np.flatnonzero(lpf & ~np.concatenate(([False], lpf[:-1])))
  if len(lpf_onsets) == 0:
    onset_delta = float("inf")
  else:
    onset_delta = float(t[lpf_onsets[0]] - t[legacy_onsets[0]])
  win_legacy = inhibit_windows(t, v_ego, legacy, dt)
  win_lpf = inhibit_windows(t, v_ego, lpf, dt)
  return {
    "dt": dt,
    "legacy_onsets": int(len(legacy_onsets)),
    "lpf_onsets": int(len(lpf_onsets)),
    "first_onset_delta_s": onset_delta,
    "onset_within_tolerance": bool(abs(onset_delta) <= ONSET_TOLERANCE_S),
    "window_overlap": jaccard(win_legacy, win_lpf),
  }


def iter_store_traces(store_dir: Path):
  events_path = store_dir / "events.jsonl"
  if not events_path.is_file():
    return
  for line in events_path.read_text().splitlines():
    line = line.strip()
    if not line:
      continue
    record = json.loads(line)
    trace_path = store_dir / record.get("trace_ref", "")
    if not trace_path.is_file():
      continue
    with np.load(trace_path) as npz:
      if not all(name in npz for name in ("t", "v_ego", "a_ego", "accel_cmd")):
        continue
      active = None
      if "enabled" in npz:
        active = np.asarray(npz["enabled"]).astype(bool)
        if "should_stop" in npz:
          active = active & np.asarray(npz["should_stop"]).astype(bool)
      yield record, (np.asarray(npz["t"], dtype=float), np.asarray(npz["v_ego"], dtype=float),
                     np.asarray(npz["a_ego"], dtype=float), np.asarray(npz["accel_cmd"], dtype=float), active)


def run(store_dir: Path, tau_s: float) -> dict[str, Any]:
  rows = []
  events_total = 0
  for record, (t, v_ego, a_ego, accel_cmd, active) in iter_store_traces(store_dir):
    events_total += 1
    finite = np.isfinite(t) & np.isfinite(v_ego) & np.isfinite(a_ego) & np.isfinite(accel_cmd)
    if not np.all(finite):
      keep = np.flatnonzero(finite)
      if len(keep) < 5:
        continue
      t, v_ego, a_ego, accel_cmd = t[keep], v_ego[keep], a_ego[keep], accel_cmd[keep]
      active = active[keep] if active is not None else None
    result = evaluate_trace(t, v_ego, a_ego, accel_cmd, tau_s, active=active)
    if result is not None:
      result["event_ref"] = "{route}--{seg}--{hold_mono_ns}".format(**record["key"])
      rows.append(result)

  push_events = len(rows)
  onset_fraction = (sum(1 for r in rows if r["onset_within_tolerance"]) / push_events) if push_events else 0.0
  overlap_fraction = (sum(1 for r in rows if r["window_overlap"] >= WINDOW_OVERLAP_FLOOR) / push_events) if push_events else 0.0
  mean_overlap = float(np.mean([r["window_overlap"] for r in rows])) if rows else 0.0

  if push_events < MIN_PUSH_EVENTS:
    status = "insufficient_data"
  elif onset_fraction >= EVENT_FRACTION_FLOOR and overlap_fraction >= EVENT_FRACTION_FLOOR:
    status = "pass"
  else:
    status = "fail"

  return {
    "artifact": "estimator_equivalence",
    "generated_utc": datetime.now(UTC).replace(microsecond=0).isoformat(),
    "status": status,
    "event_store": str(store_dir),
    "dist_lpf_tau_s": tau_s,
    "events_total": events_total,
    "push_events": push_events,
    "min_push_events": MIN_PUSH_EVENTS,
    "criteria": {
      "onset_tolerance_s": ONSET_TOLERANCE_S,
      "onset_within_tolerance_fraction": onset_fraction,
      "onset_fraction_floor": EVENT_FRACTION_FLOOR,
      "window_overlap_floor": WINDOW_OVERLAP_FLOOR,
      "window_overlap_pass_fraction": overlap_fraction,
      "mean_window_overlap": mean_overlap,
    },
    "events": rows,
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Estimator-equivalence gate artifact (spec 5.5.2 / F21)")
  parser.add_argument("--event-store", default=str(DEFAULT_EVENT_STORE))
  parser.add_argument("--tau-s", type=float, default=P.DIST_LPF_TAU_S,
                      help=f"LPF time constant under test (default: STOPPING_PARAMS.DIST_LPF_TAU_S = {P.DIST_LPF_TAU_S})")
  parser.add_argument("--output-json", default=None)
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  store_dir = Path(args.event_store).expanduser()
  if not (store_dir / "events.jsonl").is_file():
    print(f"[estimator-equivalence] event store absent: {store_dir} (build it with build_event_store.py)", file=sys.stderr)
    return 2

  report = run(store_dir, args.tau_s)
  crit = report["criteria"]
  print(f"[estimator-equivalence] status={report['status']} tau={report['dist_lpf_tau_s']}s "
        + f"events={report['events_total']} push_events={report['push_events']}")
  print(f"[estimator-equivalence] onset within +/-{crit['onset_tolerance_s']}s: "
        + f"{crit['onset_within_tolerance_fraction']:.2%} (floor {crit['onset_fraction_floor']:.0%})")
  print(f"[estimator-equivalence] window overlap >= {crit['window_overlap_floor']:.0%}: "
        + f"{crit['window_overlap_pass_fraction']:.2%} of events (mean {crit['mean_window_overlap']:.2%})")

  if args.output_json:
    out = Path(args.output_json).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(f"[estimator-equivalence] output_json={out}")

  if report["status"] == "pass":
    return 0
  if report["status"] == "insufficient_data":
    return 2
  return 1


if __name__ == "__main__":
  raise SystemExit(main())
