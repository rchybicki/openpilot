# Stopping Workflow Tooling

Scripts supporting the stopping-stack workflow on the 2022 Hyundai Santa Fe HEV. This README
documents the toolset **as deployed** after the June 2026 redesign (legacy forest controller
active, V2 dark — see `docs/stopping/architecture.md`).

## Standard simulator runner (2026-10-05)

The fixed gate list for any stopping change (PLAN sections 72 and 82: H1-H6 + comfort) runs with one command:
`python -m openpilot.tools.stopping.sim.run_gates [--base REV] [--diff FILE]`. Engine, case set, exact replay and gate rules are
in `tools/stopping/sim/` (see `sim/README.md`); large data and the content-addressed run cache live in `~/.route_sync/work/sim`.
It supersedes the per-cycle runners under `/tmp/cyc_*`. Much of the June 2026 pipeline below is stale.

## Per-drive report (2026-10-05)

Run it right after each route sync (idempotent; copied logs only, no device access, no live msgq readers):

```sh
.venv/bin/python tools/route_sync/refresh_routes.py --include-rlog && .venv/bin/python tools/stopping/drive_report.py
.venv/bin/python tools/stopping/drive_report.py --routes 00002232,00002235 [--rebuild]   # named routes (needed on an empty state)
```

`--routes` with no matching local route says so and exits 2.

- Default input: every local route with rlogs that is newer than the oldest reported route and not reported yet, or whose rlog count
  grew (a route pruned by the retention keeps its fuller report; `--rebuild` reports it again anyway). State:
  `~/.route_sync/work/drive_report/state.json`. Caches (scan 0.6 MB/segment, replay inputs, planner and replay results per arm in
  `planner_v2/`, `replay_v2/`, `delta_v2/`) are in the same directory, keyed by the arm and its replay implementation
  (`drive_replay.impl_key`: the replay / extraction code, the toggle defaults, the sender test helper and the working-tree content
  of unpinned modules that differ from the commit; a delta names both arms' keys; the replay inputs by the extraction code), so a report survives the route retention.
- Stage A (`scan_segment`): one pass per rlog segment (+ qlog bookmarks) into compact series, incl. sendcan SCC12, wheel pulses and
  IMU (`review/kcs1_reps.read_route`). 2 spawn workers.
- Stage B: engaged stops (cycle-1003 census rule, the definition behind the NEW/history case groups) and holds (engaged standstill
  >= 1 s). Interrupted approaches (a stop the census leaves out because the last 4 s were not all longActive, engaged in the 30 s
  before) and pedal takeovers (brake while longActive; gas while longActive with a braking command) get trial replay windows; they
  are not in the stop table. Per stop on the LOGGED columns: rest (census: min dRel -0.5..+1.0 s) and rest_med (median +0.3..+1.3 s), min gap, service
  entry bite (min command 0.6 s after the logged INACTIVE -> active phase change), full-approach pumps on the command and on the
  logged aEgo (from the last 3 m/s crossing; the pulse slope is quantisation noise on logs), a_stop / j300 (kcs1 terminal metrics on the IMU), felt (census), StopReq set/clear
  pairs at rest within 1 s, hold in pid, `starting` under a hold, takeovers, launch by car/driver, bookmarks (the
  find_bookmarked_bad_stops rule). ATTENTION: bookmark, rest < 3.5 or > 6, a_stop <= -0.6, bite <= -0.10, takeover, StopReq chatter,
  hold in pid, `starting` under hold, fidelity drift.
- Exact replay (`drive_replay.py`): LongControl + service + Hyundai sender on the logged inputs of every stop/hold span, controller
  code pinned to the drive's commit by `git show` (meta-path finder; a flag override rewrites one line of `stopping_flags.py`).
  Fidelity vs the log: every StopReq toggle and logged phase change matched within 50 ms, command errors > 0.05 on <= 0.1 % of the
  frames, else "SIM OUT OF SYNC WITH CAR". Python files that differ from the replayed commit outside the pinned directories
  (`selfdrive/controls/lib`, `selfdrive/controls`, `opendbc/car/hyundai`) still load from the working tree; the report names them per
  replayed commit (e.g. 1e0327943d: `frogpilot_variables.py`).
- Trials (`TRIALS`): per flag, on vs off. A build with the flag on = live: the revert rules of the trial run on the log and print
  `REVERT: <FLAG> = False` with evidence. A trial with a span whose replay failed is INCOMPLETE, never PASS (report, summary,
  INDEX); the route is not marked reported, so the next run retries the failed spans, and the exit status is 3. A build without it prints "no <trial> drive yet" and lists the counterfactual (reference
  commit, flag on vs off, open loop). The logged release-end race (the class E3 removes) is listed for every build.
  - E3 (`RELEASE_END_STOPPED_LEAD_REHOLD`, reference 2e39594627): PLAN section 60 re-hold checks; the after-re-hold grab counts only
    where the flag-off replay does not grab too (PLAN section 77).
  - LINE (`SANTA_FE_STOP_LINE`, reference 56e1512892; `GOVERNOR_BAND_PROFILE` is derived from it): neither the line nor the band latch
    is published, so the planner runs in lockstep on the logged planner inputs (`drive_replay.planner_run`, the eb5 pp6.py planner
    arm; 20 s warm-up; input bound 3 or 10 ms, whichever matches the logged aTarget better per span) with the flag on and off. The arm
    that drove (live: on; else off) replays the logged plan; the other one replays the logged plan + its planner delta, so frames
    where the arms agree stay bit-identical. Planner fidelity (reference arm vs the logged aTarget) is printed per drive.
  - LINE revert rules (`LINE_RULES`; eb4_lfl_final_build.json per-drive rule, corrected by eb5_lfl_r4_check.json and
    LFL_CODE_REVIEW_astra.md), every one relative to the flag-off replay of the same drive: R1 code defects (braking-component
    release > 0.125 per tick beyond OFF, positive floor, positive command capped, burst-hold failure, authority or deepening on a
    provenance rejection), R2 uncertified-lead episode > 0.3 deeper (certificate off, out of the stopped class beyond the burst hold,
    or a crawler: median vLead > 0.15 over 1 s), R3 queue restart binding (2 events), R4 every release/re-arm pump at the wire
    (2 >= 0.10 or one >= 0.15), R5 landing (rest < 3.5 / min gap < 3.2 / wheel-stop gap > 6 m), R6 a_stop or stop wire <= -0.65
    (2 stops), R7 crawl-then-grab, R8 downhill, R9 creeping-lead re-grab, R10 ownership / StopReq (clear -> set at rest within 1 s, `sim.gates.chatter`) / hold protocol, H5 (gap at first
    motion > 0.3 m larger than OFF, no launch where OFF launches, false launch); R11 = Radek's ratings (the per-stop table has a
    rating column). Landing rules: open loop cannot replay the motion of the arm that did not drive, so the logged landing and
    command stand for the flag-on world (exact on a live drive; 'as if live' on a counterfactual drive) and the flag-off value is the
    logged one minus the trial's command effect (travel difference up to the wheel stop, stop-level difference over the last
    0.5 s); a landing rule trips only where that effect makes it worse by > 0.2 m / > 0.05 m/s^2.
  - The per-stop table lists line (first arm time, speed, gap, armed seconds), extra braking (planner / wire), band latch, entry
    bite on/off, rest, wheel-stop gap, a_stop, stop wire, launch command on/off with the gap difference, and the rules per stop.
  - RADAR: the trial was removed with the parked radar time-alignment candidate (cycle_20261006 PLAN section 15); rebuild it
    with a future radar candidate.
- Output: `~/.route_sync/reports/drives/<route>.md` + `.json` and one line per drive in `INDEX.md`. Every section (stops, bookmarks,
  races, each trial) renders on every drive, also without census stops. Repo docs are not written; copy the summary into the
  worklog. Tests: `pytest tools/stopping/test_drive_report.py -p no:xdist -o addopts=` (the repo addopts start `-n auto` workers;
  the 2232 smoke test runs when its scan is cached). Both loaders write the flag values into the snapshot's `stopping_flags.py`
  (derived flags follow) and assert every flag against that file; nothing sets flags with setattr. They stay two loaders because
  they pin different things: `drive_replay` pins a drive's commit in the two controller directories and tolerates compiled
  drift (old drives must replay), `sim/loader.py` pins base + diff over every production root and refuses compiled or non-.py
  drift (a gate run must test exactly the candidate).

## Operating Contract

- North-star goal: **always stop perfectly** (no noticeable final jerk, no rebound/leapfrog,
  stable hold, controlled rollout). The 0-leapfrog measured baseline is a hard floor.
- Runtime source of truth: `selfdrive/controls/lib/longcontrol.py` (state machine + single
  `StopTargetArbiter`), `stopping_controller.py` (legacy forest, ACTIVE), and the dark V2 chain
  `stopping_params.py` / `stopping_plant.py` / `stopping_trajectory.py` / `stopping_tracker.py` /
  `stopping_controller_v2.py`.
- Documentation home: `docs/stopping/` (architecture, parameters, eval methodology, on-vehicle
  protocols, redesign rationale). Evidence log: `docs/stopping/worklog.md`; history:
  `docs/stopping/archive/worklog_2026H1.md`.
- Route intake contract: `docs/route_refresh_process.md` (shared route cache under
  `~/.route_sync/`).
- Promotion rule: **the sim develops, the measurement promotes.** Offline replay verdicts never
  promote a tuning change by themselves; paired measured statistics (with MDE stated) do.
  One named parameter per tuning commit, report in the commit message.

## Pipeline overview

```
device rlogs/qlogs ──refresh_routes──► ~/.route_sync/ cache
        │
        ├─ analyze_stopping_behavior.py   per-route stop-event detection + metrics + graphs
        ├─ build_event_store.py           full-corpus event store (stable keys, dual-rate metrics)
        │       └─► ~/.comma/stopping_behavior/event_store/{events.jsonl, events/*.npz}
        ├─ sim_replay.py                  closed-loop replay (legacy and/or V2) through PlantModel
        ├─ similarity_gate.py             spec-7.6 two-tier legacy-vs-V2 gate + triage table
        ├─ estimator_equivalence.py       spec-5.5.2 estimator artifact (mandatory gate row)
        ├─ check_harsh_stops.py           measured harsh/leapfrog gate (scoring_config defaults)
        ├─ paired_stats.py                paired/stratified stats, MDE-stating, refusal-capable
        └─ check_leapfrog_alignment.py    model-truthfulness loop (sim_replay predictions)
```

`scoring_config.py` is the single frozen threshold/flag definition site (generated from the
operative `check_harsh_stops.classify_event` logic; `test_scoring_config.py` diffs it against a
recorded run). Methodology details, gate protocol, and the current gate status:
`docs/stopping/eval.md`.

## The stamped cycle

`run_stopping_cycle.py` snapshots device settings, runs the shared route refresh, per-route
analysis, and the requested gate stages, then appends a dated report block to the worklog.

```bash
python tools/stopping/run_stopping_cycle.py --host comma --newest-first --max-downloads 80
```

- rlog fetch is **default ON** (`--no-include-rlog` to disable).
- New-pipeline stages: `--build-event-store` (+ `--event-store-max-routes`), `--run-sim-replay`,
  `--run-similarity-gate`. Legacy model-gate stages remain runnable until the cleanup commit.
- The shadow-analysis stage is version-aware (dispatches on the `stopping_shadow` debug-dict
  `version`; `--skip-shadow-analysis` available for v2-era routes).
- Worklog: the cycle still passes the LEGACY path (`docs/stopping_behavior_worklog.md`, now a
  stub) until its scheduled cleanup-commit `DEFAULT_WORKLOG` flip. Prefer passing
  `--worklog docs/stopping/worklog.md` explicitly. The standalone `append_*_report.py` scripts
  already default to `docs/stopping/worklog.md`.

## Script catalog

New-pipeline (redesign):

- `scoring_config.py` — frozen scoring/gate config dataclass + canonical JSON; imported by
  `check_harsh_stops.py` and the cycle. Threshold changes require a version bump + re-baseline
  note in `docs/stopping/eval.md`.
- `build_event_store.py` — full-corpus event store builder. rlog-first (qlog fallback tagged),
  stable keys `(route, seg, hold_mono_ns)`, dual-rate metric blocks, era flags
  (`--signals-version`, `--telemetry-version`, `--accel-cmd-source`), on-demand rlog fetch
  (`--fetch-missing-rlogs`).
- `sim_replay.py` — closed-loop replay of any facade-seam controller (`--controller
  legacy|v2|both`) through a `PlantModel` (`--plant ref|refit|both|<json>`) on event-store
  scenarios and/or `stop_scenarios.py` fixtures (`--include-fixtures`). Also provides the
  integrated LongControl-with-V2 replay mode used by the gate.
- `similarity_gate.py` — the spec-7.6 two-tier gate: Tier-1 outcome-envelope bounds (pass/fail,
  dual plant), Tier-2 trace-RMS diagnostics with mandatory triage classifications
  (`--triage-json`), triage-table emitter (`--triage-table-out`), estimator-artifact row
  (`--estimator-report-json`). Precondition for the `USE_STOPPING_V2` flip.
- `estimator_equivalence.py` — replays the V2 disturbance estimator against legacy single-frame
  trigger semantics on the event store (`--tau-s`); must pass before any gate run.
- `paired_stats.py` — Wilcoxon + BCa bootstrap + McNemar (paired), stratified Mann-Whitney
  (on-road); prints `n` and `mde_at_n` on every verdict and refuses below the pre-registered
  power floor (exit 2, required n printed).
- `fit_plant_model.py` — system-ID refit of the 7-feature plant on event-store rlog data
  (telemetry-era-aware exclusions, holdout RMSE reported). Acceptance to replace the reference
  fit: holdout RMSE ≤ 1.1× the archived fit AND leapfrog-alignment recall ≥ current.

Kept measurement/triage tools:

- `analyze_stopping_behavior.py` — per-route detection + metrics + plots; rlog-first; v2-aware
  shadow dispatch; `Sample.accel_cmd` sources from `carOutput.actuatorsOutput.accel` (the sent
  value) for telemetry_version ≥ 2 routes.
- `check_harsh_stops.py` — measured harsh/leapfrog gate; defaults from `scoring_config` (CLI
  flags are explicit overrides only).
- `check_leapfrog_alignment.py` — measured-vs-predicted leapfrog alignment; accepts sim_replay
  predictions (stable keys); legacy model-gate prediction JSONs accepted until cleanup.
- `find_stop_events_corpus.py`, `diagnose_stop_failures.py`, `build_review_pack.py`,
  `find_bookmarked_bad_stops.py`, `compare_stopping_runs.py`, `log_schema_helpers.py`,
  `stop_and_go_helpers.py`, `force_coast.py` — unchanged.
- `device_stop_settings.py` — device Params snapshot/apply
  (`snapshot --host comma` / `set --host comma --set Key=Value`); includes read-only rows for
  `IncreasedStoppedDistance` + the 4 weather variants (the commit-10 pre-flip check).
- `append_analysis_report.py`, `append_cycle_report.py`, `append_sync_report.py` — worklog
  appenders; default `--worklog docs/stopping/worklog.md`.
- `holdout_routes.txt` — the 5 pinned holdout routes used by gates/benchmarks. Never fit on
  them.

Legacy tools — **scheduled for deletion in the cleanup commit** (after the V2 flip + ≥ 2-week
soak; full list and triggers: `docs/stopping/architecture.md` section 5). Still functional while
the forest is the active controller:

- `stopping_model.py` (legacy `FittedStoppingModel` loader; superseded by
  `selfdrive/controls/lib/stopping_plant.py`), `fit_stopping_model.py`,
  `check_harsh_stops_model.py` (legacy replay gate), `horizon_optimizer.py`,
  `benchmark_controller_variants.py`, `train_profile_selector.py`, `analyze_stopping_shadow.py`
  (rlog-download machinery already lifted into `build_event_store.py`), plus their test files.

## Default local paths

- Route cache: `~/.route_sync/` (state: `state.json`, reports: `reports/`, data:
  `data/media/0/realdata/<route>--<seg>/`)
- Event store: `~/.comma/stopping_behavior/event_store/`
- Analysis outputs: `~/.comma/stopping_behavior/analysis/`
- Settings snapshots: `~/.comma/stopping_behavior/settings/`
- Archived plant fits (in-repo): `docs/stopping/archive/plant_model_*.json`

## Tests

Canonical build-free local invocation (the repo venv is broken; pinned in the redesign spec):

```bash
mkdir -p /tmp && touch /tmp/op_empty_pytest.ini
PYTHONPATH=<repo>:<repo>/.venv/lib/python3.11/site-packages /opt/homebrew/bin/python3.11 \
  -m pytest tools/stopping -q --timeout=300 --noconftest -p no:randomly -p no:cacheprovider \
  -c /tmp/op_empty_pytest.ini --rootdir=<repo>
```

New test modules must be import-clean without scons artifacts (pure python + numpy). Event-store-
dependent tests skip gracefully when `~/.comma/stopping_behavior/event_store` is absent.

## Deploy

Per CLAUDE.md: `ssh -tt comma 'cd /data/openpilot && ./fullupdate.sh'` (fallback `commawifi`),
then verify `git rev-parse --short HEAD` on-device. Kill-switch flips follow
`docs/stopping/on_vehicle_protocols.md` (one constant per session, first-drive checklist).

## Worklog entry template

```
### YYYY-MM-DD: <short title>

- Commands: <exact invocations>
- Artifacts: <paths>
- Before/after: <metrics, with n and MDE for any verdict>
- Decision: keep / reject / escalate (+ why)
```
