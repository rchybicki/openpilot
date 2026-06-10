# Stopping Workflow Tooling

Scripts supporting the stopping-stack workflow on the 2022 Hyundai Santa Fe HEV. This README
documents the toolset **as deployed** after the June 2026 redesign (legacy forest controller
active, V2 dark — see `docs/stopping/architecture.md`).

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
