# Stopping Log Sync Tooling

Scripts in this folder support the stopping-behavior workflow by syncing only unseen log files from a comma device over SSH
and appending a summary to the project worklog.

## Current Controller Status (2026-02-08)

- `stopping_controller` is now the only stop-controller path for new-long.
- Active stop controller:
  - module: `selfdrive/controls/lib/stopping_controller.py`
  - integration: `selfdrive/controls/lib/longcontrol.py`
- Legacy new-long stop branch was removed after rollout-focused rewrite tuning.
- Source-of-truth project narrative and progress checkpoints:
  - `docs/stopping_behavior_worklog.md`
- Next milestone:
  - on-road validation and threshold tuning of `stopping_controller`.

## Scripts

- `sync_new_logs.py`
  - Lists route log files on device (`qlog`, `qlog.bz2` by default).
  - Compares against local state.
  - Downloads only new/changed files.
  - Writes a JSON report for each run.

- `append_sync_report.py`
  - Reads a sync JSON report.
  - Appends a structured session entry to `docs/stopping_behavior_worklog.md` (or another markdown file).

- `device_stop_settings.py`
  - Reads current stop-related settings from device before analysis/tuning.
  - Can apply fine-tuning values with range checks.
  - Writes JSON snapshots/results for traceability.

- `run_stopping_cycle.py`
  - Wrapper that runs `device_stop_settings.py snapshot`, `sync_new_logs.py`, then `append_sync_report.py`.
  - Optional integrated analysis mode can run stop-event extraction and append analysis metrics.
  - Generates timestamp-matched settings/report files.
  - Useful for repeatable "one command per drive" collection cycles.

- `analyze_stopping_behavior.py`
  - Reads local qlogs directly and detects stop events.
  - Generates per-event interactive HTML plots and `summary.md`/`summary.json`.
  - Designed for before/after stop behavior review.

- `append_analysis_report.py`
  - Appends analysis summary metrics into `docs/stopping_behavior_worklog.md`.

- `compare_stopping_runs.py`
  - Compares two analysis `summary.json` files (before/after) and reports metric deltas.

- `check_harsh_stops.py`
  - Runs an offline pass/fail gate on stop-event summaries for harsh-stop symptoms.
  - Uses thresholds on end-stop jerk, command jerk, accel step, and minimum observed accel.
  - Exits non-zero on regression so it can be used in repeatable local checks.

- `stopping_model.py`
  - Shared utilities for fitting and simulating a lightweight stop-response model from log samples.
  - Captures delayed command response and low-speed/clutch-relief effects for offline replay.

- `fit_stopping_model.py`
  - Fits a stop-response model from one or more analysis `summary.json` files plus local qlogs.
  - Searches command-delay frames automatically and writes a model JSON artifact.

- `check_harsh_stops_model.py`
  - Uses a fitted model to replay each stop event and predict harsh-stop signatures.
  - Supports:
    - recorded-command replay (`--command-source recorded`)
    - controller replay (`--command-source controller`) for offline algorithm checks.
  - Runs a pass/fail gate on predicted end-stop jerk and predicted acceleration floor.

- `find_stop_events_corpus.py`
  - Scans all downloaded routes for stop events and aggregates corpus-level metrics.
  - Supports `engaged_signal`, `speed_transition`, and `hybrid` event detection modes.
  - Useful for validating stop detection coverage and building a broad baseline.

- `diagnose_stop_failures.py`
  - Consumes corpus `summary.json` and ranks likely failure modes.
  - Flags events for late signal onset, long moving stops, creep-after-stop, harsh decel, and near-hold stop release/rebound patterns.
  - Produces a markdown triage report with prioritized routes and ready-to-run graph commands.

- `build_review_pack.py`
  - Builds graph packs for top problematic routes directly from a corpus summary.
  - Uses the same diagnosis ranking logic/thresholds to pick routes.
  - Writes a `manifest.json` with route scores, commands, and output directories.

- `find_bookmarked_bad_stops.py`
  - Scans downloaded qlogs for `userFlag` bookmarks and matches each bookmark to the nearest stop event.
  - Produces bookmark triage reports (`summary.json`, `summary.md`) with event metrics and route commands.
  - Intended for "bookmark bad stop during drive, analyze later" workflow.

## Default Local Paths

- Download root: `~/.comma/stopping_behavior/downloads`
- State file: `~/.comma/stopping_behavior/sync_state.json`
- Report dir: `~/.comma/stopping_behavior/reports`
- Settings snapshots: `~/.comma/stopping_behavior/settings`

## Typical Workflow

1. Snapshot current stopping/tuning settings from device first:

```bash
python tools/stopping/device_stop_settings.py snapshot --host comma
```

2. (Optional) Apply a fine-tuning value:

```bash
python tools/stopping/device_stop_settings.py set --host comma --set LongitudinalActuatorDelay=0.35
```

3. Sync new logs from a reachable host alias:

```bash
python tools/stopping/sync_new_logs.py --host comma --include-rlog --newest-first
```

4. Append the generated report to the stopping worklog:

```bash
python tools/stopping/append_sync_report.py \
  --report-file ~/.comma/stopping_behavior/reports/<report>.json \
  --settings-file ~/.comma/stopping_behavior/settings/<settings_snapshot>.json
```

5. Generate stop-event analysis graphs and summary:

```bash
python tools/stopping/analyze_stopping_behavior.py \
  --host commawifi \
  --route <route_id> \
  --settings-file ~/.comma/stopping_behavior/settings/<settings_snapshot>.json
```

6. Append analysis notes to worklog:

```bash
python tools/stopping/append_analysis_report.py \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route>/<stamp>/summary.json
```

7. Compare before vs after runs:

```bash
python tools/stopping/compare_stopping_runs.py \
  --before ~/.comma/stopping_behavior/analysis/.../before_summary.json \
  --after ~/.comma/stopping_behavior/analysis/.../after_summary.json
```

## Useful Options

`sync_new_logs.py`
- `--host commawifi`
- `--dry-run` (discover and compare only)
- `--max-downloads N` (throttle transfer)
- `--newest-first` (pull latest files first when using `--max-downloads`)
- `--state-file ~/.comma/stopping_behavior/sync_state_stopping.json` (project-specific state)
- `--remote-root /custom/path` (repeatable)
- `--file-name qlog.bz2` (repeatable)
- `--verbose`

`append_sync_report.py`
- `--worklog docs/stopping_behavior_worklog.md`
- `--title "Log sync from comma"`
- `--note "First post-drive pull"` (repeatable)
- `--settings-file ~/.comma/stopping_behavior/settings/<snapshot>.json`
- `--dry-run`

`device_stop_settings.py`
- `list` (show supported keys and ranges)
- `snapshot --host commawifi`
- `set --host comma --set LongitudinalActuatorDelay=0.35 --set MaxDesiredAcceleration=2.5`
- `set --dry-run ...` (validate without writing)

`run_stopping_cycle.py`
- `python tools/stopping/run_stopping_cycle.py --host commawifi --max-downloads 80 --newest-first`
- `--state-file ~/.comma/stopping_behavior/sync_state_stopping.json`
- `--include-rlog`
- `--skip-settings` (if settings already captured for this run)
- `--settings-dry-run` (validate/read requested stop-tune values without writing)
- `--dry-run-sync` (discover only, still writes report and can append markdown)
- `--analyze --analysis-event-mode hybrid --analysis-min-entry-speed 0.5` (broad stop coverage)
- `--analyze --analysis-event-mode engaged_signal --analysis-min-entry-speed 2.0` (strict OP stop-signal events)
- `--analysis-route <route_id>` (pin analysis to a specific route)

`analyze_stopping_behavior.py`
- `--route 000006c0--81e575d831` (explicit route)
- `--min-entry-speed 2.0` (raise/lower event strictness)
- `--event-mode hybrid` (default; combines signal and speed transition detection)
- `--require-enabled-speed-events` (keep only speed/hybrid events with at least one enabled sample)
- `--output-dir /tmp/stopping_analysis` (custom output location)
- Produces `events/event_*.html` plus `summary.md` and `summary.json`

`append_analysis_report.py`
- `--summary-json ~/.comma/stopping_behavior/analysis/.../summary.json`
- `--note "Baseline after downtown drive"`

`compare_stopping_runs.py`
- `--before <summary_before.json> --after <summary_after.json>`
- `--output ~/.comma/stopping_behavior/analysis/comparison_<stamp>.md`

`check_harsh_stops.py`
- `--summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route>/<stamp>/summary.json`
- `--min-events 4 --min-entry-speed 0.20`
- `--max-harsh-rate 0.20`
- `--max-end-stop-jerk 0.75 --max-end-stop-cmd-jerk 3.0 --max-end-stop-accel-step 0.08 --min-a-ego-floor -1.05`
- `--output-json ~/.comma/stopping_behavior/analysis/<stamp>_harsh_check.json`

`fit_stopping_model.py`
- `--summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route>/<stamp>/summary.json` (repeatable)
- `--event-source speed` (recommended for broad engaged stop transitions)
- `--max-delay-frames 25 --min-speed 0.0 --max-speed 1.8`
- `--relief-cmd-threshold -0.25 --low-speed-ref 1.2`
- `--min-rows 120`
- `--output ~/.comma/stopping_behavior/models/stopping_model_<stamp>.json`

`check_harsh_stops_model.py`
- `--model-json ~/.comma/stopping_behavior/models/stopping_model_<stamp>.json`
- `--summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route>/<stamp>/summary.json` (repeatable)
- `--event-source speed --command-source controller`
- `--event-source speed --min-events 6 --min-entry-speed 0.20`
- `--max-harsh-rate 0.20 --max-pred-end-jerk 0.80 --min-pred-a-floor -1.10 --max-pred-rollout-m 2.0`
- `--stopping-speed-breakpoint 0.40 --stop-accel -2.0` (controller replay mode)
- `--controller-strategy v2` (single-strategy gate; current default)
- `--compare-controller-strategies baseline,v2,v3`
  (compare tuning presets within the single controller implementation)
- `--output-json ~/.comma/stopping_behavior/analysis/model_harsh_check_<stamp>.json`

`find_stop_events_corpus.py`
- `--host commawifi --verbose-routes`
- `--event-mode hybrid --min-entry-speed 0.5` (default; broad stop coverage)
- `--event-mode engaged_signal --require-enabled-speed-events --min-entry-speed 2.0` (strict OP-only focus)
- `--event-mode speed_transition --require-enabled-speed-events --min-entry-speed 0.0`
  (recommended baseline for all engaged stop transitions, including red-light/force-stop paths)
- Produces `summary.md` and `summary.json` under `~/.comma/stopping_behavior/analysis/corpus/<host>/<stamp>/`

`diagnose_stop_failures.py`
- `--summary-json ~/.comma/stopping_behavior/analysis/corpus/commawifi/<stamp>/summary.json`
- `--focus-source engaged` (default; signal + hybrid)
- `--top-n 20`
- `--positive-cmd-threshold-mps2 0.02 --hold-rebound-threshold-mps 0.15`
- Clutch-focused thresholds (when `shouldStop` stays true):
  - `--should-stop-rebound-threshold-mps 0.08`
  - `--should-stop-unexpected-accel-threshold-mps2 0.10`
  - `--should-stop-relief-spike-threshold-mps2 0.18`
- Optional comfort-calibrated run:
  - `--end-stop-jerk-threshold-mps3 0.5 --end-stop-accel-step-threshold-mps2 0.06`
  - `--cmd-jerk-threshold-mps3 0.03 --cmd-step-threshold-mps2 0.003 --creep-threshold-mps 0.08`
- `--output ~/.comma/stopping_behavior/analysis/corpus/commawifi/<stamp>/failure_diagnosis.md`

`build_review_pack.py`
- `--summary-json ~/.comma/stopping_behavior/analysis/corpus/commawifi/<stamp>/summary.json`
- `--focus-source engaged --top-routes 5`
- Supports the same clutch-disturbance thresholds as diagnosis:
  - `--unexpected-accel-threshold-mps2`, `--stable-cmd-accel-delta-threshold-mps2`
  - `--should-stop-rebound-threshold-mps`
  - `--should-stop-unexpected-accel-threshold-mps2`
  - `--should-stop-relief-spike-threshold-mps2`
- `--dry-run` (preview commands only)
- `--analysis-root ~/.comma/stopping_behavior/analysis/review_pack`

`find_bookmarked_bad_stops.py`
- `--host commawifi --event-mode engaged_signal --min-entry-speed 0.5`
- `--match-window-before 8 --match-window-after 8`
- `--output-root ~/.comma/stopping_behavior/analysis/bookmarks`

## Recommended Engaged-Stop Triage Flow

Use this when the target symptom is a near-hold disturbance while OP remains engaged.

1) Build an engaged-stop corpus (all enabled stop transitions):
```bash
python tools/stopping/find_stop_events_corpus.py \
  --host commawifi \
  --event-mode speed_transition \
  --require-enabled-speed-events \
  --min-entry-speed 0.0
```

2) Rank failures with clutch-focused thresholds:
```bash
python tools/stopping/diagnose_stop_failures.py \
  --summary-json ~/.comma/stopping_behavior/analysis/corpus/commawifi/<stamp>/summary.json \
  --focus-source speed \
  --should-stop-rebound-threshold-mps 0.08 \
  --should-stop-unexpected-accel-threshold-mps2 0.10 \
  --should-stop-relief-spike-threshold-mps2 0.18 \
  --output ~/.comma/stopping_behavior/analysis/corpus/commawifi/<stamp>/failure_diagnosis_speed.md
```

3) Generate graph packs for top routes:
```bash
python tools/stopping/build_review_pack.py \
  --summary-json ~/.comma/stopping_behavior/analysis/corpus/commawifi/<stamp>/summary.json \
  --focus-source speed \
  --top-routes 5 \
  --event-mode speed_transition \
  --min-entry-speed 0.0 \
  --require-enabled-speed-events
```

## Model-Based Offline Regression Flow

Use this to estimate harsh-stop risk from logs before another drive.

1) Fit model from recent stop summaries:
```bash
python tools/stopping/fit_stopping_model.py \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route1>/<stamp>/summary.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route2>/<stamp>/summary.json \
  --event-source speed \
  --max-delay-frames 25 \
  --max-speed 1.8 \
  --min-rows 120 \
  --output ~/.comma/stopping_behavior/models/stopping_model_<stamp>.json
```

2) Gate predicted harsh stops:
```bash
python tools/stopping/check_harsh_stops_model.py \
  --model-json ~/.comma/stopping_behavior/models/stopping_model_<stamp>.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route1>/<stamp>/summary.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route2>/<stamp>/summary.json \
  --event-source speed \
  --min-events 6 \
  --max-harsh-rate 0.20 \
  --output-json ~/.comma/stopping_behavior/analysis/model_harsh_check_<stamp>.json
```

2a) Gate candidate stop-controller behavior offline (recommended before drive):
```bash
python tools/stopping/check_harsh_stops_model.py \
  --model-json ~/.comma/stopping_behavior/models/stopping_model_<stamp>.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route1>/<stamp>/summary.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route2>/<stamp>/summary.json \
  --event-source speed \
  --command-source controller \
  --controller-strategy v2 \
  --min-events 6 \
  --max-pred-end-jerk 0.70 \
  --max-pred-rollout-m 2.0 \
  --max-harsh-rate 0.10 \
  --output-json ~/.comma/stopping_behavior/analysis/model_harsh_check_controller_<stamp>.json
```

2a.1) Rank multiple controller approaches (smoothness vs rollout):
```bash
python tools/stopping/check_harsh_stops_model.py \
  --model-json ~/.comma/stopping_behavior/models/stopping_model_<stamp>.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route1>/<stamp>/summary.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route2>/<stamp>/summary.json \
  --event-source speed \
  --command-source controller \
  --controller-strategy v2 \
  --compare-controller-strategies baseline,v2,v3 \
  --min-events 6 \
  --max-pred-end-jerk 0.70 \
  --max-pred-rollout-m 2.0 \
  --max-harsh-rate 0.10 \
  --output-json ~/.comma/stopping_behavior/analysis/model_harsh_rank_<stamp>.json
```
The output JSON contains `strategy_ranking` and `best_strategy`.

2b) Stretch gate (expected to fail until more tuning):
```bash
python tools/stopping/check_harsh_stops_model.py \
  --model-json ~/.comma/stopping_behavior/models/stopping_model_<stamp>.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route1>/<stamp>/summary.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route2>/<stamp>/summary.json \
  --event-source speed \
  --command-source controller \
  --min-events 6 \
  --max-pred-end-jerk 0.65 \
  --max-harsh-rate 0.10
```

3) Cross-check against measured harsh gate:
```bash
python tools/stopping/check_harsh_stops.py \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route1>/<stamp>/summary.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route2>/<stamp>/summary.json \
  --min-events 6 \
  --max-harsh-rate 0.20
```
