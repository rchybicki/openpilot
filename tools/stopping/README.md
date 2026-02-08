# Stopping Log Sync Tooling

Scripts in this folder support the stopping-behavior workflow by syncing only unseen log files from a comma device over SSH
and appending a summary to the project worklog.

## Current Controller Status (2026-02-08)

- Rewrite is **in progress** (scaffold implemented, not final-tuned yet).
- Current branch contains both:
  - staged improvements to the existing stop controller,
  - a new rewrite path (`stopping_v2`) behind a runtime toggle.
- Staged improvements on legacy path:
  - low-speed output slew in stop phase,
  - low-speed transition slew across active states,
  - shouldStop release-lock hysteresis for clutch leapfrogging.
- Rewrite path:
  - module: `selfdrive/controls/lib/stopping_v2.py`
  - runtime toggle param: `DisableStoppingV2`
    - unset/`false`: use `stopping_v2`
    - `true`: force legacy stop path.
- Source-of-truth project narrative and progress checkpoints:
  - `docs/stopping_behavior_worklog.md`
- Next milestone:
  - on-road A/B validation and threshold tuning of `stopping_v2`.

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
python tools/stopping/device_stop_settings.py set --host comma --set StoppingDecelRate=0.35
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
- `--state-file ~/.comma/stopping_behavior/sync_state_v2.json` (project-specific state)
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
- `set --host comma --set VEgoStopping=0.45 --set StoppingErrorFactor=1.8`
- `set --dry-run ...` (validate without writing)

`run_stopping_cycle.py`
- `python tools/stopping/run_stopping_cycle.py --host commawifi --max-downloads 80 --newest-first`
- `--state-file ~/.comma/stopping_behavior/sync_state_v2.json`
- `--include-rlog`
- `--skip-settings` (if settings already captured for this run)
- `--set-stopping-speed-breakpoint 0.35 --set-stopping-error-factor 1.5` (apply/read stop-tune pair before sync)
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

`find_stop_events_corpus.py`
- `--host commawifi --verbose-routes`
- `--event-mode hybrid --min-entry-speed 0.5` (default; broad stop coverage)
- `--event-mode engaged_signal --require-enabled-speed-events --min-entry-speed 2.0` (strict OP-only focus)
- `--event-mode speed_transition --require-enabled-speed-events --min-entry-speed 0.0` (recommended baseline for all engaged stop transitions, including red-light/force-stop paths)
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
