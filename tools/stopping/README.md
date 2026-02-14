# Stopping Log Sync Tooling

Scripts in this folder support the stopping-behavior workflow by syncing only unseen log files from a comma device over SSH
and appending a summary to the project worklog.

## Operating Contract (Mandatory)

- North-star goal: **always stop perfectly** (no noticeable final jerk, no rebound/leapfrog, stable hold, controlled rollout).
- Runtime source of truth:
  - `selfdrive/controls/lib/stopping_controller.py`
  - `selfdrive/controls/lib/longcontrol.py`
- Process source of truth:
  - `tools/stopping/README.md` (how to run/tune/decide)
  - `docs/stopping_behavior_worklog.md` (what happened, with dates and evidence)

## Continuous Improvement Loop (How We Work)

This process is self-documented and self-improving by default. Every new data batch or user suggestion is run through the same loop:

1. Build/freeze evaluation inputs first.
   - Keep a train split and holdout split.
   - Keep pinned hard routes in holdout.
2. Record baseline before any change.
   - Run benchmark/gates on current code + current inverse defaults.
3. Run one scoped experiment at a time.
   - For user suggestions, treat suggestion as experiment candidate.
   - Define success criteria before tuning.
4. Validate with tests and offline comparisons.
   - Run focused pytest suite for stopping tools/controllers.
   - Compare measured (`check_harsh_stops.py`) and replay (`check_harsh_stops_model.py` / `benchmark_controller_variants.py`).
5. Promote only if objectively better.
   - Keep change only when gates improve or stay within agreed tolerances.
   - Revert or isolate when results regress.
   - Every 3 experiments, run a path review: `inverse` track vs runtime `current` track.
6. Document every cycle before moving on.
   - Add commands used, artifacts, before/after metrics, and keep/reject decision to the worklog.
7. Clean up continuously.
   - Remove stale defaults/notes, quarantine broken inputs, and simplify unused experimental paths.

If a step is skipped, the iteration is incomplete.

## Scripts

- SSH host defaults:
  - Remote scripts default to `--host commawifi` and fall back to `comma` automatically when `commawifi` is unreachable.

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
  - `summary.json` includes per-event engagement ratios (`enabled_ratio`, `stop_signal_ratio`, etc.) to help filter for true engaged stopping.
  - Designed for before/after stop behavior review.

- `append_analysis_report.py`
  - Appends analysis summary metrics into `docs/stopping_behavior_worklog.md`.

- `compare_stopping_runs.py`
  - Compares two analysis `summary.json` files (before/after) and reports metric deltas.

- `check_harsh_stops.py`
  - Runs an offline pass/fail gate on stop-event summaries for harsh-stop symptoms.
  - Uses thresholds on end-stop jerk, command jerk, accel step, and minimum observed accel.
  - Supports filtering to engaged stopping with `--min-enabled-ratio` and `--min-stop-signal-ratio` (recommended for controller work).
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
- `--fit-model --fit-event-source all --fit-recent-summaries 8` (rebuild model from all stopping events)
- Robust delay selection passthrough:
  - `--fit-delay-min-sample-ratio 0.40`
  - `--fit-delay-rmse-tolerance 0.03`
- `--run-model-gate --model-gate-command-source controller` (run offline controller gate on engaged+stopping scope)
- Baseline gate target in cycle defaults: `--model-gate-max-harsh-rate 0.50`
  (use stricter `0.10` as a stretch target while tuning)
- Full one-shot cycle:
  `python tools/stopping/run_stopping_cycle.py --host commawifi --analyze --analysis-event-mode speed_transition`
  `--analysis-min-entry-speed 0.0 --fit-model --fit-event-source all`
  `--fit-recent-summaries 8 --run-model-gate`

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
- `--max-leapfrog-rate 0.20` (optional; keeps leapfrog regressions separate from harsh gates)
- `--max-end-stop-jerk 0.75 --max-end-stop-cmd-jerk 3.0 --max-end-stop-accel-step 0.08 --min-a-ego-floor -1.05`
- `--max-speed-rebound-while-should-stop 0.08 --max-should-stop-unexpected-accel 0.10` (leapfrog detection thresholds)
- Output includes both `harsh_events`/`harsh_rate` and `leapfrog_events`/`leapfrog_rate`
- `--output-json ~/.comma/stopping_behavior/analysis/<stamp>_harsh_check.json`

`fit_stopping_model.py`
- `--summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route>/<stamp>/summary.json` (repeatable)
- `--event-source speed` (recommended for broad engaged stop transitions)
- By default, training rows require `controlsState.enabled` (commands published while disabled are not applied and corrupt the delay fit).
  - Override only for experiments with `--include-disabled`.
- `--max-delay-frames 25 --min-speed 0.0 --max-speed 1.8`
- Delay selection robustness defaults:
  - `--delay-min-sample-ratio 0.40` (avoid sparse high-delay fits)
  - `--delay-rmse-tolerance 0.03` (prefer lower delay when RMSE is near-equal)
- `--relief-cmd-threshold -0.25 --low-speed-ref 1.2`
- `--min-rows 120`
- `--output ~/.comma/stopping_behavior/models/stopping_model_<stamp>.json`

`check_harsh_stops_model.py`
- `--model-json ~/.comma/stopping_behavior/models/stopping_model_<stamp>.json`
- `--summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route>/<stamp>/summary.json` (repeatable)
- `--event-source speed --command-source controller`
- Controller scope defaults to engaged + stopping events:
  - `--controller-scope engaged_stopping`
  - `--controller-min-enabled-ratio 0.80`
- Controller replay window defaults to actual stopping-state spans:
  - `--controller-window-mode stopping_state`
  - `--controller-end-mode last_stopping_state`
  - Override with `event` / `should_stop` / `hold` options for debugging.
- `--event-source speed --min-events 6 --min-entry-speed 0.20`
- Baseline tuning target: `--max-harsh-rate 0.50`
- Stretch target: `--max-harsh-rate 0.10`
- Optional leapfrog gate: `--max-leapfrog-rate 0.20` (kept separate from harsh gate)
- `--max-pred-end-jerk 0.80 --min-pred-a-floor -1.10 --max-pred-rollout-m 2.0`
- `--max-pred-speed-rebound-while-should-stop 0.08 --max-pred-should-stop-unexpected-accel 0.10`
- `--stopping-speed-breakpoint 0.40 --stop-accel -2.0` (controller replay mode)
- Output includes both `harsh_events`/`harsh_rate` and `leapfrog_events`/`leapfrog_rate`
- `--output-json ~/.comma/stopping_behavior/analysis/model_harsh_check_<stamp>.json`

`benchmark_controller_variants.py`
- Compares `current`, `abstract`, `inverse`, `inverse_v2`, and `legacy_32b8be` on identical event windows.
- Reports per-variant `harsh_rate`, `leapfrog_rate`, and `avg_event_score` for side-by-side tradeoff checks.
- Default inverse tuning is calibrated on the 2026-02-14 engaged-stop replay corpus:
  `tau=0.92`, `max_ref_decel=1.25`, `hold_cap=-0.26`, `hold_speed=0.14`,
  `kp=0.10`, `ki=0.01`, `step_scale=0.9`, `brake_step_scale=0.70`, `release_step_scale=1.0`.
- `inverse_v2` defaults to baseline parity with `inverse`; enable additional low-speed heuristics with
  `--inverse-v2-extra-decel-scale > 0` and a deeper `--inverse-v2-risk-hold-cmd-cap`.
- Example:
  `python tools/stopping/benchmark_controller_variants.py --model-json ~/.comma/stopping_behavior/models/stopping_model_<stamp>.json`
  `--summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route1>/<stamp>/summary.json`
  `--summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route2>/<stamp>/summary.json`
  `--event-source all --controller-scope engaged_stopping --controller-window-mode stopping_state`
  `--controller-end-mode last_stopping_state --output-json ~/.comma/stopping_behavior/analysis/controller_variant_benchmark_<stamp>.json`

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
  --event-source all \
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
  --controller-scope engaged_stopping \
  --controller-min-enabled-ratio 0.80 \
  --min-events 6 \
  --max-pred-end-jerk 0.70 \
  --max-pred-rollout-m 2.0 \
  --max-harsh-rate 0.10 \
  --max-leapfrog-rate 0.20 \
  --output-json ~/.comma/stopping_behavior/analysis/model_harsh_check_controller_<stamp>.json
```

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

## Inverse Retraining + Improvement SOP (Each New Data Batch)

Use this flow every time new stopping logs are added and before changing inverse defaults.

### 1) Freeze Evaluation Inputs First

- Keep a repeatable evaluation slice with:
  - latest engaged-stop review summaries
  - pinned regression summaries (must include known hard cases like `000006fa`)
- Keep train and holdout distinct:
  - train: summaries used for model fit + inverse tuning
  - holdout: summaries never used in tuning sweep

### 2) Refit Plant Model(s)

Fit at least one all-events model; optionally fit an engaged-only model as a sensitivity check.

```bash
python tools/stopping/fit_stopping_model.py \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<routeA>/<stamp>/summary.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<routeB>/<stamp>/summary.json \
  --event-source all \
  --max-delay-frames 25 \
  --max-speed 1.8 \
  --min-rows 120 \
  --output ~/.comma/stopping_behavior/models/stopping_model_<stamp>_all.json
```

### 3) Record Baseline Benchmark (No Tuning Yet)

```bash
python tools/stopping/benchmark_controller_variants.py \
  --model-json ~/.comma/stopping_behavior/models/stopping_model_<stamp>_all.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<routeA>/<stamp>/summary.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<routeB>/<stamp>/summary.json \
  --event-source signal \
  --controller-scope engaged_stopping \
  --controller-window-mode stopping_state \
  --controller-end-mode last_stopping_state \
  --max-pred-end-jerk 0.70 \
  --min-pred-a-floor -1.10 \
  --max-pred-rollout-m 2.0 \
  --max-pred-speed-rebound-while-should-stop 0.08 \
  --max-pred-should-stop-unexpected-accel 0.10 \
  --output-json ~/.comma/stopping_behavior/analysis/controller_variant_benchmark_<stamp>_baseline.json
```

### 4) Tune Inverse v1, Then Re-Benchmark

Run coarse-to-fine sweeps with `tune_inverse_controller.py`, then verify with
`benchmark_controller_variants.py` on the same held-out summaries.

Tuning objective (current):
- `tune_inverse_controller.py` ranks candidates by `(harsh_events, leapfrog_events, avg_score)`.
- This prevents promoting candidates that look good on harsh-only metrics but rebound more.
- Keep leapfrog thresholds aligned with benchmark/gates via:
  - `--max-pred-speed-rebound-while-should-stop`
  - `--max-pred-should-stop-unexpected-accel`

Recommended promotion checks (holdout):
- `inverse.harsh_rate <= current.harsh_rate - 0.05`
- `inverse.leapfrog_rate <= current.leapfrog_rate`
- `inverse.avg_event_score < current.avg_event_score`
- `events_considered >= 20` (or document why lower count is acceptable)

### 5) Decide Whether `inverse_v2` Is Needed

- `inverse` is the primary maintained variant.
- `inverse_v2` is experimental and should stay default-parity unless it proves clear value.
- Keep/use `inverse_v2` only if it beats tuned `inverse` on holdout by either:
  - lower harsh + no leapfrog regression, or
  - equal harsh + lower leapfrog + lower score.
- If `inverse_v2` shows no wins for 3 refresh cycles, remove it to reduce maintenance burden.

Quick v2 probe example:
```bash
python tools/stopping/benchmark_controller_variants.py \
  --model-json ~/.comma/stopping_behavior/models/stopping_model_<stamp>_all.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<routeA>/<stamp>/summary.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<routeB>/<stamp>/summary.json \
  --event-source signal \
  --controller-scope engaged_stopping \
  --controller-window-mode stopping_state \
  --controller-end-mode last_stopping_state \
  --inverse-v2-extra-decel-scale 0.4 \
  --inverse-v2-risk-hold-cmd-cap -0.35 \
  --output-json ~/.comma/stopping_behavior/analysis/controller_variant_benchmark_<stamp>_v2probe.json
```

### 6) Log and Version Every Cycle

For each refresh cycle, append to `docs/stopping_behavior_worklog.md`:
- model artifact path and fit stats
- benchmark baseline vs tuned results
- tuned parameter set
- keep/drop decision for `inverse_v2`
- exact command lines used

### 7) Iteration Definition of Done

An iteration is complete only when all items below are true:

- baseline + candidate metrics were both captured on the same holdout slice
- focused stopping tests passed
- keep/reject decision is explicit
- worklog has dated entry with artifact paths and commands
- any stale or contradicted documentation touched by the change was updated

### Worklog Entry Template (Use Every Time)

```markdown
### YYYY-MM-DD: <short iteration title>

- Trigger:
  - new data batch / user suggestion / regression follow-up
- Inputs:
  - train summaries: <paths>
  - holdout summaries: <paths>
  - model: <model path>
- Baseline:
  - current: harsh=?, leapfrog=?, avg_score=?
  - inverse: harsh=?, leapfrog=?, avg_score=?
- Experiment:
  - change(s): <params/code>
  - success criteria: <thresholds>
- Result:
  - candidate: harsh=?, leapfrog=?, avg_score=?
  - delta vs baseline: <summary>
- Tests:
  - command: <pytest/other>
  - result: pass/fail
- Decision:
  - keep/reject
  - why
- Follow-up:
  - next experiment or cleanup action
```

## General Inverse Improvement Priorities

- Improve data quality before tuning:
  - quarantine or drop corrupted qlogs from fit/eval inputs
  - keep stop-event mode/scope fixed across comparisons
- Expand pinned regression set when new failure patterns appear.
- Prefer small parameter moves + re-check over large one-shot retunes.
- Track both harsh and leapfrog metrics; do not trade one blindly for the other.
