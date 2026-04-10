# Stopping Workflow Tooling

Scripts in this folder support the stopping-behavior workflow after the shared route refresh step has populated the local route cache.

## Operating Contract (Mandatory)

- North-star goal: **always stop perfectly** (no noticeable final jerk, no rebound/leapfrog, stable hold, controlled rollout).
- Runtime source of truth:
  - `selfdrive/controls/lib/stopping_controller.py`
  - `selfdrive/controls/lib/longcontrol.py`
- Process source of truth:
  - `docs/route_refresh_process.md` (shared route refresh contract used by all route-driven processes)
  - `tools/stopping/README.md` (how to run/tune/decide)
  - `docs/stopping_behavior_status.md` (where we are and where we’re heading)
  - `docs/stopping_behavior_worklog.md` (what happened, with dates and evidence)

## Improvement Cycle (One Iteration)

This is the canonical loop for stopping improvements. The worklog records evidence; this section defines what “an iteration” is.

1. Intake new data (post-drive).
   - Refresh the shared route cache first:
     `python tools/route_sync/refresh_routes.py --host commawifi --max-downloads 80 --newest-first`
   - Or run the stamped stopping cycle, which snapshots settings and then runs the shared route refresh:
     `python tools/stopping/run_stopping_cycle.py --host commawifi --max-downloads 80 --newest-first`
   - Optional (slower): add `--include-rlog` when you need rlog-only signals.
2. Refresh corpus + triage (weekly or when behavior shifts).
   - Build an engaged-stop corpus (`speed_transition` + enabled).
   - Rank failures and generate graph packs for top routes.
3. Freeze evaluation inputs before tuning.
   - Maintain two sets:
     - train summaries (for model fit)
     - holdout summaries (for gates + variant benchmark)
   - Pin “hard routes” (harsh + leapfrog) in holdout.
   - Recommended: keep a pinned route list at `tools/stopping/holdout_routes.txt` and pass it to the cycle runner:
     `--gate-route-file tools/stopping/holdout_routes.txt`
     This also prevents accidentally fitting on the holdout when fit summaries are auto-discovered.
   - If pinned holdout discovery fails, fix the path/plumbing before trusting the cycle output. A manual pinned-summary fallback is acceptable only as a documented exception in the worklog.
   - Do not treat the newest downloaded routes as proof of improvement by themselves. New routes are for post-promote validation, or they become future train/holdout inputs only after a new split is frozen.
4. Baseline on holdout (no code changes).
   - Measured: `check_harsh_stops.py` on holdout summaries.
   - Measured comfort lane: on fresh enabled `hybrid` review summaries with real brake command, run `check_harsh_stops.py` with:
     - `--min-should-stop-ratio`
     - `--require-brake-command-below`
     - entry-side harshness thresholds (`--max-entry-stop-jerk`, `--max-entry-stop-accel-step`)
     - mini-leapfrog/dropout flags (`--count-stop-signal-drop-as-leapfrog`, `--count-exit-stop-as-leapfrog`)
   - Replay: `check_harsh_stops_model.py --command-source controller` on the same holdout.
   - Compare variants: use `benchmark_controller_variants.py`, but keep decisions focused on active lanes:
     - `current` (shipping lane),
     - `horizon_v1` (offline sequence-optimizer probe),
     - `legacy_32b8be` (sanity baseline).
5. Turn new failures into regressions.
   - Add a unit seed when a new on-road failure mode is identified:
     `selfdrive/controls/lib/tests/test_stopping_controller.py`
   - Add/refresh pinned holdout routes and record in the worklog entry.
6. Refresh the stop-response model (as needed).
   - Fit with `fit_stopping_model.py` on the train summaries.
   - Record model path + fit stats; keep the previous model for comparison.
   - Refit when the train corpus changes or replay trust/alignment is in doubt; do not churn the model between baseline and candidate comparisons on the same frozen slice.
7. Choose what to improve.
   - If the failure shows up in measured events and in `current` controller replay: improve runtime (`StoppingController`).
   - If measured failures exist but replay misses them: fix replay windows/model/alignment before tuning the runtime controller.
   - If `horizon_v1` is better in replay for a specific event class: extract that behavior into runtime-safe logic (or use it as the next controller-shaping target). Do not ship the offline optimizer directly.
8. Run one scoped change.
   - Change one thing at a time (runtime heuristic, gate threshold, model-fit constraint, replay window semantics).
   - Define success criteria up front (harsh improves, leapfrog does not regress, and stop distance stays within contract: no-lead rollout budget or lead-follow final-gap band).
9. Validate and decide.
   - Rerun measured + replay gates on the frozen holdout.
   - Rerun the measured comfort lane on the newest real stop-go routes before calling the iteration good.
   - Update the worklog with commands, artifacts, before/after metrics, and keep/reject decision.
   - If promoted, deploy and collect a validation route.

If a step is skipped, the iteration is incomplete.

## Current Operational Exception (2026-03-07)

The target promotion contract is still “measured + model frozen-holdout both pass,” but current practice is slightly narrower:

- The measured frozen historical holdout is still recorded every cycle, but it is currently used as a monitoring signal rather than the sole blocker.
- Runtime promotion currently requires:
  - no replay/model regressions on frozen holdout,
  - no regressions on pinned harsh/leapfrog benchmark routes,
  - stop distance within budget (`<= 2.0m` rollout for no-lead, `2.0-4.0m` final hold gap for lead-follow),
  - fresh-route on-device validation after deploy.
- Any manual holdout-resolution fallback must be called out explicitly in `docs/stopping_behavior_worklog.md`.
- This exception is temporary process debt; once the measured holdout and route resolution are refreshed, return to the full contract.

## Scripts

- SSH host defaults:
  - Remote scripts default to `--host commawifi` and fall back to `comma` automatically when `commawifi` is unreachable.

- Shared route refresh:
  - `tools/route_sync/refresh_routes.py` owns route discovery, download, shared refresh state, and JSON refresh reports.
  - `docs/route_refresh_process.md` defines the shared contract and when consumers should refresh.

- `append_sync_report.py`
  - Reads a shared route-refresh JSON report.
  - Appends a structured session entry to `docs/stopping_behavior_worklog.md` (or another markdown file).

- `device_stop_settings.py`
  - Reads current stop-related settings from device before analysis/tuning.
  - Can apply fine-tuning values with range checks.
  - Writes JSON snapshots/results for traceability.

- `run_stopping_cycle.py`
  - Wrapper that runs `device_stop_settings.py snapshot`, the shared route refresh, then `append_sync_report.py`.
  - Optional integrated analysis mode can run stop-event extraction and append analysis metrics.
  - Optional gate/model/benchmark stages can run and append a single stamped cycle summary to the worklog (model fit, measured gate, model gate, leapfrog alignment, variant benchmark).
  - Generates timestamp-matched settings/report files.
  - If launched under a Python interpreter missing repo deps, it re-execs via a dependency-ready `python` when available and otherwise fails with an explicit environment error.
  - Useful for repeatable "one command per drive" collection cycles.

- `analyze_stopping_behavior.py`
  - Reads local qlogs directly and detects stop events.
  - Generates per-event interactive HTML plots and `summary.md`/`summary.json`.
  - `summary.json` includes per-event engagement ratios (`enabled_ratio`, `stop_signal_ratio`, etc.) to help filter for true engaged stopping.
  - Skips live segment directories that still contain `*.lock` files, so freshly copied tails are treated as provisional until they close cleanly.
  - Designed for before/after stop behavior review.

- `append_analysis_report.py`
  - Appends analysis summary metrics into `docs/stopping_behavior_worklog.md`.

- `append_cycle_report.py`
  - Appends a stamped "cycle results" section to the worklog, summarizing model fit and gate/benchmark outputs.

- `compare_stopping_runs.py`
  - Compares two analysis `summary.json` files (before/after) and reports metric deltas.

- `check_harsh_stops.py`
  - Runs an offline pass/fail gate on stop-event summaries for harsh-stop symptoms.
  - Uses thresholds on end-stop jerk, command jerk, accel step, and minimum observed accel.
  - Supports filtering to engaged stopping with `--min-enabled-ratio` and `--min-stop-signal-ratio` (recommended for controller work).
  - Also supports a measured comfort lane for messy stop-go review:
    - filter on `shouldStop` / `stopping` presence (`--min-should-stop-ratio`, `--min-stopping-state-ratio`)
    - require real brake command (`--require-brake-command-below`)
    - gate stop-entry harshness (`--max-entry-stop-jerk`, `--max-entry-stop-cmd-jerk`, `--max-entry-stop-accel-step`)
    - count stop-signal/state dropout as mini leapfrog (`--count-stop-signal-drop-as-leapfrog`, `--count-exit-stop-as-leapfrog`)
  - Exits non-zero on regression so it can be used in repeatable local checks.
  - Output JSON now includes full `harsh_event_keys` and `leapfrog_event_keys` (not only truncated examples) for downstream alignment tooling.

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

- `check_leapfrog_alignment.py`
  - Compares measured leapfrog events (`check_harsh_stops.py`) vs predicted leapfrog events (`check_harsh_stops_model.py`).
  - Reports exact overlap, measured-only/predicted-only mismatches, and optional near-matches (`event_id` tolerance).
  - Supports optional fail thresholds (`--min-overlap-recall`, `--max-count-delta`) for stricter cycle gating.

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
  - Scans downloaded qlogs for `userBookmark` markers (with `userFlag` legacy fallback) and matches each bookmark to the nearest stop event.
  - Produces bookmark triage reports (`summary.json`, `summary.md`) with event metrics and route commands.
  - Intended for "bookmark bad stop during drive, analyze later" workflow.

## Default Local Paths

- Route-sync root: `~/.route_sync`
- Download root: `~/.route_sync`
- State file: `~/.route_sync/state.json`
- Report dir: `~/.route_sync/reports`
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

3. Refresh shared route logs from a reachable host alias:

```bash
python tools/route_sync/refresh_routes.py --host comma --include-rlog --newest-first
```

4. Append the generated report to the stopping worklog:

```bash
python tools/stopping/append_sync_report.py \
  --report-file ~/.route_sync/reports/<report>.json \
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

## Device Update / Deploy Workflow

`fullupdate.sh` only installs commits that exist in the device's Git remote. If you changed code locally, you must `git commit` + `git push` before the device can pick it up.
Device policy for this project: keep the device on branch `!my-fp`.

`fullupdate.sh` decides what to fetch like this:
- If the current branch has an upstream (`@{u}`), it fetches that remote branch.
- Otherwise, it fetches `origin` `refs/heads/<current_branch>`.

1. Check what branch the device will update from (and its current commit + upstream):

```bash
# Prefer commawifi; use comma if commawifi is unreachable.
ssh -o BatchMode=yes -o ConnectTimeout=8 commawifi 'cd /data/openpilot && echo BRANCH=$(git branch --show-current) && echo HEAD=$(git rev-parse --short HEAD) && echo UPSTREAM=$(git rev-parse --abbrev-ref --symbolic-full-name @{u} 2>/dev/null || echo none)'
```

2. Make sure the device is on the branch you intend to deploy.

```bash
ssh -tt commawifi "cd /data/openpilot && git checkout '!my-fp'"
```

If `@{u}` points somewhere unexpected and you want `fullupdate.sh` to fetch by branch name, unset upstream:

```bash
ssh -tt commawifi 'cd /data/openpilot && git branch --unset-upstream'
```

3. Push your local commit to the branch the device will fetch.

```bash
# zsh note: branch names like !my-fp require quotes to avoid history expansion.
git push origin HEAD:'!my-fp'
```

4. Trigger the update (SSH may close during reboot; this is expected):

```bash
ssh -tt commawifi 'cd /data/openpilot && ./fullupdate.sh'
```

5. Verify the device is on the new commit after it comes back:

```bash
ssh -o BatchMode=yes -o ConnectTimeout=8 commawifi 'cd /data/openpilot && git branch --show-current && git rev-parse --short HEAD'
```

Troubleshooting:
- If `fullupdate.sh` keeps resetting to an older commit, the usual cause is that the remote branch tip hasn't moved (push didn't happen), or the device is updating a different branch than you think (check `BRANCH` and `UPSTREAM`).

## Useful Options

`refresh_routes.py`
- `--host commawifi`
- `--dry-run` (discover and compare only)
- `--max-downloads N` (throttle transfer)
- `--newest-first` (pull latest files first when using `--max-downloads`)
- `--state-file ~/.route_sync/state.json`
- `--remote-root /custom/path` (repeatable)
  - Default shared refresh scans the active device roots under `/data/media/0` and normalizes them into one local cache path.
  - Use `--remote-root /data/media/0/realdata` only when you intentionally want to narrow discovery.
- `--file-name qlog.zst` (repeatable)
- `--verbose`

`append_sync_report.py`
- `--worklog docs/stopping_behavior_worklog.md`
- `--title "Route refresh from comma"`
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
- Cycle summaries now record the local repo branch and commit used for the run.
- `--state-file ~/.route_sync/state.json`
- `--include-rlog`
- `--skip-settings` (if settings already captured for this run)
- `--settings-dry-run` (validate/read requested stop-tune values without writing)
- `--dry-run-sync` (discover only, still writes report and can append markdown)
- `--analyze --analysis-event-mode hybrid --analysis-min-entry-speed 0.5` (broad stop coverage)
- `--analyze --analysis-event-mode engaged_signal --analysis-min-entry-speed 2.0` (strict OP stop-signal events)
- `--analysis-route <route_id>` (pin analysis to a specific route)
- `--fit-model --fit-event-source all --fit-recent-summaries 12` (rebuild model from all stopping events; default widened to reduce overfit)
- Robust delay selection passthrough:
  - `--fit-delay-min-sample-ratio 0.40`
  - `--fit-delay-rmse-tolerance 0.03`
- Gates and comparisons:
  - `--run-measured-gate` (measured harsh/leapfrog gate on the same fit summaries)
  - `--run-model-gate` (model harsh/leapfrog gate; requires `--fit-model`)
  - `--run-leapfrog-alignment` (measured vs predicted leapfrog overlap; requires `--run-model-gate`)
  - `--run-variant-benchmark` (compare `current` vs `inverse*` variants on a chosen holdout summary; requires `--fit-model`)
  - `--fit-delay-rmse-tolerance 0.03`
- `--run-model-gate --model-gate-command-source controller` (run offline controller gate on engaged+stopping scope)
- Controller replay `shouldStop` semantics in cycle model-gate default to recorded values:
  - `--model-gate-controller-should-stop-source recorded`
  - use `constant_true` only for legacy comparison/debugging.
- Baseline gate target in cycle defaults: `--model-gate-max-harsh-rate 0.50`
  (use stricter `0.10` as a stretch target while tuning)
- `--run-leapfrog-alignment` (runs measured check + predicted overlap report after model gate)
- Model-gate thresholds now exposed in-cycle:
  - `--model-gate-controller-should-stop-source recorded`
  - `--model-gate-min-entry-speed 0.20`
  - `--model-gate-max-leapfrog-rate 1.0` (set `< 1.0` to enforce leapfrog gate)
  - `--model-gate-max-leapfrog-count 0` (set `> 0` to enforce count gate)
  - `--model-gate-max-pred-end-cmd-jerk 3.0`
  - `--model-gate-max-pred-end-accel-step 0.08`
  - `--model-gate-max-pred-speed-rebound-while-should-stop 0.08`
  - `--model-gate-max-pred-should-stop-unexpected-accel 0.10`
- Alignment controls:
  - `--alignment-event-id-tolerance 1` (diagnostic near-match window)
  - `--alignment-min-overlap-recall 0.0` (set `> 0` to fail low overlap)
  - `--alignment-max-count-delta -1` (set `>= 0` to fail count mismatches)
  - `--alignment-min-enabled-ratio` (defaults to model-gate controller enabled ratio)
  - `--alignment-output ~/.comma/stopping_behavior/analysis/leapfrog_alignment_<stamp>.json`
- Full one-shot cycle:
  `python tools/stopping/run_stopping_cycle.py --host commawifi --analyze --analysis-event-mode speed_transition`
  `--analysis-min-entry-speed 0.0 --fit-model --fit-event-source all`
  `--fit-recent-summaries 12 --run-model-gate --run-leapfrog-alignment`

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
- Output JSON includes full `harsh_event_keys` / `leapfrog_event_keys` for exact event-set comparison across runs.
- `--output-json ~/.comma/stopping_behavior/analysis/<stamp>_harsh_check.json`
- Comfort-lane example for enabled stop-go routes with real braking:
  ```bash
  python tools/stopping/check_harsh_stops.py \
    --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<route>/<stamp>/summary.json \
    --event-source hybrid \
    --min-events 1 \
    --min-entry-speed 0.20 \
    --min-should-stop-ratio 0.15 \
    --require-brake-command-below -0.10 \
    --max-entry-stop-jerk 0.75 \
    --max-entry-stop-accel-step 0.10 \
    --max-end-stop-jerk 0.90 \
    --max-end-stop-accel-step 0.10 \
    --count-stop-signal-drop-as-leapfrog \
    --count-exit-stop-as-leapfrog
  ```
  Use this lane for fresh-route comfort review even when the stricter enabled `speed_transition` gate finds no clean controller seeds.

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
  - `--controller-should-stop-source recorded` (recommended; use recorded shouldStop instead of forcing true)
  - `stopping_state` / `should_stop` replay windows use the last contiguous active span near hold (not first-to-last sparse spans),
    which avoids false rollout/leapfrog inflation on long hybrid events with sparse stop flags.
  - In `recorded` mode, replay clamps positive standstill drift only after sustained true-zero standstill while command remains braking,
    to reduce synthetic creep/leapfrog artifacts from the first-order model.
  - Override with `event` / `should_stop` / `hold` options for debugging.
- `--event-source speed --min-events 6 --min-entry-speed 0.20`
- Baseline tuning target: `--max-harsh-rate 0.50`
- Stretch target: `--max-harsh-rate 0.10`
- Optional leapfrog gate: `--max-leapfrog-rate 0.20` (kept separate from harsh gate)
- `--max-pred-end-jerk 0.80 --min-pred-a-floor -1.10 --max-pred-rollout-m 2.0`
- `--max-pred-end-cmd-jerk 3.0 --max-pred-end-accel-step 0.08` (predicted low-speed command/accel sharpness)
- `--max-pred-speed-rebound-while-should-stop 0.08 --max-pred-should-stop-unexpected-accel 0.10`
- `--stopping-speed-breakpoint 0.40 --stop-accel -2.0` (controller replay mode)
- Output includes both `harsh_events`/`harsh_rate` and `leapfrog_events`/`leapfrog_rate`
- Output JSON includes per-event replay rows (`event_rows`) with `is_leapfrog` and event keys.
- `--output-json ~/.comma/stopping_behavior/analysis/model_harsh_check_<stamp>.json`

`check_leapfrog_alignment.py`
- `--measured-json ~/.comma/stopping_behavior/analysis/measured_harsh_gate_<stamp>.json`
- `--predicted-json ~/.comma/stopping_behavior/analysis/model_harsh_check_<stamp>.json`
- `--event-id-tolerance 1`
- Optional strictness:
  - `--min-overlap-recall 0.5`
  - `--max-count-delta 2`
- `--output-json ~/.comma/stopping_behavior/analysis/leapfrog_alignment_<stamp>.json`

`benchmark_controller_variants.py`
- Compares `current`, `horizon_v1`, and `legacy_32b8be` on identical event windows.
- Active decision lanes:
  - `current` for shipping decisions.
  - `horizon_v1` for offline sequence optimization against the fitted plant.
  - `legacy_32b8be` for sanity checks.
- Reports per-variant `harsh_rate`, `leapfrog_rate`, and `avg_event_score` for side-by-side tradeoff checks.
- Harsh classification includes predicted `end_stop_jerk`, `end_stop_cmd_jerk`, and `end_stop_accel_step` plus floor and stop-distance guards.
- Stop-distance guard is split by context: no-lead events use rollout, lead-follow events use final `LeadHold` gap (`2.0-4.0m`, with `~3.0m` preferred for scoring).
- Uses the same contiguous-span replay window semantics as `check_harsh_stops_model.py` for
  `--controller-window-mode should_stop|stopping_state`, so benchmark and model-gate results are directly comparable.
- `horizon_v1` is a deterministic tail-sequence search on top of the current replay command trace:
  - default horizon: `1.2s`
  - control block: `0.10s`
  - beam width: `24`
  - residual grid: `{-0.12, -0.06, 0.0, +0.06, +0.12}` m/s²
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

## Horizon Optimizer + Improvement SOP (Each New Data Batch)

Use this flow every time new stopping logs are added and before changing runtime stop behavior.

### 1) Freeze Evaluation Inputs First

- Keep a repeatable evaluation slice with:
  - latest engaged-stop review summaries
  - pinned regression summaries (must include known hard cases like `000006fa`)
- Keep train and holdout distinct:
  - train: summaries used for plant fit and runtime/horizon iteration
  - holdout: summaries never used to pick the keep/reject decision

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

Notes:
- Refit the plant model whenever the train corpus changes materially.
- Keep baseline and candidate comparisons on the same frozen model.
- The fitted plant is for replay and sequence optimization, not a shippable controller by itself.

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

### 4) Compare `horizon_v1`, Then Re-Benchmark

Use `benchmark_controller_variants.py` on the same held-out summaries.

`horizon_v1` is a deterministic tail-sequence search on top of the current replay command trace:
- default horizon: `1.2s`
- control block: `0.10s`
- beam width: `24`
- residual grid: `{-0.12, -0.06, 0.0, +0.06, +0.12}` m/s²

Recommended keep checks (holdout):
- `horizon_v1.harsh_rate <= current.harsh_rate`
- `horizon_v1.leapfrog_rate <= current.leapfrog_rate`
- `horizon_v1.avg_event_score < current.avg_event_score`
- `events_considered >= 20` (or document why lower count is acceptable)

If `horizon_v1` wins:
- inspect the winning events
- extract the sequence-shaping idea into runtime-safe `LongControl` / `StoppingController` logic
- do not ship the offline optimizer directly

### 5) Keep Variant Scope Narrow

- Runtime controller (`current`) is the only shipping lane.
- `horizon_v1` is the only actively maintained offline optimizer lane.
- `legacy_32b8be` is retained as a regression sanity baseline.

Quick focused `horizon_v1` probe example:
```bash
python tools/stopping/benchmark_controller_variants.py \
  --download-root ~/.route_sync \
  --model-json ~/.comma/stopping_behavior/models/stopping_model_<stamp>_all.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<routeA>/<stamp>/summary.json \
  --summary-json ~/.comma/stopping_behavior/analysis/commawifi/<routeB>/<stamp>/summary.json \
  --event-source signal \
  --controller-scope engaged_stopping \
  --controller-window-mode stopping_state \
  --controller-end-mode last_stopping_state \
  --horizon-v1-horizon-s 1.2 \
  --horizon-v1-block-s 0.10 \
  --horizon-v1-beam-width 24 \
  --horizon-v1-residual-grid=-0.12,-0.06,0.0,0.06,0.12 \
  --output-json ~/.comma/stopping_behavior/analysis/controller_variant_benchmark_<stamp>_horizonprobe.json
```

### 6) Log and Version Every Cycle

For each refresh cycle, append to `docs/stopping_behavior_worklog.md`:
- model artifact path and fit stats
- benchmark baseline vs tuned results
- tuned parameter set
- keep/drop decision for the active candidate
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
- If runtime tuning starts accumulating many narrow guards with weak generalization, step back and test broader stop-profile families on the same frozen slice instead of stacking more special cases.
- Track both harsh and leapfrog metrics; do not trade one blindly for the other.
