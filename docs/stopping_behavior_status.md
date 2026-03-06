# Stopping Behavior Project: Status and Direction

- Updated: 2026-03-06
- Scope: OpenPilot/FrogPilot longitudinal stopping behavior (stop execution, not stop decision timing)
- Worklog (evidence, commands, artifacts): `docs/stopping_behavior_worklog.md`
- Tooling workflow (how to run cycles): `tools/stopping/README.md`
- Improvement cycle (process definition): `tools/stopping/README.md`

## Problem Statement

Stopping quality is still inconsistent near standstill across real routes, especially on vehicles/gearboxes that exhibit low-speed push/release disturbances. The two main user-visible failure modes:

- **Harsh end-stop jerk** at the wheel-stop transition.
- **Rebound / “leapfrog”** behavior: “almost stop -> slight re-accel/roll -> stop again”, or creep while `shouldStop` remains true.

The project goal is to reduce both without materially increasing rollout distance.

## Scope and Constraints

In scope:

- Longitudinal stop **execution** while OpenPilot is engaged (`shouldStop` true, `longControlState` stopping).
- Controller-side behavior: unwind, step limits, disturbance lock, anti-rebound and rollout management.
- Offline evaluation tooling and regression gates for repeatable iteration.

Out of scope / treated as inputs:

- Planner/model “stop decision timing” (late/early stop intent transitions). We measure it, but we don’t try to redesign it here.
- Manual driver stopping (useful context, not an acceptance target).

Acceptance constraints:

- Wheel-stop should feel smooth (minimal perceived jerk/force spike).
- Avoid increasing stopping distance: rollout budget target `<= 2.0m` over the low-speed stop window.

## Current Runtime Implementation (On-Device)

Runtime source of truth:

- `selfdrive/controls/lib/longcontrol.py`
  - Single stop-controller path in the `LongCtrlState.stopping` branch.
  - Uses stop-speed breakpoints and expected accel bounds as inputs to the stop controller.
- `selfdrive/controls/lib/stopping_controller.py`
  - Stateful low-speed stop controller with explicit phases:
    - `APPROACH` (higher low-speed),
    - `NEAR_HOLD`,
    - `HOLD`.
  - Key mechanisms:
    - disturbance / clutch push detection with release-lock,
    - low-speed rollout accounting,
    - anti-rebound arrest windows,
    - dt-aware step sizing (important for offline replay parity).
- `selfdrive/controls/lib/stopping_guard.py`
  - Low-speed slew limiter used outside the explicit stopping phase (prevents abrupt command moves when transitioning out of stop intent).

Planner context that affects “how late” the stop begins:

- `selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py` (e.g., `STOP_DISTANCE`)

## Offline Tooling and Regression Gates (Local)

- Operational docs + command lines live in `tools/stopping/README.md`.
- Primary entrypoint is `tools/stopping/run_stopping_cycle.py` (snapshot + sync + optional analysis/model-fit/gates + worklog append).
- Frozen holdout routes for gates live in `tools/stopping/holdout_routes.txt` (use `--gate-route-file`).

## Parallel Solutions (What We’re Testing in Parallel)

Parallel policy exploration happens in `tools/stopping/benchmark_controller_variants.py`:

- Active decision lanes:
- `current`: runtime source of truth and only shippable controller.
- `inverse_v3`: offline idea-source only (used to extract targeted logic into runtime).
- `legacy_32b8be`: sanity baseline for large-regression detection.

Guiding rule: only the runtime controller ships; benchmark scope is intentionally kept to these three variants to reduce decision noise.
Legacy `abstract` / `inverse` / `inverse_v2` benchmark+tuner code paths have been removed from active tooling.

## Where We Are Now (Snapshot)

- Runtime stop execution is consolidated into one controller path (`StoppingController`).
- Offline tooling is mature enough to run “freeze/benchmark/promote” cycles:
  - measured gates (`check_harsh_stops.py`),
  - model-based controller replay gates (`check_harsh_stops_model.py`),
  - variant comparisons (`benchmark_controller_variants.py`),
  - inverse tuning sweeps (`tune_inverse_controller.py`),
  - leapfrog alignment (`check_leapfrog_alignment.py`).
- Latest fitted model artifact (local): `~/.comma/stopping_behavior/models/stopping_model_20260306T182846Z_all.json`
- Latest full cycle stamp: `20260306T182846Z` (see `docs/stopping_behavior_worklog.md` for evidence and artifacts)
- Latest holdout replay baseline (controller, recorded shouldStop): `harsh=8/15` (`0.533`), `leapfrog=0/15`.
- Latest measured holdout gate remains fail (`11/11` harsh on frozen historical logs), so current tuning decisions are driven by replay/model gate plus fresh-route on-device validation.
- Device policy: always deploy branch `!my-fp` (see deploy workflow in `tools/stopping/README.md`)
- Current benchmark snapshot (`0000071c--fb4cca0034`): cycle baseline `current=5/24` harsh (`0.208`, avg score `0.570`, leapfrog `1/24`); current kept candidate improves to `4/24` (`0.167`, avg score `0.559`, leapfrog `1/24`).
- Restored two-route holdout (`0000071c` + `00000721`): same-model baseline (before soft-landing widen) is `7/29` harsh (`0.241`) with leapfrog `1/29`; kept runtime candidate is `6/29` harsh (`0.207`) with leapfrog unchanged `1/29`.
- Deterministic replay regression seeds are now in-tree for persistent harsh holdout events (`0000071c` events `14/15/19`, `00000721` event `4`) in `selfdrive/controls/lib/tests/test_stopping_controller.py`.
- Latest-model strict failing targets are now in-tree for remaining harsh events (`0000071c` events `2/14/15/19`) to drive next tuning iterations.
- Current focus: reduce high-rollout final-stop jerk and remaining accel-step events while keeping leapfrog at 0 and avoiding regressions in low-rollout stops.
- Secondary focus: eliminate stop-intent dropouts that allow a rapid low-speed “resume” (driver intervention/disengage class).
- 2026-03-06 latest cycle outcome (cleaned benchmark lanes): with model `stopping_model_20260306T200122Z_all.json` on holdout `0000071c`, baseline runtime is `5/24` harsh and `1/24` leapfrog. A runtime-only `soft_landing_release` window widen (`v_ego<1.05` with smoother high-speed soft target/steps) improves runtime to `4/24` harsh with leapfrog unchanged (`1/24`). A follow-up “slower release slew” variant regressed avg score and was rejected.
- 2026-03-06 offline refit sanity check (no controller code change): widening fit inputs to 12 recent summaries produced a much more stable gate result on the same holdout (`model replay 3/15`, benchmark `1/10`, leapfrog `0`). Process implication: treat narrow-fit model swings as process noise; default to wider fit windows when selecting the next runtime tuning target.
- Remaining work is mostly *quality and maintainability*:
  - stop-controller tuning still needs iterations on fresh routes,
  - the runtime controller has grown a large number of narrow guards (harder to reason about),
  - inverse variants show promise in replay, but require careful translation into runtime-safe logic.

## Risks and Tech Debt (Why Cleanup/Refactor Is Timely)

- `StoppingController` is now a large rule stack; guard ordering matters and is easy to regress.
- Many thresholds are “magic numbers” spread across the update path, making review and systematic tuning hard.
- Offline controllers (`inverse*`) and runtime are intentionally different; without an abstraction boundary, idea transfer is manual and error-prone.
- Documentation is split between:
  - a detailed, chronological worklog (`docs/stopping_behavior_worklog.md`),
  - an operational playbook (`tools/stopping/README.md`),
  - but previously lacked a stable “status + direction” snapshot (this file).

## Heading: Plan for the Next Phase

### Phase A: Documentation and Hygiene (short)

- Keep `docs/stopping_behavior_worklog.md` chronological only (evidence and commands).
- Keep this file as the living snapshot (current architecture + plan).
- Pin and name “frozen” evaluation slices and the current model artifacts in the worklog entry template.

### Phase B: Refactor for Maintainability (no behavior change first)

Goal: make `StoppingController` reviewable and parameterizable without changing behavior.

- Extract tuning curves and thresholds into a `dataclass` (or module-level structure) with names that map to failure modes.
- Group the rule stack into explicit stages (base target, disturbance lock, rollout management, anti-rebound, comfort release).
- Add a small “trigger summary” structure for offline replay so we can see which guards fired on harsh/leapfrog events (no runtime log spam).
- Remove redundant computations and dead locals once the refactor makes them visible.

### Phase C: Fewer Parallel Variants, Cleaner Promotion Rules

- Keep active decision lanes narrow:
- `current` for shipping decisions.
- `inverse_v3` as the single inverse idea-source.
- `legacy_32b8be` for sanity checks.
- Define a single “promotion gate contract” for any runtime change:
  - no regressions on frozen holdout,
  - no regressions on pinned harsh/leapfrog routes,
  - rollout within budget,
  - leapfrog does not worsen when harsh improves.

### Phase D: Try Something New (only if Phase B/C stalls)

If rule-stack tuning stops moving the needle:

- Consider a small, runtime-safe “inverse-inspired” submodule:
  - keep the runtime phase machine,
  - replace parts of target selection with a model-informed reference (`a_ref`) and a small PI term,
  - preserve all step limiting and lock semantics.

This keeps the benefits of inverse tuning without committing to a full model-inversion policy on-device.

## Definition of Done (for a Meaningful “Stopping Improvement”)

A change is considered a “good stopping improvement” only if:

- It improves harsh metrics and does not regress leapfrog on the same evaluation slice(s).
- It stays within rollout budget (`<= 2.0m` in low-speed stop window).
- It passes both measured and model-based offline gates on the frozen holdout set.
- It is documented in the worklog with commands and artifact paths.
