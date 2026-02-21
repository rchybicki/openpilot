# Stopping Behavior Project: Status and Direction

- Updated: 2026-02-21
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

- Runtime baseline: `current` (the actual `StoppingController` behavior replayed offline).
- Offline-only baselines: `abstract`, `legacy_32b8be`.
- Offline-only tunables: `inverse`, `inverse_v2`.

Guiding rule: only the runtime controller ships; the other variants exist to measure tradeoffs and extract ideas safely.

## Where We Are Now (Snapshot)

- Runtime stop execution is consolidated into one controller path (`StoppingController`).
- Offline tooling is mature enough to run “freeze/benchmark/promote” cycles:
  - measured gates (`check_harsh_stops.py`),
  - model-based controller replay gates (`check_harsh_stops_model.py`),
  - variant comparisons (`benchmark_controller_variants.py`),
  - inverse tuning sweeps (`tune_inverse_controller.py`),
  - leapfrog alignment (`check_leapfrog_alignment.py`).
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

- Keep `abstract` and `legacy_32b8be` as offline baselines.
- Maintain `inverse` as the primary tuning probe.
- Keep `inverse_v2` only if it continues to beat `inverse` on rebound/leapfrog without hurting harshness on frozen slices.
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
