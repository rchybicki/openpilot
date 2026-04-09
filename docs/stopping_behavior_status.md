# Stopping Behavior Project: Status and Direction

- Updated: 2026-04-09
- Scope: OpenPilot/FrogPilot longitudinal stopping behavior (stop execution, not stop decision timing)
- Worklog (evidence, commands, artifacts): `docs/stopping_behavior_worklog.md`
- Shared route refresh contract: `docs/route_refresh_process.md`
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
- Avoid increasing stopping distance:
  - no-lead stops: rollout budget target `<= 2.0m` over the low-speed stop window
  - lead-follow stops: final hold gap target `2.0-4.0m`, with `~3.0m` preferred inside that band
- Fresh stop-go review should not regress on measured comfort:
  - entry bite (`EntryJerk` / `EntryStep`) should improve or stay flat
  - mini leapfrog / dropout (`SigDrop`, `ExitStop`) should improve or stay flat on enabled events with real brake command

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

- Shared route intake lives in `docs/route_refresh_process.md` and `tools/route_sync/refresh_routes.py`.
- Operational docs + command lines live in `tools/stopping/README.md`.
- Primary stopping entrypoint is `tools/stopping/run_stopping_cycle.py` (snapshot + route refresh + optional analysis/model-fit/gates + worklog append).
- Frozen holdout routes for gates live in `tools/stopping/holdout_routes.txt` (use `--gate-route-file`).
- Measured review now has two lanes:
  - clean deterministic lane: enabled `speed_transition` / holdout summaries for controller regression gates
  - comfort lane: enabled `hybrid` fresh-route summaries with real brake command, using `check_harsh_stops.py` entry-jerk filters plus `SigDrop` / `ExitStop` as mini-leapfrog

## Parallel Solutions (What We’re Testing in Parallel)

Parallel policy exploration happens in `tools/stopping/benchmark_controller_variants.py`:

- Active decision lanes:
- `current`: runtime source of truth and only shippable controller.
- `horizon_v1`: offline sequence-optimizer probe built on the fitted plant and current replay trace.
- `legacy_32b8be`: sanity baseline for large-regression detection.

Guiding rule: only the runtime controller ships; benchmark scope is intentionally kept to these three variants to reduce decision noise.
Legacy `abstract` / `inverse` / `inverse_v2` lanes are gone from active tooling, and `inverse_v3` is no longer a maintained decision lane.

## Where We Are Now (Snapshot)

- Runtime stop execution is consolidated into one controller path (`StoppingController`).
- Offline tooling is mature enough to run “freeze/benchmark/promote” cycles:
  - measured gates (`check_harsh_stops.py`),
  - model-based controller replay gates (`check_harsh_stops_model.py`),
  - variant comparisons (`benchmark_controller_variants.py`),
  - horizon-sequence comparison (`horizon_v1` inside `benchmark_controller_variants.py`),
  - leapfrog alignment (`check_leapfrog_alignment.py`).
- Latest fitted model artifact (local): `~/.comma/stopping_behavior/models/stopping_model_20260324T200003Z_all_manual.json`
- Latest variant benchmark cycle: `2026-04-09 horizon_v1 prototype` (see `docs/stopping_behavior_worklog.md` for commands and JSON artifacts)
- Latest maintained holdout benchmark (`0000071c` + `00000721`, 29 events):
  - `current`: `0/29` harsh, `0/29` leapfrog, avg score `0.279`
  - `horizon_v1`: `0/29` harsh, `0/29` leapfrog, avg score `0.196`
- Latest measured holdout gate remains fail (`11/11` harsh on frozen historical logs), so current tuning decisions are driven by replay/model gate plus fresh-route on-device validation.
- Device policy for the current line: deploy branch `!my-fp-new` (see deploy workflow in `tools/stopping/README.md`)
- Latest recent clean-slice benchmark (`00000815` + `00000816` + `00000824`, 14 events):
  - `current`: `6/14` harsh, `0/14` leapfrog, avg score `0.928`
  - `horizon_v1`: `4/14` harsh, `0/14` leapfrog, avg score `0.654`
- Deterministic replay regression seeds are now in-tree for persistent harsh holdout events (`0000071c` events `14/15/19`, `00000721` event `4`) in `selfdrive/controls/lib/tests/test_stopping_controller.py`.
- Latest-model strict failing targets are now in-tree for remaining harsh events (`0000071c` events `2/14/15/19`) to drive next tuning iterations.
- Current focus: use `horizon_v1` event wins to simplify and improve runtime stop shaping, especially late stopping-mode harshness without giving back lead-gap control.
- Secondary focus: eliminate stop-intent dropouts that allow a rapid low-speed “resume” (driver intervention/disengage class).
- Remaining work is mostly *quality and maintainability*:
  - stop-controller tuning still needs iterations on fresh routes,
  - the runtime controller has grown a large number of narrow guards (harder to reason about),
  - inverse variants show promise in replay, but require careful translation into runtime-safe logic.
- 2026-03-14 fresh-route note: newly synced route `000007df--73ac2a18cd` is a replay-alignment trap, not yet a clean tuning target. The enabled measured slice has 4 events, but the current `should_stop -> hold` replay only sees 3 events with avg score `0.381`, and several controller-side attempts around rebound-arrest / glide-handoff / pre-standstill release did not improve that slice or worsened it. Treat the next step on this route as replay-window/model-alignment plus route-derived pre-standstill seeds, not more ad hoc runtime guard stacking.
- 2026-03-14 kept runtime direction: a small `clean_settle_profile` for the moderate-rollout, pre-standstill settle band improved fresh-route replay without moving the frozen slice. On the March 14 plant-model replay, `000007df` improved from avg `0.379 -> 0.364`, `000007d0` improved from `0.622 -> 0.614`, `000007af` stayed flat, and the frozen holdout stayed `0/26` harsh, `0/26` leapfrog, avg `0.185`. This is still a small runtime step, not the full landing-planner rewrite, but it is the first kept candidate from the broader “replace the cap stack with a profile” direction.
- 2026-03-14 analysis tooling now tracks stop-entry sharpness separately from end-stop sharpness. Summaries include `EntryJerk` / `EntryStep` / `EntryCmdJerk` / `EntryCmdStep` around the first `stopping` transition (fallback: first `shouldStop` sample). Early review says both command-side and accel-side entry metrics matter: `000007df` event `1` is mostly command-sharp (`EntryCmdJerk=1.37`), while `000007d0` event `5` is mostly accel-bite (`EntryStep=0.13`) despite almost no command jerk.
- 2026-03-14 historical check on pinned holdout routes says entry-side measurement is worth keeping, but not yet ready for hard gating. On the canonical enabled holdout slice (`commawifi`, `0000071c` + `00000721`), `EntryCmdJerk` stays mostly low (median `0.01`, p95 `1.01`) while recent bad routes push higher (`p95 1.33`), but accel-side `EntryJerk` / `EntryStep` already have real holdout outliers. Treat `Entry*` as triage and seed-building signals first, not as immediate gate thresholds.
- 2026-03-14 lead-distance review says real lead-following stops usually finish well beyond the tooling-only `< 2 m` rollout budget. On the last `40` clean recent lead-following stops, `LeadStart` was `1.10-6.40 m` (median `3.81`) and `LeadHold` was `1.00-5.89 m` (median `3.06`). Treat that `< 2 m` number as a no-lead rollout budget only. If we add an explicit lead-stop target, the change belongs in longitudinal MPC lead-obstacle construction, not in `stopping_controller.py`.
- 2026-03-14 next initial-jerk work should move to the stop-entry envelope. Current target seeds are `000007df/1`, `000007d0/4`, `000007af/3`, `000007b1/25`, and holdout sentinel `0000071c/1`, with `00000721/19` as the first regression check after any entry-envelope change.
- 2026-03-14 kept stop-entry runtime step: a short `stop_entry_soften` window now handles the actual controller-visible entry-jolt cases instead of softening every first `shouldStop` frame. It improves the two clearest route-derived onset seeds without touching the deep-brake sentinel:
  - `000007af/2`: onset output `-0.412 -> -0.396`
  - `000007b1/32`: onset output `-0.050 -> -0.020`
  - `000007df/1` sentinel: unchanged `-0.816`
  - full controller tests are green (`39 passed`)
  - frozen holdout replay sanity stayed clean: `0/29` harsh, `0/29` leapfrog, avg `0.212`
- 2026-03-14 next infrastructure step is now in place: model replay reports `pred_entry_*` and `pred_lead_*` metrics, so the same offline seed lane can evaluate stop-entry sharpness and lead-gap behavior without waiting for measured re-analysis only. Targeted replay tests pass in `tools/stopping/test_check_harsh_stops_model.py`.
- 2026-03-15 lead-stop target is isolated from generic following again: the live lead obstacle path is back on legacy follow-gap semantics, while the explicit stopped-lead target only feeds `distanceToStopTarget` for `longcontrol` / `stopping_controller.py`. Current lead target is `3.0 m`; no-lead behavior still uses the existing `STOP_DISTANCE`, FrogPilot `increasedStoppedDistance` compensation stays only in the runtime MPC path, and shared helper `desired_follow_distance()` keeps legacy generic-follow-gap semantics unless a caller explicitly opts into the stopped-lead target.
- 2026-03-15 post-update route review (`000007ea`, `000007ec`) says the newest clean lead-follow stops are mostly short, not wide. Clean enabled `LeadHold` values came out `0.90`, `0.99`, `1.10`, `1.88`, `2.40`, and `3.50 m`; the likely near-crash case (`000007ec` event `4`) is a dirty manual-brake/disengage window, not a clean autonomous hold-gap sample.
- 2026-03-15 kept runtime direction from that review: start `LongControl` stopping slightly earlier when planner already exposes a close stopped-lead target and planner accel is meaningfully negative. This keeps generic follow-gap semantics unchanged, but stops waiting for raw `shouldStop` to rise before `StoppingController` can spend the remaining distance. Route sanity check: on `000007ec` event `2`, the new rule would activate about `4.8 s` before the recorded stop event (`vEgo 1.84 m/s`, `aTarget -0.81`, `lead 4.66 m`, `distanceToStopTarget 1.16 m`).
- 2026-03-15 next runtime step is now in place too: a separate stopped-lead approach band in `LongControl` PID mode. When planner already exposes a moderate stopped-lead target window, runtime now applies a mild brake-cap/freeze before full stopping takes over. This is deliberately earlier than terminal stopping and is aimed at the “5-6 m actual lead distance at >10 kph” handoff. Route sanity check: on `000007ec` event `2`, the new soft band would engage around `lead 5.72 m`, `distanceToStopTarget 2.22 m`, `vEgo 2.28 m/s`, before the later stop-entry latch at `4.66 m`.
- 2026-03-18 current harshness focus moved one layer up from `StoppingController`: the newest measured bite is the first `LongControl -> StoppingController` handoff frame when a deep inherited brake command is carried into stopping. Kept runtime fix is a one-frame `stop_entry_handoff_soften` in `longcontrol.py`, scoped to non-urgent mid-low-speed stop entry only. Sanity seeds:
  - smooth handoff seed (`v=1.03`, `a=-0.79`, `aTarget=-0.35`, `last=-0.77`, `distanceToStopTarget=0.9`) now enters stopping at about `-0.565` instead of `-0.770`
  - urgent close-stop sentinel (`v=0.55`, `distanceToStopTarget=0.10`) stays deep at about `-0.758`
- 2026-03-14 lead-gap objective order for future tuning:
  - smooth stop behind lead takes priority over minimizing final distance
  - if two candidates are similarly smooth, prefer the shorter stopped gap
  - practical preference order is smooth `3 m` over smooth `4 m`, and even smooth `2 m` over a harsh stop
- 2026-03-14 fresh lead-gap route `000007e3--69f1c0a24d` is now a clean runtime-tuning target after replay alignment was fixed to fall back to the final hold-window lead gap when replay does not quite cross standstill inside the measured window. On the enabled speed slice, the active misses are:
  - event `1`: long final lead gap (`4.42 m` on `HEAD`)
  - event `4`: short final lead gap with slight rebound (`1.93 m` plus leapfrog on `HEAD`)
- 2026-03-14 kept runtime direction for `000007e3`: a distance-aware `distance_carry_settle` branch in `stopping_controller.py` improves the long-gap case without adding new test regressions. On the same March 9 plant-model replay and lead-gap gate:
  - fresh `000007e3` improved from avg score `0.582 -> 0.527` with counts unchanged at `2/5` harsh and `1/5` leapfrog
  - combined pinned holdout (`0000071c` + `00000721`) stayed flat on counts at `16/26` harsh, `0/26` leapfrog and improved slightly on avg score `1.933 -> 1.921`
  - rejected sub-approach: a tighter distance-capture handoff removed the leapfrog on event `4` but made the same event harsh on end-stop jerk and worsened the route average, so it was dropped
- 2026-03-20 newest measured routes `00000815` and `00000814` looked bad at first glance, but their harshest windows are not valid direct runtime seeds:
  - the worst events are off-state / zero-command windows (`enabled_ratio=0`, `stopping_state_ratio=0`, `min_cmd=0`)
  - use them as route-review warnings, not as deterministic controller targets
- 2026-03-20 kept runtime direction for harshness is still at the `LongControl -> StoppingController` boundary:
  - widened the first-frame `stop_entry_handoff_soften` into a bounded moderate-speed non-urgent band (up to about `2.3 m/s`)
  - fresh clean replay slice `000007e3` improved from `2/5` harsh, `1/5` leapfrog, avg `0.527` to `2/5` harsh, `0/5` leapfrog, avg `0.503`
  - clean recent seed `00000804/3` stayed flat rather than regressing
- 2026-03-20 latest clean lead-gap review points one layer earlier than the final stop shaper:
  - `00000815/1` still lands too short (`LeadHold=0.79 m`), but the route evidence shows planner is surfacing only a tiny `distanceToStopTarget` (`0.078 m`) when actual free space is still about `3.57 m`
  - kept bounded fix:
    - `long_mpc.py` now only exposes the stronger stopped-lead target inside the last `4.5 m` of free space and only for truly creeping leads
    - `longcontrol.py` now requires at least `0.2 m` of meaningful stop-target distance before full stop mode takes over, so near-zero crumbs stay in the soft approach band but the short-gap `00000815/1` seed can hand off one frame earlier
  - route sanity check from recorded `00000815` samples:
    - short-gap event `1`: at `distanceToStopTarget=0.23`, the old `0.5 m` floor stayed in PID at about `-1.300`, while the new `0.2 m` floor enters stopping and softens to about `-1.000`
    - clean reference `00000816/3`: low-speed output stays essentially flat (`-0.543 -> -0.550`) while handing off a little earlier
- 2026-03-21 follow-up on the “still stopping 6-7 m behind lead” report:
  - fresh post-update route `0000081d--48e10ce7bf` does not show a new clean wide lead-follow miss; its clean enabled lead holds are `2.29 m`, `3.35 m`, and `3.99 m`
  - the remaining likely bug is structural: `distanceToStopTarget` was keyed to the instantaneous MPC source and raw stopped-lead factor, so brief source flips or noisy `vLead` could drop the explicit `3.5 m` target and let runtime fall back to the ordinary wider stop
  - kept bounded fix:
    - compute stopped-lead stop-target candidates for both leads every cycle, not just the winning MPC source
    - latch the last positive `distanceToStopTarget` for `0.6 s` across brief source / `vLead` dropouts
    - keep the ordinary moving-follow obstacle model unchanged, so generic follow distance should not move
- 2026-03-22 newest harsh/leapfrog route `00000824--e42e1042fc` points to stop-target mode chatter, not another tail-planner-only issue:
  - clean enabled event `1` is pure PID braking with `shouldStop=False` and `distanceToStopTarget=-1.0`, so it is outside the explicit stopped-lead target lane
  - the useful stop-target seed is hybrid event `5`: raw qlog frames show the old logic entering `stopping` on a tiny `distanceToStopTarget=0.28 m` while the lead is still moving about `1.6 m/s`, then dropping back to PID at `1.68 m`
  - kept bounded fix in `longcontrol.py`:
    - make the minimum meaningful stop-target distance dynamic, so tiny non-urgent targets stay in the soft approach band
    - add stop-target hold hysteresis once `stopping` is already active, so a real positive target does not chatter back to PID on the next frame
  - route-shaped checks:
    - `00000824/5`: old logic would go `pid -> stopping` at `0.28 m`, then `stopping -> pid` again at `1.68 m`; new logic stays `pid` at `0.28 m` and stays `stopping` at `1.68 m`
    - `00000815/1`: first stop-target handoff point stays unchanged on the real route (`0.435 m`, `aTarget=-1.306`)
- 2026-03-22 follow-up stopping-mode harshness fix on the same `00000824/5` route:
  - user feedback matched the raw route: the felt harshness is inside `stopping`, not in the earlier stop-target chatter lane
  - the low-speed corner at about `v=0.19 m/s`, `distanceToStopTarget=0.93 m`, `last_output=-0.335` was still hitting `low_rollout_soft_landing_cap`, which unwound the brake earlier than we now want for lead-target stops
  - kept bounded fix in `stopping_controller.py`:
    - do not apply `low_rollout_soft_landing_cap` while an explicit stop target is still materially ahead (`remaining_m >= 0.70`)
  - direct route-shaped effect:
    - the same representative low-speed stop-target probe moved from about `-0.319` with `low_rollout_soft_landing_cap` active to about `-0.334` with only `stop_entry_soften` + `rollout_tighten`
  - replay outcome:
    - `00000824` aligned stopping-mode slice improved from `1/8` harsh, `0/8` leapfrog, avg `0.335` to `0/8` harsh, `0/8` leapfrog, avg `0.321`
    - `0000081d` counts stayed flat at `1/3` harsh, `0` leapfrog; avg score drifted slightly from `0.645` to `0.647`
- 2026-03-28 process upgrade:
  - shared route refresh plus stopping analysis now read the current `qlog.zst` device logs directly, so “download all missing routes” works again on `realdata_konik` without manual decompression or targeted pulls
  - measured comfort is now a first-class lane in `check_harsh_stops.py`: we can filter for enabled `hybrid` events with real brake command, gate entry harshness, and count `SigDrop` / `ExitStop` as mini leapfrog
  - March 27 review confirmed the old process gap: across routes `0000099e` .. `000009a3`, enabled `speed_transition` found `0` clean stops, but the comfort lane still found `11` relevant events, `6/11` harsh, and `11/11` mini-leapfrog/dropout failures
- 2026-03-28 kept runtime fix for that March 27 harshness lane:
  - the harshness is inside `StoppingController` rebound-arrest on moderate-rollout low-speed stops, not only in planner/stop-target chatter
  - kept bounded fix: `moderate_rollout_rebound_soften` now applies only while inherited brake is still mild (`last_output_accel > -0.56`), so it softens the first arrest beat but hands back to the normal arrest lane before a later bigger jab
  - route-shaped March 27 seeds (`000009a0` events `2/6/7`) now keep the softer first arrest (`drop ~0.12-0.13` instead of `~0.30`) while aggregate max-drop returns to the old baseline (`0.307-0.311` instead of `0.360-0.375`)
- 2026-03-28 newest clean March 28 stopping lane is a late stop-mode reacquire problem, not a planner-entry problem:
  - fresh routes `000009a7`, `000009a8`, and `000009ac` still have `0` clean enabled `speed_transition` stops, so the useful evidence is the final contiguous `longState=stopping` spans inside the hybrid events
  - strongest seeds are `000009ac` events `2` and `4`: `shouldStop` turns on after brake is already built, then the controller unwinds too early into the terminal cap stack
  - kept bounded fix in `stopping_controller.py`:
    - add a short `stop_reacquire_hold` lane when stop intent reappears while inherited brake is already meaningful
    - keep `end_stop_cap_active` out of that hold window
    - hold the reacquire window slightly longer so the first stopping-mode beat does not immediately hand back to soft-landing unwind
  - deterministic direct seeds now capture the improvement:
    - `000009ac/2`: first `shouldStop` beat improves from `-0.876` to `-0.959`, and the next four beats stay near `-0.959` instead of unwinding toward `-0.464`
    - `000009ac/4`: first stopping-mode beats improve from `-0.654 / -0.519 / -0.546` to `-0.793 / -0.793 / -0.793`, while the later unwind is still present but later and weaker
  - process note: current fitted-model replay is mostly blind to this lane, so keep the direct route-derived controller seeds as the primary acceptance evidence for this specific fix
- 2026-04-09 fresh stopping harshness review used the shared route-refresh process, not the old stopping-only sync:
  - route refresh source of truth is now `tools/route_sync/refresh_routes.py`
  - `commawifi` was down during this cycle, so refresh/analyze ran against `comma` and the shared cache at `~/.comma/route_sync/downloads`
  - newest usable fresh routes were `000009cb` and `000009cc`; enabled `speed_transition` still found `0` clean stop events, so the acceptance lane stayed on direct route-derived controller seeds from the final contiguous `longState=stopping` spans
- 2026-04-09 kept runtime fix for the new harshness lane in `StoppingController`:
  - failure shape: once brake is already built in stopping mode and there is no explicit planner stop target, the terminal unwind stack can still grab the stop too early
  - strongest fresh seeds were `000009cb/3` and `000009cb/4`
  - kept fix:
    - add `terminal_unwind_delay` for no-target, built-brake, still-moving stopping beats
    - `distance_carry_settle` now requires either a real planner stop target or a much tighter no-target fallback window
    - while `terminal_unwind_delay` is active, hold the inherited brake envelope instead of letting `soft_landing_release`, `distance_carry_settle`, `low_rollout_soft_landing_cap`, or `end_stop_cap_active` unwind early
  - direct seed effect versus `HEAD`:
    - `000009cb/3`: old beats `4..11` unwound from `-0.683` down to `-0.272`; kept fix holds the same lane near `-0.739 .. -0.734`
    - `000009cb/4`: old beats `4..10` unwound through `-0.649`, `-0.733`, `-0.688`, `-0.646`, `-0.596`, `-0.541`, `-0.482`; kept fix holds that same lane at `-0.779` through beat `10`
  - acceptance evidence:
    - deterministic direct controller seeds added for `000009cb/3`, `000009cb/4`, and the corroborating `000009cc/1` no-target lane
    - focused tests passed:
      - `selfdrive/controls/lib/tests/test_stopping_controller.py` -> `53 passed`
      - `tools/stopping/test_check_harsh_stops_model.py` -> `30 passed`
- 2026-04-09 measured comfort lane does reflect today's bad stopping quality, but only partly in deterministic runtime seeds:
  - fresh measured comfort check on `000009ca`, `000009cb`, `000009cc` failed hard on the filtered enabled+braking slice: `5/5` harsh and `5/5` mini-leapfrog/dropout
  - clean replay/controller windows are still mostly blind on this lane, so the acceptance contract stays split:
    - measured comfort lane for route discovery and keep/reject review
    - direct route-derived `StoppingController` seeds for deterministic runtime iteration
  - deterministic coverage was extended with a late standstill restart seed from `000009ca/2`
- 2026-04-09 kept follow-up runtime fix for the fresh harshness lane in `StoppingController`:
  - failure shape: the late `shouldStop` reacquire floor was still too deep for the moderate-speed `000009cb` lane, which preserved distance but kept a noticeable jab
  - kept fix:
    - add `high_speed_reacquire_soften` inside `stop_reacquire_hold`
    - only applies for `0.45 < vEgo < 0.95`, strong ongoing decel, and mildly deep inherited brake (`-0.74 < last_output < -0.50`)
    - low-speed/deeper inherited-brake reacquire lanes stay on the older stronger floor
  - direct seed effect:
    - `000009cb/3`: first reacquire beats soften from about `-0.739` to about `-0.732`
    - `000009cb/4`: first two reacquire beats soften from about `-0.779` to about `-0.737`, then the stronger hold lane still takes over on the next beat (`~ -0.757`)
    - `000009ac/4` stays on the older floor; `high_speed_reacquire_soften` does not arm there
  - acceptance evidence:
    - deterministic runtime coverage now includes `000009ca/2`, `000009cb/3`, `000009cb/4`, and `000009cc/1`
    - focused tests passed:
      - `selfdrive/controls/lib/tests/test_stopping_controller.py` -> `54 passed`
      - `tools/stopping/test_check_harsh_stops_model.py tools/stopping/test_check_harsh_stops.py` -> `44 passed`

## Risks and Tech Debt (Why Cleanup/Refactor Is Timely)

- `StoppingController` is now a large rule stack; guard ordering matters and is easy to regress.
- Many thresholds are “magic numbers” spread across the update path, making review and systematic tuning hard.
- Offline controllers (`inverse*`) and runtime are intentionally different; without an abstraction boundary, idea transfer is manual and error-prone.
- Documentation is split between:
  - a shared route-refresh contract (`docs/route_refresh_process.md`),
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
- `horizon_v1` as the single maintained offline optimizer probe.
- `legacy_32b8be` for sanity checks.
- Only rerun the full three-lane comparison when the fitted plant model changes materially or when a runtime approach changes phase behavior; do not treat every micro-guard tweak as a new variant campaign.
- Define a single “promotion gate contract” for any runtime change:
  - no regressions on frozen holdout,
  - no regressions on pinned harsh/leapfrog routes,
  - rollout within budget,
  - leapfrog does not worsen when harsh improves.

### Phase D: Try Something New (only if Phase B/C stalls)

If rule-stack tuning stops moving the needle:

- Use `horizon_v1` to identify the winning command-sequence shape offline.
- Port only the winning shape into runtime-safe controller logic.
- If the same optimizer behavior keeps winning, simplify the runtime phase logic around that phase instead of stacking more guards.

## Definition of Done (for a Meaningful “Stopping Improvement”)

A change is considered a “good stopping improvement” only if:

- It improves harsh metrics and does not regress leapfrog on the same evaluation slice(s).
- It stays within the active stop-distance contract:
  - no-lead: rollout budget `<= 2.0m`
  - lead-follow: final hold gap `2.0-4.0m`
- It passes both measured and model-based offline gates on the frozen holdout set.
- It is documented in the worklog with commands and artifact paths.
