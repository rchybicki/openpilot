# Stopping Behavior Project: Status and Direction

- Updated: 2026-04-26
- Vehicle focus: Hyundai Santa Fe HEV 2022
- Scope: OpenPilot/FrogPilot longitudinal stop execution, especially the final low-speed stop tail
- Worklog: `docs/stopping_behavior_worklog.md`
- Route intake contract: `docs/route_refresh_process.md`
- Operational commands: `tools/stopping/README.md`

## Current Problem

Stopping is materially better than earlier iterations, but it is not solved. Recent drive feedback says many stops are now gentle, while the remaining failures cluster around:

- harsh force spikes near the final stop,
- stop/release/re-stop or "leapfrog" behavior,
- late stop-state reacquire/dropout,
- low-speed drivetrain disturbances from the automatic clutch / EV-to-combustion transition,
- stopped-lead cases where the final gap can be safe but the brake command still does the wrong shape.

The current rule stack can patch individual cases, but the full Wi-Fi corpus review says the next major improvement should be learned/profile-based rather than another narrow guard.

## Current Runtime

Runtime source of truth is still deterministic code:

- `selfdrive/controls/lib/longcontrol.py`
  - owns the `LongCtrlState` lifecycle and handoff into stopping,
  - includes Santa Fe-specific Experimental close-lead accel limiting,
  - includes Santa Fe low-speed stopped-lead glide protection.
- `selfdrive/controls/lib/stopping_controller.py`
  - owns the final stop execution in `APPROACH`, `NEAR_HOLD`, and `HOLD`,
  - contains the current low-speed rollout, release-lock, rebound-arrest, comfort-release logic,
  - now includes local candidate profiles extracted from `horizon_v1` teacher behavior.
- `selfdrive/controls/lib/stopping_guard.py`
  - provides low-speed slew limiting outside the explicit stopping phase.

Important status: there is no learned runtime stopping controller on-device yet. The deployable local candidate is still deterministic code; it uses offline teacher evidence to add bounded runtime profiles, not an on-device ML model.

## Existing Learned / Offline Pieces

We already have learning in the process, but it is advisory/offline:

- `tools/stopping/fit_stopping_model.py`
  - fits a learned vehicle response / plant model from qlogs,
  - predicts how `aEgo` responds to delayed accel commands at low speed,
  - supports linear, speed-band, and low-speed-blend model families.
- `tools/stopping/check_harsh_stops_model.py`
  - replays recorded or controller-generated command traces through the learned plant model,
  - gates predicted harshness, rollout, and leapfrog risk.
- `tools/stopping/benchmark_controller_variants.py`
  - compares `current`, `horizon_v1`, and `legacy_32b8be`,
  - emits `horizon_teacher` summaries showing what command-shape the offline optimizer wanted.

What is missing is the next layer: a deployable learned residual/profile selector that can translate the offline teacher into a bounded runtime command envelope.

## Latest Corpus State

Full Wi-Fi intake on 2026-04-26:

- Source: `commawifi:/data/media/0/realdata`
- Route-sync-indexed qlogs verified: `2337/2337`
- Raw remote qlogs: `2363`; the extra `26` are truncated/corrupt `qlog.zst` files and are locally byte-matched too.
- Local all-history cache: `13023` qlog files, `42G`
- Routes scanned in both corpora: `806`

Latest artifact paths:

- Hybrid enabled corpus:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/commawifi/20260426_full_wifi_hybrid_enabled/summary.json`
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/commawifi/20260426_full_wifi_hybrid_enabled/diagnosis.md`
- Speed-transition enabled corpus:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/commawifi/20260426_full_wifi_speed_enabled/summary.json`
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/commawifi/20260426_full_wifi_speed_enabled/diagnosis.md`
- Route-sync reconcile report:
  - `/Users/radoslawchybicki/.route_sync/reports/route_refresh_commawifi_20260426T141555Z_rsync_reconcile.json`

Measured review summary:

- Hybrid enabled lane:
  - all-history lane: `736` events, `622` harsh (`84.5%`), `93` rebound/leapfrog (`12.6%`), `225` strict dropout/rebound (`30.6%`)
  - route-id slice `000008ee` and newer: `50` events, `38` harsh (`76.0%`), `3` rebound/leapfrog (`6.0%`), `13` strict dropout/rebound (`26.0%`)
  - latest named fresh slice (`000009bf`, `000009c0`, `000009c2`, `000009ca`, `000009cb`, `000009cc`): `11` events, `7` harsh (`63.6%`), `0` rebound/leapfrog, `2` strict dropout/rebound (`18.2%`)
- Speed-transition enabled lane:
  - all-history lane: `849` events, `722` harsh (`85.0%`), `112` rebound/leapfrog (`13.2%`), `272` strict dropout/rebound (`32.0%`)
  - route-id slice `000008ee` and newer: `57` events, `45` harsh (`78.9%`), `3` rebound/leapfrog (`5.3%`), `19` strict dropout/rebound (`33.3%`)
  - latest named fresh slice: `12` events, `8` harsh (`66.7%`), `0` rebound/leapfrog, `3` strict dropout/rebound (`25.0%`)

Interpretation:

- Recent rebound/leapfrog is much lower than the historical corpus, matching recent subjective feedback.
- Harshness remains common in the latest filtered slices.
- The remaining failures are not one obvious threshold miss; they are repeated command-shape failures around late stop intent, stop-state dropout/reacquire, and low-speed drivetrain disturbance.

## Current Deploy Candidate

The first deployable bridge from the learned/offline process is now implemented locally:

- lead-aware terminal rollout teacher profile for no-target stop tails,
- explicit-target far-tail teacher release for non-close leads,
- far-lead high-rollout release profile for stopped-lead tails with large available gap,
- `LongControl` now forwards lead status, lead speed, and lead distance into `StoppingController`,
- controller replay tooling forwards the same lead signals so offline tests match runtime inputs.

Frozen-slice replay result with the current local candidate:

- latest named hybrid slice: `8/12 -> 7/12` harsh, leapfrog `0/12 -> 0/12`, average score `2.136 -> 2.103`
- recent hybrid slice: `25/52 -> 23/52` harsh, leapfrog `1/52 -> 1/52`, average score `1.624 -> 1.601`
- recent speed slice: current candidate `33/69` harsh, leapfrog `2/69`, average score `1.879`

The speed slice baseline was not captured before the local source change, so it is recorded as candidate-only evidence for this iteration.

## Current Decision

Stop adding route-specific runtime guards unless a route exposes a safety-critical issue.

The longer-term stopping improvement should still come from a constrained learned residual/profile-selector path:

- keep `LongControl` and the stop lifecycle deterministic,
- keep hard-coded safety limits for accel, jerk, rollout, lead gap, and lead-closing risk,
- use the learned model and `horizon_v1` teacher to select or adjust the final stop-tail envelope,
- shadow-log the learned selector before it is allowed to command.

This is not a raw end-to-end brake model. It is a small learned decision layer inside a deterministic safety envelope.

## Next Plan

### Phase 1: Freeze Inputs

- Keep the full Wi-Fi corpus artifacts above as the current data baseline.
- Split by route into train / validation / holdout.
- Keep known bookmarked failures and pinned harsh/leapfrog routes out of train.
- Quarantine corrupt/truncated qlogs from model fit inputs, but keep them noted in corpus health.

### Phase 2: Refit the Plant

- Refit at least one all-events plant model on the updated train split.
- Fit a sensitivity model on engaged-only or low-speed-only windows.
- Record model paths, row counts, delay frames, RMSE/MAE/R2, and selected model family in the worklog.
- Do not change the plant model between baseline and candidate comparisons on the same frozen slice.

### Phase 3: Build a Teacher Dataset

For each stop window, export:

- route, segment, event id, source, train/validation/holdout split,
- `vEgo`, `aEgo`, command history, lead distance / velocity, stop-target distance, stop-state flags,
- current controller output trace and trigger trace,
- measured labels: harsh flags, leapfrog/dropout flags, rollout, final lead gap,
- `horizon_v1` output trace and teacher intent (`soften`, `deepen`, `tail_deepen`, `soften_then_deepen`, `reshape`, etc.).

The label should be a bounded profile/residual target, not driver brake imitation.

### Phase 4: Train a Small Selector

Initial model shape:

- classifier or small regressor that chooses a stop-tail profile family or residual cap,
- inputs limited to signals available in runtime,
- output limited to a bounded accel envelope or residual correction,
- monotonic/safety constraints encoded outside the model.

Candidate families:

- `preserve_brake`: hold inherited brake longer when stop intent reacquires late,
- `soften_then_deepen`: reduce initial bite but rebuild smoothly before hold,
- `tail_deepen`: prevent clutch push / rebound while already near hold,
- `glide_soften`: avoid late far-gap brake spikes behind stopped leads,
- `no_change`: keep deterministic controller behavior.

### Phase 5: Offline Gates

A candidate learned selector is not deployable unless it passes:

- measured harsh/leapfrog gates on holdout,
- plant-model replay gates on holdout,
- latest-route comfort lane review,
- no worse final lead-gap contract,
- no worse no-lead rollout contract,
- no increase in strict dropout/rebound rate.

Current acceptance contract:

- no-lead rollout target: `<= 2.0m`
- lead-follow final hold gap target: `2.0-3.5m`, with `~2.75m` preferred
- harsh improves without leapfrog regression on the same slice

### Phase 6: Shadow Mode

Before runtime command authority:

- run the learned selector on-device in shadow mode,
- log selected profile, bounded residual, guard limits, current controller command, and model disagreement,
- compare shadow predictions against real route feedback and corpus gates.

### Phase 7: Gated Runtime Integration

Only after shadow validation:

- allow the learned selector to modify final stop-tail commands inside hard-coded caps,
- default to deterministic `current` behavior when inputs are missing or confidence is low,
- include a runtime kill switch / compile-time constant for rapid rollback,
- deploy only after focused tests and frozen-slice replay pass.

## Promotion Rules

A stopping change is deployable only when:

- it has a frozen before/after evaluation slice,
- it improves harsh metrics and does not regress leapfrog/dropout metrics,
- rollout and final lead-gap contracts still pass,
- focused controller/tooling tests pass,
- worklog records commands, artifacts, metrics, and keep/reject decision,
- any changed runtime behavior has a route-derived regression seed or a documented reason why a seed is not possible.

## Deprecated / De-Emphasized

- Do not maintain broad legacy variant campaigns. Active comparison lanes are:
  - `current`
  - `horizon_v1`
  - `legacy_32b8be` as sanity baseline only
- Do not treat every new route as a tuning target. New routes are first validation or future split inputs.
- Do not ship the plant model, `horizon_v1`, or an opaque learned brake model directly.
- Detailed historical route notes belong in `docs/stopping_behavior_worklog.md`, not in this status snapshot.
