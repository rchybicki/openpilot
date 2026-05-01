# Stopping Behavior Project: Status and Direction

- Updated: 2026-05-01
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

Latest live bookmark reviewed on 2026-05-01:

- route `00000916--fd3c4a348d`, segment `15`, event `4`,
- user bookmark was about `3.24s` after hold,
- lead gap was too large: `LeadEntry=9.26m`, `LeadHold=8.99m`,
- measured check marks it harsh with `far_lead_brake_spike`, `end_stop_jerk`, and `end_stop_accel_step`,
- root cause: no explicit stop target was available, but a stopped/slow lead around `9m` kept stop-hold authority active and allowed a late brake-force spike.

## Current Runtime

Runtime source of truth is still deterministic code:

- `selfdrive/controls/lib/longcontrol.py`
  - owns the `LongCtrlState` lifecycle and handoff into stopping,
  - includes Santa Fe-specific Experimental close-lead accel limiting,
  - includes Santa Fe low-speed stopped-lead glide protection,
  - now releases no-target stopped-lead holds above `6.0m` into a capped crawl instead of continuing to stop at a far gap.
- `selfdrive/controls/lib/stopping_controller.py`
  - owns the final stop execution in `APPROACH`, `NEAR_HOLD`, and `HOLD`,
  - contains the current low-speed rollout, release-lock, rebound-arrest, comfort-release logic,
  - now includes local candidate profiles extracted from `horizon_v1` teacher behavior.
- `selfdrive/controls/lib/stopping_shadow.py`
  - runs a shadow-only learned profile oracle using the broad-corpus plant fit and bounded learned residual templates,
  - writes profile, score delta, predicted rollout/harsh/leapfrog flags, and guard rejection reason into stopping debug/log data,
  - never modifies the commanded acceleration.
- `selfdrive/controls/lib/stopping_guard.py`
  - provides low-speed slew limiting outside the explicit stopping phase.

Important status: there is still no learned runtime brake authority on-device. The deployable 2026-04-30 shadow candidate keeps deterministic stopping in command, but runtime can now log what the learned plant/profile oracle would have selected.

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
  - emits `horizon_teacher` summaries showing what command-shape the offline optimizer wanted,
  - exports runtime-available selector feature snapshots and teacher-derived selector labels,
  - can evaluate a learned profile library with a plant-model oracle that rejects worse scores and new harsh/leapfrog flags.
- `selfdrive/controls/lib/stopping_profile_selector.py`
  - defines the bounded profile classes plus prototype and k-nearest-neighbor selectors that can be trained offline without new dependencies.
- `tools/stopping/train_profile_selector.py`
  - trains an auditable profile library from benchmark output instead of hand-writing another `interp(...)` table.

Current offline result: the classifier alone is not good enough for command authority, but the learned profile library plus plant-model oracle is now a credible next architecture.

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

Expanded selector seed refresh on 2026-04-30:

- Route refresh:
  - `/Users/radoslawchybicki/.route_sync/reports/route_refresh_comma_20260430T131126Z.json`
- Hybrid corpus:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/comma/20260430_selector_expanded_hybrid/summary.json`
- Result:
  - `20` routes scanned,
  - `318` enabled hybrid stop events,
  - `25` engaged stopping replay windows used for the first broad selector benchmark.

Expanded partial broad refresh on 2026-04-30:

- Local cache after interrupted broad pull:
  - `627` qlogs under route-sync cache.
- Hybrid corpus:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/comma/20260430_selector_broad_partial_hybrid/summary.json`
- Result:
  - `38` routes scanned,
  - `718` enabled hybrid stop events,
  - `126` engaged stopping replay windows used for broad teacher/profile-oracle evaluation.
- Broad plant model:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/models/stopping_model_20260430_selector_broad_partial_hybrid.json`
  - windows `718`, rows `1294`, RMSE `0.0522`, MAE `0.0283`, R2 `0.9628`.

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

The current local candidate combines the learned/offline bridge with one safety/comfort fix from the latest live bookmark:

- lead-aware terminal rollout teacher profile for no-target stop tails,
- explicit-target far-tail teacher release for non-close leads,
- far-lead high-rollout release profile for stopped-lead tails with large available gap,
- `LongControl` now forwards lead status, lead speed, and lead distance into `StoppingController`,
- controller replay tooling forwards the same lead signals so offline tests match runtime inputs,
- learned profile-oracle shadow mode logs sampled on-device decisions through `cloudlog.event("stopping_shadow", ...)`,
- Santa Fe no-target far stopped-lead gap release prevents holding/stopping around `9m`; explicit stop targets and inside-`6m` stopped-lead cases keep existing stopping authority.

Frozen-slice replay result with the current local candidate:

- latest named hybrid slice: `8/12 -> 7/12` harsh, leapfrog `0/12 -> 0/12`, average score `2.136 -> 2.103`
- recent hybrid slice: `25/52 -> 23/52` harsh, leapfrog `1/52 -> 1/52`, average score `1.624 -> 1.601`
- recent speed slice: current candidate `33/69` harsh, leapfrog `2/69`, average score `1.879`

The speed slice baseline was not captured before the local source change, so it is recorded as candidate-only evidence for this iteration.

## Current Decision

Stop adding route-specific runtime guards unless a route exposes a safety-critical issue.

The longer-term stopping improvement should come from a constrained learned residual/profile-oracle path:

- keep `LongControl` and the stop lifecycle deterministic,
- keep hard-coded safety limits for accel, jerk, rollout, lead gap, and lead-closing risk,
- use the learned plant model to evaluate a small learned library of bounded stop-tail residual profiles,
- only accept a profile when predicted score improves and no new harsh/leapfrog flag appears,
- use the new shadow logs to compare oracle decisions against real route outcomes before any command authority.

This is not a raw end-to-end brake model. It is a small model-predictive decision layer inside a deterministic safety envelope.

## Current Offline Candidate

Best current offline candidate:

- Model:
  - KNN profile library trained from `benchmark_broad_teacher_targets_12.json`,
  - profile-oracle mode in `benchmark_controller_variants.py`,
  - exemplar distance cap `0.90`,
  - candidate rejection if score worsens or a new harsh/leapfrog flag appears.
- Same-corpus broad replay:
  - artifact: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/comma/20260430_selector_broad_partial_hybrid/benchmark_profile_oracle_broad_knn3_exemplar090_rescaled.json`
  - current: harsh `21/126`, leapfrog `0/126`, avg score `0.410`
  - profile oracle: harsh `12/126`, leapfrog `0/126`, avg score `0.314`
  - selector improved `108`, worsened `0`
- Leave-one-route-out replay:
  - artifacts: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/comma/20260430_selector_broad_partial_hybrid/loo_oracle_knn3_exemplar090_rescaled/`
  - current: harsh `21/126`, leapfrog `0/126`, avg score `0.410`
  - profile oracle: harsh `15/126`, leapfrog `0/126`, avg score `0.343`
  - selector improved `81`, worsened `0`

Decision: this is a real offline improvement, but it is not ready to deploy as brake authority. Runtime shadow logging is now the next validation step: deploy shadow-only, drive normal routes, then compare `stopping_shadow` decisions with bookmarks, harsh-force review, and corpus replay.

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

Current model shape:

- KNN/profile library from teacher output plus plant-model oracle,
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

Current deployable shadow mode:

- samples stopping at low rate from `LongControl`,
- logs selected profile, bounded residual preview, current controller command, predicted score delta, predicted rollout, and guard rejection reason,
- keeps deterministic `StoppingController` output as the only command source.

Next validation:

- deploy shadow-only code,
- collect routes with and without bookmarks,
- compare `stopping_shadow` decisions against actual force spikes, leapfrog/rebound, and final lead gap.

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
