# Stopping Behavior Project: Status and Direction

- Updated: 2026-06-02
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
- stopped-lead cases where the final hold gap is now treated as unacceptable outside `2.75m..5.0m`.

The current rule stack can patch individual cases, but the full Wi-Fi corpus review says the next major improvement should keep using learned/profile-based replay as the teacher while shipping only bounded deterministic command changes.

Latest deployable candidate from the 2026-06-02 offline goal cycle:

- validation gate: 2026-05-14 fresh hybrid corpus, `19` engaged-stopping replay events, strict absolute stopped-lead band `2.75m..5.0m`;
- baseline current HEAD before the cycle: harsh `18/19`, leapfrog `0/19`, avg score `1.751`, good-or-better `0/19`;
- candidate: harsh `16/19`, leapfrog `0/19`, avg score `1.571`, good-or-better `1/19`;
- validation score improvement: `10.25%` better than baseline;
- tuning cross-check: 2026-04-30 broad partial hybrid corpus, `25` engaged-stopping replay events, avg score `1.219 -> 1.052` (`13.7%` better), harsh `10/25 -> 7/25`, leapfrog stayed flat at `2/25`;
- runtime change is deterministic: it adds bounded explicit-target tail releases for adequate-gap lead stops, a low-rollout strong-decel release lane seeded from `0000090b` event `3`, and a softer long-gap lead glide that only fires while the car is still decelerating.

Prior deployable candidate from the 2026-05-31 offline goal cycle:

- hard-route model gate: pass, harsh `3/11`, leapfrog `0/11`, avg score `0.434`;
- realistic hard-route comfort slice, excluding non-actionable far-lead association: avg score `0.364 -> 0.318` (`12.7%` better);
- comfort quality score on that slice: `2.0 -> 1.6` (`20.0%` better);
- perfect stops: `1/10 -> 2/10`; good-or-better stops: `3/10 -> 6/10`;
- harsh/leapfrog counts stayed flat on the realistic slice: harsh `2/10`, leapfrog `0/10`;
- runtime change is deterministic: `explicit_target_mid_tail_teacher_profile` releases a far-lead explicit-target tail one beat earlier while the target is still about `0.8m..1.7m` ahead, then leaves the normal tail/hold logic to rebuild brake if decel fades.

Prior 2026-05-14 reference candidate:

- frozen intake covers `119` fresh routes and `423` enabled hybrid stop events,
- fresh plant model fit: `862` rows, delay `1`, RMSE `0.0660`, R2 `0.9444`,
- frozen engaged-stopping replay baseline: harsh `11/31`, leapfrog `1/31`, avg score `0.8648`,
- first explicit lead-follow glide soften: harsh `10/31`, leapfrog `1/31`, avg score `0.759` on the old relaxed lead-gap gate,
- corrected absolute lead-gap gate (`2.5m..5.0m`, no recorded-wide slack): current candidate harsh `23/31`, leapfrog `0/31`, avg score `1.734`,
- old relaxed sanity gate after the 5m policy change: harsh `11/31`, leapfrog `0/31`, avg score `0.800`.

Important: the new candidate is not a proof that lead distance is solved. It makes `>5m` final lead holds visible as failures and changes runtime policy so stopped-lead crawl/release starts above `5m`, but the frozen replay still has long-gap failures in late-start windows.

## Current Runtime

Runtime source of truth is still deterministic code:

- `selfdrive/controls/lib/longcontrol.py`
  - owns the `LongCtrlState` lifecycle and handoff into stopping,
  - includes Santa Fe-specific Experimental close-lead accel limiting,
  - includes Santa Fe low-speed stopped-lead glide protection,
  - now releases stopped-lead holds above `5.0m` into a capped crawl until an explicit stop target is close (`<=1.8m`) instead of continuing to stop at a far gap.
- `selfdrive/controls/lib/stopping_controller.py`
  - owns the final stop execution in `APPROACH`, `NEAR_HOLD`, and `HOLD`,
  - contains the current low-speed rollout, release-lock, rebound-arrest, comfort-release logic,
  - now includes local candidate profiles extracted from `horizon_v1` teacher behavior.
  - adds `explicit_lead_glide_soften` for explicit stopped-lead tails in the `3.2m..5.0m` gap band, preventing unnecessary final brake-force spikes when the lead gap is already reasonable.
  - adds a separate `explicit_lead_long_gap_glide` lane for `5.0m..8.0m` stopped-lead tails so the controller crawls rather than settling early above the new max gap.
- `selfdrive/controls/lib/stopping_shadow.py`
  - runs a shadow-only learned profile oracle using the fresh 2026-05-14 plant fit and bounded learned residual templates,
  - writes profile, score delta, predicted rollout/harsh/leapfrog flags, and guard rejection reason into stopping debug/log data,
  - never modifies the commanded acceleration.
- `selfdrive/controls/lib/stopping_guard.py`
  - provides low-speed slew limiting outside the explicit stopping phase.

Important status: there is still no learned runtime brake authority on-device. The direct learned-authority attempt was tested offline on 2026-05-14 and rejected because it did not improve the recorded frozen gate. The deployable command change is deterministic, but it was derived from the learned profile-oracle winning cases and keeps the learned oracle in fresh shadow logging for the next live review.

## Existing Learned / Offline Pieces

We already have learning in the process, but it is advisory/offline:

- `tools/stopping/fit_stopping_model.py`
  - fits a learned vehicle response / plant model from qlogs,
  - predicts how `aEgo` responds to delayed accel commands at low speed,
  - supports linear, speed-band, and low-speed-blend model families.
- `tools/stopping/check_harsh_stops_model.py`
  - replays recorded or controller-generated command traces through the learned plant model,
  - gates predicted harshness, rollout, and leapfrog risk.
  - now supports `--absolute-pred-lead-hold-distance`, which disables the old recorded-wide slack and treats `2.5m..5.0m` as the hard lead-hold acceptance band.
- `tools/stopping/benchmark_controller_variants.py`
  - compares `current`, `horizon_v1`, and `legacy_32b8be`,
  - now reports comfort-quality buckets: `perfect`, `good`, `mediocre`, `poor`, and `hard_fail`,
  - emits `horizon_teacher` summaries showing what command-shape the offline optimizer wanted,
  - exports runtime-available selector feature snapshots and teacher-derived selector labels,
  - can evaluate a learned profile library with a plant-model oracle that rejects worse scores and new harsh/leapfrog flags.
- `selfdrive/controls/lib/stopping_profile_selector.py`
  - defines the bounded profile classes plus prototype and k-nearest-neighbor selectors that can be trained offline without new dependencies.
- `tools/stopping/train_profile_selector.py`
  - trains an auditable profile library from benchmark output instead of hand-writing another `interp(...)` table.

Current offline result: the classifier alone is not good enough for command authority, but the learned profile library plus plant-model oracle is now a credible next architecture.

## Latest Corpus State

Full fresh intake on 2026-05-14:

- Source: `comma:/data/media/0/realdata`
- New qlogs downloaded: `2245` across the capped and uncapped refreshes
- New routes detected: `119`
- Hybrid enabled corpus:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/comma/20260514_full_new_hybrid_enabled/summary.json`
- Speed-transition enabled corpus:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/comma/20260514_full_new_speed_enabled/summary.json`
- Bookmark scan:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/bookmarks/comma/20260514_full_new/summary.json`
- Fresh plant model:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/models/comma/20260514_full_new_low_speed_blend.json`
- Frozen replay gate for the deployable candidate:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/comma/20260514_full_new_hybrid_enabled/model_gate_absolute_2p5_5p0_final_v2.json`

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

The current local candidate is deployable for device testing as a lead-gap policy iteration:

- keeps learned profile-oracle command authority disabled,
- refreshes shadow/profile logging to the 2026-05-14 plant fit,
- adds deterministic `explicit_lead_glide_soften` in the final controller output,
- applies the normal glide soften only inside the new acceptable lead band (`3.2m..5.0m`),
- treats `>5.0m` stopped-lead holds as a crawl/release problem instead of an acceptable final hold,
- preserves close-lead authority and the `2.5m` lower bound.

Frozen-slice replay result with the prior relaxed gate:

- recorded-shouldStop model gate:
  - before: harsh `11/31`, leapfrog `1/31`, avg score `0.8648`
  - after current 5m policy: harsh `11/31`, leapfrog `0/31`, avg score `0.800`
- broad benchmark harness:
  - before: harsh `17/31`, leapfrog `1/31`, avg score `0.837`
  - after: harsh `15/31`, leapfrog `1/31`, avg score `0.715`
  - remaining profile-oracle ceiling on the same slice: harsh `9/31`, leapfrog `1/31`, avg score `0.543`

Frozen-slice replay result with the corrected absolute lead-gap gate:

- prior explicit glide candidate re-scored against `2.5m..5.0m`: harsh `24/31`, leapfrog `6/31`, avg score `1.696`; `15/31` predicted final lead holds were above `5.0m`.
- current 5m policy candidate: harsh `23/31`, leapfrog `0/31`, avg score `1.734`.
- interpretation: this is a safety/contract improvement, not a strict score improvement. It removes predicted leapfrog on the hard gate and marks long final lead holds as failures instead of masking them with recorded-wide slack.

## Current Decision

Stop adding route-specific runtime guards unless a route exposes a safety-critical issue.

The longer-term stopping improvement should come from a constrained learned residual/profile-oracle path:

- keep `LongControl` and the stop lifecycle deterministic,
- keep hard-coded safety limits for accel, jerk, rollout, lead gap (`2.5m..5.0m`), and lead-closing risk,
- use the learned plant model to evaluate a small learned library of bounded stop-tail residual profiles,
- only accept a profile when predicted score improves and no new harsh/leapfrog flag appears,
- use the new shadow logs to compare oracle decisions against real route outcomes before any command authority.

This is not a raw end-to-end brake model. It is a small model-predictive decision layer inside a deterministic safety envelope.

## Current Offline Candidate

Best current offline candidate on the 2026-05-14 frozen slice:

- Model:
  - fresh low-speed-blend plant model at `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/models/comma/20260514_full_new_low_speed_blend.json`
  - `423` windows, `862` rows, delay `1`, RMSE `0.0660`, R2 `0.9444`
- Profile oracle:
  - artifact: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/comma/20260514_full_new_hybrid_enabled/benchmark_final_explicit_lead_glide_soften.json`
  - current after deployable deterministic cap: harsh `15/31`, leapfrog `1/31`, avg score `0.715`
  - profile oracle: harsh `9/31`, leapfrog `1/31`, avg score `0.543`
  - selector improved `21`, worsened `0`

Older 2026-04-30 broad-corpus candidate:

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

Decision for learned authority: the profile oracle remains a real offline improvement, but it is still not ready to deploy as brake authority. Runtime shadow logging is the validation path: deploy deterministic command changes only, drive normal routes, then compare `stopping_shadow` decisions with bookmarks, harsh-force review, and corpus replay.

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
- lead-follow final hold gap target: `2.5-5.0m`, with `5.0m` as the absolute max and `2.5m` as the lower bound
- harsh improves without leapfrog regression on the same slice

### Phase 6: Shadow Mode

Current deployable shadow mode:

- samples stopping at low rate from `LongControl`,
- logs selected profile, bounded residual preview, current controller command, predicted score delta, predicted rollout, and guard rejection reason,
- keeps deterministic `StoppingController` output as the only command source.

2026-05-16 validation result:

- `tools/stopping/analyze_stopping_shadow.py` now parses `stopping_shadow` from targeted `rlog.zst` files and attaches decisions to detected qlog stop events.
- On old deployed route `00001421--4090dede0b`, shadow produced useful brake-relief candidates on two harsh stops, but mixed them with unsafe accepted candidates and missed two harsh stops entirely.
- On current deployed route `00001429--32d16f6f48`, shadow produced comfort candidates on some gentle/leapfrog-ish events, but all three actual harsh events had no shadow decisions in the event window.
- Verdict: shadow mode is useful as observability, but not ready for command authority. The current sampling scope is too narrow because it only runs once `LongControl` is already in `stopping`.

Implemented immediately after the 2026-05-16 validation:

- shadow sampling now also covers low-speed stop-intent / braking-like PID windows, including Force Coast, explicit target approach/carry, low-speed braking command, and close stopped-lead approach cases.
- `run_stopping_cycle.py --analyze` now runs targeted `rlog.zst` shadow review by default after qlog stop analysis.
- `append_analysis_report.py` includes the shadow verdict, event coverage, harsh-event coverage, unsafe-candidate count, and shadow artifact links when `shadow_summary.json` exists.

Next validation/code direction:

- collect routes with the broader shadow scope,
- compare `stopping_shadow` decisions against actual force spikes, leapfrog/rebound, and final lead gap,
- require clean shadow verdicts on harsh events before considering any learned command authority.

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
