# Motion estimator consolidation (cycle-11 design)

## Why
The stopping service's laws are clean (min() of demand lanes + one jerk limiter; one anchored
glide). What has accreted is the MOTION-EVIDENCE machinery: ~10 interacting predicates spread
across the monitor and RAMP branch (hover, roll, displacement deficit, dither immunity,
queue-creep suppression, entry grace, ramp grace + evidence termination, gap-trust freezing).
Three of the last five defects were these predicates' evidence baselines going stale
(dither false-arrest, entry-grace poisoning, crawl blindness). This is the conditional-tree
pattern regrowing. Consolidation target: ONE estimator answering ONE question — "what is the
car actually doing?" — with every evidence rule named, and the service reading a single enum.

## The abstraction
`MotionEstimator` lives in stop_context.py (it is signal conditioning). Inputs per frame:
v_meas, standstill flag, conditioned d_gap + gap_trust, lead_v, dt, under_control flag
(service active) and stop intent. Output on StopSignals:

    motion: MOVING | FINISHING | STOPPED | CREEPING | ROLLING
    displacement_deficit: float   (trusted-gap consumption since the wheel-stop reference)

State semantics and the SINGLE evidence rule set:
- MOVING: default while rolling. Evidence baselines (v_min, hover window) accumulate ONLY
  while under_control and past the entry grace — the rule that fixes the whole
  "poisoned baseline" bug class in one place (entry mid-motion, aborted go, re-entry).
- FINISHING: wheel-stop latched, residual v >= MON_V_MIN, within the finish window. Exits to
  STOPPED (v < MON_V_MIN or window elapsed with no displacement) or ROLLING (v rise above the
  post-latch minimum — the grace-yields-to-evidence rule, now a state transition).
- STOPPED: at rest; velocity dither (readings 0.03-0.05, zero displacement) stays STOPPED —
  dither immunity is the state definition, not a bolt-on.
- CREEPING: displacement_deficit > threshold while readings are low (includes sub-quantization
  crawls), or pre-latch walking-pace hover with stop intent and a non-receding lead
  (the queue-creep gate becomes the ONE suppression input: lead receding => stays MOVING).
- ROLLING: v risen above the applicable baseline (post-latch min or post-grace running min).

Consumers become table lookups:
- Monitor floor policy: arm/escalate iff motion in (CREEPING, ROLLING); ratchet semantics
  unchanged (floor persists until RELEASE/INACTIVE).
- RAMP build: gentle-finish while FINISHING; A_HOLD/A_HOLD_SECURE build on STOPPED;
  fast arrest on CREEPING/ROLLING (unchanged depths).
- Crawl arrest: CREEPING (the displacement lane's home).

## Invariants (unchanged, now enforced in one place)
1. Evidence only accumulates under control (no pre-entry/pre-latch poisoning).
2. Dither (zero net displacement) is never motion.
3. Displacement is trusted-gap only; references freeze on distrust, re-base on re-trust,
   re-base upward only on genuine departure (> RELEASE_GAP_GROW_M).
4. Deepen-only outputs; the estimator never shallows anything by itself.

## Non-goals
No change to demand lanes, anchor valve, phase laws, thresholds, or depths. Every existing
fixture must pass unchanged — the fixtures ARE the behavior contract; this moves where the
decisions are computed, not what they decide.

## REVISION after sol red-team (2026-07-18, adopted)
The five-state enum is REJECTED (sol xhigh review, 9 findings, all arbitrated as valid except
naming nits): it collapses evidence channels whose thresholds, clocks, epochs, and trust flavors
intentionally differ (ramp +0.02 fast-build vs monitor +0.06 arm; entry-graced vs latch-immediate
epochs; measured/held/queue trust distinctions; ratchet pause-vs-reset). Adopted instead:

**StandstillEvidence ledger, service-owned.** One object inside stopping_service.py that owns
EVERY motion-evidence baseline, window, reference, and epoch. Advanced by the service after
current-frame phase transitions (no one-frame edge shift, no second phase authority in
StopContext); reset synchronously with the service episode. Emits named orthogonal fields --
hover, entry_roll, finish_roll, post_latch_escape, decreasing, queue_suppressed(+deadline),
crawl_deficit(+reference validity), genuinely_stopped (dither-immune: readings within the
quantization band, zero crawl deficit, sustained) -- which the monitor/RAMP laws consume as
read-only inputs. All baseline MUTATIONS live in the ledger; the laws keep their thresholds.
Two epochs stay explicit: entry-graced (monitor history) and latch-immediate (_ramp_v_min,
crawl reference). Behavior-preserving: every existing fixture passes unchanged; new fixtures
per red-team finding 8 (fast-finish arrest with ratchet unarmed; same-frame retrust/deficit
rebase ordering).
