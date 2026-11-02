# Stopping Redesign Rationale

Why the stopping stack was redesigned in June 2026, what was decided, and where the program
stands. Companion docs: [architecture.md](architecture.md) (what was built),
[eval.md](eval.md) (how it is judged).

## 1. The June 2026 review verdict

A six-agent deep review (2026-06-09) of the stopping-behavior project concluded the prior
approach was wrong on three levels:

1. **Wrong layer (the runtime).** `stopping_controller.py` had grown into a hand-trained decision
   forest: ~130 behavioral branches, ~3,500 tuned constants, 406 interp tables. MPC output is
   discarded in stopping state (`output_accel = min(output_accel, −0.1)` seed); there was no
   setpoint, no trajectory, no error signal — every new failure mode got a new lane.
2. **Unmeasurable gates (the eval).** The offline pipeline could not measure the 10–20% effects
   it adjudicated: the plant model was a 7-coefficient linear AR(1) fit on 862 rows (~86 s of
   driving), validated in-sample only; teacher, selector, and scorer shared the same plant (triple
   circularity); n = 19–31 gates gave p ≈ 0.3–0.6; the headline average score was dominated by a
   single outlier event (36.2 of avg 2.389); gate definitions changed ≥ 14 times in 4 months.
3. **Root causes mostly outside the stop-tail layer.** The fork removed stock's 1 s `should_stop`
   lookahead (no hysteresis at the source); StopReq is suppressed until vEgo < 0.01 so the SCC's
   managed final stop never runs; radard mutates published `dRel` by IncreasedStoppedDistance with
   a partial compensation web; SCC14 jerk limits are static.

The ML attempts (inverse policy, KNN profile selector, direct authority, shadow oracle) failed
because they were trained and judged inside the unstable plant model, not because ML cannot work
here. Shadow mode structurally cannot validate a selector — the counterfactual is never driven.
The one corroborated real win of the old process: leapfrog reduction (the 0-leapfrog baseline),
which the redesign treats as a hard floor.

## 2. What was decided

The implementation contract (FINAL_SPEC, 2026-06-09) adopted the risk-first / maximum-continuity
design: smallest on-vehicle delta per commit, every change revertible via one in-code constant,
and the new controller must **reproduce the forest's envelope** on the recorded corpus before any
improvement is attempted. Binding principles:

- **Dark-launch everything**; CAN semantics change nothing on day one.
- **One owner per signal** (the stop-target arbiter); constants over Params.
- **The sim develops, the measurement promotes** — the AR(1) plant sim is a promotion gate exactly
  once (the dual-plant similarity gate), then demoted to smoke; tuning promotes only on measured
  paired statistics with mandatory minimum-detectable-effect reporting.
- **Phases:** (0) fix source signals (shouldStop hysteresis, dRel honesty, StopReq semantics,
  dynamic SCC14 jerk — the latter two staged via on-vehicle protocols); (1) replace the lane
  forest with arbiter + jerk-limited distance-feedback trajectory + plant-referenced tracker with
  disturbance estimator (~39 named parameters, every one with preserve-group provenance); (2)
  rebuild eval on rlogs at 100 Hz with a full-corpus event store, frozen scoring, paired stats;
  (3) ML only where ground truth exists.
- Vehicle knowledge was preserved as 17 itemized parameter groups
  ([parameters.md](parameters.md)); the forest's lane composition, teacher lanes, shadow residual
  templates, and the predictive rebound-risk regression were deliberately dropped (the arrest
  AUTHORITY — depth and rate — was kept).
- Key physics finding pinned forever: the identified plant's DC gain collapses and inverts sign
  near v ≈ 0.21 m/s. No code may ever divide by the plant gain; no model-inverse feedforward in
  any phase.

## 3. Where the program stands (2026-06-10)

Phases 0–1 are **implemented and merged dark**; the eval stack is rebuilt and live. The spec
section-7.6 similarity gate ran on the dual-plant deck and **did not pass** (details and numbers:
[eval.md](eval.md) section 6). `USE_STOPPING_V2` remains `False`; the forest is the active
controller; zero CAN delta has shipped.

The open blockers, per the gate's triage:

- Frozen-plant rollout / time-to-standstill / one leapfrog stratum: authority-collapse stall
  artifacts of the inverted-gain plant that no provenance-legal parameter move can fix —
  escalated to the operator: either re-scope those Tier-1 rows or supply per-event triage.
- Refit-plant hold_gap p95: event-specific stop-position divergences that map to no named
  parameter; fixing by tuning would be free tuning against the sim, which the gate forbids.
- 81 / 69 Tier-2 flagged events need honest per-event written classification.

The deterministic exit criterion stands: Tier 1 green on both plants, zero open class-C events,
triage table committed with the flip. Until then every cleanup deletion stays scheduled
([architecture.md](architecture.md) section 5) and the forest stays one constant away from the
new stack in either direction.
