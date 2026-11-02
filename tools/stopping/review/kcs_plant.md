# KCS1 offline plant: V2 validation

Offline evaluation only. No runtime changes, device contact, controller candidate, or safety claim.
The first-run outputs are retained. New outputs must stay under
`~/.route_sync/corpus/stopping_decision_20260926/sim/` and have a `v2_` filename prefix.

Constraints: Skip and report speculative work. Reuse existing code, then stdlib/platform/DB
constraints, then installed dependencies; add none. Only then write minimal code. No
single-implementation interfaces, fixed-value config, or one-caller helpers. Preserve
trust-boundary validation, data-loss handling, and security. Never fit per-stop residuals;
keep failed and incomplete episodes visible; keep fit and held-out data separate;
do not tune the plant on natural stops.

## Reproduce

From the repository root, activate `.venv` and set `PYTHONDONTWRITEBYTECODE=1`:

```sh
python -m tools.stopping.review.plant_sim --v2-gate
python -m tools.stopping.review.plant_sim --census
ruff check --no-cache tools/stopping/review/{kcs_plant,plant_data,plant_sim,test_kcs_plant}.py
pytest --noconftest -n 0 -p no:cacheprovider tools/stopping/review/test_kcs_plant.py \
  --basetemp="$HOME/.route_sync/corpus/stopping_decision_20260926/sim/v2_pytest_tmp"
```

`v2_GATE_SPEC.md` is the preregistered evaluation contract; its SHA256 is included in
`v2_gate.json`. `v2_selection.json` is written before natural validation. Repeated V2 runs
replace their V2 outputs, never the first-run artifacts. The pulse-boundary test uses the
local frozen reps 1–4 calibration fixture. No automatic formatter is used.

## Plant changes and limitations

- Apply `-9.81 * grade_percent / 100` in both engaged and brake-off regimes. The old
  engaged path implicitly assumed full grade compensation. KCS uses its recorded grade;
  natural grade remains unknown (nominal zero). Measured gain and grade remain confounded:
  the change is a required physical assumption correction, not a new identification fit.
- Seed physical motion once, at the beginning of at least two seconds of recorded SCC12
  warm-up. Roll speed, lag, delay, and regime freely through the entry. Reset only the
  distance origin at entry. No speed or observation reseed at the crossing.
- At pulse-cache boundaries, divide displacement by the actual available time interval.
  The old fixed `.2` denominator halved the initial speed when only `.1` s was available;
  speed-forced warm-up masked that bug. Boundary speed is an interval mean, with that
  remaining timing uncertainty, not an independently observed instantaneous speed.
- Remove the zero-acceleration latch on StopReq/deep requests. The forward-only speed
  clamp still omits reverse motion and static friction. **Every stationary outcome is
  UNVALIDATED**, including creep, relaunch, hold security, rollback and breakaway. A
  downhill unit test proves the absence of a latch, not a calibrated breakaway law.

Existing training-based gain, SCC14 slew, delay/lag, release history, shift dip, additive
loss/push and wheel/KF observation assumptions are otherwise retained. Parameter sources
are in `parameter_manifest()` and `Plant.__doc__`; original documentation is preserved in
`sim/v2_before_kcs_plant.md`. Nothing is fitted on held-out or natural episodes.

## Regimes and diagnosis

`regime_diagnosis()` partitions each original held-out/natural episode three ways:
plant OFF state; recorded TCS13 BrakeLight; and recorded wire-history hysteresis.
Brake lights are an indicator, not proof of friction-brake torque (A5/A6 lights stay off
while their wire remains engaged by the requested definition). The wire episode class
uses the frozen -0.45 threshold below 2.6 m/s. These definitions are deliberately distinct.

`v2_DIAGNOSIS.md` contains every original episode and partition; its JSON includes
signed/absolute acceleration-error integrals, frame quantiles, and KCS-only diagnostic
perturbations. Distance attribution is the integral of speed error within each regime,
not causal attribution: excess speed persists after rebuild. Tail distance after the
recorded stop ends at the first predicted stop, excluding later departures. Sampling,
pulse-distance endpoints, and the entry integration step mean these partitions need
not sum exactly to final stopped-distance error. The original source snapshots are
`v2_before_*.py`; the diagnosis uses that original model, not the corrected V2 model.

Recorded SCC12 directly drives gate A. Neither a planner replay nor lead reconstruction
can cause its physical-distance error. Lead reconstruction affects rest-gap error only;
the observation model affects aEgo error but does not feed back into gate A motion.
There is evidence of a brake-off regime mismatch and carry-over after rebuild, but no
unique identification of release loss versus gain/delay versus grade from these stops.

## Frozen evaluation and stopping rule

Fit gain on reps 1–4 only; choose among the 14 existing screening cells on valid KCS
training stops only. Score KCS reps 5–6, natural route 2086, and separate 2129. Keep
20bf/20ff/2100/2102 as candidate-entry routes; do not score their corrected plant outcomes.
All 28 saved stops can take the recorded-motion gate-2 twin check, which is independent
of physical simulation. It uses actual cached publication timestamps, `request_time`,
`observe_accel_request`, MAE <=.01, and separate release/rebuild counts (including a
release that never rebuilds). Subscription timing remains approximate; historical source
commits differ from HEAD. No source overlay is loaded to force agreement.

Gate A retains >=100 clean route-grouped held-out stops and all frozen acceleration,
distance/rest-gap, rest-time and newly explicit event timing/count thresholds. Missing
metrics, missing cohorts and incomplete stops cannot pass. BrakeLight transitions are
not smoothed; counts and timing therefore expose regime mismatch rather than hiding it.
A passing small numerical cohort cannot waive the count requirement.

The census scans local 2031–2128 rlogs, sorts relevant publications by monotonic timestamp,
preserves cross-segment history, rejects stale/missing/driver-controlled windows, and
prevents reusing a pre-stop crossing. It lists all standstill detections and rejected
windows plus corrupt files. Counts are conservative eligibility, not pulse-validated
rollouts or proof that historically exposed routes are pristine held-out evidence.

V2 fails both regime gates. Stop after step 3: no wire-invariant checker or synthetic
entry generator was built; no simulated HEAD/candidate comparison was run. HEAD's
comparison uses recordings. Their implementation is meaningful only after the engaged
gate passes; brake-off simulation additionally needs its own passing gate.
