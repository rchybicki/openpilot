# Full-planner stopping audit — 2026-09-12

The deployed `7c861b7` service remains the supported runtime candidate. Comparing it
against the previous service in a further 3,888 paired stress cases with recorded vehicle
settings, plus 108 scenarios with the native planner and controller, introduces no new
3 m floor crossings, incomplete stops, or creep. These are tests of hypothetical plants, not proof
of real-car performance or superiority to the driver's marked stops.

The whole-approach governor remains OFF. More persistent stop tracking reduces repeated
entries, but exposes excessive braking from the current demand calculation. Substituting
the whole-approach output into the native planner/controller loop also worsens comfort
in many otherwise successful scenarios. No further runtime change is made in this audit.

## Actual vehicle settings

The original low-speed stress sweep uses dummy CP and an always-asserted `shouldStop`.
It tests the stopping state, without a natural arrival through the planner/PID state.
This audit keeps that test and adds two distinct checks:

1. Repeat its 1,944 cases for each observation mode with recorded CP and compatible
   recorded toggles. Load both service implementations from pinned Git revisions.
2. Run production `LongitudinalPlanner` with both native MPC solvers, `LongControl`,
   `FrogPilotFollowing`, and `FrogPilotAcceleration` in a closed loop. Read CP and the
   serialized `frogpilotPlan.frogpilotToggles` from route `000020c1--8f82c447fd`.

Recorded settings include zero longitudinal PID gains, `stopAccel=-2`,
`startingState=True`, `vEgoStopping=.5`, `startAccel≈.7`, human following enabled,
short/long distance factors `0/1.8`, lead probability threshold `.35`, and longitudinal
delay `.5`. The standard-personality planner weights are `200/100/5`, danger factor
`.75`, and following time `.85 s` at 6–10 m/s. Production helpers calculate the weights,
following time, and acceleration limits in the native experiment.

The native fixture uses the recorded model's direct-action semantics:
`shouldStop = v_ego < .3 and raw_desired_accel < .1`, then production `smooth_value`
with a `.3 s` time constant. A stop somewhere in the 10 s trajectory is not this flag.
Coherent lead position/velocity forecasts supply the enabled human-following branch.

Initial exploratory outputs with a horizon-based stop flag or dummy planner weights
are superseded by `native_planner_20260912/closed_loop.json`.

## Paired results

The previous service comes from `f1954d2` (runtime `761f4c`); the deployed service comes
from `7c861b7`. All other control code is identical, at checkout `331088c`.

| Recorded-CP low-speed sweep | Ideal observation | Wheel observation |
|---|---:|---:|
| Paired cases | 1,944 | 1,944 |
| New floor crossing / incomplete stop / creep | 0 / 0 / 0 | 0 / 0 / 0 |
| Existing floor crossings, both services | 149 | 148 |
| Existing incomplete stops, both services | 7 | 7 |
| Worst minimum-gap change | −0.34964 m | −0.39777 m |
| Worst absolute 300 ms jerk change | +0.51443 m/s³ | +0.68141 m/s³ |

No extra intrusion occurs in cases where the previous service already crosses the
floor. The absolute minimum remains 0.985263 m. Comfort is not uniformly better in
this cold, always-stopping-state test.

The native loop covers nine stopped/braking-lead scenarios, three actuator plants,
two synthetic model forecasts, and ideal/wheel observations: **108 scenarios per
arm**. It compares previous service, deployed service, and an offline substitution of
the whole-approach shadow output. All native solver status checks pass.

| Native-loop result | Previous | Deployed | Whole-approach substitution |
|---|---:|---:|---:|
| Floor crossings | 36 | 36 | 28 |
| Incomplete stops | 33 | 33 | 24 |
| Creep | 0 | 0 | 0 |
| Absolute minimum gap | 0.96104 m | 0.96104 m | 0.95427 m |

The deliberately adverse plants expose existing failures. Zero **new** failures must
not be described as all scenarios passing. The whole-approach arm rescues some weak-plant
stops, but does not provide a general comfort improvement.

There are 72 comparisons where both relevant arms stop, stay at least 3 m away, and do
not creep. Each pair is selected separately; the resulting indices happen to be identical
because all 36 weak-plant cases already fail in the deployed arm. The per-case indices
and metrics are stored in `vehicle_params_stress_20260912/native_audit.json`; the
zero-capped jerk recomputation is in `native_audit_details.json`. Within those comparisons:

| Final 2.5 m/s metric | Deployed versus previous | Whole approach versus deployed |
|---|---:|---:|
| Negative jerk improves / worsens / unchanged | 28 / 0 / 44 | 30 / 26 / 16 |
| Minimum acceleration becomes shallower / deeper / unchanged | 29 / 1 / 42 | 24 / 36 / 12 |
| Largest extra braking at the minimum | 0.00228 m/s² | 0.79812 m/s² |

Negative jerk here is capped at zero, so gentler positive jerk during pure release
does not count as a braking-jerk improvement. Both endpoints must be within the final
2.5 m/s band. Including intervals that cross into that band changes the whole-approach
counts to 29 improvements, 31 regressions, and 12 unchanged cases.

In the native loop, for deployed versus previous, the worst gap reduction is 0.19130 m
and the longest added stopping time is 0.21 s. Whole-stop negative jerk never worsens; the largest
positive-jerk increase is 0.05152 m/s³. These describe simulated motion only.

Across all 108 cases, whole approach versus deployed worsens negative jerk by up to
1.62877 m/s³ and positive jerk by 6.06310 m/s³; these extremes include failed stops and
the physical rest clamp. Its worst gap reduction is 0.30890 m. The artifact's empty
`regressions` list records only new floor, incomplete-stop, and creep failures; it does
not mean zero comfort regressions or unchanged distance margins.

For example, from 6 m/s and 30 m behind a stopped lead on the nominal plant, the whole
approach reaches service takeover at a 12.05 m gap rather than 7.40 m. Its early peak
braking is stronger, and stopping takes 11.41 s instead of 7.44 s. From 10 m/s and 50 m,
the same approach can produce a long crawl. `native_approaches.png` shows both cases.
The remaining task is a better arrival and finishing trajectory, not simply earlier
ownership of the existing curve.

## Rejected changes and causal checks

- An implicit lag-aware reference produced a new 3 m floor crossing in its first 81
  hard comparisons: 3.0309 → 2.9668 m. Rejected.
- Seeding takeover from `min(legacy, previous wire)` produced two new floor crossings
  in 3,888 comparisons, and slightly deepened both latest marked entry minima. Rejected.
- Accepting conservative outward gap holds reduced shadow entries from 19 to 15.
  Restricting that acceptance to continuation reduced them to 14; suspending demand
  through rejected inward readings reduced them to 10. All retain a common-frame
  extra-braking case of 1.5833 m/s². Rejected under the current demand law.

The baseline also has 19 releases (14 gap, four reversal, one disengagement); entry and
release totals coincide in this replay. These are separate counters.

In that moving-lead case, the raw comfort law requests +0.5 m/s², while the predictive
lead-braking lane requests −1.434653 m/s², equal to the final shadow output. The previous
gap release had bypassed this lane. Lower tracking churn is therefore not a comfort
result, and is not permission to remove the safety lane. The stricter variants pass
128 original gate checks, 116 hold/hazard checks, and three shadow-publication checks.

An exact 53,346-frame replay also reconstructs the two marked negative-jerk windows.
The deployed code holds or releases braking within those windows; there is no additional
deepening, terminal transition, recovery action, safety binding, or untrusted geometry
inside them. However, segment 5 still ramps from −0.9421 to −1.5921 m/s² in about .25 s
**before** its window. Delayed vehicle response can still produce a physical dip.
Flat command at the measured peak does not prove that the deployed car's dip is gone.
The small coast estimate relieves braking locally and is not the source of deepening.

## Entry-conditioned trajectory experiment

A separate 17-line prototype fits a cubic speed trajectory to entry speed, measured
acceleration, and remaining distance, with zero terminal speed and acceleration. The
smaller positive duration root solves `a0*T² + 6*v0*T - 12*D = 0`. Both marked entry
states admit a monotone curve, with initial jerk −0.0734 and +0.3733 m/s³. This directly
tests whether matching the arrival state avoids the fixed profile's pursuit transient.

Four initial native comparisons improve the final approach substantially. The corrected
108-scenario native loop, now using the Hyundai standstill threshold and corrected stop
timestamp, also introduces no new floor crossing, incomplete stop, or creep.

The broader 3,888-case low-speed comparison **rejects the prototype**: 16 new floor
crossings, including weak/delayed-response variants of both real-like entry states.
The largest gap reduction is 2.53125 m. An otherwise passing native approach grid is
therefore insufficient to qualify a new service law.

Independent review also finds structural faults:

- The curve can request more than the existing net comfort authority. This changes
  safety-rate attribution even though the safety formulas themselves are unchanged.
- The entry clock and target survive lead changes, dropout, and aborted release.
  Speed feedback cannot correct accumulated position error or a changed stop target.
- Near-zero measured acceleration or the cubic feasibility boundary can abruptly
  select a different law. This is a controller admission discontinuity, not a failure
  of the cubic's endpoint equations.

The first expanded test set passes 306 of 307 existing tests, while the baseline passes
307. The failure is a moving-lead braking-response requirement. The prototype is not
merged. Its complete source, analytical checks, traces, and stress results are in
`entry_curve_20260912/`; independent checks are in `vehicle_params_stress_20260912/`.

A distance-driven variant removes the clock, projects entry acceleration continuously
into the cubic's admissible family, restores the existing net authority limits, and
clears its state in RELEASE. It recovers six of the original 16 new floor failures;
ten remain. No full sweep follows a failed subset.

The weak-response trace shows why: the curve captures measured acceleration while a
stronger brake command is still in flight. It initially releases that command. Capturing
the stronger of measured braking and coast-adjusted wire braking recovers the remaining
stationary cases, but six new floor failures remain across three braking-lead cases and
both observation modes. The worst remaining gap changes from 3.00885 to 2.75392 m.
For a moving lead, advancing an absolute-speed reference by relative gap scales the
entry feedforward by `(v - v_lead) / v`; it no longer matches the captured braking.
These are structural failures, not grounds for lead-specific tuning.

Independent review also finds a shared rate-arbitration defect. At `v=1`, gap `4.5`,
measured acceleration `-2.5`, previous wire `-.3`, and planner demand `-1.5`, the deployed
phase target is `-.3`: the planner binds and the next command is `-.38` at `J_SAFE`.
The spatial prototype requests a stronger phase target of `-2.5`, making the same planner
lane nonbinding; the next command is only `-.325` at `J_DOWN`. A stronger comfort request
can therefore slow the response to an unchanged safety demand. Keeping safety formulas
unchanged does not preserve their command-rate behavior. Any replacement trajectory
must pass this interaction as well as the distance and comfort checks.

An isolated alternative limits the existing comfort and effective safety demands from
the same previous wire, then selects the stronger braking command. A safety demand below
the previous wire receives `J_SAFE`, whether or not it is deeper than the comfort target.
It fixes all nine
recorded rate-masking counterexamples and passes all 307 unchanged control tests.
The existing attributed-relief target is reconstructed exactly; its eligibility, dwell,
and release bounds are unchanged on identical input and state. However, the independent
safety rate can reduce the relief actually returned on the wire. Existing target-binding
telemetry would not fully describe the new rate decision.

The 53,346-frame latest-route replay gives no comfort reason to ship that change alone:
45 commands deepen, none become shallower, and ownership is unchanged. All differences
occur in bad segment 5, where the first-second minimum changes from -1.592131 to
-1.593021 m/s². The maximum pointwise extra braking is 0.016121 m/s², caused by the lag
barrier at entry. The overall negative command-jerk extreme improves slightly, so this
is not evidence of a worse global jerk peak either. A planner-comfort demand can still
be an admitted safety lane; granting every such shortfall an urgent rate is a policy
change that the passing counterexamples do not validate. These observed command deltas
are not authority bounds. The replay does not separately inventory internal latch-state
differences, so it is not a complete lifetime comparison. An independent rerun reproduced
all 36 arm/case counterexample outputs exactly and verified the unchanged test hashes.

A fourth curve uses captured ego travel, capped by the current trusted gap, rather than
advancing solely from relative gap. Its budget starts with no entry-frame travel debit;
subsequent updates subtract trapezoidal observed travel. Feedforward uses the derivative
of the active distance bound. This fixes all 16 development failures without adding the
independent-rate modification. The frozen source then fails the full 3,888-case comparison: **17 new floor
crossings**, despite recovering 28 existing failures. Aggregate floor failures fall from
297 to 286 and hide these regressions. There are no new incomplete stops or creep.
The largest minimum-gap reduction is 2.475006 m; negative jerk worsens by up to 2.491935
m/s³ and positive jerk by 6.662242 m/s³. These extrema span different cases and include
failed stops; the physical rest clamp contributes to positive jerk.

One stationary-lead case starts at 2.4 m/s, an 11 m gap, and a -0.3 m/s² command, with
gain .7, lag .7 s, delay .3 s, and push +.45 m/s². Minimum gap falls from 3.52948 to
2.47455 m. Correcting the moving-lead coordinate therefore does not solve weak-response
entry. The candidate source hash is
`4d9b84dd9e58ed488edcfc4ecbaa99400d0a5a976c89092f5be42094e309e78a`;
all 3,888 baseline results reproduce exactly, and this source did not change during testing.

The corrected native 108-case test again introduces no new floor, incomplete-stop, or
creep failure, but comfort is mixed. Existing control tests pass 296 of 307. Six failures
cover dynamic behavior: two crawler requirements and four recovery-on/off gap differences.
Five pin phase values or change counts; the warm-takeover case also exposes slower response
to an admitted safety demand. Crawler rest gap grows from 4.7395 to 5.3648 m. Recovery
enabled/disabled gap differences within the candidate are 0.1577–0.2916 m, above the test's
0.10 m limit; these are not candidate-versus-deployed deltas. Direct-input checks also
reproduce stale curve state through a same-update
HOLD → RELEASE → APPROACH transition, and a finite-speed brake-release request when the
ego travel budget reaches zero. Those checks are lifecycle/join evidence, not simulated
collision claims. The reference derivative is singular near zero remaining distance at
positive speed, while the exhausted-budget fallback has no proven smooth join. Discrete
accepted gap changes also have no finite derivative; removal of a gap cap can restore
reference room up to the consumed ego budget. These bounds are not a complete moving-target
trajectory model.

All four curve variants are rejected. The 16 → 10 → 6 → 0 development sequence is not
independent acceptance evidence; the full comparison exposes the fourth variant's new
failures. No lead-specific condition or further parameter search was added to this family.

## Lower comfort rate with the deployed law

A final alternative keeps the deployed target equations and adds the independent safety
projection, while using the existing whole-approach `J_DOWN=.6 m/s³` for the governor's
pre-terminal approach. Terminal and hold rules remain unchanged; their capture inputs
and physical results can change. The projection acts on the post-attributed-relief
safety minimum, so it does not silently restore the released planner demand.

This fails the selected 18-case regression set: the 16 prior failure cells plus stationary
case 39 under both observation modes. There are **12 new floor crossings**, no new
incomplete stop or creep; this fraction is not an estimate of normal-drive failure frequency.
The artifact field `remaining_new_floor` counts candidate gap below 3 m with deployed gap
at least 3 m. The stationary case above changes from 3.52948 to 2.38373 m. Negative physical
surrogate jerk worsens by up to 0.77750 m/s³ and positive jerk by 3.19657 m/s³. All 36
arm/case results reproduce exactly in an independent rerun. Slower requested comfort
braking therefore does not guarantee smoother motion or sufficient stopping margin,
even with the separate safety rate.
In case 39, terminal command capture deepens from -1.18069 to -3.21825 m/s² after the
earlier delay: unchanged terminal rules receive a substantially worse arrival state.

The prototype scopes its rate by governor mode, approach phase, and terminal latch; it
does not fully distinguish a missing-lead legacy fallback inside that phase. The failing
stationary cases have valid lead geometry, so that scope limitation does not explain
their failures. No scope patch, rate sweep, full grid, or recorded-route replay follows
this rejection. This prototype also remains outside the repository and device.

## Native planner reuse

A read-only source and route audit finds no evidence of a planner reset or state-lifetime
defect behind the two marked dips, so it supports no corrective reset change. It inspects
[`LongitudinalPlanner.update`](../../selfdrive/controls/lib/longitudinal_planner.py),
[`LongitudinalMpc`](../../selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py), and
`live_reference_probe_20260912/causal_results.md`. The native MPC already shifts its previous acceleration solution
and corrects its speed state from measured speed. Its predicted acceleration can differ
from measured acceleration by design; replacing it with `aEgo` is not an established fix.
The direct model action remains a separate minimum before the existing planner caps, so
changing MPC comfort alone need not change the final arrival demand. Existing route
evidence identifies the service's entry-reference mismatch, not a planner reset edge.

The native MPC is a possible integration point for a future arrival trajectory. Its
position/speed/acceleration model does not include actuator gain or delay states, however.
Reusing it does not remove the need to validate changed command-to-motion behavior.

## Evidence and remaining limits

Artifacts are under `~/.route_sync/corpus/radek_baseline_20260912/`:

- `vehicle_params_stress_20260912/`: pinned CP sweep, independent native-harness review,
  summary recomputation, and a full three-arm deterministic rerun.
- `native_planner_20260912/`: corrected `probe.py`, `evaluate.py`, full traces,
  summary, plot, device check, and evidence manifest.
- `live_reference_probe_20260912/`: rejected reference/seed changes and exact command
  component attribution with manifests.
- `hold_policy_20260912/continuation/`: isolated certificate variants, replay, hard-gate
  checks, and predictive-lane attribution.
- `entry_curve_20260912/`: rejected clock, distance, and in-flight capture prototypes,
  corrected standstill/timing fixtures, and paired results. `ego_distance_results.md`
  and `ego_distance_comparison.json` cover the separately frozen fourth prototype.
- `independent_rate_20260912/`: separate-rate prototype, nine counterexamples, unchanged
  control tests, and latest-route replay.
- `ego_distance_validation_20260912/`: independent corrected native 108-case comparison,
  all 307 unchanged tests for each arm, and exact failed-assertion diagnostics.
- `comfort_rate_20260912/`: rejected fixed-rate alternative, limiter-equivalence check,
  18 paired plant cases, and terminal-capture diagnostics.

The original 27-file audit manifest has SHA256
`c5ea96dc7c009c030dcd0884b5a9de78cab52c2f52ff315c53c6bff296b58e1f`.
It includes native solver binaries and excludes the later entry-curve experiments.
The three entry-curve experiments have a separate manifest SHA256
`ddee405f359c4ea3366bc4dc1f02e103cba4c6257d12fa87018c3c0a681f3f31`.
The fourth prototype's manifest SHA256 is
`dcbce7df3f58a8b7546506fb0b3dd3c540eef399fd09690b6e870622e2611449`.

The native fixture starts with seeded PID state and zero wire, not a warmed controller.
It uses synchronized fresh radar at 20 Hz and control at 100 Hz. The plant is an
uncalibrated lag/gain/delay/push model. Wheel quantization is hypothetical; its observer
uses the native KF recurrence. Brake actuator state persists at rest, while physical
acceleration reaches zero. This stop constraint contributes to positive jerk.
This initial native run uses `raw < .005` for standstill, which differs from Hyundai's
wheel-based threshold of 0.375 km/h. Its terminal timing therefore is not fully native.
The reported first-stop timestamp is one 10 ms step early; paired duration differences
are unchanged. A new candidate must also be checked with the actual standstill rule.

The whole-approach experiment substitutes a finite shadow demand, including release,
after planner update and clips it to the control acceleration bounds. It preserves the
independent MPC trajectory input and solver/filter state. It does not implement the full
live hazard contract, sensor timing, or a production consumer, so it cannot authorize
activation.

Passive device checks at about 17:50 UTC found checkout `331088c`, the exact `7c861b7`
service hash, unchanged stopping flags, manager processes, and off-road state. The
latest route is still `000020c1--8f82c447fd--30`; there is no new recorded drive after
the deployed correction. A second passive file/hash check at 18:25 UTC confirmed the
same checkout, service, flags, off-road state, and latest route segment. No device update
was issued for this audit. Only the 11
explicitly marked manual stops remain the personal baseline.
