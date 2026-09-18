# New-route stopping cycle, 2026-09-18

The new automatic cohort has substantially higher median negative wheel-jerk than
Radek's marked manual reference. No additional driving change qualifies in this
cycle. The supported runtime remains `7c861b7`,
present in device/source revision `f4c3eed94fd56b570ebeb0deb8fae081624f270c`.
The controller trials below remain offline. None was deployed.

## Data and reference

The shared refresher discovered 57 routes after `000020c1`, through
`000020fa--32a67f8d0c`. All 940 finalized rlogs (9.96 GiB) are downloaded and
all 57 route packets pass provenance and recomputed-census checks. They contain
466 observed stops, all with init revision `f4c3eed94fd56b570ebeb0deb8fae081624f270c`.
`cycle_summary.json` records exact coverage and source hashes.
Locked segments are excluded. Frozen source copies, sizes, SHA256 hashes, init
revisions, and per-window control classifications accompany the analysis.
The canonical refresh exits successfully: 2,000 files downloaded, including all
940 new qlogs and 940 new rlogs, with zero failures or report errors. Ordinary
cache retention remains within its 30 GiB limit. All 196 archived reference rlogs
are rehashed after retention and remain byte-exact.

The latest three routes contain 19 engaged final-2.5-m/s windows: F8 has 13,
F9 has three, and FA has three. The sole new bookmark is F8 segment 27,
event `000020f8--3495298f77@40846112178200`. Its label is **mediocre**, as stated
by the user. Other new stops have no inferred rating or driver identity.

The personal reference remains the same 11 explicitly attributed manual stops.
Their original labels and packets are unchanged. Before ordinary-cache retention,
all 196 source rlogs behind the earlier packets were copied outside the cache and
hashed. Rebased reference packets change source paths only; their relocation
manifest records both original and relocated hashes.

The new `tools/stopping/review/cycle_review.py` retains every observed stop,
including slow starts and mixed-control windows that the older v2 comparator
omits. It validates source hashes and packet identity, reuses existing phase
boundaries and signed-jerk calculations, and retains unavailable-window reasons.
It never turns an unlabelled manual stop into a Radek reference. At the final
2.5 m/s band, 354 stops classify as manual, 19 as engaged, one as mixed, and 92
as unknown, including missing crossings. All 19 engaged terminal windows have
the same identities as the 19 final-2.5 windows. No additional engaged stop is
lost merely because it starts below 2.5 m/s. Mixed handoffs are reviewed separately.

The three mixed terminal windows do not reveal an additional low-speed
automatic-to-manual intervention. CF and DE remain disengaged; brake release
leaves a mixture of manual and unclassified samples. In the FA window, cruise
is canceled 20.535 s before rest while the accelerator overrides control; manual
braking starts 16.559 s before rest. Control is re-enabled near rest while the
brake remains pressed. The earlier cancellation stays in the evidence: its
reason, driver identity, and relationship to prior automatic behavior are unknown.

| Observed final 2.5 m/s descriptor, median | Marked manual, n=11 | Latest automatic, n=19 |
|---|---:|---:|
| Negative 300 ms wheel-jerk magnitude | 0.415 m/s³ | 1.236 m/s³ |
| Positive 300 ms wheel-jerk magnitude | 1.107 m/s³ | 1.115 m/s³ |
| Acceleration at band entry | −1.009 m/s² | −0.786 m/s² |
| Time to filtered rest | 3.299 s | 4.561 s |
| Wheel distance to filtered rest | 3.248 m | 4.758 m |

Seventeen of the 19 automatic negative-jerk magnitudes exceed the manual maximum,
0.794 m/s³. Below 0.5 m/s the median negative magnitude is zero in the manual
reference and 0.172 m/s³ in these automatic stops. The final-2.5 positive-jerk
medians are similar; one signed metric is not a complete comfort score.

These are different, unmatched traffic situations. Acceleration is a wheel-filter
state, and filtered rest is not an independently measured physical rest instant.
The windows end 0.5 s after filtered rest; both jerk endpoints must lie inside
their own valid window. Wider manual windows must not be pooled indiscriminately:
only eight of the 11 final-5-m/s windows are fully manual, and ten last-30-s
windows are mixed. None of these results establishes physical safety or superiority.

## More than the bookmark

Exact-current control replays and original CAN distinguish several mechanisms:

- **FA segment 5, unbookmarked:** the service takes ownership at 2.253 m/s,
  measured acceleration −0.837 m/s², and gap 7.100 m. Requested braking then
  rises from about −0.95 to −1.90 m/s² in 0.38 s. CAN transmission and echoes
  confirm the request; wheel, ESP, and pose signals show the later dip. The
  request ramp is phase-driven. Neither that ramp nor the later measured peak
  has binding safety or dropout. This does not isolate calibrated body-motion
  causality. LongControl still reports PID while the service owns its output.
- **FA segment 17:** the same service-entry mechanism produces a smaller pulse.
- **F8:** 11 of 13 final-band negative-jerk peak windows are service-owned
  APPROACH_GLIDE. The bookmarked segment 27 and segment 36 peak before service
  entry. Segment 63 also has a later release/rebrake; segment 29 has excessive
  terminal negative jerk. A service-entry correction cannot cover all of these.
- **F9 segment 9:** the planner weakens braking while a moving lead slows. The
  service is inactive through the weak-braking plateau, then enters after the
  lead qualifies as stopped. Its command-history mismatch is negligible there.

At 18 of the 19 service entries, ego is already faster than the nominal easing
profile permits even with the 0.45 s lag reserve removed. The median distance
deficit to that profile is 1.720 m. Removing the reserve is therefore not a
solution to the main mismatch, nor is the reserve proved to duplicate wheel
filtering: the wheel estimate is a current state, not a future state.

For FA segment 5, the remaining nominal-anchor distance is 2.800 m. Ideal
constant braking to a fixed anchor is −0.907 m/s²; the production measured-lag
calculation gives −0.941 m/s². Upstream requested about −0.9 m/s². The service
phase requests −2.033 m/s², dominated by tracking its lower reference speed.
The planner was already braking. This is a handoff between different trajectories,
not evidence that upstream simply forgot to brake. These kinematic calculations
do not identify actuator gain, delay, grade, or a safe replacement request.

## Offline alternatives and their limits

| Alternative | Finding | Decision |
|---|---|---|
| Stateless distance arc | Deepens FA5 entry and fails nine existing tests, including crawler behavior | Reject |
| Warm the whole StopContext from 4.5 m/s | Earlier entries deepen every changed moving command; stale geometry can cause a false takeover | Reject |
| Receding cubic ending at zero speed/acceleration | No real duration on 384/747 inspected moving-owned FA frames above 0.5 m/s | Reject before controller trial |
| Replace bypassed-planner history with previous final LongControl request | Confirms an input-contract defect but deepens many commands and worsens a native comfort case | Not a standalone comfort fix |
| Corrected history plus queued-brake forecast | Softens several entry requests, but worsens another native stop and credits gas-suppressed requests | Not qualified |
| Extrapolate recent measured acceleration instead | Forecast error worsens at 18/19 stops; largest optimistic-braking error reaches 1.698 m/s² | Reject before controller trial |
| Use calibrated pose acceleration for comfort prediction | Reduces FA5/FA17 entry request minima by 0.050/0.066 m/s², but leaves FA5 request jerk unchanged; ideal-sensor native test gives only small gains | Not a significant demonstrated improvement |
| Track measured acceleration toward the rest-anchor demand | Changes two command-semantic tests; a weak delayed plant then crosses the 3 m floor | Reject on physical regression |

The queued-brake trial passes 342 existing tests and introduces no new floor,
incomplete-stop, or creep failures in 3,888 paired hypothetical-plant cases.
The current arm still has 297 floor failures and 14 incomplete stops; the trial
has 291 and 14. These are not 3,888 successful stops. It improves the FA5-like
native moving negative jerk from −1.418 to −1.273 m/s³, but worsens FA17-like
from −0.273 to −0.342 and extends that stop from 5.36 to 6.32 s. Most of that
second change comes from the history correction. Native expansion stopped there.

More decisively, an override counterexample sends zero braking for 25 frames
while the prototype accumulates −0.578 m/s² of supposed pending braking. False
credit persists on accelerator release. That defect blocks deployment regardless
of its favorable entry results. Internal requests are not applied commands.

The producer audit also reproduces an odd-frame request `[0, −1, 0]` reported in
carOutput while SCC12 sends only `[0, 0]`. Holding the existing `accel_last_scc`
would correct that reporting error without changing CAN bytes. It does not alone
establish source age or uninterrupted authority: card republishes previous output
before apply, and can publish while apply is skipped. A separate, narrow contract
sketch and counterexamples are retained; no telemetry or schema change is applied.

The acceleration-tracking trial replaces the existing comfort forecast with
`max(raw_phase, last_cmd + dt * min((a_stop - measured_decel) / GOV_TAU, 0))`.
It adds no state or tuning constant and leaves raw safety, recovery, and terminal
code unchanged. Two existing command assertions fail, but their frozen-input
grade case and shallower crawler command do not alone establish worse motion;
the crawler's closure and handback conditions still pass.

The decisive rejection is a physical stress regression. With gain 0.7, lag 0.7 s,
delay 0.6 s, and push 0.45 m/s², minimum gap falls from 3.135 to 2.941 m. Early
braking reduction creates a response deficit. The unchanged barrier binds earlier
and the candidate later saturates braking, but it cannot recover the extra
0.194 m of travel. All three current stress results exactly reproduce the prior
reference. The trial stops at this first new floor failure; no gain tuning or
native expansion follows. Valid algebra and unchanged safety code do not by
themselves preserve the complete controller's behavior.

## Recheck the response model before trusting optimization

The earlier rejected fit already used automatic requests, not manual braking.
This cycle repeats its fixed model family on new CAN-backed automatic phases:
12 F8 training phases, three F9 development phases, and three FA held-out phases.
One of the 19 is excluded because the required two seconds of automatic command
history is not present. Cohort, split, family, and tolerances were frozen before
fitting; development and held-out data did not select parameters or delay.

The prediction gates remain speed RMSE ≤0.1 m/s, absolute travel error ≤0.2 m,
and absolute filtered-rest time error ≤0.3 s. Pass counts are 7/12, 3/3, and 2/3.
FA5 fails: rest is predicted 0.630 s early, when recorded filtered speed is still
0.178 m/s. The model also predicts its moving negative jerk as −1.103 m/s³,
versus measured −2.551 m/s³. It captures approximate dip timing while smoothing
away much of its amplitude. The failure is not confined to quantized rest samples.

Below 0.1 m/s, all admitted cohorts request only −0.70 or −0.69 m/s² before rest.
This does not identify the result of a substantially softer terminal request.
The model is useful diagnostic progress, but cannot rank a new stopping controller
or justify a better-than-driver claim. After inspecting these results, FA is
development evidence for later experiments; it cannot be called held out again.

A separate fixed-objective trial adds acceleration error to the existing speed
and distance residuals. Its normalization is calculated from F8 training alone;
the model family, bounds, delay search, and data split remain unchanged. There is
no weight sweep. Mean acceleration RMSE improves on F9 and FA, but gate counts
remain 7/12, 3/3, and 2/3. FA5's predicted negative jerk improves from −1.103 to
−1.262 m/s³, still far from −2.551, and predicted rest remains 0.580 s early.
All 36 original and revised prediction traces reproduce exactly. Changing the
objective does not resolve the model's main error. A larger learned model does
not remove the missing excitation or independent validation requirement.

A mechanism audit uses all 12 F8 training stops above the existing 1.2 m/s
gain-schedule breakpoint. Replaying measured raw wheel speed through the current
filter reproduces acceleration with only 0.0017–0.0044 m/s² RMSE. The dips also
appear in raw-speed differences and the pose proxy. In six complete pulses the
model minimum is 0.11–0.20 s late and too shallow. This is a response-shape error,
not mainly the wheel filter or the low-speed gain schedule.

Forty-four repeated request levels show different acceleration while tightening
and releasing, but the first-order model already produces such a loop. A wider
observed loop does not identify hydraulic hysteresis. Five of six complete pulses
recover above the model after the deeper dip, contradicting a fixed-target
explanation that only tightens faster and releases slower. Constant-request runs
last only 0.13–0.46 s; the decoded pressure channel is zero throughout these
automatic intervals. Neither a steady command-to-force map nor a physical
asymmetry is established.

The final predictive trial adds just one parameter: separate time constants for
upward and downward target error, both selected from the candidate's own latent
acceleration. Their ordering is free. All other features, bounds, training data,
objective weights, initialization, and delay search stay fixed. Equal time
constants reproduce the original model exactly; no future observation resets
the predicted state.

The selected fit improves the training gate count from 7/12 to 9/12, but F9
regresses from 3/3 to 0/3 and FA from 2/3 to 0/3. Training acceleration error also
slightly worsens despite the lower total fitting loss. All seven delay fits reach
the existing 80-iteration limit; convergence is not claimed. The selected
down/up constants are 0.394/0.034 s, which must not be read as identified brake
physics. Independent checks reproduce all 36 symmetric/asymmetric prediction
traces and the resulting metrics. This tested fit is rejected for predictive use;
no further architecture or optimizer search is used to rescue it in this cycle.

## Independent acceleration observation

The existing pose estimator offers a useful diagnostic: it fuses inertial and
camera motion without fusing wheel velocity. Applying the existing calibrated
pose transform gives a longitudinal acceleration estimate available to controlsd.
The executable frame transforms were checked; the stale schema comment alone
would give the wrong interpretation of the camera frame.

Across the three FA stop windows, pose and calibration are valid. Camera motion
arrives about 58 ms after capture. Estimated longitudinal acceleration standard
deviation is 0.33–0.44 m/s², and full covariance is not logged. This is an
estimator at the device origin, not calibrated passenger-comfort truth.
Its negative-jerk peak becomes available 66 ms before wheel acceleration in FA5,
but the advantage is not uniform across stops. Both signals still earn zero
forecast credit through 0.307 s after that entry, when the command is already
−1.728 m/s². A faster observation alone cannot remove the initial ramp.

An offline prototype changes only the comfort forecast's acceleration input.
Its current arm exactly reproduces all 126,450 FA replay frames. Actual pose
changes 811 requests without changing ownership or the motion ledger; raw safety
calculations retain wheel acceleration. Entry minima change from −1.903 to −1.853
for FA5 and −1.059 to −0.993 for FA17. FA5's request-jerk minimum is unchanged.
Some later requests become slightly stronger. Existing source-liveness limits
are used only for this diagnostic; they are not validated observer-age limits
for a deployed braking controller.

A separate six-case native test supplies the candidate's own simulated physical
acceleration, held at 20 Hz. Even this ideal observation leaves the FA5-like
moving negative jerk unchanged at −1.418 m/s³. FA17-like improves slightly,
−0.273 to −0.263, and FA1-like has a small terminal positive-jerk regression.
There are no new floor, incomplete-stop, or creep failures. All three weak delayed
adverse cases already fail the current controller and remain exactly unchanged;
they are not successful safety cases. The first two current traces exactly match
the prior native results, and four selector/fixture tests pass. This does not
support a significant improvement or justify a new runtime sensor dependency.

## Reproduction and next boundary

All cycle evidence is under
`~/.route_sync/corpus/stopping_cycle_20260918/`. `remote_manifest.json` pins the
discovered finalized rlogs; each route has original source copies, baseline,
signals, and review packets. `summarize_cycle.py` requires all 57 route packets
before producing `cycle_summary.json`. `labels.json` preserves the sole mediocre
bookmark and points to the unchanged personal-reference labels.

The main audit directories are `causal_fa`, `causal_f8`, `actuator_fa`,
`f9_triage`, `governor_architecture_audit`, `pending_brake_trial`,
`effective_command_contract`, `automatic_response_trial`,
`response_objective_trial`, `motion_observation_audit`, `pose_forecast_trial`,
`pose_forecast_native`, `accel_tracking_trial`, `mixed_handoff_audit`,
`response_mechanism_audit`, and `response_asymmetric_trial`. Their reports,
prototype sources, result traces, and manifests retain rejected experiments.
Recorded-input command comparisons are not changed-controller vehicle rollouts.
`cohort_comparison.png` and `.pdf` show every personal-reference and new automatic
observation, with the mediocre bookmark identified. They do not pool unlabelled
manual stops or imply matched traffic conditions.

```sh
source .venv/bin/activate
python tools/stopping/review/cycle_review.py --packets PACKET_DIRS --output REVIEW.json
pytest -q tools/stopping/review/test_cycle_review.py tools/stopping/review/test_marked_comparison.py tools/stopping/review/test_bookmarked_baseline.py
```

The census check passes 25 tests and targeted Ruff. Optional Fable review could
not run: installed Claude Code 2.1.123 rejected the requested model and required
2.1.251 or later. Independent Codex reviews and counterexample probes ran instead;
no automatic CLI update or model substitution was made.

The next controller design must reconcile the earlier approach and the finishing
trajectory, including moving-lead arrivals, rather than extend ownership of the
current profile. It also needs a trustworthy command-response comparison that
reproduces the actual dip and finishing motion. This cycle supplies a broader
regression cohort and isolates these failures; it does not supply a qualified
runtime improvement or a superhuman-stopping result.
