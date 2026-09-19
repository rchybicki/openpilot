# New-route stopping review, 2026-09-19

The additional routes confirm that the braking pump remains. This cycle has not
qualified a new comfort law. A separate three-line repair prevents stale pre-entry
diagnostics; it does not change braking. See
[the context repair](pre_entry_context_2026-09-19.md).

## Captured evidence

The frozen snapshot contains 32 finalized rlogs: route `00002100--1d4b9d86ff`
segments 0–20, `00002101--965129ea02` segments 0–1, and
`00002102--505ee0f006` segments 0–8. Route 2100 records `ab3464a77d`; routes 2101
and 2102 record `6e25c96d37`. Unfinalized segments are not counted as complete.
All source hashes were verified. No new bookmarks were found in this capture.

There are 17 rests, including six automatic final-2.5 m/s windows. Five automatic
stops precede the latest braking change; only one belongs to `6e25c96d37`.
Route 2101 never engaged. The personal reference remains Radek's same 11 explicitly
marked manual finishes. Their longer approach contexts are mixed or unknown;
neither those contexts nor other drivers' stops are relabelled as Radek's driving.

At the automatic 2102 stop (`@1202469868148`, segment 5), the whole last 30 seconds
are engaged, valid and authorized. Signed 300 ms wheel jerk in the final 2.5 m/s
window is −1.824/+1.519 m/s³. Radek's unmatched descriptive medians are
−0.415/+1.107. These are different scenes and filtered wheel measurements, not a
controlled comparison or proof of perceived comfort.

The ownership signal follows the intended contract: 167,644 paired
`controlsState`/`carControl` records in the through-segment-5 snapshot have no
authority, pedal or contract contradiction. On 2102, ownership starts at about
1.975 m/s, 3.177 seconds before legacy STOPPING. Actual screen receipt is not logged.

## Recorded pulses and control inputs

At takeover, the request falls from approximately −0.436 to −0.786 m/s², then
releases. SCC12 transmission and its CAN echo confirm the requested pulse; wheel
deceleration follows about 0.46 seconds later. A second release reaches about
−0.181 m/s² before braking builds again. The strongest final-2.5 negative wheel
jerk belongs to that later rebrake. Final-0.5 SCC12 requests are nearly constant
at −0.70; wheel quantization and the delayed first raw zero prevent an inference
of physical creep from the small velocity tail alone.

The lead was still moving when ego entered the observation band. Both contexts
observed 175 frames before takeover and earned the same 300 ms stopped-lead dwell.
Earlier context warmup would not remove that legitimate wait. An earlier pulse,
about 20 seconds before rest, follows the planner request on this stop.

The coast observer has a separate input mismatch: while the stopping service owns
the output, it still subtracts the legacy would-have command. The two differ by
as much as 0.484 m/s² on this approach. Correcting the input is necessary for that
observer's interpretation, but it is not sufficient for a stable controller:
subtracting a purely delayed request still mistakes actuator lag/gain for external
push or drag. Neither a finite command-window bound nor exact command steadiness
resolves that ambiguity for the existing delayed, first-order test plants.

## Why the tested alternatives remain offline

- Replacing the coast input with the published request changes the later pulse,
  but does not improve this stop's entry jerk. Broad native simulations expose
  worse comfort and wider final gaps. Adding positive-coast compensation to the
  safety barrier does not repair nominal comfort: among 38 native cases, negative
  moving jerk worsens in 25 and positive terminal jerk in 29; 17 rest gaps exceed
  5.1 m, versus none for the current controller. These are hypothetical plants,
  not predicted outcomes of the recorded routes. A separate actual-CAN-timing
  challenge reaches a no-push restart transient where the raw barrier is zero,
  but the estimated coast makes this candidate add braking. That test retains
  synthetic stop intent; it is not a claim about production planner behavior.
  True positive push also benefits from compensation, so the unresolved issue
  is whether the observer can distinguish those situations.
- A bounded negative-drag correction passes the 3,888 relative physical checks,
  but still worsens terminal positive jerk in 24 of 38 native cases. The worst
  increase is 0.428 m/s³. Passing a clearance check alone is not a comfort win.
- Stronger forecast credit either exceeds the existing noise-sensitivity limit
  or leaves all six entry extrema unchanged. No new coefficient was selected by
  tuning those failures away.
- Recovery laws derived from the governor profile stop too far back in existing
  lag/push fixtures. The fitted-profile variant still fails four absolute 5.1 m
  rest-gap checks; the direct profile variants fail twelve.
- Replacing prediction with a per-frame brake-build budget changes safety-rate
  selection even with zero earned surplus and loses predictor fault containment.
  Six existing entry tests fail; the new stop's entry request jerk worsens from
  −1.340 to −1.460 m/s³. A separate raw-demand blend also loses the prior bookmark's
  earned-hold behavior. Neither was applied to runtime code.
- Holding the pre-takeover coast estimate avoids learning from the legacy command
  while the service owns braking, but a one-line hold leaves unobserved commands
  in the delayed buffer at handback. A separate variant clears missing-observation
  history and waits for a full delay window before learning again. Its observer
  contract tests pass, but priority case 179 crosses the 3 m floor: current
  clearance 3.135 m, candidate 2.970 m. Testing stopped before route replay; no
  modified coast observer was deployed.
- The lag-adjusted reference's feedforward derivative was checked again. The
  continuous one-sided correction is the prior rejected `governor_forecast`
  mechanism, not a new law. Rechecking its decisive wheel case 1711 on current
  `6e25c96d37` reproduces the same completed clearance loss exactly:
  2.762307 to 2.633738 m. No further sweep or tuning followed.

The hypothetical matrix itself needed an audit. Its old early exit at gap <1 m
could compare failed stops at different times. At case 1151, the apparent candidate
loss reverses to 22 mm more clearance when both are run to completion. The original
failure is preserved. Completing all 3,888 coast-plus-barrier pairs finds no new
floor crossing, incomplete stop or creep, but four already-unsafe cases worsen.
Wheel case 549 loses 0.164 m and takes 2.19 seconds longer; this is a real completed
comparison, not the early-exit artifact. Baseline failures remain visible:
297 cases are below 3 m and 14 are incomplete in the original current matrix.

Actual Hyundai sender/packer timing also matters. The old proxy applies a 100 Hz
unquantized request; SCC12 is sent at 50 Hz in 0.01 m/s² increments. In adverse
case 1273, applying actual sender/packer semantics enlarges the published-coast
clearance loss from 1.6 mm to 23–33 mm across the two send phases. SCC14 and StopReq
effects inside the ECU remain unidentified; their transmitted values are not an
achieved-force model.

## Model adequacy

The unchanged previously fitted response model reproduces all 18 old saved traces
exactly, but only two of the six new automatic stops pass its original speed,
distance and rest-time gates. Even on 2102, which passes those geometric gates,
predicted negative jerk is −0.583 versus observed −1.816 m/s³ and the predicted
peak is about 3.43 seconds early. The model understates all six negative extrema.
It cannot rank changes intended to remove this pulse.

The additional calibrated pose/camera/IMU audit covers the same six stops. On
2102, the separate pose estimator also contains the entry and later braking
pulses, with no calibration change at either event. Wheel filtering alone does
not account for the missing model waveform. Pose uses camera/IMU estimation;
its reported uncertainty, shared sensor lineage and device mounting prevent
treating it as calibrated passenger-comfort ground truth.

Near rest, the channels need separate treatment. At 2100 `@2109304757140`, pose
velocity rises to about 0.181 m/s inside the wheel-defined rest interval. The
model's larger predicted departure differs from both recorded estimates, but wheel
speed below 0.05 m/s alone does not prove physical immobility or achieved brake hold.

The new routes add 1.01 seconds of requests above −0.25 m/s², mostly between 0.5
and 1.2 m/s. They add no softer-command evidence below 0.1 m/s. No model refit,
new dependency, scripted driving hook or comfort activation accompanies this review.

A bounded audit of documented CAN state channels adds no validated measurement
of achieved brake/drive force. During the two inspected stop windows, gear is D,
pedals are released, auto-hold and parking-brake indications are off, and decoded
ESC control flags remain inactive. The brake lamp follows the 2102
entry/release/rebrake sequence. Undocumented reference/limiting fields and
messages with a mismatched DBC payload size are not promoted to force evidence.
At 2100's positive request, no decoded hold indication supports a claim that the
ECU rejected acceleration; the pose departure caveat remains applicable.

The response model is not adequate to rank changes intended to remove the recorded
pulse. Further work needs a model sufficient for the specific decision, with
explicit uncertainty; a complete accurate vehicle model is not a prerequisite.
Request transmission, motion estimation and achieved actuator force must remain
distinct. The [next investigation](factory_comparison_plan_2026-09-19.md) uses
factory SCC as an additional reference and tests trajectory feasibility locally.

## Reproducibility

Frozen rlogs, scripts, source hashes, exact baseline reproductions and rejected
counterexamples are under
`~/.route_sync/corpus/post_6e25_review_20260919/`. Main packets are `metrics/`,
`ui_audit/`, `actuator_audit/`, `entry_lifetime_audit/`, `late_release_audit/`,
`coast_input_audit/`, `coast_observer_review/`, `frozen_response_evaluation/`,
`body_motion_audit/`, `can_transport_audit/`, and the separately named experiment
directories. `actuator_state_observability/` retains the bounded CAN state audit.
Recorded-input replay compares final published requests with motion fixed; it
does not predict changed physical motion. Native/proxy plants test mechanisms
and counterexamples; they do not establish superiority to Radek's stops.

The diagnostic repair `dfa8dbb3fb` was pushed and applied with `fullupdate.sh`
while off-road. After reboot, device HEAD, the manager's `GitCommit` parameter
and all three stopping source hashes match; the build completed and UI/pandad
are running. Verification used passive files/processes and the existing console,
without message subscribers or driving-module imports. This proves deployment
and off-road startup, not a new on-road comfort result. The unchanged braking
service remains the `6e25c96d37` version.
