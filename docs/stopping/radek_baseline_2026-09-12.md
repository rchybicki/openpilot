# Radek's marked stopping baseline — 2026-09-12

## Decision

Use Radek's explicitly marked manual stops as the personal reference. The previous Claude claim that
openpilot was smoother than Radek used another driver's manual stops. That personal comparison is withdrawn.
Other drivers' data can still test vehicle dynamics and regressions, with identity and purpose kept separate.

The target is a complete, predictable stop: timely braking, a continuous final ease, little body motion at rest,
the intended gap, and no creep or second brake application. Retain every marked example; do not select a
favourable subset or declare success from a lower median alone.

The recommended direction is one planned motion reference through the approach and finish, followed by a
verified hold transition. Learn vehicle response where a simple model fails. The recorded reference is useful
now; a superior controller has not been demonstrated. No driving code, flag, or device setting changed.

## Source and provenance

Recovered Claude session `dd2f1735-085c-43ac-b2c5-347e09a5a928`, including Radek's correction at
2026-09-12 09:38 UTC and the Fable usage-limit response. The current user request explicitly says the bookmarks
are his manual baseline stops. That statement supplies identity and meaning; the logs do not.

Cycle 53 remains the driving code. Cycle 54 rejected further tuning and left two open mechanisms: arrival above
the comfort profile, and terminal descent building secure hold before rest. Current-route `initData.gitCommit`
is `b3ead04479a0b087fe0c470f55368fe436801d3f` in all 26 captured segments. The three previous comparator routes
report `5fc2db9920`; the intervening diff in selfdrive/frogpilot/opendbc is empty.

Snapshot: route `000020bf--9dcbe4db3b`, completed segments **0–25**, **10 bookmark presses**, **14 detected rests**.
The route was still active. This is a fixed snapshot, not a claim that the route is complete. Only completed
files without a segment lock were copied. No live message subscribers were opened. The shared refresher was
stopped during slow full-device discovery; a route-specific SSH tar stream populated the same canonical cache.
Its state file was not advanced. Strict decompression and Cap'n Proto decoding passed.

Local packet: `~/.route_sync/corpus/radek_baseline_20260912/`:

- `baseline.json`: exact event IDs and bookmark associations, source paths/bytes/SHA-256, scorer SHA-256,
  init commits/settings, all detected rests, missingness and window classifications.
- `signals.json`: continuous signals used for analysis; no video or account credentials.
- `terminal_traces.png`: every selected finish, with identical time alignment and axes.
- `000020b7/`, `000020b8/`, `000020bc/`: comparator packets from 42 previously cached rlogs.
- `labels.json`: explicit IDs for 11 manual references (the original ten plus segment 26) and the later bad
  openpilot stop. The original table/figure remain frozen at ten; the addendum is described below.

Route-init settings: IncreasedStoppedDistance **0.3**, CEForceCoastStrength **1.4**, ExperimentalMode **1**.
These are init snapshots, not evidence that settings were constant at every stop. No causal gap comparison uses them.

## Attribution and measurements

One screen press produces `bookmarkButton` followed by `userBookmark`. Both timestamps are retained; each pair
counts once. Every mark matches exactly one sustained rest. Marks follow the detected rest by **0.393–2.187 s**.
Four unmarked rests remain outside Radek's labelled baseline.

All ten terminal windows are fully manual. Only three complete last-ten-second windows are manual. Openpilot
controlled earlier parts of the other approaches. Continuous manual control before rest lasts **4.65–16.38 s**.
All ten support a manual finish reference; seven cannot represent a wholly manual last-ten-second approach.
This is a provenance limit, not a negative rating of those stops.

Terminal window: final crossing below 0.45 m/s through 0.5 s after sustained vEgo <0.05 m/s. Jerk reuses the v2
exact 300 ms difference, with its full window inside the scoring interval. Both arms use the same definitions.
The separate 30-second diagnostic is not metric v2. Invalid samples, CAN-invalid carState, gaps >100 ms, stale
engagement state, truncated files and unknown phase starts cannot silently become valid scores. Segment clocks
remain continuous. No p90 or statistical significance claim is made.

| Segment | Rest, route s | Manual before rest, s | Last 10 s | Terminal jerk, m/s³ | aEgo at last v≥0.10, m/s² | Radar gap, m |
|---|---:|---:|---|---:|---:|---:|
| 5 | 335.155 | 9.57 | mixed | 2.033 | -0.470 | 3.05 |
| 6 | 398.484 | 8.13 | mixed | 1.030 | -0.417 | 3.29 |
| 7 | 455.754 | 10.45 | manual | 0.692 | -0.311 | 3.00 |
| 9 | 546.245 | 7.26 | mixed | 1.183 | -0.248 | none |
| 10 | 625.144 | 8.36 | mixed | 1.049 | -0.308 | 3.60 |
| 12 | 723.997 | 16.38 | manual | 0.676 | -0.274 | 4.20 |
| 14 | 852.594 | 6.24 | mixed | 0.639 | -0.272 | 2.50 |
| 17 | 1072.405 | 9.74 | mixed | 0.839 | -0.292 | 3.07 |
| 21 | 1292.625 | 11.73 | manual | 1.526 | -0.376 | 27.37 |
| 25 | 1550.477 | 4.65 | mixed | 1.256 | -0.286 | 3.00 |

These are radar readings, not surveyed bumper gaps or target positions. Segment 12 has a moving lead (0.37 m/s
at ego rest); segment 21's lead is far away. Do not pool them as stopped close-lead examples. Do not copy the
2.50 m gap into the controller: the existing target remains 4–5 m, with the 3 m floor.

## Findings

The previous-route packets contain 21 detected rests; eight have complete engaged v2 windows and peak speed
≥3 m/s. Source coverage is incomplete (42 cached segments across three routes). This is an observed comparator,
not a full-drive census. One comparator rests 25 m from the reported lead. Grade, traffic, choice of manual mode,
target intent and per-stop settings are unmatched. All detected rests and window exclusions remain in the packets.
Selection also differs: the personal examples were deliberately marked, while the comparator was selected by
complete engagement and signal windows. This is not evidence that the marks are Radek's best-ever stops.
Both arms use identical metric definitions, but the manual arm only requires manual terminal control; the
comparator requires ten seconds of engagement. Selecting the comparator by terminal engagement instead gives
n=10 and medians -0.463 / -0.318 / 0.901 / 2.908 for the table below. These are sensitivity results, not rankings.
The original route snapshot also contains one unmarked engaged terminal stop in segment 14; it remains separate.

| Descriptive median | Radek's 10 marked finishes | 8 observed engaged finishes |
|---|---:|---:|
| aEgo at last v≥0.50 m/s | -0.810 | -0.433 |
| aEgo at last v≥0.10 m/s | -0.300 | -0.301 |
| v2 terminal jerk | 1.039 | 0.881 |
| livePose device-x 300 ms jerk, rest−2 to rest+1 s | 2.617 | 2.908 |

The channels do not give a consistent ranking. Radek's examples show a larger reduction in measured deceleration
between 0.5 and 0.1 m/s; engaged examples often carry a flatter late demand. This is a candidate shape difference,
not proof of its cause or superior comfort. A lower scalar score cannot disprove the user's perceived reference.

`aEgo` comes from the wheel-speed Kalman filter (`opendbc/car/interfaces.py`, `update_speed_kf`). It is not direct
body acceleration. The livePose traces show the stop transient before the vEgo<0.05 alignment; these must not be
treated as the same physical event. livePose is also filtered and in device axes. Its acceleration/pitch-rate
metrics remain diagnostic until orientation, timing, noise and road effects are accounted for. Neither these
proxies nor v2 inherit the legacy `felt <=0.8` acceptance threshold.

## Architecture and next experiments

1. **Establish a physical finish measurement.** Reuse the frozen events. Align the wheel-speed filter, individual
   wheel CAN observations, livePose acceleration/rotation and hold. Identify which onset/release/body-motion
   feature separates the reference finishes. Test timing/filter sensitivity before interpreting a number at
   0.1 m/s. Keep mixed-control approaches labelled. Later confirmed routes must provide independent validation.
2. **Validate command-to-motion prediction through a complete stop.** Correct the known `_Sim` acceleration
   feedback defect from cycle 54. Reuse the fitter/evaluator from cycles 44–47; score free-rollout speed, travel,
   stop time and creep/hold, with a stationary observation model. Do not feed the recorded answer back each frame.
   Prior travel errors (+5.790, +0.698, -19.097 m) disqualify that model from ranking new laws. If existing engaged
   data cannot identify the missing response, specify the gap before proposing new collection.
3. **Test one continuous stop reference offline.** Plan from when a trusted target and remaining distance permit
   the reference, rather than entering an unrelated profile at 2.5 m/s. Plan deceleration and its rate through
   the final ease, then track with the measured delay. Keep one comfort source, explicit hazard arbitration and
   the existing disengagement/fault path. `_terminal_descent_target` deliberately approaches
   `A_HOLD_SECURE=-0.70` before 0.10 m/s. Replacing this requires anticipating creep and the hold transition;
   sending zero brake near rest is not the proposal. A physical hold mode still needs separate handling.
4. **Require measured improvement before activation.** Compare unchanged and candidate controllers on identical
   held-out scenarios and plausible plant uncertainty. Count noncompletion, short gaps, relaunch, intervention
   and safety demand separately from comfort. A lower jerk score cannot excuse them. Then use a separately
   authorized vehicle evaluation with Radek's explicit ratings. One route cannot establish superhuman performance.

First decision: does the reference's advantage lie in approach timing, final release, or body motion absent from
v2? If release differs, test a terminal-reference/hold transition in the validated plant. If approach timing
differs, test planner arrival. If the signals do not explain perceived quality, calibrate the objective first.
Do not declare every possible result proof that the architecture must be rewritten.

**Machine learning:** these marks are trajectory examples, not brake-command training labels. While manual,
`carOutput` is not the driver's applied brake. Fit response from valid engaged command/response data; use manual
events to specify desired motion. Start with the existing physical model, then a small learned residual for
delay/creep/friction only if it improves unseen full-stop predictions and uncertainty bounds. A preference model
needs more explicitly rated episodes. This packet does not justify end-to-end imitation or RL control.
Residual learning inside constrained MPC has research precedent in a different vehicle setting:
[Hewing et al.](https://arxiv.org/abs/1705.10702). That is not validation for this car. Upstream also documented
how its planner's distance objective changed stopping shape in the historical
[0.8.12 longitudinal change](https://blog.comma.ai/0812release/); that change is not a proposed import.

## Reproduction and review

Activate `.venv`. Run `python tools/stopping/review/bookmarked_baseline.py --output <new-packet-directory>` with
the frozen `baseline.json` source-file list. A glob will include later downloads and change the snapshot.
Source and scorer hashes are stored in that file. Targeted verification:
`pytest -q tools/stopping/review/test_bookmarked_baseline.py tools/stopping/review/test_human_baseline.py`.

Plan review: Fable was at its usage limit; the permitted Opus fallback completed the review. Adopted exact
bookmark pairs, partial-manual attribution, equal windows, hashes, sample/context limits and model validation.
Discarded its proposed confirmation question: Radek had already explicitly supplied bookmark meaning and identity.

Final Opus review reproduced the counts, code equivalence, tables and tests; no blocker. Applied its cohort
selection clarification and terminal-selection sensitivity. Fixed a diagnostic pitch-peak window that included
the interpolation bracket samples outside its stated interval; added a regression check. No reviewer source edits.

## Live-route bad example — user correction during this analysis

The user's later message labels the **last bookmark as bad**: unnecessary braking, acceleration, then harsh
stopping. It is `userBookmark` at mono **4779896644117**, route **2099.500 s**, segment **34**, matched to rest
**4774151422860** (route **2093.755 s**). It is outside the frozen manual packet and must never inherit its label.
The preceding further mark in segment 26 is also outside that initial snapshot. It matches manual rest
`4269070390092` (route 1588.674 s), with 10.37 s of continuous manual control and complete manual last-ten-second
and terminal windows. Its packet is `manual_addendum_26/`: terminal jerk 0.873, aEgo at last v≥0.10 of -0.198,
radar gap 2.60 m. It is the **eleventh manual reference**, retained separately from the frozen ten-row comparison.
Future labels must be explicit event IDs, not a rule that every bookmark in this route is a manual reference.

Full-rate segments 32–35 and service telemetry are preserved in the packet's `bad_stop_34/` subdirectory.
The bad stop is engaged with no pedal input throughout the preceding 30 seconds. Its recorded sequence:

- At route 2088.8 s the wire is about -0.64 m/s² and the ego speed is about 1.0 m/s.
- The service releases braking to about -0.08. Speed bottoms near **0.71 m/s**, then rises to about **0.95 m/s**.
- The wire then deepens to about **-0.89 m/s²**, followed by a smaller speed rebound around 0.43→0.48 m/s.
- It reaches the observed rest at a radar gap of **4.60 m**. This gap does not make the approach comfortable.

The service's phase timeline stays **APPROACH_GLIDE** through this sequence; no phase exit/re-entry caused it.
`attr_ring` identifies the release: after the braking-lead veto clears and the 0.30 s eligibility dwell passes,
the live attributed-safety path removes the planner's deeper demand and raises the output at 0.8 m/s³. At service
t=1.63 s, the service's `a_plan` is -0.454, phase demand -0.030, candidate -0.070 and wire -0.301; by t=1.93 s the wire
is -0.080. The governor later demands more braking as speed and gap change. `attr_live_release_frames=78`,
`attr_eligibility_flips=1`, `attr_live_reentries=0`. This identifies the command-release mechanism from telemetry
and source; it does not prove that all earlier braking was unnecessary or predict the result of disabling it.

**The metric failure is now directly labelled.** Terminal 300 ms jerk is only **0.468 m/s³**, while last-ten-second
jerk is **2.497 m/s³**. The v2 census excludes the episode entirely: speed last exceeded 3 m/s **10.45 seconds**
before rest, just outside its ten-second inclusion window. The new packet retains it. Its poor approach is outside
the terminal window. A terminal-only improvement or a census that drops this stop cannot satisfy the user.

This becomes the first negative regression case for the continuous-reference experiment. The required behaviour
is a smooth reduction of motion toward rest without the observed release/creep/re-brake sequence, while preserving
lead-departure response and hazard braking. A single persistent owner alone is insufficient: this event already
had one. Compare the current law and candidate reference with the same lead history and a validated response
model. Do not simply disable attributed release on the vehicle or add another instantaneous brake clamp.

Final verification: 9 targeted tests pass, ruff and diff checks pass, and the three personal/incident packets'
source/scorer hashes match their saved manifests. The later incident analysis was checked by the main agent;
the independent final review covered the original frozen baseline, not this later user-supplied incident.
