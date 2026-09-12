# Whole-stop comparison against Radek's marks

**No broad improvement has been demonstrated.** Cycle 55 changes one release mechanism; its recorded-input
replay leaves the eight earlier comparison stops unchanged. This assessment expands the personal comparison
to all 11 marked manual stops. It changes analysis only, not the controller or device.

## What the marks now establish

All 11 reference stops are continuously manual from the last 2.5 m/s crossing through filtered rest +0.5 s.
This gives a common final approach at the stopping-service entry speed. It is longer than the terminal-only
window. At 5 m/s, eight are manual, two are mixed and one has no crossing in the 30 s lookback. These examples
cannot establish fully manual high-speed brake onset in every case.

The pre-existing shape hypothesis survives the expanded sample and a timing check: every manual example has
less measured deceleration at 0.1 than at 0.5 m/s. Seven of eight earlier engaged examples have a smaller ease
than every manual example. Using vEgoRaw for the crossing anchors, with aEgo unchanged, preserves that separation.
This is a descriptive difference in these selected events, not proof of better comfort or its cause. The closest
filtered-anchor separation is only 0.025 m/s². These are single-sample descriptors without uncertainty bounds;
the anchor check does not establish resistance to sensor noise.

The bad automated stop's easing falls inside the manual range. Its 2.5 m/s approach contains a 0.245 m/s maximum
speed recovery, versus 0–0.001 in these manual examples. The same detector finds no recovery in the earlier
eight engaged examples at the displayed precision. It identifies this known failure; it does not identify a
general fault in every automated stop. The previous 0.239 figure described the incident's selected subinterval;
0.245 uses the full native-rate final 2.5 m/s interval defined here.

Neither descriptor alone is a stopping objective. In particular, the existing livePose body-motion proxy has
overlapping values across all three groups. A terminal-only score would still miss the labelled bad approach.

## Every retained event

`Time` and `travel` run from the last vEgo >=2.5 sample to the existing vEgo <0.05 sustained-rest sample.
Travel integrates wheel speed; it is not a surveyed stopping distance. `Recovery` is the maximum increase from
an earlier running speed minimum inside that interval, with no added filter or noise threshold. `Ease` is
aEgo(last vEgo >=0.1) minus aEgo(last vEgo >=0.5), in m/s². Positive means measured deceleration decreases.
`Raw ease` changes only the crossing channel to vEgoRaw. Crossings are recorded samples, not interpolated times.
Pose jerk uses the existing full-window 300 ms device-x diagnostic, filtered rest −2 to +1 s, in m/s³.

| Event | Time, s | Travel, m | Recovery, m/s | Ease | Raw ease | Pose jerk |
|---|---:|---:|---:|---:|---:|---:|
| Radek 20bf s5 | 2.292 | 2.125 | 0.000 | 0.873 | 0.891 | 2.376 |
| Radek 20bf s6 | 2.890 | 2.887 | 0.000 | 0.608 | 0.590 | 3.382 |
| Radek 20bf s7 | 4.386 | 4.716 | 0.000 | 0.253 | 0.239 | 1.832 |
| Radek 20bf s9 | 2.849 | 2.544 | 0.000 | 0.621 | 0.620 | 2.743 |
| Radek 20bf s10 | 3.609 | 3.825 | 0.000 | 0.418 | 0.460 | 1.974 |
| Radek 20bf s12 | 4.300 | 4.696 | 0.000 | 0.374 | 0.358 | 2.005 |
| Radek 20bf s14 | 2.740 | 2.167 | 0.000 | 0.346 | 0.342 | 3.607 |
| Radek 20bf s17 | 3.590 | 3.967 | 0.000 | 0.459 | 0.458 | 2.965 |
| Radek 20bf s21 | 2.670 | 2.633 | 0.000 | 0.644 | 0.650 | 2.994 |
| Radek 20bf s25 | 3.299 | 3.248 | 0.000 | 0.662 | 0.634 | 2.492 |
| Radek 20bf s26 | 6.590 | 7.411 | 0.001 | 0.219 | 0.309 | 1.460 |
| Bad 20bf s34 | 9.639 | 11.491 | 0.245 | 0.430 | 0.390 | 2.740 |
| Engaged 20b7 s3 | 4.700 | 4.475 | 0.000 | 0.058 | 0.067 | 3.112 |
| Engaged 20b8 s4 | 3.056 | 2.787 | 0.000 | 0.572 | 0.572 | 2.167 |
| Engaged 20b8 s16 first | 3.811 | 3.427 | 0.000 | 0.194 | 0.185 | 2.703 |
| Engaged 20b8 s16 second | 4.487 | 4.724 | 0.000 | 0.132 | 0.136 | 2.953 |
| Engaged 20bc s3 | 3.732 | 3.270 | 0.000 | 0.183 | 0.172 | 2.863 |
| Engaged 20bc s17 | 8.740 | 11.646 | 0.000 | 0.087 | 0.065 | 3.858 |
| Engaged 20bc s18 | 7.621 | 10.463 | 0.000 | 0.019 | 0.017 | 2.036 |
| Engaged 20bc s37 | 3.841 | 3.997 | 0.000 | -0.068 | -0.014 | 3.523 |

All 20 have complete 2.5, 1.0, 0.5 and 0.1 m/s cells. Segment 26 and the bad stop have no 5 m/s crossing;
Radek s7 and s25 have mixed 5 m/s windows. No row is removed for these exclusions. The JSON retains exact event
IDs, every crossing timestamp, per-cell control mode, continuity reasons, last-30-second scope and lead context.
The two s16 events remain separate. The comparison is the same previously defined engaged-v2 cohort, not a
new selection made to favour the result. That v2 rule excludes this particular bad stop because its last speed
above 3 m/s falls outside the ten-second lookback. The eight-event cohort cannot estimate how common such
excluded slow approaches are. The negative label remains independent of the v2 inclusion rule. In both mixed
manual-reference windows, fresh engagement samples explicitly show openpilot followed by manual control.

## Measurement limits that matter to the next change

Both speed channels come from the same wheel observations. vEgoRaw remains positive around filtered rest in
these examples; changing anchors does not independently locate physical rest or validate body acceleration.
The persistence of the ease difference rules out this particular anchor choice as its sole cause. It does not
rule out ECU filtering, differing brake response, road grade, low-speed quantisation or traffic context.

A raw CAN probe found the DBC-labelled ESP12.CYL_PRES signal changing during the manual stops, but zero during
the bad automated stop. Its pressure/status semantics are not validated for this vehicle. It is excluded from
the comparison and from control training. Manual carOutput is also not the driver's brake command. The marks
provide desired motion examples; they do not provide a calibrated actuator model.

The earlier limitations still apply: deliberately marked manual examples versus incompletely cached engaged
routes; unmatched grade, lead motion, target gap and per-stop settings; no new user comfort ratings. Radek s9
has no lead, s12 has a moving lead at rest, and s21 has a distant lead. Short manual gaps are not target settings.
No pooled rank, significance claim, personal-superiority claim or legacy `felt <=0.8` threshold follows.

## Engineering decision

Keep the bounded cycle-55 release change separate from the broader target. Its tests and command replay
support evaluating that mechanism, not claiming a better general finish. Changing the final brake constant
would be an unsupported shortcut: `_terminal_descent_target` explicitly builds secure hold before filtered
rest, and earlier lighter-hold experiments produced creep. The current response model also fails to reproduce
the bad stop's surge. Neither fact is resolved by a smoother-looking reference curve.

The next falsifiable experiment is a continuous motion reference through the approach and final ease, with
the existing hazard arbitration and secure hold. Test whether it can remove recovery *and* produce a gradual
physical finish without extra stopping distance, rollback, late hazard response or delayed departure. The
reference must use the available distance and lead motion; it must not simply copy a manual speed trace.

Before ranking that candidate, the response model must reproduce whole-stop motion under the **recorded**
commands, including the known surge and hold. Use additional route groups for validation because this packet
has now informed design. If a simple model cannot do that, test a small learned response correction against
the same full-stop prediction checks. Learning a driver's command policy from these 11 marks is not supported.
This is the specific open gate to a broad controller improvement; another terminal jerk score cannot close it.

## Reproduce and verify

From the repository, activate `.venv`, then use the exact frozen packet directories:

```sh
packet_root=/Users/radoslawchybicki/.route_sync/corpus/radek_baseline_20260912
python tools/stopping/review/marked_comparison.py \
  --labels "$packet_root/labels.json" \
  --reference-packets "$packet_root" "$packet_root/manual_addendum_26" "$packet_root/bad_stop_34" \
  --comparison-packets "$packet_root/000020b7" "$packet_root/000020b8" "$packet_root/000020bc" \
  --output "$packet_root/marked_comparison.json"
pytest -q tools/stopping/review/test_marked_comparison.py tools/stopping/review/test_bookmarked_baseline.py tools/stopping/review/test_human_baseline.py
```

The tool checks all source-rlog hashes against the frozen manifests before using the packets. Output records
packet, label and scorer hashes. It rejects missing labels, conflicting labels, duplicate selected events and
broken timestamp order. Each descriptor checks its own signal continuity and validity. These checks protect
the comparison; they do not validate a controller.

Main and independent Opus checks: 17 targeted tests pass and ruff is clean. All 120 displayed numeric cells
match the JSON. The final reviewer independently verified the 20-event counts, mode classifications, ease
separation and recovery values; no blocker and no reviewer edits. Added its uncertainty-margin disclosure,
precise v2 selection limit, explicit mixed-window evidence, and shared lookback bound. The review's broader
claim that v2 cannot contain any bad-stop-type event is not established; only this episode's exclusion is known.
