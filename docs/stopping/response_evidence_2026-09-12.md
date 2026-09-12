# Offline stopping response check, 2026-09-12

The existing recordings do **not yet support a significant new stopping-control change**.
The corrected simulator still misses recorded stops with their original brake requests.
It must not rank a larger change or support a claim of better-than-Radek stopping.
No driving code, settings, arm files, or device files changed in this work.
The small entry correction at `761f4cef14` remains the existing candidate; this work
does not establish an additional comfort gain or its on-device performance.

## Data and observation checks

- Retained the 11 explicitly attributed Radek manual references and all four bad
  automatic bookmarks. Identity and ratings remain in `labels_with_20c1.json`.
  All 11 reference windows from the last 2.5 m/s crossing are manual; at 5 m/s,
  eight are manual, two are mixed, and one has no crossing. These are different
  cohorts, not eleven fully manual approaches from an arbitrary earlier time.
- Decoded the cached classic-CAN Hyundai requests from `sendcan` bus 0 and their
  `can` bus 128 echoes, plus ESP12/TCS13 on bus 0. Every source rlog was checked
  against the existing packet hash and required a complete single zstd frame.
  Echoes establish transmission, not that the ECU applied the requested force.
- On route 20c1, the native wheel-filter recursion reproduces the recorded vEgo
  and aEgo to less than `5e-7` on 4,602 moving adjacent samples below 5 m/s.
  The implementation is `x_next = (A - K C) x + K vEgoRaw`, with
  `K = [0.17406038913518396, 1.6592563982783999]` and `A[0,1] = 0.01`.
  aEgo is a wheel-filter state, not an independent physical acceleration sensor.
- ESP12 and TCS13 provide additional acceleration proxies. They qualitatively show the dips
  in both latest bookmarks too. Their calibration, body orientation, latency,
  and status-bit semantics are not established by a DBC signal name.
  ESP12 cylinder pressure is zero in the inspected bad-stop segment, although
  it is nonzero elsewhere. It is not a verified automatic brake-force signal.
- The new extractor preserves every CAN frame in a batch. Route 20c1 contains
  2,771 equal-timestamp ESP12 frames. Collapsing to one latest value per batch
  loses observations. It also keeps nanosecond stamps as integers, retains
  invalid frames as invalid, and rejects short payloads instead of zero-padding
  them into apparently valid requests. Equal timestamps do not give each frame
  a known individual acquisition time. These sensor streams are diagnostic only;
  they are not samples in the fitted model. No independent checksum/counter
  validation is claimed; the selected DBC messages supply no such validators.
  Segment gaps remain explicit and must fail downstream continuity checks.

## Corrected model experiment

The final experiment covers 37 admitted final stopping phases on seven routes.
Training: 26 phases from 2072, 20b7, 20b8, and 20c0. Development: 11 phases from
20bc, 20bf, and 20c1. These are development routes, not untouched acceptance data.
The manual references are targets for comparison, not actuator-training samples.

Each phase starts at the last observed 2.5 m/s crossing, bounded after the
previous qualified rest, with two seconds of command history. Native input
validity, no pedals, and SCC12 ACCMode=1 are required. Post-stop observation
continues for up to two seconds, censored at the recorded rest end or the first
invalid, missing, or driver/ECU-control boundary. No stale command is extended
past that boundary. This is not a simulation of the full approach from road speed.

The selected development model uses one actuator lag, positive high/low-speed
command gains, a constant offset, and the existing relief and low-speed feature
shapes (`-0.25` request threshold, `1.2 m/s` low-speed reference). It integrates
nonnegative speed, then applies the native wheel observation filter. Initial
speed/filter state and a slope from the preceding 0.30 s initialize it once.
There are no future measured-state resets and no fitted per-stop offsets.

Fit loss includes the entire predicted raw-speed trajectory and integrated
distance. Delay was selected on training only over 0 to 0.6 s in 0.1 s steps.
Development checks require all three: speed RMSE <=0.1 m/s, absolute distance
error at the recorded stop <=0.2 m, and absolute stop-time error <=0.3 s.
Only the numeric tolerances were fixed before the initial fits. The cohort and
censoring were subsequently revised as described below. Passing would still not
establish passenger comfort or physical safety under different requests.

| Corrected experiment | Training, n=26 | Development, n=11 |
| --- | ---: | ---: |
| Phases passing all three checks | 5 | 1 |
| Median absolute distance error | 0.244 m | 0.466 m |
| Maximum absolute distance error | 1.366 m | 2.446 m |
| Median speed RMSE | 0.092 m/s | 0.131 m/s |
| Maximum speed RMSE | 0.273 m/s | 0.268 m/s |
| Rest not verified before the observation boundary | 1 | 0 |

The first latest bookmark has +0.466 m distance error and +0.311 s stop-time
error. The second has +0.034 m distance error but predicts rest 0.491 s early.
An average score would hide that difference. The selected delay is 0.0 s and
the lag hits its lower fit bound of 0.03 s. The high/low-speed gains are
0.94510/0.86243, offset 0.01931, relief coefficient -1.49286, and low-speed
offset 0.23991. This failed fit supplies no reliable physical parameter interval;
in particular it does not establish a 30 ms physical actuator lag.

Initial simpler fits, including a pre-entry-only offset estimate, did not resolve
the errors. Their scratch results are superseded by the corrected experiment:
the original observation ended at rest+0.5 s, which could not verify a slightly
later predicted half-second rest, and its search was not bounded by prior rest.
The updated admission/censoring code and final cohort are retained with the result.
Adding the older cached route did not make the model adequate. This rejects
these fitted models; it does not prove that every possible offline model must fail.
Geographic independence between the development and training routes was not tested.

## What input would help

More marked manual stops are not the current bottleneck. The missing evidence is
how the car responds to *different automatic requests* at comparable low speeds.
In the final 37-phase cohort, before recorded rest and below 0.1 m/s, training
contains 9.09 sampled seconds with requests from -0.94 to -0.54 (median -0.7).
Development contains 4.33 sampled seconds, all at -0.7. Those recordings cannot establish
what a substantially softer terminal request would do. Older data contains
additional commands, including departures; those are not interchangeable with
braking trials. Neither driver identity nor a smooth manual stop supplies an
automatic actuator response for a request that was not sent.

Alternative data sources were checked before recommending collection. The 131
rlogs behind the seven packets contain no valid incoming SCC12 ACCMode=1 frames
on buses 0–2. An initial-configuration inventory of all 166 locally cached rlog
routes found one Santa Fe route configured for stock longitudinal control,
2080. Its 15 rlogs contain active stock cruise, but none below 2.5 m/s.
The old event store has 116 additional 100 Hz traces marked as carOutput/v2,
some with softer low-speed requests. They omit gas input and CAN-validity
fields, and their original rlogs are not in the local cache. Three representative
original segments checked on the device were absent too. These derived traces
are not promoted to trusted command-response evidence. Restoring their original
logs would be another possible way to obtain the missing information without
collecting a new drive; availability of another archive has not been established.

A supervised comparison of the existing entry candidate can check regressions
and give qualitative feedback on the pulse. One unblinded session would not
establish superiority over the marked manual references. For a larger controller or learned response model, the
high-value new input is repeatable command-response evidence in the stopping
speed range, with original and changed requests, on a controlled test site.
That experiment needs its own concrete design before activation. No experiment
was armed here. The existing identification hook starts at 7–11 m/s and ends at
4.5 m/s: running it alone would not fill the near-stop gap discussed here.

The prior architecture direction remains a continuous approach and a controlled handoff
into the stopping service, evaluated against the marked reference phases. This
2.5 m/s experiment does not validate that earlier-approach design. Adding
bookmark-specific branches or a larger learned model does not repair an invalid
response comparison. The whole-approach governor stays OFF.

## Reproduction

The reusable source is `tools/stopping/review/can_response.py`; no dependencies
were added. It accepts `bookmarked_baseline.py`'s `signals.json` and writes a new,
hashed CAN packet. Invalid values are accompanied by explicit validity arrays.

```sh
source .venv/bin/activate
python tools/stopping/review/can_response.py PATH/TO/signals.json --output NEW_DIRECTORY
pytest -q tools/stopping/review/test_can_response.py tools/stopping/review/test_bookmarked_baseline.py
```

Local evidence is under
`~/.route_sync/corpus/radek_baseline_20260912/response_model/`.
The final model result and cohort are `corrected/scheduled_fit.json` and
`corrected/cohort.json`; prototype sources, the sensor plot, and source hashes
are retained in `evidence_manifest.json`. From the repository's managed Python
environment, run `response_model/reproduce_corrected.py --output NEW_DIRECTORY`
with the full local path to recheck hashes and repeat the rejected fit.
The plot's sensor axes are diagnostic
proxies, not calibrated passenger-comfort measurements.

Validation: 14 targeted tests passed; targeted Ruff passed. The new extractor
also matched every original decoded field on all six route-20c1 CAN streams.
The corrected model result and cohort were reproduced exactly in a fresh output
directory. Constant-braking distance and standstill-observation checks passed.
No new comfort result, deployment, or device revision is claimed by these checks.
