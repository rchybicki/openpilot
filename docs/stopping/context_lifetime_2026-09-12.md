# Stop-context lifetime: deployment candidate, 2026-09-12

The stopping service could start an approach with a frozen distance and coast estimate from an
earlier stop. Reset `StopContext` whenever observation ends, including after a natural RELEASE
has already put the service in INACTIVE. The brake laws, safety arbitration, terminal descent,
hold, and observation-band boundary are unchanged. This is a control-state correction, not a
claim of better-than-human stopping.

## New bookmark and personal baseline

Downloaded complete rlogs 0–28 of `000020c0--418683bc8f`. Their init data and a read-only SSH
check identify device revision `ecd3fee0337f250acc075b4e53b48c8bddc46582`, which includes the
earlier recovery-release limit. No diagnostic message subscribers or device updates were run.

The sole bookmark in those segments is in segment 22: button `1558338708405`, userBookmark
`1558349708572`. The following continuous stop is `1558542312527`. The button was 0.203604122 s
before the first sustained `vEgo < 0.05` sample. The automatic rest-only matcher correctly says
unmatched; Radek explicitly reported a bookmark **before** stopping, so the separate label file
records this association and its reason. No other manual stops on this route receive Radek's label.

The original eleven marked manual references remain the baseline. The labelled comparison now
has 21 events: eleven manual, eight earlier automated comparators, and two explicitly bad stops.
For the new bad stop, the final 2.5 m/s approach lasts 6.050 s, travels 6.030 m by filtered wheel
speed, and has a 0.218 m/s speed recovery. Its 300 ms wheel-acceleration jerk is 2.973 m/s³.
These are recorded descriptors, not predicted candidate outcomes or a single comfort score.

## Defect and scope

`StoppingService.update()` completes RELEASE by resetting its own state. The service can then
remain INACTIVE while the car leaves the observation band. Previously, `_run_stopping_service`
reset the context only if the service was still active. No context updates or resets occurred
during the subsequent out-of-band interval. On the next approach, even a new outward radar
track had to recover from the old distance at the normal 0.5 m/s outward limit.

The fix ends that stale observation interval. Persistence, dropout protection, and trust remain
continuous while `run` stays true. The existing summary is still emitted once. Inactive service
observation can now restart its 0.3 s entry dwell after a band crossing; a repeated 2.45/2.55 m/s
flap test confirms entry after 0.3 s of uninterrupted observation. No hysteresis was added.

The separate pre-entry **shadow** context has similar stale-state exposure above 4.5 m/s. It is
not changed here. Do not treat its old `gov_trace` pre-entry distances as raw radar or ground
truth. This analysis uses the raw radar stream and the live service context separately.

## Recorded-input comparison

`context_lifetime_replay.py` loads the actual shipped `longcontrol.py` from `ecd3fee` and compares
it with this change. Both arms enable the existing recovery brake. Other runtime files must be
unchanged. Source rlogs, both LongControl implementations, context/service/flags, and runner are
hashed. Missing source segments reset replay state. Full new-route replay includes segments 0–28.

Seventeen automated stop windows were inspected: the eight earlier comparators, the old bad
bookmark, and eight engaged stops on the new route. Seven have identical commands; ten change.
All seventeen retain the same service ownership in the inspected valid, active, pedal-free
last-10-second through rest+0.5-second windows. The full new-route replay also has zero ownership
differences across 173,211 frames. Replay summaries compare frames owned by either arm and count
ownership differences over all frames.

The older `20b8` packet does have 1,210 ownership differences outside those stop windows:
735 stationary brake-pedal frames, plus 475 frames of a far-lead approach that includes driver
gas override. At `39511203379481`, raw gap is 27.507 m but the old context says 7.359 m and
incorrectly enters APPROACH_GLIDE. With fresh geometry the service stays INACTIVE and the legacy
controller requests −0.1114 instead of −0.1364 m/s²: 0.025 m/s² less braking at that frame. A
regression test pins this far-lead false entry. The stationary group is reported, not declared
physically equivalent: pedal/ECU blending is not identified here. These mixed-pedal intervals are
not comfort comparisons; no claim of unchanged ownership is made for this packet.

Two new-route stops directly expose the stale-distance defect. Entry is the first baseline-owned
frame in that window; pulse depth is the minimum command during the following one second:

| New-route segment | Raw radar gap | Old context gap | Fixed context gap | Old pulse | Fixed pulse |
|---|---:|---:|---:|---:|---:|
| 20, not user-rated | 12.499 m | 8.494 m | 12.499 m | −1.010 | −0.545 |
| 22, bad bookmark | 10.800 m | 6.881 m | 10.800 m | −1.491 | −0.741 |

Commands are acceleration requests in m/s². Segment 22's full moving-owned baseline command
MAE is 0.0073 m/s², maximum 0.0554; segment 20's is 0.0038, maximum 0.0486. This supports input
reconstruction. The later segment-22 rebrake remains in fixed-input replay: the recorded rebound
is still fed into both controllers. Its removal in the physical car is **not** proved.

The coast estimate and command history also restart. This is material: other windows can ask
for more brake, up to 0.316 m/s² in segment 12. The old bad bookmark changes by at most
0.0092 m/s², so this fix does not resolve every cause of its rebound. The first new-route
automated stop (segment 3) and the new segment-24 stop are command-identical.

Fidelity limits remain: the first earlier `20b8` segment-16 stop has baseline MAE 0.092,
maximum 0.199 m/s²; it is not a precise command oracle. New segment 6 has logged input ages up
to 0.125 s. Other inspected moving-owned windows have ages below 0.095 s. The replay approximates
subscription timing, uses initial toggle snapshots and Hyundai limits, and holds motion fixed.

## Validation and deployment interpretation

All 24 natural-release lifetime cases failed on the old code and passed after the
fix. Added coverage includes all three service modes, disengagement, out-of-band exit, same/new
tracks, inward/outward next gaps, exactly-once summary, and band flaps. The existing stopping,
context, governor, recovery, LIVE, LIVE_TERMINAL, and tracking-trim suites passed (300 checks).
The full controls-lib suite also passed: 754 passed, 19 skipped; ruff passed on the changed Python
files. Independent plan and final reviews found no blocking runtime issue. Review findings about
reproducing the table, ownership accounting, and runtime source binding were addressed in the
offline tools. The committed audit reproduces the 17-stop table byte for byte.

A separate two-episode simulation uses actual LongControl and StopContext with a simple lagged
actuator. Across 24 combinations of gain 0.8/1.0/1.2, lag 0.15/0.4 s, delay 0/0.3 s and load
−0.1/+0.2 m/s², all arms stopped and held for one second without creep. Rest gaps changed from
4.597–7.764 m to 4.475–5.686 m. Maximum 300 ms acceleration jerk changed from 2.855 to 2.638;
individual cases can worsen by 0.073 m/s³. This is a regression probe, **not a validated vehicle
model**; it neither establishes the real rest gap nor proves superior comfort.

Full-motion fitting, simple actual-command estimator feedback, and generic acceleration damping
were investigated before selecting this fix. Fits did not reproduce the bad-stop rebound and
travel together. Feedback correction alone worsened coupled simulations. Damping improved some
cases but regressed others. None is enabled. The manual motion references do not identify the
actuator response, especially under the fixed terminal hold command. Terminal ease alone remains
insufficient to rank whole-stop comfort. A larger learned or motion-reference controller needs
better response validation; clearing stale context is useful independently of that architecture.

This candidate is ready for device evaluation after code review and checks. It retains the 4–5 m
target and 3 m floor logic. The next recorded drive must assess unnecessary pulses, speed recovery,
body motion, final ease, rest gap and hold together against the explicit manual references. Also
check for a new early pulse at 2.0–2.5 m/s after the coast estimate restarts. Neither
a green test run nor a lower replay command proves that the car now stops better than Radek.

## Local evidence and reproduction

Artifacts are under `~/.route_sync/corpus/radek_baseline_20260912/`:

- `live_20c0_full/{baseline,signals,context_replay}.json`: 29 complete segments and both commands.
- `labels_with_20c0.json`, `marked_comparison_with_20c0.json`: explicit labels and 21 descriptors.
- `{000020b7,000020b8,000020bc,bad_stop_34}/context_replay.json`, `context_comparison.json`: 17-stop audit.
- `context_lifetime_new_bookmark.png`: raw/context distance, old/fixed commands, recorded speed.
- `context_lifetime_sweep.{py,json,log}`: reproducible synthetic probe, separate from road evidence.

```sh
source .venv/bin/activate
python tools/stopping/review/context_lifetime_replay.py --output /tmp/context_replay.json \
  ~/.route_sync/data/media/0/realdata/000020c0--418683bc8f--{0..28}/rlog.zst
pytest selfdrive/controls/lib/tests/test_stopping_context_lifetime.py
python tools/stopping/review/context_comparison.py \
  --labels ~/.route_sync/corpus/radek_baseline_20260912/labels_with_20c0.json \
  --output /tmp/context_comparison.json \
  ~/.route_sync/corpus/radek_baseline_20260912/{000020b7,000020b8,000020bc,bad_stop_34,live_20c0_full}
```

`context_evidence_manifest.json` SHA256:
`1e28a8b60b1730428b61de3dc8306af59a5f980e6b845f3ff3cb8e6260fb536e`.
All five replays' runtime-source hashes and source rlog hashes were independently rechecked.
