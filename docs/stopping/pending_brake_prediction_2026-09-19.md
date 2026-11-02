# Pending brake-request prediction, 2026-09-19

This change reduces repeated brake demand while an earlier request is still taking effect.
It is a qualified offline improvement for the next driving cycle. Real stopping comfort,
and superiority to Radek's manual stops, still require road evidence.

## Runtime change

LongControl retains a complete, timestamped 300 ms window of finalized nonpositive
acceleration requests. The window uses the existing response-delay constant. Its
time-weighted mean minus the oldest held request estimates additional pending braking.
Only a negative difference contributes to the existing comfort prediction.

controlsd records requests after acceleration limits, gas arbitration, finite-value
sanitization, cruise override calculation, and successful carControl publication.
This is request history; it is not CAN-transmission or brake-force confirmation.
Startup, gas/brake override, inactivity, input/ACC faults, invalid or stale subscribed
data, non-increasing time, and request gaps clear the window. Re-entry must collect
another full window. Source freshness uses existing SubMaster alive timeouts and
source timestamps; no new timing parameter or Params setting is introduced.

The existing coast observer, raw safety demands, recovery demand, terminal descent,
hold and release rules remain unchanged. The comfort forecast retains its existing
previous-command bound. No model, planner, CAN, or SCC14 jerk-limit change is included.

## Evidence

The [September 18 census](new_route_cycle_2026-09-18.md) remains the source cohort:
57 downloaded routes, 466 stops, and 19 fully automatic final-band stops. The manual
reference remains the same 11 explicitly attributed Radek stops; other drivers are excluded.

- All 3,888 paired stress cases reproduce the frozen baseline exactly. The candidate
  introduces no new 3 m floor crossing, incomplete stop, or creep. Existing baseline
  failures remain: 297 floor crossings and 14 incomplete stops in adverse hypothetical
  plants. No already-below-floor case loses clearance with this candidate.
- Native planner/controller simulations use all 19 observed entry geometries with
  two fixed hypothetical plants. Moving negative jerk improves in 33 of 38 scenarios,
  is unchanged in three, and worsens slightly in two (at most 0.00272 m/s³).
  Maximum additional stopping time is 0.13 s. Terminal-phase tradeoffs remain;
  the largest increase in negative-jerk magnitude is 0.0170 m/s³.
- The original FA5-like native case improves moving negative jerk by 10.5%
  (−1.41824 to −1.26927 m/s³), with stop time +0.02 s and rest gap −0.01784 m.
- Chronological recorded-input replay reduces final-band negative **request** jerk
  in 18 of 19 automatic stops, with one unchanged. Peak brake request improves in
  16, with three unchanged. FA5's first-second minimum is −1.91393 → −1.65327 m/s².
  Positive request jerk has two small regressions (at most 0.00239 m/s³).
- No manual/gas/brake final request changes. All 8,164 negative requests suppressed
  by recorded override arbitration receive zero prediction credit. Three F8 ownership
  differences are earlier RELEASE completion by one or two control frames; credit
  is zero at those transitions.
- The concrete runtime LongControl/Service reproduces all 38 complete native traces
  and all 3,888 candidate stress results exactly. The targeted controller, boundary,
  recovery, publication and new request-history suites pass: 532 tests.

Recorded-input replay holds measured motion fixed and cannot predict physical jerk
or stop gap after changing a request. Timestamp ordering is necessary: file-order
replay sometimes associates future carState samples with earlier carControl messages.
Both chronological arms use the same ordered inputs. Owned-wire baseline mean error
is 0.00358/0.00352/0.00417 m/s² for FA/F9/F8; maximum error reaches 0.244 m/s² on F8.
The simulations use uncalibrated hypothetical plants, not fitted route predictions.

The planner roll-in deletion, weak-braking surplus change, acceleration-tracking
variant, and lag-profile correction remain rejected or offline. The profile correction
creates another terminal dip and longer stops in some cases; it is not combined here.

Frozen scripts, raw-source references, traces, comparisons and hashes are under
`~/.route_sync/corpus/stopping_improvement_20260919/`, including `pending/`,
`native_cohort.json`, and `runtime_candidate/runtime_equivalence.json`.
The next cycle must compare all automatic stops and the unchanged personal reference,
including onset dips, later rebraking, terminal jerk, rest gap and post-stop motion.
