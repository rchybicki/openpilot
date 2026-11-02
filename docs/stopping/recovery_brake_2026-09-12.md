# Limit profile-recovery brake release — cycle 55

Radek asked for work to continue until a stopping improvement is ready for the device. This change limits
the unnecessary brake release in the segment-34 regression. It is a bounded first step for supervised
device evaluation. It does not establish better physical stopping or solve the earlier brake onset.
The 11 explicitly labelled manual stops remain the desired minimum; the bad bookmark remains a negative case.

## Change and invariant

The original governor tries to regain its profile speed after excessive early braking. At the bad stop it
requests positive net acceleration while ego speed is about 0.72 m/s. The phase reaches its shallow -0.03
command ceiling; attributed safety then permits release of the planner's deeper brake. Speed rises to 0.95
m/s and the governor brakes again. There is no phase handoff in this sequence.

`GOVERNOR_RECOVERY_BRAKE` limits release only in APPROACH_GLIDE, above 0.50 m/s, while closing on a trusted
lead and below the selected ego-speed reference. It uses closing-speed braking over the existing lag-adjusted
remaining margin. The net reference is bounded at `GOV_A_C=0.60`; the existing coast compensation and planner
limits still apply. No fitted coefficient, learned model, new Params key, new state or new owner is added.

The phase adjustment is `min(old_phase, max(last_cmd, recovery_reference))`. Thus:

- If the phase already demands deeper braking than `last_cmd`, it is unchanged.
- Otherwise, the result lies between `last_cmd` and the old phase. It can hold or limit release, but cannot
  create a deeper command step. An existing safety demand below `last_cmd` still binds and retains `J_SAFE`.
- The floor is the current service command, including a takeover reseed. It cannot restore brake already
  released before takeover. Terminal descent and hold keep their laws but can inherit a changed approach seed.

The original `governor_demand` and its `gov_*` shadows stay unchanged. `a_phase`, `attr_released`,
`attr_unexplained` and binding counts can change; do not compare their distributions without the activation SHA.
The selected reference is an ego speed in both states of `GOVERNOR_PROFILE_REFERENCE`. Both flag combinations
are tested. The legacy-only late-entry corridor cannot arm under this governor law.

## Evidence and limits

`service_replay.py` runs real LongControl with the recorded CarParams, planner, radar, control activity,
model stop provenance and force-coast inputs. Both arms keep recorded motion. The immutable rlog hashes,
source hashes, runtime flags, settings, skipped startup frames and resets at missing segments are saved.
It approximates subscription timing with the latest messages at each carControl publication. It does not
reconstruct live toggle changes. Gaps and poor agreement must remain visible.

Packet root: `~/.route_sync/corpus/radek_baseline_20260912/`. Each named subdirectory now contains
`recovery_replay.json`. The bad stop is the exact existing label, rest mono `4774151422860`.

| Recorded-input check | Result |
|---|---|
| Bad stop, original command reproduction | 599 frames; MAE 0.00164 m/s², maximum error 0.0483 |
| Bad release interval, original → candidate shallowest command | -0.0752 → -0.1422 m/s² |
| Maximum command difference below 0.50 m/s, same incident | 0.00559 m/s² |
| All owned moving frames in segments 32–35 | 926; 364 changed; maximum deepening 0.0670, release 0.00040 m/s² |
| Eight prior complete engaged comparison windows | Candidate command unchanged in all eight |

Prior windows: 20b7 s3; 20b8 s4 and two stops in s16; 20bc s3, s17, s18, s37. Seven reproduce the original
command with MAE 0.0025–0.0083 m/s² and maximum error ≤0.071. The first 20b8 s16 stop has MAE 0.092 and
maximum error 0.199: retain it as a limited-fidelity comparison, not a passed reproduction gate. Whole-route
errors are larger, up to 1.31 m/s², outside these complete windows; the tool reports them without filtering.
These are previously engaged examples, not newly asserted user ratings of "good".

All reported rest gaps, speeds and physical jerk in the route packet remain measurements of the original
drive. The candidate command does not predict a new rest gap, reduced surge, body jerk or comfort score.
The invariant compares a single step from the same controller state. Different subsequent histories can
produce small wire releases relative to the original replay, including the reported 0.00040 m/s².

The new tests cover the release bound, unchanged safety lanes and urgent braking, both reference modes,
following/departure, hot entry, dropout/untrusted gaps, no lead, terminal band and takeover reseeding.
Sixteen synthetic actuator cases span lag 0.15–0.60 s and known constant push 0.10–0.50 m/s². Both arms stop
in the 4.0–5.1 m band and hold securely, with rest-gap change ≤0.10 m. This is a controller regression test,
not a validated Santa Fe model or evidence that the physical surge is fixed.

## Rejected changes and review

The estimator currently receives a pre-service command. Correcting it changes this incident only slightly
and does not remove the release; it is deferred as a separate correctness task. The test plant's `aEgo=0`
defect was corrected separately in `ecf0c623f1`; 69 existing tests passed with unchanged expectations before
the new feature was added. That result validates the fixture correction, not feature coverage.

A local linear response fit trained before the incident under-predicts held-out travel by 0.239 m out of
4.106 m, with acceleration RMSE 0.190 m/s². Its full-controller rollout predicts surge 0.015 instead of the
recorded 0.239 m/s. It fails as an outcome comparator; its candidate rankings were discarded.

Plan red-team used the permitted Opus fallback after Fable exhausted its quota. Adopted consumption-site
scoping, trust and terminal exclusions, bounded authority, unchanged shadow definition and explicit outcome
limits. Testing exposed a safety-rate demotion in the first draft; the release-only invariant above fixes it.
Rejected the later corridor concern because its entry is explicitly `not governor_law`; existing tests pin
that exclusion. Kept the selected absolute speed reference and added both-mode coverage.

Final independent Opus review found no blockers for bounded supervised evaluation and independently ran
all 277 selected tests successfully. Main-agent lint and diff checks also pass. The 1,500-case sweeps in
each reference mode are entry-frame checks; trajectory coverage comes from the 16 synthetic cases and replay.

## Reproduction and vehicle evaluation

Activate `.venv`. Run `python tools/stopping/review/service_replay.py --output <result.json> <rlogs...>` using
the exact `sources[*].path` list from the frozen replay packet. Run the stopping/context/governor/attributed
and LongControl service/terminal suites, including `test_stopping_recovery.py`, with the flag enabled.

Activation is a separate one-line commit; revert is `GOVERNOR_RECOVERY_BRAKE=False` plus the normal updater.
The first vehicle comparison must retain approach surge, repeated braking, rest gap, terminal body motion,
secure hold, departure and Radek's explicit rating. Earlier unnecessary braking and the broader stop
architecture remain open. No physical improvement or superhuman performance is claimed before that evidence.
