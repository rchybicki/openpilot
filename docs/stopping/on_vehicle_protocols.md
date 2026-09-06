# On-Vehicle Protocols

Staged enablement procedures for the CAN-semantics changes that the redesign plumbed dark, plus
the first-drive checklist for any kill-switch flip. Vehicle: 2022 Hyundai Santa Fe HEV.

## 0. Ground rules (every protocol)

- **One constant change per session, deployed alone** — never combined with any other change.
- Deploy: push branch, then `ssh -tt comma 'cd /data/openpilot && ./fullupdate.sh'` (fallback
  `commawifi`); verify `git rev-parse --short HEAD` on-device.
- Any fault ⇒ revert the constant, redeploy, verify hash, record the finding in
  `docs/stopping/worklog.md`.
- Watch signals (CAN capture): TCS13 `ACCEnable` (any nonzero = accFaulted), TCS13 `PBRAKE_ACT`,
  TCS15 `AVH_LAMP`; plus openpilot `accFaulted` events, FCW/AEB alerts.
- **Sequencing:** stage ordering and entry criteria live in the living
  [rollout plan](rollout_plan.md) (re-scoped 2026-06-10: the StopReq and dynamic-jerk stages are
  independent of the V2 flip — both constants live in the carcontroller, downstream of controller
  dispatch, and apply to whichever controller is active. They start only after the rollout plan's
  baseline-corpus stage exits). This doc remains authoritative for the per-stage mechanics,
  thresholds, and promotion criteria below.

## 1. StopReq escalation (carcontroller constants)

Background: the fork gates StopReq at `vEgo < 0.01` (fork deviation, no recorded rationale;
upstream asserts it for the whole stopping state). Receivers are EPB and ESC; behavior while
moving is the protocol's deliberately-last unknown. Kalman vEgo dithers near zero, so the raw
gate can chatter at 50 Hz once asserted.

Constants (in `opendbc_repo/opendbc/car/hyundai/carcontroller.py`):

- `STOP_REQ_MAX_SPEED` — assert gate, m/s. `0.01` == legacy.
- `STOPREQ_RELEASE_SPEED` — latch speed-release, m/s. **Always active at every stage**: the latch
  may NEVER hold StopReq on a rolling car. `0.10` sits just below the 0.104 m/s wheel-speed
  standstill threshold. On this HEV, creep-push events that roll the car to 0.2–0.3 m/s while the
  long state stays `stopping` are routine — a state-exit-only latch would assert StopReq to
  EPB+ESC on a moving car. Legacy deasserts the instant vEgo ≥ 0.01; with the speed release,
  stage 0 is conservative at standstill only.
- `STOPREQ_LATCH` — `False` == legacy chatter-prone gate (deployed). `True` enables the latch:
  set on `stopping ∧ vEgo < STOP_REQ_MAX_SPEED`; cleared when the state leaves stopping OR
  `vEgo > STOPREQ_RELEASE_SPEED`.

### Stage table (exact values; one stage per session)

| Stage | Change | What it probes | Promotion criteria |
|---|---|---|---|
| 0 | `STOPREQ_LATCH = True` @ gate 0.01, release 0.10 | deassert-side chatter fix — conservative at standstill only (StopReq held while wheels are provably stopped; cleared the moment vEgo exceeds the wheel-standstill threshold OR state leaves stopping) | ≥ 10 stops + 1 × 60 s standstill hold + **1 deliberate creep-push/hill stop with the latch active** (verify StopReq drops as the car rolls); no TCS13 `ACCEnable != 0`, no `PBRAKE_ACT`, no TCS15 `AVH_LAMP`; clean launch with `startAccel = 0.7` |
| A | gate → 0.04, release 0.10 | below wheel-standstill 0.104 m/s — wheels provably stopped; isolates the Kalman-dither band from genuine rolling | same as stage 0 |
| B | gate → 0.10, release 0.12 | still at the wheel-standstill edge; full dither band covered | same as stage 0 |
| C | gate → 0.35–0.50, release = gate + 0.05 | upstream managed-stop contract; StopReq-while-rolling probed last, deliberately, in isolation | same as stage 0 + one EPB/auto-hold watch ≥ 60 s, launch verified |

Only after stage C passes may Phase 2 consider letting the ESC manage the final ~0.3 m.

## 2. Dynamic SCC14 jerk (`DYNAMIC_SCC14_JERK`)

Deployed: `False` — static 3.0 (pid) / 1.0 (stopping) upper, 5.0 lower. The dynamic path derives
the true 50 Hz command slope inside the SCC send block (`(accel_sent − accel_last_scc)/(2·DT_CTRL)`,
updated only on sent frames) plus `SCC14_JERK_MARGIN = 0.5` m/s³, **floored at the legacy static
values** (the dynamic path may only ADD headroom over what the car runs today, never advertise
less). Both jerk fields are unconditionally clipped to [0, 12.7] before packing in every mode
(CANPacker wraps out-of-range — 13.0 would become 0.2 m/s³ and the ESC would rate-limit releases).

Enable protocol:

1. Precondition: StopReq enablement settled at its chosen sub-stage ([rollout plan](rollout_plan.md)
   stage 4 entry; applies to whichever controller is active).
2. Flip `DYNAMIC_SCC14_JERK = True` alone; deploy; verify hash.
3. First session: compare commanded-vs-realized accel slew on identical stops (rlog CAN capture),
   static vs dynamic.
4. Alert = realized accel rate-limited below planned jerk, or any `ACCEnable != 0`. Any alert ⇒
   revert the constant, record.

## 3. dRel-honesty flip (`PUBLISH_TRUE_LEAD_DISTANCE`, commit 10)

Pre-flip check (hard gate): read on-device `IncreasedStoppedDistance` + the 4 weather variants
via `tools/stopping/device_stop_settings.py` (read-only PARAM_SPECS rows). If all are 0 the flip
is a provable no-op — assert via replay on recorded routes before flipping. For ISD > 0 the
following distance grows by ISD and the queue-stop resting gap moves ISD farther back —
**EXPECTED, not anomalous** (the new single ISD meaning). The longcontrol stopping layer is exact
for any ISD by construction (single-point `lead_d_rel_eff` compensation); planner-side Santa Fe
lead-gap caps and moving-lead consumers shift by ISD (accepted residual, scheduled retirement).

## 4. First-drive checklist (the V2 flip and any kill-switch flip)

Empty-lot session first:

- [ ] 5 no-lead crawl stops
- [ ] 5 explicit-target stops behind a parked car
- [ ] 2 uphill + 2 downhill stops, including one deliberate hill-rebound/creep-push (exercises
      the arrest path)
- [ ] 1 stop with brake-tap override — expected: full USER_DISABLE disengage + chime, brakes
      released on lift; verify clean re-engage with fresh tracker state (the system does NOT keep
      stopping through a tap)
- [ ] 1 green-light departure release

Watch live during the session:

- [ ] `longControlState` transitions sane (no stopping-state chatter)
- [ ] **post-cap (sent)** accel vs `aEgo`: no step > 0.10 m/s² between frames on the sent signal
      (the Santa Fe quirk caps mutate the command after the facade — judge the sent signal, not
      the facade output); no command < −1.4 m/s² below 0.1 m/s
- [ ] rollout < 2.0 m from 2 m/s; final hold gap 2.5–5.0 m
- [ ] no `accFaulted` / `AVH_LAMP` / `PBRAKE_ACT`; no FCW/AEB alerts; clean resume

Then 3 commute drives before declaring soak start. Any anomaly: flip the constant back,
`fullupdate.sh`, verify hash, re-run the last good route's telemetry to confirm reversion.
Soak = ≥ 2 weeks of V2 drives with paired on-road stats (eval.md section 3) showing no
harsh/leapfrog regression against the frozen scoring config.
