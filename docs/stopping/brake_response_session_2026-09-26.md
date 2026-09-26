# Brake-response test program KCS1/KCS2: session runbook (revised 2026-09-26 evening)

**Current block: KCS2, revised 26 September evening** (the device runs `PLAN_ID = "KCS2"` after the next deploy).
KCS1 (B, A, C, D, E; 6 reps each) was completed on 26 September on routes 0000212e, 0000212f, 000021ef and 000021f0;
analysis: `~/.route_sync/corpus/kcs1_drive1_20260926/ANALYSIS_1.md` and `ANALYSIS_2.md`. The first KCS2 table (G, F,
H) never reached the car (the deploy timed out); both plan red-teams of the stopping decision
(`~/.route_sync/corpus/stopping_decision_20260926/DECISION_v2.md`) replaced it. KCS2 pairs held and released commands
at the same level, in the pid state as the normal chain runs them, to set the release floor of the next stopping law:

| id | Script (from the 20 km/h cruise) | Question |
|---|---|---|
| P | -0.7 to the stop | The uninterrupted control for K; first rep of the session (site and hold check) |
| L | -0.5 to the stop | Held from the cruise: the pair for I/M/N, and the second-day control for KCS1 A |
| I | -1.0 to 2.5 m/s (9 km/h), then ease to -0.5 at 1.5 m/s^3, to the stop | Is a release to -0.5 held at the start of the pump band? |
| M | -1.0 to 1.5 m/s (5 km/h), then ease to -0.5 at 1.5 m/s^3, to the stop | The same at pump speeds |
| J | -1.0 to 1.5 m/s (5 km/h), then ease to -0.6 at 1.5 m/s^3, to the stop | Is -0.6 held where -0.5 is not (pairs with M)? |
| N | -1.0 to 0.8 m/s (3 km/h), then ease to -0.5 at 1.5 m/s^3, to the stop | A deep capture's landing |
| K | -0.7 to 1.9 m/s (7 km/h), -0.45 for 0.5 s, then -0.7 to the stop | Does route 2129's fade under a constant -0.7 come back? |

6 reps each (42), fewest done first (P, L, I, M, J, N, K, P, ...); the counts persist, so the block can span two
drives. Changes against KCS1 for the driver:
- The stop intent now comes at 0.5 m/s (2 km/h), not 2 m/s: the car stays in normal cruise control (pid) until then,
  as it does in a real stop. A press or an abort above 2 km/h releases the braking and cruise takes over again (it
  can accelerate); below 2 km/h the stop finishes and holds.
- The eases are ramps, not steps: the braking lightens over about 0.3 s.
- The banner asks for the brake after 5 s (KCS1: 3 s), so the holds also show the car's StopReq hold (about 2.3 s
  after the stop).
- If you can, do 2-3 of the L reps on a gentle downhill (a held -0.5 into the stop, then the -0.7 hold build).
  Alternate the directions on one stretch as before.
The progress file restarts at zero for the new table (a KCS1 or old KCS2 record gives zero counts). Everything below
applies unchanged except the maneuver tables and the intent speed; the KCS1 text is kept as the reference.

Status: implemented on 26 September without red-team or adversarial review, at Radek's request (fast-iteration day).
Verification is host tests only. No rep has run on the car. The session build has `IDENTIFICATION_HOOK = True`.
Plan: `~/.route_sync/corpus/test_program_plan_20260926/PLAN.md`. Where the plan and the code are different, the code
and this runbook apply. This version replaces the 30 km/h one-step procedure (-0.5 m/s^2 for 3 s) of the earlier
26 September versions.

What a rep is: one scripted open-loop braking run from a steady 20 km/h cruise to a held stop. The device cycles a
fixed table of 5 maneuvers (B, A, C, D, E). The commands are constant, from -0.3 to -1.0 m/s^2 (D also has 2 s at
0.0). Each maneuver needs 6 counted reps: 30 stops in total. The block measures how the car responds to a known
command from 5.56 m/s to rest: delay, gain against depth and speed, release, creep, the last metre and the hold. It
does not rank a stopping law and it does not prove better stops. There are no ratings.

## Part A: operator (driver)

The driver is responsible for steering and pedals at all times. The test never commands positive acceleration.
After a cancel, normal cruise takes over again and CAN ACCELERATE to the set speed. The only cue is the on-screen
banner. There is no tone.

### A0. What the test build changes (for every drive while it is installed)

- The wheel distance button does only the test functions (A3). Force Coast, personality, Traffic, Experimental and
  pause actions are not available from it (short, long or very long). The on-screen distance button is ignored.
- The personality is Standard. Traffic mode is off (also from an LKAS mapping). Saved settings are not changed.
- Ordinary engagement is not changed. SET engages at the saved Initial Set Speed (160 km/h on 26 September) or at
  the current speed if faster. RESUME returns to the last set speed.
- Every long press of the distance button sets the set speed to 20 km/h. If openpilot is engaged, the change is
  immediate. If it is not engaged, the change occurs at the next engagement (SET or RESUME). This also applies to the
  long press that turns test mode off, and to a long press during a rep or a hold. At a higher speed the car slows
  to 20 km/h, as after any set-speed change. While the gas pedal is pressed the set speed follows the car as usual:
  release the gas before the long press.
- The device keeps a count of the completed reps (A5). The count stays after a restart.
- Nobody else drives the car while the test build is installed. A long press (a personality habit) followed later
  by a short press (a Force Coast habit) can start a scripted stop. Remove the test build (B5) before anyone else
  drives.

### A1. Site go/no-go (before the first rep; parked)

- Measure the length of the straight that is continuously flat, clear and has good grip. A turn or a downhill part
  ends it. A -0.3 or 0.0 command has the same size as a 1-3 % grade, so use flat road only.
- Mark the END of the usable straight (a fixed object before the turn).
- 20 km/h needs at least 90 m of usable straight. Put a LAST-PRESS marker 55 m before END (the longest press-to-rest
  distance, 33.4 m, plus 20 m margin, rounded up). Start each rep from rest at least 35 m before the LAST-PRESS
  marker (launch to 20 km/h 22 m, plus the 2 s READY wait 11 m).
- Distances per rep at 20 km/h (from PLAN.md section 2). They assume a NOMINAL car response: 0.45 s delay, gain 1.0,
  no creep push unless stated. A slower real response, a grade or a late press makes every row longer.

| Rep | Banner profile (m/s^2) | Press to rest | Footprint from RESUME |
|---|---|---:|---:|
| B | -1.0 to stop | 17.9 m | 71 m |
| A | -0.5 to stop | 33.4 m | 86 m |
| C | -1.0 to 9 km/h, -0.3 to stop | 25.2 m (28 m if -0.3 stalls at 1.0 m/s) | 78-81 m |
| D | -1.0 to 9 km/h, 0 for 2 s, -0.8 to stop | 23.7 m (32.6 m with +0.25 creep push) | 77 m (86 m) |
| E | -0.8 to 5 km/h, -0.3 for 2 s, -0.8 to stop | 23.3 m (33.2 m with +0.25 push) | 76 m (86 m) |

- Footprint = launch from rest at 0.7 m/s^2 (22 m) + READY 2 s (11 m) + press to rest + 20 m margin.
- Short site (60-90 m): use 15 km/h (B2 values). After arming, lower the set speed to 15 km/h with SET- and check
  the display. Put LAST-PRESS 40 m before END. Less than 60 m: NO-GO.
- No other people or vehicles near. No vehicle close behind: the brake lamp is off in 72-100 % of frames at -0.5 and
  shallower above 2.5 m/s (it is on at standstill). Whether in-lane stops on a public road are acceptable is
  Radek's decision.
- Do not accelerate harder, raise the speed or use a downhill to make the site fit.

### A2. Settings

- Change nothing. The test build ignores the saved distance mappings (2/1/6) and does not change them. The saved
  Initial Set Speed stays in use for an ordinary SET (A0). After a stop, use RESUME, not SET.

### A3. Buttons and banners

| Gesture | In state | Result |
|---|---|---|
| Distance button held 0.5 s (long press) | OFF | Test mode ARMED at the 0.5 s mark. No braking. The rest of that press does nothing. Set speed 20 km/h (now if engaged, else at the next SET or RESUME). |
| Long press | ARMED or READY | Test mode OFF. Set speed 20 km/h (as above). |
| (nothing) | READY | After a 2 s countdown the maneuver shown on the banner starts by itself. Holding the button pauses the countdown. |
| Short press, released in less than 0.5 s | READY | Starts the maneuver at once, 50 ms after the release. |
| Short press | OFF, ARMED (not ready) | Nothing. It is not remembered for later. |
| Any press | ACTIVE, second line `press = cancel (releases, cruise resumes)` | Cancels at once. The braking releases at 0.8 m/s^3 (at most 1.25 s from -1.0). Then normal cruise resumes and can accelerate. Not counted. Test mode stays ARMED. |
| Any press | ACTIVE, second line `press = finish and hold` | The current command stays until the car stops, then the car holds. Not counted. |
| Any press | HELD | Ignored by test mode (a long press still sets 20 km/h). |
| Long press | LOCKED | Shows why it is locked. It stays locked. |
| Brake pedal | HELD | The normal end of every rep. The hold ends, openpilot disengages, test mode stays ARMED. The rep counts if the hold lasted at least 1.0 s. |
| Brake pedal, gas pedal or cruise cancel | ACTIVE (moving) | The test releases braking control at once. Test mode OFF. Not counted. A new long press is needed. |
| Gas pedal or cruise cancel | HELD | Test mode OFF. Not counted. Never use the gas to leave a hold (A4). |
| RESUME | disengaged after a stop | Engages at the last set speed (20 km/h). Normal cruise launches the car. |
| SET | disengaged | Engages at the saved Initial Set Speed (160 km/h), except after a long press while disengaged (then 20 km/h). Do not use SET after a stop. |

The second banner line during a rep tells what a press does. It shows `press = finish and hold` from the stop
intent: at 2.0 m/s in the braking-to-stop segments (not in D's 2 s segment at 0.0).

| Banner (first line / second line) | Meaning |
|---|---|
| (none) | Test mode OFF. |
| `TEST ARMED - waiting: <reason>` / `next B 1/6: -1.0 to stop; long press = off` | Armed. A start condition is not met yet (`settling` = all met for less than 2 s). The second line shows the next maneuver and its rep number. |
| `TEST B 1/6 STARTS IN 1.4 s` / `B: -1.0 to stop; brake = not now` | Every start condition held 2 s; the maneuver starts by itself when the countdown ends. |
| `TEST B 1/6 s1 -1.00 - 4.3 m/s` / `press = cancel ...` or `press = finish and hold` | The script brakes: segment, command, speed. |
| `TEST B 1/6 ABORTED - <reason>` / `releasing; cruise resumes and can accelerate` | Aborted before the stop intent. The release runs, then normal cruise. |
| `TEST B 1/6 ABORTED - <reason>` / `finishing the stop; brake to end` | Aborted after the stop intent. The car stops under the current command and holds. Not counted. |
| `TEST B 1/6 STOPPED - hold 1.2 s` / `brake to finish` | Held at rest. From 3.0 s the first line ends with `- BRAKE NOW`. `(not counted)` follows after an abort. |
| `TEST ARMED - waiting: disengaged` / `next A 1/6: -0.5 to stop; long press = off` | After the brake. A counted rep moves the next maneuver on. A rep that did not count shows the same maneuver and rep number again. |
| `TEST MODE OFF` / `long press distance to arm` (3 s) | Turned off with a long press. |
| `TEST B 1/6 ABORTED - pedal` (or `disengaged`) / `test mode off; long press distance to arm` (3 s) | The driver ended a moving rep. |
| `TEST MODE LOCKED - <reason>` / `restart the car to use test mode again` (3 s) | A fault. No more reps until the car is switched off and on again. |
| `TEST LOCKED - <reason> - HELD` / `brake to end; restart the car` | A fault while stopping or held. The hold stays until the brake. |
| `TEST BLOCK COMPLETE - long press = off` / `plan KCS1: 6 reps of every maneuver` | All 30 reps counted. No more starts. |

After the brake, the rep result (`B 1/6 DONE` or `B 1/6 NOT COUNTED - <reason>`, second line `next ...`) shows for
3 s. The script owns the braking command for the whole rep and the hold; a lead, stop, FCW, steering, press or fault aborts it.

A button held while openpilot starts, or while a fault is reported, never counts. Release it and press again.

### A4. Drive procedure

1. Parked, ignition on. The build is installed and verified (B4). B3 shows the saved counts. The site is measured
   (A1).
2. Arm: long press. `TEST ARMED` shows. Engage with RESUME or SET if not engaged yet, or continue if engaged. Check
   that the displayed set speed is 20 km/h.
3. Drive on the straight at 20 km/h: no car ahead, wheel straight, no blinker, feet off the pedals, mirror clear.
   Hazard lights can read as a blinker (`waiting: steer`); keep them off.
4. Wait for `TEST <id> <n>/6 STARTS IN ...`. READY needs, continuously for 2 s: openpilot longitudinal
   engaged in normal cruise control; measured speed 12.6-32.4 km/h and within 1.1 km/h (0.3 m/s) of the set speed;
   acceleration within 0.2 m/s^2; planner target within 0.15 m/s^2; no planner or controller demand deeper than the
   maneuver's first command; no lead (radar, or model probability 0.1 or more); no stop sign or stop target within
   200 m; wheel within 5 degrees; yaw rate within 0.03 rad/s; no blinker; no pedal; Drive gear; no fault. If
   `waiting: settling` stays at a steady speed, the speed is not within 1.1 km/h of the set speed.
5. The countdown runs 2 s, then the maneuver starts by itself (a short press starts it at once). Read which maneuver
   the banner shows and check the road ahead and the mirror. If the start would come after the LAST-PRESS marker,
   brake (the rep does not start; test mode stays armed) and come back.
6. The car brakes under the script and stops. Hands on the wheel, foot over the brake. Before the stop intent
   (second line `press = cancel ...`), a lead, a stop sign, a steering input or a speed rise ends the rep: the
   braking releases and normal cruise takes over (it can accelerate). After the stop intent (`press = finish and
   hold`, from 2 m/s) the stop always finishes and holds. While the rep runs and holds, the script owns the braking
   command; after an abort any deeper openpilot braking passes.
7. The car holds (`STOPPED - hold`). On the FIRST hold of the session, watch the cluster for 20 s (auto-hold lamp,
   parking brake, any fault) before you brake. Later holds: 3-10 s. `BRAKE NOW` shows at 3 s.
8. Press the brake pedal to end the hold. The rep counts after 1.0 s of hold. openpilot disengages. The banner shows
   the next maneuver.
9. Turn around if necessary. Press RESUME (not SET): the car launches to 20 km/h under normal cruise. Check the set
   speed shows 20 km/h. Continue at step 4.
10. At the end: long press, test mode OFF. Ignition off, copy logs (B6), remove the build (B5) before anyone else
    drives.

- Never use the gas pedal to leave a hold. A gas take-off from a no-lead StopReq hold latched an ACC fault 2 times in
  2. An ACC fault needs an ignition cycle. Use the brake, then RESUME.
- Abort while moving: use the brake pedal. It ends test braking at once and turns test mode off (re-arm with a long
  press). A press above 2 m/s also cancels, but cruise then accelerates again: use it only when the road ahead is
  clear.
- If the car does not stop within 30 s after the press, test mode locks (`vehicle`). Stop with the brake.
- Below 9 km/h, if the car slows less than 0.15 m/s in 2 s, the command deepens to -0.70 m/s^2 at 0.6 m/s^3 and
  continues to the stop (stall rule). The rep still counts.

### A5. Progress

- A rep counts only if the car stopped under the script (no abort), the hold lasted at least 1.0 s, the brake
  pedal ended it and test mode was not locked. A cancel, an abort for any reason, a hold shorter than 1.0 s, the
  gas pedal, a cruise cancel or the brake while moving does not count.
- Order: the device shows the maneuver with the fewest counted reps; ties go in the order B, A, C, D, E. The
  sequence is B A C D E B A C D E ... A rep that did not count shows the same maneuver again. There is no skip.
- Do one rep per pass and turn around after each rep. Because the table has 5 maneuvers, each maneuver then runs 3
  times in each direction. 30 reps take about 30-36 min, plus about 5 min of checks.
- Persistence: after each counted rep the device saves the counts to `/data/identification_progress.json`. The
  counts stay after a restart, an ignition cycle and on the next drive. Arming never stays: every start is OFF, and
  a long press is needed again. A lock also ends at the restart; the counts stay.
- To stop early: stop after a complete round if possible (10 reps = 2 each, 15 = 3 each). The next drive with the
  build continues from the saved counts.
- Reset (new block): delete the file with the ignition OFF: `ssh comma 'rm -f /data/identification_progress.json'`.
  With the ignition on, the running process keeps its counts and writes them again at the next counted rep. A
  missing, unreadable or foreign file gives zero counts (never more).
- Block complete: after 6 counted reps of every maneuver the banner shows `TEST BLOCK COMPLETE - long press = off`.
  No more reps start. Long press to turn test mode off.

### A6. Stop the session on

Any unexpected alert, fault, warning or noise; `TEST MODE LOCKED` or `TEST LOCKED`; a cruise or ACC fault; the test
banner disappearing while armed, or an update banner (`Update Running`, `Update Ready`, ...) showing (the banner
channel was taken: test mode is locked); an unexpected restart; unclear state; a missed marker; a vehicle behind
during a rep; a change in grip or visibility; any person on the road. Do not start Full Update, the settings Reboot
or longitudinal maneuver mode while test mode is armed (each locks it or blocks it).

## Part B: engineering

B1. Source anchors
- `selfdrive/controls/lib/stopping_flags.py` `IDENTIFICATION_HOOK`: True in the session build (`ffeccdb208` on
  `!my-fp-new`); False = nothing is constructed.
- `selfdrive/controls/lib/identification_hook.py`: `BLOCKS` (KCS1, KCS2), `PLAN_ID = "KCS2"`, `MANEUVERS = BLOCKS[PLAN_ID]`,
  `N_REPS = 6`, `HOLD_BRAKE_S = 5.0` (the KCS1 table below; KCS2 at the top).
  States OFF/ARMED/READY/ACTIVE/HELD/HANDBACK/LOCKED, in memory only (every controlsd start is OFF).
  - Button (`PressTimer`, shared with card): debounced (a press ends after `MIN_PRESS_S` 50 ms of release); a press
    held at construction or across an input gap never counts; arm/disarm at `LONG_PRESS_S` (= `CRUISE_LONG_PRESS`,
    0.5 s) on the hook's own timer; start on a 50 ms-0.5 s press that began in READY with every precondition held
    through its end; the card's long flag only discards a start. A raw press in ACTIVE aborts on its first frame;
    presses in HELD and HANDBACK do nothing.
  - Start gate `precondition_failure(..., start_accel)`, `PRECONDITION_S` 2.0 s. Faults first (`FAULT_ENDS`:
    inputs, car, mapping, fcw, vehicle, fault, exception, banner), then disengaged, state (not `pid` or standstill),
    pedal, speed (`V_ARM_MIN`-`V_ARM_MAX` 3.5-9.0 m/s), lead (radar status, model prob >= `LEAD_PROB_MAX` 0.10,
    plan hasLead), stop (planner shouldStop or target 0-200 m), steer (`STEER_MAX_DEG` 5, `YAW_MAX` 0.03 rad/s,
    blinker, steer fault), demand (min(normal final command, plan aTarget) below the first segment's command),
    settling (|v - vCruise| > `V_STEADY` 0.3, |aEgo| > `A_STEADY` 0.2, |aTarget| > `PLAN_STEADY` 0.15).
  - ACTIVE: a segment ends on the frame with v <= `v_end` or t >= `t_s`; the last segment ends at `CS.standstill`
    (then HELD). Stop intent at v <= `V_INTENT` 2.0 in a segment without `t_s`; it stays until the rep ends. Rep
    abort: any precondition failure (with the intent, `state` and `stop` are ignored; the start band is replaced by
    v > (speed at the current segment's first frame) + `V_OVER` 0.5), or a press. Abort above `V_INTENT` without the
    intent: HANDBACK (cap = min(last + `RELEASE_JERK` 0.8 x dt, 0), wire = min(normal, cap), ends when the cap reaches
    0 or the normal chain is deeper), then ARMED (OFF after pedal/disengaged, LOCKED after a fault). Abort with the
    intent or at v <= `V_INTENT` (also in D's 0.0 segment): finish (intent on, the last command kept to standstill
    under min() semantics, then HELD, not counted).
  - Stall (segments without `t_s`): v < `STALL_V` 2.5 and less than `STALL_DV` 0.15 slowing over `STALL_T` 2.0 s:
    deepen at `J_HOLD` 0.6 m/s^3 to `A_HOLD` -0.70, sticky; sets the intent; the rep still counts. `CAP_S` 30 s after
    the start without standstill: lock `vehicle` (handback before the intent, HELD after it).
  - HELD: the floor deepens at 0.6 m/s^3 to -0.70 if it is shallower, else it stays (-0.80, -1.00); the intent
    stays. A fault sets the lock but keeps the hold. Brake without gas = the rep end: counted if not a finish, not
    locked and hold >= `HOLD_MIN_S` 1.0 s; then `done[id] += 1`, `rep_done = id` on that frame only, ARMED. Gas,
    disengagement or brake while moving: floor and intent drop on that frame, OFF, not counted.
  - While a rep runs or holds, the floor IS the wire (`own`): on the car the normal chain only lagged behind the scripted
    braking (D's 0.0 coast carried -0.2) or held its own -0.70 at standstill (it overwrote the hold build). An abort
    hands back or finishes under min(normal, floor). A counted rep logs `reason=complete` or `reason=stalled`.
  - An exception locks; a driver action still ends authority at once; a release still runs out.
- `selfdrive/controls/lib/longcontrol.py`: constructs the hook OFF under the flag in the Santa Fe HEV scope and logs
  `identification hook constructed: OFF`. `hook_intent` (the previous frame's `stop_intent`) is ORed into the
  arbiter's `raw_should_stop` only (`service_should_stop` is computed before, without it), so the state machine
  enters `stopping` (carcontroller: SCC14 upper 1.0, StopReq at v < 0.01, latched). The hook is the final writer
  after every cap, service and hold writer: `output_accel = floor` if `hook.own`, else `min(output_accel, floor)`.
  `own` = after the intent while no abort ended the rep; finish, HELD and HANDBACK use the min. On every frame with
  a floor the tracking trim is zeroed and a pid integrator is reseeded. Fault frames (`input_hold`) call
  `interrupt()` instead: a rep, hold or release locks; ARMED/READY only restart qualification. `reset()` keeps
  ARMED/READY.
- `selfdrive/controls/controlsd.py`: `ID_PROGRESS_FILE = "/data/identification_progress.json"`, read once in
  `__init__` (`hook.load`), then `identification hook progress loaded: {...}`. On a frame with `rep_done`:
  `_save_id_progress` in a daemon thread (write `.tmp`, fsync, `os.replace`) and `identification hook progress
  saved: {...}` (logged when the save starts; an OSError logs `identification hook progress not saved`).
  `_identification_inputs` builds `HookInputs` (with `a_ego` and `vCruise` in m/s). `_publish_id_banner` registers
  the `alertDebug` publisher only when the hook first has text, sends only non-empty text, and on any `IpcError`
  locks the hook (`banner`) and never publishes again. `LongitudinalManeuverMode` blocks arming.
- `selfdrive/car/card.py`: its own `PressTimer` (`id_press`) sees the same fresh 0.5 s press as the hook and sets
  `v_cruise_kph` to `SET_SPEED_KPH` (20). If `carControl.enabled` is False it also sets `id_set_speed_pending`; the
  next engagement applies 20 km/h after `initialize_v_cruise` and clears it. Card cannot see the hook state, so
  every long press does this. `frogpilot/common/frogpilot_variables.py` `initial_set_speed` is the saved
  `InitialSetSpeed` (not changed by the test build).
- Test scope `identification_mode` (flag + `HYUNDAI_SANTA_FE_HEV_2022` + openpilot longitudinal) in
  `frogpilot_variables.py` (every distance mapping NOTHING, no personality via LKAS), `card.py`,
  `frogpilot/controls/frogpilot_card.py` (Traffic off), `selfdrive/selfdrived/selfdrived.py` (Standard).
- Button mappings: keys `DistanceButtonControl`, `LongDistanceButtonControl`, `VeryLongDistanceButtonControl`; the
  device held 2 / 1 / 6 on 25 and 26 September. They must be the same before and after the session.

Maneuvers (`MANEUVERS`, table order = tie order; commands in m/s^2; 9 km/h = 2.5 m/s, 5 km/h = 1.5 m/s):

| id | Segments | Stop intent | Hold after the wheel stop |
|---|---|---|---|
| B | s1 -1.00 to standstill | s1 at 2.0 m/s | stays -1.00 |
| A | s1 -0.50 to standstill | s1 at 2.0 m/s | -0.50 to -0.70 at 0.6 m/s^3 |
| C | s1 -1.00 until v <= 2.5; s2 -0.30 to standstill | s2 at 2.0 m/s (or a stall) | -0.30 to -0.70 at 0.6 m/s^3 |
| D | s1 -1.00 until v <= 2.5; s2 0.00 for 2.0 s; s3 -0.80 to standstill | s3 at 2.0 m/s (never in s2) | stays -0.80 |
| E | s1 -0.80 until v <= 1.5; s2 -0.30 for 2.0 s; s3 -0.80 to standstill | s1 at 2.0 m/s (s2 and s3 run in `stopping`) | stays -0.80 |

Persistence file: `/data/identification_progress.json` = `{"plan": "KCS1", "done": {"B": n, "A": n, "C": n, "D": n,
"E": n}}`. Written only after a counted rep. `load()` accepts only plan `KCS1`, known ids and integers (clamped
0-6); another plan, a parse error or a missing file gives zeros. Arming, state, floor, lock and set speed are never
saved.

B2. Footprint per rep (PLAN.md section 2; sensitivity only, NOT a braking-distance guarantee)
Assumptions: nominal plant (delay 0.45 s, gain 1.0, no creep push unless stated); press to rest from the start
press at 20 km/h; footprint = launch from rest at 0.7 m/s^2 (22 m, 7.9 s) + READY 2 s (11 m) + press to rest + 20 m
margin. 15 km/h values in brackets (short-site fallback). The D/E push footprints use the same formula.

| id | Press to rest, 20 km/h [15 km/h] | Footprint from RESUME, 20 km/h [15 km/h] |
|---|---|---|
| B | 17.9 m, 5.9 s [10.6 m, 4.5 s] | 71 m [50 m] |
| A | 33.4 m, 11.4 s [19.2 m, 8.6 s] | 86 m [59 m] |
| C | 25.2 m, 11.5 s; -0.3 stalls at 1.0 m/s: ~28 m, ~12.5 s [17.8 m, 10.1 s] | 78-81 m [58 m] |
| D | 23.7 m, 8.5 s; +0.25 push in s2: 32.6 m, 11.8 s [16.3 m, 7.1 s] | 77 m, push 86 m [56 m] |
| E | 23.3 m, 8.5 s; +0.25 push: 33.2 m, 12.2 s [14.2 m, 6.8 s] | 76 m, push 86 m [55 m] |

Site minimum: 90 m at 20 km/h, 60 m at 15 km/h. Faults, slope, grip and real delays can make every row longer.

B3. Snapshot (read-only file reads; no msgq readers, no driving imports). Save as `b3.sh`, run with `bash`.
```sh
set -euo pipefail
S=~/.route_sync/corpus/brake_response_session_20260926; mkdir -p "$S"
EXPECT_FLAG="${EXPECT_FLAG:-}"        # True with the session build, False after removal; empty = record only
EXPECT_MAP="${EXPECT_MAP:-}"          # the restore record's values (e.g. 2/1/6) at every later check; empty = record only
EXPECT_INITIAL="${EXPECT_INITIAL:-}"  # the saved value (160 on 26 September); the test build never changes it
OUT=$(mktemp "$S/snapshot_$(date +%Y%m%dT%H%M%S)_XXXXXX")
REMOTE='set -eu; cd /data/openpilot; echo "head=$(git rev-parse HEAD)"
echo "tracked_changes=$(git status --porcelain --untracked-files=no | wc -l | tr -d " ")"
echo "flag=$(sed -n "s/^IDENTIFICATION_HOOK = \([A-Za-z]*\) .*/\1/p" selfdrive/controls/lib/stopping_flags.py)"
for k in DistanceButtonControl LongDistanceButtonControl VeryLongDistanceButtonControl LKASButtonControl InitialSetSpeed LongitudinalManeuverMode IsOnroad IsOffroad; do
  if [ -e /data/params/d/$k ]; then echo "$k=$(cat /data/params/d/$k)"; else echo "$k=ABSENT"; fi; done
if [ -e /data/identification_progress.json ]; then echo "progress=$(tr -d "\n" < /data/identification_progress.json)"; else echo progress=ABSENT; fi
p=$(cat /data/fullupdate_reboot.pid 2>/dev/null || true)
if [ -n "$p" ] && grep -qa __reboot_when_parked /proc/$p/cmdline 2>/dev/null; then echo supervisor=running; else echo supervisor=none; fi
echo SNAPSHOT_OK'
ssh -o BatchMode=yes -o ConnectTimeout=8 comma "$REMOTE" > "$OUT" || ssh -o BatchMode=yes -o ConnectTimeout=8 commawifi "$REMOTE" > "$OUT"
python3 - "$OUT" "$EXPECT_FLAG" "$EXPECT_MAP" "$EXPECT_INITIAL" <<'PY'
import json, re, sys
lines = open(sys.argv[1]).read().splitlines()
if not lines or lines[-1] != 'SNAPSHOT_OK':
  sys.exit('FAIL: snapshot incomplete')
s = dict(line.split('=', 1) for line in lines[:-1])
maps = [s[k] for k in ('DistanceButtonControl', 'LongDistanceButtonControl', 'VeryLongDistanceButtonControl')]
try:
  prog = None if s['progress'] == 'ABSENT' else json.loads(s['progress'])
except ValueError:
  prog = 'unreadable'
checks = {'mappings present, 0-6': all(re.fullmatch('[0-6]', v) for v in maps),
          'maneuver mode off': s['LongitudinalManeuverMode'] in ('0', '', 'ABSENT'),
          'no supervisor': s['supervisor'] == 'none', 'no tracked changes': s['tracked_changes'] == '0',
          'expected flag': sys.argv[2] in ('', s['flag']),
          'expected mappings': sys.argv[3] in ('', '/'.join(maps)),
          'initial speed present, 8-170': s['InitialSetSpeed'].isdigit() and 8 <= int(s['InitialSetSpeed']) <= 170,
          'expected initial speed': sys.argv[4] in ('', s['InitialSetSpeed']),
          'progress absent or a KCS1 record': prog is None or (isinstance(prog, dict) and prog.get('plan') == 'KCS1'
                                                              and isinstance(prog.get('done'), dict))}
print('\n'.join(lines))
print('progress counts:', 'none (the block starts from zero)' if prog is None else prog)
if not all(checks.values()):
  sys.exit(f'FAIL: {[k for k, ok in checks.items() if not ok]}')
print('PREFLIGHT PASS')
PY
```
The first PASS snapshot is the restore record (the three mappings and `InitialSetSpeed`). `head` must be the expected
commit; `IsOnroad` is recorded, not required. A foreign or unreadable progress file fails the check: the device
would start the block from zero. Delete or restore it (ignition off) first. The source HEAD alone does not prove the
running process (B4.3).

B4. Install the test build (only when Radek confirms he is the only driver until B5)
1. `!my-fp-new` already has `IDENTIFICATION_HOOK = True` (`ffeccdb208`, pushed). Commit the KCS1 code on it and
   push; `fullupdate.sh` resets the device to the pushed branch. The enable patch is not needed.
2. `EXPECT_MAP=2/1/6 bash b3.sh` PASS (records the running flag and the counts). Deploy:
   `ssh -tt comma 'cd /data/openpilot && ./fullupdate.sh'` (on a timeout at once `ssh -tt commawifi ...`).
   Parked/off-road it reboots at once. On-road it stages and applies after cruise off (AGENTS.md device workflow).
3. After the restart: `EXPECT_FLAG=True EXPECT_MAP=2/1/6 bash b3.sh` PASS with HEAD = the KCS1 commit, and the
   running controlsd was built from it:
   `ssh comma 'grep -h "identification hook \(constructed\|progress loaded\)" /data/log/swaglog.* | tail -n 4'`
   shows `constructed: OFF` and `progress loaded: {'plan': 'KCS1', ...}` with a `created` time after the restart
   (the earlier step build wrote no `progress loaded` line).
4. New block: no progress file (or all zeros). Continued block: the file shows the saved counts.

B5. Remove the test build (always, before anyone else drives)
1. Turn test mode off (long press) and park. (Initial Set Speed was never changed.)
2. Copy logs (B6) before other changes.
3. `git apply --check ~/.route_sync/corpus/button_session_v2_20260926/build/disable_identification_hook.patch`, apply
   it, commit and push (`True` -> `False`); deploy as in B4.2. Then
   `EXPECT_FLAG=False EXPECT_MAP=2/1/6 EXPECT_INITIAL=<recorded> bash b3.sh` PASS with HEAD = the disable commit, and
   no new `identification hook constructed` line after the next start.
4. Keep `/data/identification_progress.json` while the block is not complete: nothing reads it with the flag False,
   and the next session build continues from it. Delete it (ignition off) only to restart the block or after the
   data is archived. `/data/identification_hook.arm` is obsolete; `rm -f` it if a stale one exists.

B6. Logs (host, `.venv` active, repository root; ignition off so segments are final). A block can take more than one
drive: run this once per session route. Set `S=~/.route_sync/corpus/brake_response_session_20260926`. Then
`python tools/route_sync/refresh_routes.py --host comma --include-rlog --newest-first --report-file "$S/refresh_report_$(date +%Y%m%dT%H%M%S).json"`
and, with the session route from the report, `R=<route> bash b6.sh`:
```sh
set -euo pipefail
S=~/.route_sync/corpus/brake_response_session_20260926
R="${R:?set R to the session route}"
RD=~/.route_sync/data/media/0/realdata
D="$S/$R"
shopt -s nullglob; SEGS=("$RD/$R"--*)
[ ${#SEGS[@]} -gt 0 ] || { echo "FAIL: no segments for $R"; exit 1; }
for ((i = 0; i < ${#SEGS[@]}; i++)); do [ -f "$RD/$R--$i/rlog.zst" ] || { echo "FAIL: $R--$i has no rlog"; exit 1; }; done
mkdir -p "$S"; mkdir "$D" "$D/source_rlogs"
for d in "${SEGS[@]}"; do mkdir "$D/source_rlogs/${d##*/}"; cp -p "$d/rlog.zst" "$D/source_rlogs/${d##*/}/"; done
(cd "$D" && shasum -a 256 source_rlogs/*/rlog.zst > SOURCE_SHA256)
python tools/stopping/review/bookmarked_baseline.py --output "$D/packet" "$D"/source_rlogs/*/rlog.zst
python tools/stopping/review/can_response.py "$D/packet/signals.json" --output "$D/can"
python ~/.route_sync/corpus/calibration_session_prep_20260925/build/extract_hook_session.py "$D/packet/signals.json" --output "$D/hook"
```
`mkdir "$D"` fails if the route was already copied; never overwrite. The extractor keeps the `alertDebug` text and
every "identification hook" cloudlog line:
- `identification hook constructed: OFF`, then `identification hook progress loaded: {'plan': 'KCS1', 'done': {...}}`
  at each controlsd start;
- `identification hook <STATE> man=<id> rep=<n> seg=<k> reason=<r> floor=<a> intent=<0|1> done=<id> v=<v>` on every
  state change, segment change and stall (`floor=None` without a floor; the brake frame of a counted rep has
  `ARMED ... reason=complete done=<id>`; a rep that did not count has an empty `done=` and the reason, e.g.
  `short-hold`);
- `identification hook progress saved: {...}` after each counted rep (or `progress not saved` on a write error);
- `identification hook banner lost: test mode locked` if the banner channel was lost.
The banner carries `<id> <n>/6 s<k>` on every rep frame; use it and the SCC12 edges for timing, not the cloudlog
time. `can_response.py` and the extractor do not yet read WHL_SPD11, the raw IMU, `orientationNED`, GPS, the
auto-hold lamp or the parking brake; build that extraction before the B7 analysis. Never edit the originals.

B7. Review each rep before drawing conclusions (frozen before the drive; PLAN.md sections 2 and 8)
- Identity: route, segments, SHA256, commit, construction and `progress loaded` lines, maneuver/rep/segment from
  the banner and the cloudlog, the counts before and after.
- Command: sent SCC12 `aReqValue` (bus 0, even frames, with the bus-128 echo) equals the script within 0.005 after
  0.01 quantisation on every frame; SCC14 upper 3.0 in `pid`, 1.0 from the stopping state; `StopReq` 0 while
  v >= 0.01, latched in the hold, 0 on the brake frame. `longControlState` is `pid` until the intent and `stopping`
  from one or two frames after it. The sent command is not achieved acceleration.
- Validity: pre-window >= 2.0 s at vCruise +-0.3 m/s, |aEgo| <= 0.2, sent SCC12 within +-0.15 of 0; no gap > 0.1 s
  in carState, WHL_SPD11, sendcan or echo; no pedal, override, ESP, ABS or accFaulted; standstill under the script
  (or the stall rule); hold >= 1.0 s; grade from the pre-window within +-2 % for gain use. Maneuver checks: C s1
  >= 2.0 s and the s1-s2 edge at 2.5 +-0.1 m/s; D s2 the full 2.0 s with 0.8 <= v <= 3.3; E s1 reached 1.5 m/s
  and s2 the full 2.0 s with v >= 0.5 at its end, `stopping` on every s2 frame.
- Labels: `complete`, `stalled`, `short-hold`, `aborted(<reason>)`. Keep every rep as labelled data. Only `complete` and
  `stalled` are fit data.
- Signals and onset rule: motion = raw WHL_SPD11 mean and its 0.3 s slope; body = raw accelerometer with the
  calibrated orientation, and ESP12 LONG_ACCEL; aEgo only through its known Kalman recursion; livePose at 20 Hz
  only; jerk over 300 ms windows only. Onset = the first of three consecutive 50 Hz wheel samples whose 0.3 s slope
  departs from the pre-edge slope by more than 3 SD of the pre-window slope; delay = that time minus the send time
  of the first frame of the new command.
- Hold-out: reps 1-4 of each maneuver train; reps 5-6 are held out and not inspected before the fit. Tolerances for
  held-out reps: speed RMSE <= 0.1 m/s, distance at rest +-0.2 m, stop time +-0.3 s. Then free rollout from the
  recorded SCC12 on the natural closed-loop stops in the corpus, with no per-stop offsets.
