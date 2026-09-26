# Brake-response test mode: session runbook (revised 2026-09-26)

Status: prepared and reviewed offline; the source has `IDENTIFICATION_HOOK = False`. The test build is the same code
with that flag set to True (one-line enable patch, B4). No trial has run on the car. This procedure replaces the
one-attempt arm-file procedure of the first 26 September version and the collection steps in
`universal_stop_program.md` (2026-09-05 identification entries).

What a trial is: one open-loop braking request of -0.5 m/s^2 for up to 3 s from a steady ~30 km/h. Every trial is
the same step; the build has no stronger profile. It measures how the car responds to a known request (delay, gain,
release). It does not fit a full response model, it does not qualify the 0.5-2.4 m/s stopping problem, and it does
not show better-than-human stops. Deeper, repeated-depth or near-stop trials need a separate reviewed protocol.

## Part A: operator (driver)

The driver is responsible for steering and pedals at all times. Lateral control stays engaged. The test never
commands positive acceleration, but when a trial ends, normal cruise takes over again and CAN ACCELERATE back to the
set speed. The only cue is the on-screen banner; there is no tone.

### A0. What the test build changes (for every drive while it is installed)

- The wheel distance button does only the test functions below. Force Coast, personality, Traffic, Experimental and
  pause actions are not available from it (short, long or very long). The on-screen distance button is ignored.
- The personality is Standard; Traffic mode is off (also from an LKAS mapping). Saved settings are not changed.
- Engaging cruise starts at 30 km/h (or at the current speed if faster), not at the saved Initial Set Speed.
- Every long press of the distance button while cruise is engaged sets the set speed to 30 km/h. This includes the
  long press that turns test mode off. At a higher speed the car slows to 30 km/h like after any set-speed change.
- Nobody else drives the car while the test build is installed. A long press (a personality habit) followed later by
  a short press (a Force Coast habit) can start a trial. Remove the test build (B5) before anyone else drives.

### A1. Site go/no-go (before any trial; parked)

- Measure the length of the straight that is continuously flat, clear and has good grip. A turn ends it. Do not
  count road after the turn. Do not use downhill parts.
- Mark the END of the usable straight (a fixed object before the turn).
- Choose the LAST-PRESS point from the measured geometry: everything after the start press (B2 worksheet: press,
  step, release, takeover, manual stop, unused runout, plus your own margin) must fit before END. Setup and the 2 s
  READY wait must fit between the start of the straight and the LAST-PRESS point.
- No other people or vehicles near, and no vehicle close behind. If the space or margin is not enough: NO-GO.
- Do not accelerate harder, raise the speed target or use a downhill to make the site fit.

### A2. Settings

- Change nothing. The test build ignores the saved distance mappings and the saved Initial Set Speed (160 km/h on
  26 September) without changing them; it uses 30 km/h (A0).

### A3. Buttons and banners

| Gesture (wheel distance button) | In state | Result |
|---|---|---|
| Hold 0.5 s (long press) | OFF | Test mode ARMED at the 0.5 s mark. No braking. The rest of that press does nothing. Set speed 30 km/h if engaged. |
| Hold 0.5 s | ARMED or READY | Test mode OFF. Set speed 30 km/h if engaged. |
| Short press, released in less than 0.5 s | READY | Starts ONE trial 50 ms after the release. |
| Short press | OFF, ARMED (not ready) | Nothing; not remembered for later. |
| Any press | trial running (ACTIVE) | Cancels the trial at once. That press does nothing else, however long it is held. |
| Hold 0.5 s | LOCKED | Shows why it is locked. It stays locked. |

| Banner (top line) | Meaning |
|---|---|
| (none) | Test mode OFF. |
| `TEST MODE ARMED - waiting: <reason>` | Armed; a start condition is not met yet (`settling` = all met, less than 2 s). |
| `TEST READY - short press distance to start` | Every condition held 2 s. The second line shows the last result. |
| `TEST ACTIVE -0.50 m/s^2 - 1.2 s` | The trial owns the braking request. Any press cancels. |
| `TEST n COMPLETE` / `TEST n ABORTED - <reason>` | The braking releases (about 0.6 s); then normal cruise resumes and can accelerate. |
| `TEST MODE OFF` (3 s) | Turned off by a long press or by the driver during a trial. |
| `TEST MODE LOCKED - <reason>` (3 s) | A fault. No more trials until the car is switched off and on again. |

A button held while openpilot starts, or while a fault is reported, never counts; release it and press again.

### A4. Running trials

- Arm with a long press (anywhere, also parked). Engage cruise (or, if already engaged, the long press set 30 km/h).
  Check the displayed set speed IS 30 km/h. No car ahead, wheel straight, no blinker, feet off the pedals.
- Wait for `TEST READY`. READY needs, continuously for 2 s: openpilot
  longitudinal engaged, 25-40 km/h measured speed, no lead (radar, or model probability 0.1 or more), no stop sign or
  stop target within 200 m, no planner or controller braking deeper than -0.5 m/s^2, wheel within 5 degrees, no yaw,
  no blinker, no pedal, no Force Coast or pause, Drive gear, and no fault.
- Only if READY shows BEFORE the LAST-PRESS point: press the distance button once, short. Do not press while waiting.
- During `TEST ACTIVE` the car requests -0.5 m/s^2 for up to 3 s (it ends early at 16 km/h). This measures the real
  response; do not expect exactly gentle braking.
- It ends by itself, or you cancel with one press. The trial also ends at once if a lead appears, a stop appears,
  the speed leaves 14-43 km/h, you steer, or the planner/controller wants to brake harder (that deeper braking
  passes at once). Then the braking releases and normal cruise resumes and can accelerate. Test mode stays armed.
- If you do not want cruise to accelerate again, brake to a stop as usual (this does not turn test mode off after
  the release has started).
- Brake, gas or cruise cancel DURING the step: the test gives up braking control at once and test mode turns OFF.
  A new long press is needed to arm again.
- The car does not stop: from 30 km/h the step ends near 25 km/h (the real response is what we measure), then
  cruise returns to 30 km/h.
- Another trial: READY must come back (2 s of all conditions), then a new short press. Every trial is the same
  -0.5 m/s^2 step; nothing gets stronger. Check the space for each trial again; do not start one after LAST-PRESS.
  One trial, from READY to the end of the release, covers about 45 m at 30 km/h (2 s wait, 3 s step, 0.6 s release).
- Turn test mode off with a long press when finished.

### A5. Stop the session on

Any unexpected alert, fault, warning or noise; `TEST MODE LOCKED`; the test banner disappearing while armed, or an
update banner (`Update Running`, `Update Ready`, ...) showing (the banner channel was taken: test mode is locked);
unclear state; a missed marker; another person or vehicle; a change in grip or visibility; an unexpected restart.
Do not start Full Update, the settings Reboot or longitudinal maneuver mode while test mode is armed (each locks it).

## Part B: engineering

B1. Source anchors
- `selfdrive/controls/lib/stopping_flags.py` `IDENTIFICATION_HOOK` (False): nothing is constructed when False.
- `selfdrive/controls/lib/identification_hook.py`: states OFF/ARMED/READY/ACTIVE/HANDBACK/LOCKED, in memory only (every
  controlsd start is OFF). Button: debounced (a press ends after `MIN_PRESS_S` 50 ms of release); a press held at
  construction or across an input gap never counts; arm/disarm at `LONG_PRESS_S` (= `CRUISE_LONG_PRESS`, 0.5 s) on the
  hook's own timer; start on a 50 ms-to-0.5 s press that began in READY with every precondition held through its
  end; the card's long flag only discards a start. `STEP_ACCEL, STEP_S = -0.5, 3.0`; `V_END` 4.5 m/s; `RELEASE_JERK`
  0.8 m/s^3. `precondition_failure` reports faults first (`FAULT_ENDS`); `demand` = min(normal final command,
  `longitudinalPlan.aTarget`) below the step (no margin: the Santa Fe HEV runs kp = ki = 0, so the final command is the
  planner demand plus caps; the planner target also covers a tune with an integrator, whose per-frame reseed would
  anchor the final command at the step). Driver action during ACTIVE -> OFF; during HANDBACK -> the trial's own end
  state. Faults during ACTIVE/HANDBACK, an exception, or a lost banner -> LOCKED for the process.
- `longcontrol.py`: constructs the hook OFF (no file I/O); `reset()` keeps ARMED/READY armed (qualification restarts);
  an input fault (`input_hold`) calls `interrupt()`: ACTIVE/HANDBACK lock, ARMED/READY only restart qualification.
  The final writer applies the step while ACTIVE and min(normal, release cap) during HANDBACK.
- `controlsd.py`: `_publish_id_banner` registers the `alertDebug` publisher only when the hook first has text (a long
  press), sends only non-empty text, and on any `IpcError` (msgq: the old publisher's send fails after another
  process registers; ZMQ: registration fails) locks the hook and never publishes again. `fullupdate.sh` and
  `maneuversd` keep the channel. `LongitudinalManeuverMode` (read every 10 frames) blocks arming.
- Test scope `identification_mode` (flag + `HYUNDAI_SANTA_FE_HEV_2022` + openpilot longitudinal) in
  `frogpilot/common/frogpilot_variables.py`, `selfdrive/car/card.py`, `frogpilot/controls/frogpilot_card.py`,
  `selfdrive/selfdrived/selfdrived.py`. Set speed: `SET_SPEED_KPH` (30) is the scope's `initial_set_speed`, and
  `card.py` sets it on FrogPilotCard's long-press frame (`gap_counter == long_press_threshold`) while
  `carControl.enabled`. Card cannot see the hook state, so the disarming long press sets it too.
- Button mappings: keys `DistanceButtonControl`, `LongDistanceButtonControl`, `VeryLongDistanceButtonControl`; the device
  held 2 / 1 / 6 on 25 and 26 September. They must be the same before and after the session.

B2. Distance worksheet at 30 km/h per trial (sensitivity only; NOT a braking-distance guarantee)
Assumptions: constant 8.33 m/s through the 2 s READY wait, 1.5 s for the press (a short press takes less), the 3 s
step (no credit for its braking) and the 0.625 s release; then 2 s to manual braking with the listed normal-cruise
acceleration; 1.5 m/s^2 manual braking; 20 m unused runout. Faults, slope, grip, overshoot and real delays can make
every row longer. Source: `HOST_30KPH_PROBE.json` and `codex/LOWER_SPEED_REVIEW.md` (25 September corpus).

| Setup accel | Takeover accel | Setup + qualification | Press to stop + runout | Total |
|---:|---:|---:|---:|---:|
| 1.0 | 0 | 51.4 m | 102.5 m | 153.9 m |
| 1.0 | 1.0 | 51.4 m | 117.0 m | 168.4 m |
| 1.0 | 2.0 | 51.4 m | 134.1 m | 185.5 m |
| 0.7 | 2.0 | 66.3 m | 134.1 m | 200.3 m |
| 0.5 | 2.0 | 86.1 m | 134.1 m | 220.2 m |

B3. Snapshot (read-only file reads; no msgq readers, no driving imports). Save as `b3.sh`, run with `bash`.
```sh
set -euo pipefail
S=~/.route_sync/corpus/brake_response_session_20260926; mkdir -p "$S"
EXPECT_FLAG="${EXPECT_FLAG:-False}"
EXPECT_MAP="${EXPECT_MAP:-}"          # the restore record's values (e.g. 2/1/6) at every later check; empty = record only
EXPECT_INITIAL="${EXPECT_INITIAL:-}"  # the saved value (160 on 26 September); the test build never changes it
OUT=$(mktemp "$S/snapshot_$(date +%Y%m%dT%H%M%S)_XXXXXX")
REMOTE='set -eu; cd /data/openpilot; echo "head=$(git rev-parse HEAD)"
echo "tracked_changes=$(git status --porcelain --untracked-files=no | wc -l | tr -d " ")"
echo "flag=$(sed -n "s/^IDENTIFICATION_HOOK = \([A-Za-z]*\) .*/\1/p" selfdrive/controls/lib/stopping_flags.py)"
for k in DistanceButtonControl LongDistanceButtonControl VeryLongDistanceButtonControl LKASButtonControl InitialSetSpeed LongitudinalManeuverMode IsOnroad IsOffroad; do
  if [ -e /data/params/d/$k ]; then echo "$k=$(cat /data/params/d/$k)"; else echo "$k=ABSENT"; fi; done
p=$(cat /data/fullupdate_reboot.pid 2>/dev/null || true)
if [ -n "$p" ] && grep -qa __reboot_when_parked /proc/$p/cmdline 2>/dev/null; then echo supervisor=running; else echo supervisor=none; fi
echo SNAPSHOT_OK'
ssh -o BatchMode=yes -o ConnectTimeout=8 comma "$REMOTE" > "$OUT" || ssh -o BatchMode=yes -o ConnectTimeout=8 commawifi "$REMOTE" > "$OUT"
python3 - "$OUT" "$EXPECT_FLAG" "$EXPECT_MAP" "$EXPECT_INITIAL" <<'PY'
import re, sys
lines = open(sys.argv[1]).read().splitlines()
if not lines or lines[-1] != 'SNAPSHOT_OK':
  sys.exit('FAIL: snapshot incomplete')
s = dict(line.split('=', 1) for line in lines[:-1])
maps = [s[k] for k in ('DistanceButtonControl', 'LongDistanceButtonControl', 'VeryLongDistanceButtonControl')]
checks = {'mappings present, 0-6': all(re.fullmatch('[0-6]', v) for v in maps),
          'maneuver mode off': s['LongitudinalManeuverMode'] in ('0', '', 'ABSENT'),
          'no supervisor': s['supervisor'] == 'none', 'no tracked changes': s['tracked_changes'] == '0', 'flag': s['flag'] == sys.argv[2],
          'expected mappings': sys.argv[3] in ('', '/'.join(maps)),
          'initial speed present, 8-170': s['InitialSetSpeed'].isdigit() and 8 <= int(s['InitialSetSpeed']) <= 170,
          'expected initial speed': sys.argv[4] in ('', s['InitialSetSpeed'])}
print('\n'.join(lines))
if not all(checks.values()):
  sys.exit(f'FAIL: {[k for k, ok in checks.items() if not ok]}')
print('PREFLIGHT PASS')
PY
```
The first PASS snapshot is the restore record (the three mappings and `InitialSetSpeed`). `head` must be the expected
commit; `IsOnroad` is recorded, not required. The source HEAD alone does not prove the running process (B4.3).

B4. Install the test build (only when Radek confirms he is the only driver until B5)
1. Enable patch: `~/.route_sync/corpus/button_session_v2_20260926/build/enable_identification_hook.patch` (one line,
   `False` -> `True`); reverse: `disable_identification_hook.patch`. `git apply --check` first. `fullupdate.sh` resets
   to the pushed branch, so commit and push the enable change; push the reverse as soon as the session ends.
2. `EXPECT_MAP=2/1/6 bash b3.sh` PASS. Deploy: `ssh -tt comma 'cd /data/openpilot && ./fullupdate.sh'` (on a timeout at
   once `ssh -tt commawifi ...`). Parked/off-road it reboots at once. On-road it stages and applies after cruise off
   (AGENTS.md device workflow); the test build no longer conflicts with the updater banner while test mode is OFF.
3. After the restart: `EXPECT_FLAG=True EXPECT_MAP=2/1/6 bash b3.sh` PASS with HEAD = enable commit, and the running
   controlsd was built from it: `ssh comma 'grep -h "identification hook constructed" /data/log/swaglog.* | tail -n 2'`
   shows `constructed: OFF` with a `created` time after the restart (a flag-False build writes no such line).

B5. Remove the test build (always, before anyone else drives)
1. Turn test mode off (long press) and park. (Initial Set Speed was never changed; the normal build uses it again.)
2. Copy logs (B6) before other changes.
3. Push the reverse commit; deploy as in B4.2. Then `EXPECT_MAP=2/1/6 EXPECT_INITIAL=<recorded> bash b3.sh` PASS with
   HEAD = disable commit, and no new "identification hook constructed" line after the next start.
4. `/data/identification_hook.arm` is obsolete; nothing reads it. `rm -f` it if a stale one exists.

B6. Logs (host, `.venv` active, repository root; ignition off so segments are final). Set
`S=~/.route_sync/corpus/brake_response_session_20260926`. Then
`python tools/route_sync/refresh_routes.py --host comma --include-rlog --newest-first --report-file "$S/refresh_report.json"`
and, with the session route from the report, `R=<route> bash b6.sh`:
```sh
set -euo pipefail
S=~/.route_sync/corpus/brake_response_session_20260926
R="${R:?set R to the session route}"
RD=~/.route_sync/data/media/0/realdata
shopt -s nullglob; SEGS=("$RD/$R"--*)
[ ${#SEGS[@]} -gt 0 ] || { echo "FAIL: no segments for $R"; exit 1; }
for ((i = 0; i < ${#SEGS[@]}; i++)); do [ -f "$RD/$R--$i/rlog.zst" ] || { echo "FAIL: $R--$i has no rlog"; exit 1; }; done
mkdir "$S/source_rlogs"
for d in "${SEGS[@]}"; do mkdir "$S/source_rlogs/${d##*/}"; cp -p "$d/rlog.zst" "$S/source_rlogs/${d##*/}/"; done
(cd "$S" && shasum -a 256 source_rlogs/*/rlog.zst > SOURCE_SHA256)
python tools/stopping/review/bookmarked_baseline.py --output "$S/packet" "$S"/source_rlogs/*/rlog.zst
python tools/stopping/review/can_response.py "$S/packet/signals.json" --output "$S/can"
python ~/.route_sync/corpus/calibration_session_prep_20260925/build/extract_hook_session.py "$S/packet/signals.json" --output "$S/hook"
```
The extractor keeps `alertDebug` text and every "identification hook" cloudlog line (`constructed: OFF`, then
`identification hook <STATE> trial=<n> reason=<r> accel=<a> v=<v>` on each state change). Never edit the originals.

B7. Review each trial before drawing conclusions
- Identity: route, segments, SHA256, commit, construction line, trial number, banners, state changes.
- Command: sent SCC12 `aReqRaw`/`aReqValue`/`ACCMode`/`StopReq` and SCC14 jerk limits (`can_response.py`), with
  `carControl` enabled/longActive/override and the pedals. The sent command is not achieved acceleration.
- Response: measured speed/acceleration, grade, handback tail, and whether the driver cut it.
- Outcome: complete, speed-truncated (`complete` also covers the 4.5 m/s end), or aborted (reason). `demand`, `lead`,
  driver and fault ends are not clean open-loop samples. Keep every trial as labelled data.
