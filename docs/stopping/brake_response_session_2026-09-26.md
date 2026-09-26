# Brake-response smoke trial: session runbook, 2026-09-26

Prepared; calibration disabled. Activation waits for site geometry and parked/off-road checks. The source has
`IDENTIFICATION_HOOK = False`; the disabled preparation can be deployed through the normal workflow. The enable and
disable patches stay unapplied until the site checks pass. Still open: the device/source checks (B3, B4) and the site
facts (A1). This procedure replaces the collection steps in `universal_stop_program.md` (2026-09-05 identification entries).
Button mode (prepared offline, not activated): with the flag True on this car, a SHORT press of the wheel distance
button starts the attempt and a second press cancels it; the saved button settings are not changed.

The only purpose is ONE mild diagnostic attempt: the existing -0.5 m/s^2 profile for up to 3 s, from a steady
~30 km/h (8.33 m/s, inside the unchanged 7-11 m/s start gate). It does not fit a full response model. It does not show
"better than human" stops. It does not qualify the 0.5-2.4 m/s stopping problem. Deeper, repeated or near-stop trials
need a separate reviewed protocol. Budget about 30 min on site; the two off-road updates and the log review vary.

## Part A: operator (driver)

The driver is responsible for steering and pedals at all times. Lateral control can stay engaged; this procedure does
not disable it. The hook never commands positive acceleration, but after it releases, normal cruise resumes and CAN
accelerate. The only cue is the on-screen banner; there is no tone.

Test build behaviour (flag True, Santa Fe HEV, openpilot longitudinal; every drive, with or without the arm token):
- All three distance presses (short, long, very long) do nothing except the test start/cancel. Force Coast,
  personality, Traffic, Experimental and pause actions are not available from the distance button.
- Only the physical wheel button counts. The on-screen distance button is ignored.
- The driving personality is Standard. The saved personality and the saved button settings are not changed.
- Traffic mode is off (also from an LKAS mapping). The LKAS button cannot change the personality.

A1. Go/no-go (parked at the site, before any setting change)
- Walk or drive slowly along the straight. Measure the length that is continuously flat, straight, clear and has good
  grip. The turn ends it. Do not add the ~100 m after the turn. Do not use downhill parts.
- Mark the END of the usable straight (a fixed object before the turn, not the turn itself).
- Choose the LAST-PRESS point from the measured geometry, so that everything after the press (B2 worksheet: press,
  profile, release, takeover, manual stop, unused runout, plus your own margin for uncertainty) fits before END.
  Also check that setup and qualification fit between the start and the LAST-PRESS point.
- If the space or the margin is not enough, or other people/vehicles are near: NO-GO, no trial. Record only normal
  driving/manual logs, or use a longer flat straight another day.
- Do not accelerate harder, change the speed target, or use the downhill to make the site fit.

A2. Session settings (parked, through the existing UI only; the engineer records the originals first, B3)
- Do NOT change the WHEEL CONTROLS distance mappings. The test build ignores them in the test scope (above) and
  keeps the saved values for normal builds.
- FrogPilot -> GAS / BRAKE -> Quality of Life: set "Initial Set Speed" to 30 km/h BEFORE first engagement.
  The device held 160 km/h on 25 September; the real cruise helper selects that target on a normal first engagement.
  Record the current value and restore it after the session. This setting has not been changed during preparation.
- Openpilot longitudinal must be active on this car. The engineer confirms this from logs (B3), not from a toggle.

A3. The attempt (one only)
- Start from the start of the usable straight after A2 and B4 checks. Engage at or below 30 km/h; do not resume an old higher target.
  Check that the displayed set speed IS 30 km/h before any press. If it differs, cancel and park. No car ahead, wheel
  straight, no blinker, feet off the pedals.
- Wait for a steady speed. The screen shows `STEP TEST READY` when every condition held for 2 s (measured speed, not
  the set speed, is what the gate checks). A short glance is allowed. Do not touch the screen while driving (the
  on-screen distance button is ignored in the test build). No phone or chat use; nobody sends the driver messages.
- If the test banner is absent after engagement, do not press the distance button. Drive manually and park; a
  consumed token, a restart or another fault may have left the test unavailable.
- Only if READY shows BEFORE the LAST-PRESS point: press the wheel distance button once, SHORT (release in less
  than 0.5 s; isolated presses shorter than 50 ms are ignored). The attempt starts when you RELEASE.
  Do not press or hold while you wait for READY. If READY is late
  or does not come, do not press; continue manually.
- These presses start nothing and are not remembered: a press begun before READY, a press held from the start of
  the drive, a press of 0.5 s or longer, or a press during which any condition failed. If a press does not start
  the attempt, do not try again this run.
- The screen shows `STEP TEST ACTIVE`: openpilot requests -0.5 m/s^2 for up to 3 s. This is a braking pulse, not a
  stop. How the car actually responds is what this trial measures; do not expect exact gentle braking.
- To cancel: press the distance button once. The attempt ends at once, the braking releases gradually
  (0.8 m/s^3), then NORMAL CRUISE RESUMES and can accelerate. Steering stays engaged.
- A double-tap can start and immediately cancel the attempt. It still uses the one attempt; do not repeat it.
- When the screen shows `STEP COMPLETE` or `STEP ABORTED` with `normal cruise resumes; brake to a stop, do not repeat`,
  brake normally to a stop. Do not wait for cruise to pick up speed again.
- The brake pedal, gas pedal or cruise cancel ends the hook's authority at once (banner `park and review; do not
  repeat`). An input fault can instead hold the previous braking command for a moment; brake if needed.

A4. After the attempt
- Park. Do not press the distance button again. Do not drive the straight again. There is only ONE attempt per
  arm token; a second attempt is not possible (`STEP TEST DONE - all trials used`). Then follow B5 (teardown) before
  driving anywhere.
- Wait for the engineer's log review (B6, B7) before any further collection.

A5. Stop the session on: any unexpected alert, fault, warning or noise; unclear state; a missed marker; another person
or vehicle; any change in grip or visibility; an unexpected restart (the token is consumed at start, so after a
restart there is no banner and no attempt); `STEP TEST ARMED` or `READY` shown again after the attempt or after
`STEP TEST DONE` (a defect: report it); a waiting reason that does not clear.

## Part B: engineering

B1. Source anchors (commit to be recorded)
- `selfdrive/controls/lib/stopping_flags.py:183` master `IDENTIFICATION_HOOK = False`; nothing is constructed when False.
- `selfdrive/controls/lib/longcontrol.py:770` consumes the one-shot token `/data/identification_hook.arm` at
  LongControl construction (controlsd start, i.e. each ignition-on): only a successful unlink plus a directory fsync
  arms the hook. Missing token = unarmed; any removal/fsync error = unarmed with a logged exception. A restart
  (controlsd crash, next ignition) finds no token and stays unarmed. No file I/O while driving.
- `selfdrive/controls/lib/identification_hook.py`: `MAX_TRIALS = 1` per hook instance; first `TRIALS` profile
  `step -0.5` for 3.0 s. Later profiles stay defined but unreachable. Start band 7-11 m/s (first smoke targets
  8.33 m/s); `V_END` ends the command early at vEgo <= 4.5 m/s; `RELEASE_JERK` bounds release to 0.8 m/s^3.
  `_update` removes hook authority on a pedal or disengagement; a fresh ACTIVE press cancels the test.
  The trigger judges READY before crediting the press frame and requires at least 50 ms and less than 0.5 s of
  observed press duration; the upper bound uses the existing `CRUISE_LONG_PRESS`. A failed gate or long flag discards it.
  `_handback_text` names cruise resuming for ordinary ends; input/control faults instead say park and review.
- Test scope `identification_mode` = flag True + `HYUNDAI_SANTA_FE_HEV_2022` + openpilot longitudinal
  (`frogpilot/common/frogpilot_variables.py:575`): the three distance mappings are treated as 0 (No Action) and LKAS
  cannot select a personality, without Params writes. Readers: `selfdrive/car/card.py:274` (restores the raw wheel
  button before FrogPilotCard), `frogpilot/controls/frogpilot_card.py:84,116` (no on-screen merge, Traffic off),
  `selfdrive/selfdrived/selfdrived.py:173,594,654,663` (Standard at start, Standard in every published
  `selfdriveState`, setter ignored, Standard on every Params refresh), `controlsd.py:255` (hook `mapping` gate).
- `selfdrive/controls/controlsd.py:55,211` the banner uses `alertDebug`; `fullupdate.sh:63,176` (staging banner and
  reboot supervisor) and the longitudinal maneuver mode also publish it. The source implies that competing publishers
  can end controlsd (a restart and an SCC12 gap; the restart comes back unarmed); this was not reproduced on the car.
  For the WHOLE flag-True period: no `fullupdate.sh` with the ignition on, no settings Reboot with the ignition on, no
  `LongitudinalManeuverMode`, no running reboot supervisor, no raw ignition-on reboot, no new msgq subscriptions.
  Test-mode updates and teardown are OFF-ROAD only. Travel to and from the site with the flag False and no arm file.
- `frogpilot/controls/frogpilot_card.py:94-111`: a press released before 0.5 s (`CRUISE_LONG_PRESS`) is short; the
  long/very-long flags (0.5 s / 2.5 s) stay published and the hook rejects a press that carried them.
- Mappings (read only; the procedure no longer changes them): keys `DistanceButtonControl`, `LongDistanceButtonControl`, `VeryLongDistanceButtonControl`
  (`common/params_keys.h:234,313,513`); values `frogpilot/common/frogpilot_variables.py` `BUTTON_FUNCTIONS`
  (0 No Action, 1 Personality, 2 Force Coast, 3 Pause Steering, 4 Pause Accel/Braking, 5 Experimental, 6 Traffic);
  UI `frogpilot/ui/qt/offroad/wheel_settings.cc:7-25`. The host read 2 / 1 / 6 on 2026-09-25 (`DEVICE_BEFORE.json`);
  re-read at the site. They must be the same before and after the session.

B2. Distance worksheet at 30 km/h (sensitivity only; NOT a braking-distance guarantee)
Assumptions: start from rest inside the usable straight; constant 8.33 m/s through 2 s qualification, 1.5 s for the
press (a short press takes less; the allowance stays as margin), the 3 s profile (no credit for its braking) and the 0.625 s nominal release; then 2 s to manual braking with the listed
normal-cruise acceleration; 1.5 m/s^2 manual braking; 20 m unused runout. Faults, slope, grip, overshoot and real
delays can make every row longer. Source: `HOST_30KPH_PROBE.json` and `codex/LOWER_SPEED_REVIEW.md`.

| Setup accel | Takeover accel | Setup + qualification | Press to stop + runout | Total |
|---:|---:|---:|---:|---:|
| 1.0 | 0 | 51.4 m | 102.5 m | 153.9 m |
| 1.0 | 1.0 | 51.4 m | 117.0 m | 168.4 m |
| 1.0 | 2.0 | 51.4 m | 134.1 m | 185.5 m |
| 0.7 | 2.0 | 66.3 m | 134.1 m | 200.3 m |
| 0.5 | 2.0 | 86.1 m | 134.1 m | 220.2 m |

Worksheet example, not a rule: with +2 m/s^2 takeover, 134.1 m from press to END. The LAST-PRESS point is chosen on
site from the measured END, all the phases and the extra margin; no row approves a site by itself.

B3. Snapshot and preflight check (read-only file reads; no msgq readers, no driving imports). Save this block as
`b3.sh` and the B6 block as `b6.sh`; run them with `bash` (their `set -e` would close an interactive shell).
```sh
set -euo pipefail
S=~/.route_sync/corpus/brake_response_session_20260926; mkdir -p "$S"
EXPECT_FLAG="${EXPECT_FLAG:-False}"   # True only at B4.3
EXPECT_MAP="${EXPECT_MAP:-}"          # the restore record's values (e.g. 2/1/6) at every later check; empty = record only
EXPECT_INITIAL="${EXPECT_INITIAL:-}"  # 30 before activation; original recorded value after restoration
OUT=$(mktemp "$S/snapshot_$(date +%Y%m%dT%H%M%S)_XXXXXX")
REMOTE='set -eu; cd /data/openpilot; echo "head=$(git rev-parse HEAD)"
echo "tracked_changes=$(git status --porcelain --untracked-files=no | wc -l | tr -d " ")"
echo "flag=$(sed -n "s/^IDENTIFICATION_HOOK = \([A-Za-z]*\) .*/\1/p" selfdrive/controls/lib/stopping_flags.py)"
if [ -e /data/identification_hook.arm ]; then echo arm_file=present; else echo arm_file=absent; fi
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
checks = {'mappings present, 0-6': all(re.fullmatch('[0-6]', v) for v in maps), 'maneuver mode off': s['LongitudinalManeuverMode'] in ('0', 'ABSENT'),
          'off-road': s['IsOnroad'] == '0' and s['IsOffroad'] == '1', 'no supervisor': s['supervisor'] == 'none',
          'no tracked changes': s['tracked_changes'] == '0', 'flag': s['flag'] == sys.argv[2], 'arm file absent': s['arm_file'] == 'absent',
          'expected mappings': sys.argv[3] in ('', '/'.join(maps)),
          'initial speed present, 8-170': s['InitialSetSpeed'].isdigit() and 8 <= int(s['InitialSetSpeed']) <= 170,
          'expected initial speed': sys.argv[4] in ('', s['InitialSetSpeed'])}
print('\n'.join(lines))
if not all(checks.values()):
  sys.exit(f'FAIL: {[k for k, ok in checks.items() if not ok]}')
print('PREFLIGHT PASS')
PY
```
The first PASS snapshot at the site is the mandatory restore record (exact values of the three mappings and
`InitialSetSpeed`). A missing or malformed setting key fails the check; resolve how to
restore it before any setting change. Confirm the car and openpilot longitudinal from the newest copied route's
`carParams` (`HYUNDAI_SANTA_FE_HEV_2022`, `openpilotLongitudinalControl` True) with the B6 extractor.
Immediately after a reboot, wait for the off-road state to settle and retry; missing state keys remain a failed check.

B4. Activation (only after the A1 GO, the B3 PASS and the A2 settings)
0. Connectivity, before enabling: host to device (B3 PASS) and device fetch:
   `ssh comma 'cd /data/openpilot && git ls-remote origin "refs/heads/!my-fp-new"'` (on timeout at once `commawifi`).
1. Enable patch (not applied): `~/.route_sync/corpus/button_calibration_prep_20260926/build/enable_identification_hook.patch`
   (one line, `False` -> `True`); reverse: `disable_identification_hook.patch` in the same directory. The 25 September
   patches no longer apply (the flag comment changed); `git apply --check` the enable patch before use. `fullupdate.sh` resets to the pushed
   branch, so the host commits and pushes the enable change, and pushes the reverse as soon as the session ends.
2. Ignition OFF, `EXPECT_MAP=2/1/6 EXPECT_INITIAL=30 bash b3.sh` PASS (the restore record's mapping values). Deploy: `ssh -tt comma 'cd /data/openpilot && ./fullupdate.sh'` (on timeout at once
   `ssh -tt commawifi ...`). Off-road it reboots at once. If the output says "staged", run
   `ssh comma 'touch /data/fullupdate_reboot.cancel'` (same fallback), then verify the supervisor has exited and stop collection.
3. After the reboot: `EXPECT_FLAG=True EXPECT_MAP=2/1/6 EXPECT_INITIAL=30 bash b3.sh` PASS, HEAD = enable commit.
4. Ignition still OFF: `ssh comma 'touch /data/identification_hook.arm'` (one token = one attempt). Then ignition ON,
   parked. Never create the token while controlsd runs (ignition on): the running controlsd does not read it, and
   the next start would consume it.
5. Confirm from files, not a subscriber:
   `ssh comma 'grep -h "identification hook constructed" /data/log/swaglog.* | tail -n 2; test ! -e /data/identification_hook.arm && echo token=consumed'`:
   the newest line (check its `created` time) must be from this start and show `armed=True`, and the token must be
   consumed. `armed=False` or a "not consumed durably" exception: no attempt this session. Then the driver follows A3.

B5. Teardown (always, also after NO-GO or abort; parked, before driving anywhere)
If the enable commit was never deployed, restore any changed settings and verify flag False / arm absent; no reverse commit is needed.
Otherwise:
1. Park and switch cruise off. Restore Initial Set Speed in the UI to the restore record (the mappings were not
   changed; do not touch them).
2. `ssh comma 'rm -f /data/identification_hook.arm && test ! -e /data/identification_hook.arm'` must succeed (same
   fallback). Normally the token is already consumed; this removes an unconsumed one (e.g. after a NO-GO).
3. Ignition OFF. Copy logs (B6) before any other change.
4. Push the reverse commit. With `EXPECT_FLAG=True bash b3.sh` PASS, deploy off-road as in B4.2. Then
   `EXPECT_MAP=2/1/6 EXPECT_INITIAL=160 bash b3.sh` (use the actual restore record's values) must PASS with HEAD = disable commit.
5. After the next ignition-on, no new "identification hook constructed" line appears in `/data/log/swaglog.*`.
Nobody drives home with the flag True (the distance button would still do only the test functions) or the arm file present.

B6. Logs (host, `.venv` active, repository root; ignition off so segments are final). Set
`S=~/.route_sync/corpus/brake_response_session_20260926` in this shell (B3 ran in a separate shell). Then
`python tools/route_sync/refresh_routes.py --host comma --include-rlog --newest-first --report-file "$S/refresh_report.json"`
(falls back to `commawifi` itself and skips locked segments; a clean run does not prove coverage). Then, with the
session route from the report, run `R=00002130--abcdef0123 bash b6.sh` (example name):
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
Check that the last segment reaches past the stop. The extractor (local artifact) re-checks size, sha256 and complete
zstd of every packet file. It keeps `logMonoTime` and `valid` on every row of: `initData` (commit, branch, dirty,
mappings), `alertDebug` text, "identification hook" cloudlog lines, `carControl` (enabled, longActive, override, accel),
`controlsState.longControlState`, `frogpilotCarState` (distance presses, forceCoast, pauseLongitudinal), `carParams`
and the gate inputs (carState faults, plan lead/stop/fcw, model lead probabilities, radar leads/errors). The baseline
packet omits these. Non-finite values stay as JSON NaN/Infinity; `valid` True does not make them valid physics.
Never edit or recompress the originals.

B7. Review before any further collection (acceptance fields)
- Identity: route, segments, SHA256, commit/dirty, controlsd construction lines (`armed=True`, any restart), trial 1,
  banners.
- Command: actual sent SCC12 `aReqRaw`/`aReqValue`/`ACCMode`/`StopReq` and SCC14 jerk limits (`can_response.py`), with
  `carControl` enabled/longActive/override and the pedals. A banner or a unit test is not proof of the sent command;
  the sent command is not achieved acceleration.
- Response: measured speed/acceleration, grade and direction, handback tail, and whether the driver's brake cut it.
- Outcome: complete, speed-truncated (`complete` also covers an early end at 4.5 m/s; use timing and speed), or
  aborted (reason), faults, missing segments, gaps. Keep all of them as labelled data. Decide only after this review;
  this session authorizes no second attempt.
