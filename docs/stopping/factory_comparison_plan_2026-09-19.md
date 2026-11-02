# Next stopping investigation: factory SCC comparison

## Decision and scope

Radek reports that factory cruise is smoother on average than our controller,
although it also makes harsh stops. The next investigation will compare command
history, control modes and measured motion to choose the next experiment: a
coherent stopping trajectory, or a correction for the car's command response.
An inconclusive comparison will identify the missing measurement explicitly.
It will not trigger another unrestricted coefficient search.

This step does not change braking code. The target is the complete normal stop:
approach, brake build/release/rebuild, settling, completion and hold. Emergency
responses and driver interventions remain separate evidence. Clearance and hold
requirements remain constraints; comfort includes body-motion estimates and
Radek's assessment, with measurement uncertainty retained.

## What is already available

- Radek's 11 explicitly marked manual finishes remain the personal reference.
  Their earlier approach contexts are mixed/unknown, so they do not supply 11
  complete demonstrations of his stopping policy.
- The six automatic stops in the latest review remain development evidence.
  Route `00002102--505ee0f006@1202469868148` is the current-version pump example.
- A bounded inventory found `radek_baseline_20260912/stock_2080/`, recorded on
  2026-09-05. Raw segment 0 confirms the Santa Fe fingerprint,
  `openpilotLongitudinalControl=false`, `pcmCruise=true`, `passive=false` and
  Hyundai safety parameter 2. However, its only indexed rest,
  `00002080--01bc05dde8@524143289404`, was manual: raw segments 2/3 contain 3,051
  carState samples from rest-30 s through rest+0.5 s, all with cruise disabled.
  Pedal counts are 1,285 gas, 903 brake and 863 neither. It is not a factory-SCC
  automatic stop or an additional labelled Radek reference.

This inventory does not establish that no factory stop exists elsewhere. It
establishes that the known stock-labelled candidate does not fill the gap.

## Minimal request to Radek

One ordinary trip using factory cruise, with comma connected and logging. Aim for
about five naturally occurring complete stops behind traffic. Include ordinary
and poor stops; do not select only the best. Drive and intervene as usual. Pedal
overrides are retained and labelled, not treated as failed data collection.

After parking, provide the approximate trip time and any memorable good or harsh
stops, preferably with location or stop order. No device interaction while moving
is needed. This is an initial diagnostic sample, not enough to demonstrate average
superiority. Additional manual drives are not requested for this first step.

Codex handles configuration checks, download, event selection and analysis.
Before the trip, Radek supplies a parked setup window. Snapshot the original
control/logging settings before changing them, then restore those exact settings
after the capture in another parked setup window.

## Supported recording configuration

Code inspection supports passive factory recording. With ignition off, while
comma remains powered, set **Settings -> Developer -> openpilot Longitudinal
Control (Alpha)** OFF and **Settings -> Toggles -> Enable openpilot** OFF. Apply
at the next normal ignition start. Openpilot steering assistance is also off.
The Developer sidebar requires FrogPilot Tuning Level Developer; preserve its
original value if changing it is needed. Do not use the toggle click as a live
handoff. Experimental Mode OFF alone does not return longitudinal control to SCC.

`card.py` reads these settings at initialization; the Hyundai interface selects
factory longitudinal, and passive card selects Panda noOutput. On-road logging
does not require openpilot actuation, but the FrogPilot logging-disable setting
must be checked. Stock initialization clears ExperimentalMode, so restore its
saved value as well as the two control toggles when returning to normal operation.

Before accepting the data, verify copied logs contain the intended fingerprint,
`openpilotLongitudinalControl=false`, `pcmCruise=true`, `passive=true`, noOutput
safety and complete raw CAN. For each admitted stock stop, verify fresh factory
SCC activity and pedal/override status through the approach and settle. Preserve
SCC11/12/14; lead/radar availability can differ in factory mode. CAN echo is not
a measurement of delivered force. Initial settings alone do not prove ownership.

Sources: `selfdrive/ui/qt/offroad/developer_panel.cc`, `selfdrive/car/card.py`,
`opendbc_repo/opendbc/car/hyundai/interface.py`, `system/manager/process_config.py`,
`selfdrive/selfdrived/selfdrived.py`. This path is code-verified; the capture's
actual configuration and logging remain to be verified. No device settings have
been changed for this investigation. Longitudinal Maneuver Mode is not used.

## Analysis and next decision

1. Freeze source hashes and classify every complete stop before comfort ranking.
   Keep driver identity, authority, pedals, scene, speed, gap and available grade
   information separate. Do not call unmatched events a controlled comparison.
2. Align final CAN commands, SCC modes, measured speed/acceleration and body-motion
   estimates without fitting away their delays. Compare build, release and
   rebuild timing, terminal settling and final gap. Determine whether the current
   comfort measures agree with Radek's good/poor assessments.
3. Use existing command episodes to test a bounded response hypothesis: recent
   command history predicts motion that the delayed-request coast observer calls
   external push. Evaluate release and build separately and preserve unseen
   events for validation; do not fit and score the same stops as independent data.
4. Construct an offline feasibility bound at the current pump's takeover, using
   actual motion and pending commands. Ask whether a smooth complete stop can
   meet the same clearance/completion requirements across plausible local
   responses. Future lead motion may be supplied only as an explicitly optimistic
   bound. Feasibility is not proof of an implementable online policy; failure
   under an uncertain model is not proof of a physical interface limit.
5. Report the command/motion comparisons and select one falsifiable next change.
   A smoother stock command through comparable modes prioritizes the trajectory
   and ownership path. Different responses under comparable histories prioritize
   actuator state/mode identification. In either case preserve uncertainty and
   test alternative explanations before attributing cause.

The first factory sample is for discovery. Any resulting candidate needs a later
untouched acceptance set, appropriate runtime/safety checks and measured complete
stops. Hypothetical-plant failures remain visible, with their assumptions stated;
millimetre-level proxy differences are not treated as measurement certainty.
