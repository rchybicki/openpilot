"""Phase-0 kill-switch constants for the stopping-stack redesign (FINAL_SPEC §3, §4.1, §4.2).

Constants only -- no logic lives here. Each flag flips in its own one-line commit;
revert = flip back + redeploy. Consumers read these as module attributes
(``stopping_flags.PUBLISH_TRUE_LEAD_DISTANCE``) so both states stay unit-testable.
"""

# KILL SWITCH: 0.0 restores the raw planner shouldStop flag (no falling-edge hold). §4.1
SHOULD_STOP_FALLING_EDGE_HOLD_S = 0.4

# Assert-side early-entry lookahead seconds; OFF by design (> 0.0 enables) -- it delays
# stop entry and is not strictly conservative. §4.1
SHOULD_STOP_LOOKAHEAD_S = 0.0

# KILL SWITCH: False restores the current Santa Fe stopped-lead smooth-approach cap
# (activation floor v_ego >= 2.50 m/s). True extends the SAME one-way brake cap down
# into the 0.80-2.50 m/s creep-to-stop band so a confirmed stopping/creeping-then-
# stopping lead is carried under a gentle controlled approach toward the 4.0 m hold
# gap, instead of bleeding speed to the ~6 m MPC follow gap and far-crawling in.
# Strictly additive braking-toward: the cap is applied as min(output_a_target, cap)
# and returns None once remaining-to-hold-gap <= 0, so it can never reduce braking and
# never commands a rest inside 4.0 m. Flip back to False to restore prior behavior. (seg30 wide-stop)
SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_EXTENSION = True

# KILL SWITCH: False restores the prior behavior where the MPC is free to over-brake a
# confirmed creeping-then-stopping lead all the way to a near-stop at the ~6 m follow gap
# (before any explicit stop target exists), then far-crawls the remaining distance. True
# enables santa_fe_stopping_lead_roll_in: the MIRROR of the stopped-lead smooth-approach
# cap. The cap DEEPENS brake (min) "brake AT LEAST enough to stop at the 4.0 m hold gap";
# this floor RAISES brake (max) "do not brake HARDER than needed to stop at the 4.0 m hold
# gap", so the car carries the minimum speed to roll continuously to ~4 m instead of
# near-stopping far back. Floor and cap SHARE one hold-gap/buffer/required-decel source of
# truth (get_santa_fe_stopped_lead_hold_gap_required_decel), so they converge on the SAME
# target: cap deepens up to it, floor raises down to it -> command clamped to exactly that
# gentle stop-at-hold-gap decel. Strictly bounded: gated OFF once output_should_stop (so the
# seg24 anti-collision net keeps full authority), on force-coast, on a latched lead hard-stop
# dwell (real sudden stops hand full brake to the MPC), and once remaining-to-hold-gap <=
# margin. By construction the floor's required-decel == the cap's, so max() can never carry
# speed PAST the hold gap. Flip back to False to restore prior behavior.
SANTA_FE_STOPPING_LEAD_ROLL_IN = True

# KILL SWITCH: the stop-commitment necessity floor (longitudinal_planner.py, route 00001f47
# seg6 driver takeover). DEEPEN-ONLY min() to the decel that rests the ego at the 3.0 m band
# floor behind a persistence-confirmed stopping/stopped lead, Schmitt-gated on kinematic
# necessity exceeding the current command. Flip to False to remove the lane entirely.
SANTA_FE_STOP_COMMIT_ENVELOPE = True
SANTA_FE_STOP_AIM_ENVELOPE = True   # cycle-24: aim-commitment necessity floor (00001f90 seg22)
SANTA_FE_REST_CLOSE_FLOOR = True    # cycle-31: E1-R rest-close reference floor (00002011 s22)
# cycle 2026-10-04 entry bite, upstream part: ONE STOP LINE (longitudinal_planner.py, SANTA_FE_STOP_LINE_*). Behind a
# certified STOPPED radar lead the planner floors aTarget at the StoppingService governor law projected to the V_ENTER
# hand-over, so the car reaches the service band on the governor profile. ONE switch for the whole entry-bite fix: the
# in-band part GOVERNOR_BAND_PROFILE is derived from it (each part alone fails the gates). Revert = False.
SANTA_FE_STOP_LINE = False

# KILL SWITCH: False restores the legacy producer behavior where
# get_stopped_lead_control_target keeps re-asserting a synthetic stop target on a STOPPED
# lead during the low-speed crept-to-rest window. On bouncing radar (dRel jitter, e.g.
# 2.0<->3.9 m) each re-assertion re-arms the low_speed_stopped_lead_glide_accel_cap
# near_hold_gap_cap (indexed on lead_d_rel, gated by stop_request_active OR stopping), which
# fires repeated firm brake re-grabs that walk the car INWARD toward the lead. True adds an
# arrived-state early-return: once v_ego <= STOPPED_LEAD_ARRIVED_V_EGO_MAX and we are within
# STOPPED_LEAD_REST_GAP_M + STOPPED_LEAD_ARRIVED_GAP_MARGIN_M of the lead, the producer
# returns None so stop_request_active is not re-asserted and the glide-cap re-grab is not
# re-triggered. Strictly inactive above the arrived band (all existing pins run at
# v_ego > 0.35). Flip back to False to restore prior behavior. (route 00001786 seg5)
STOPPED_LEAD_ARRIVED_GATE_ENABLED = True

# KILL SWITCH: False restores the legacy radard publish-side dRel mutation
# (published dRel = true dRel - increasedStoppedDistance) and the exactly-cancelling
# compensation terms in long_mpc / stop_target_helpers / the planner Santa Fe caps.
# True publishes the TRUE lead distance and re-expresses ISD as the single
# "rest ISD meters farther back" term (§4.2.3). Flipped 2026-06-10 after the §4.2.5
# device read: IncreasedStoppedDistance == 0.0 on the target car (all weather variants 0),
# so the flip is bit-identical today; later ISD raises are compensated by design
# (single-point lead_d_rel_eff, rest-gap equality pinned for ISD in {0, 1.5, 3.0}).
PUBLISH_TRUE_LEAD_DISTANCE = True

# KILL SWITCH: False restores the legacy Santa-Fe terminal-stop patchwork (the synthetic
# stopped-lead control target rests at STOPPED_LEAD_MIN_CONTROL_GAP_M 2.75 m, the
# STOPPED_LEAD_ARRIVED_GATE early-return is live, and the over-brake caps
# low_speed_stopped_lead_glide_accel_cap / low_speed_close_lead_accel_cap fire). True replaces
# that patchwork with a single terminal-glide PROFILE: feed the EXISTING jerk-limited trajectory
# tracker the right target -- the synthetic close-band rest gap becomes LEAD_STOP_DISTANCE_TARGET
# (4.0 m), so the synthetic target, the MPC/explicit target, the planner roll-in floor and the
# smooth-approach cap ALL rest at 4.0 m; the a=-v^2/(2*d_eff) jerk-limited law then lands v=0 AT
# 4.0 m in one monotonic glide instead of near-stopping short and creeping in. THREE coupled
# corrections under this ONE flag (Santa-Fe-fingerprint scoped exactly like the should_apply_*
# predicates):
#   (1) TARGET: get_stopped_lead_control_target returns max(lead_d_rel - 4.0, 0.05) in the close
#       band (the trigger band math still uses 2.75 so control still FIRES -- only the returned
#       rest distance moves). The STOPPED_LEAD_ARRIVED_GATE early-return is retired (bypassed):
#       the corrected stable target removes the synthetic jitter the arrived-gate patched.
#   (2) HOLD: the stop_reference SETTLE/HOLD branch uses A_HOLD_FIRM (-0.32 ==
#       FORCE_COAST_STANDSTILL_HOLD_ACCEL, the proven HEV creep-counter magnitude) instead of the
#       gentle A_HOLD (-0.16..-0.10) that creep torque overpowers, eased in by J_SETTLE_RELEASE.
#   (3) RETIRE the over-brake patchwork: low_speed_stopped_lead_glide_accel_cap (the binding
#       leapfrog over-brake) and low_speed_close_lead_accel_cap are BYPASSED via their
#       should_apply_* predicate gates (a clean bypass, not a deletion).
# STAGED ROLLOUT: STOPPING_CLOSE_GAP_CREEP_ENABLED, the seg24 STOPPING_PLANNER_FLOOR,
# FORCE_COAST_STANDSTILL_HOLD and the far_stopped_lead releases all stay ENABLED as safety
# closers until the on-road landing distribution confirms <= ~4.6 m. Flip back to False to
# restore the prior patchwork. (routes 0000178a seg19 / 00001aea engaged leapfrogs)
SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED = True

# --- Stopping Service V3 staged-rollout mode (docs/stopping/stopping_service_v3_plan.md §6) ------
# The clean-slate stopping service (stop_context.py + stopping_service.py + stopping_telemetry.py)
# ships behind this single mode string; each stage flip is its own one-line commit, revert = flip
# back + redeploy. Stages:
#   "OFF"           -- stage 0: service modules + tests exist but nothing instantiates them on-car.
#   "SHADOW"        -- stage 1: the service is computed per stopping-band frame for the Santa Fe HEV
#                      fingerprint ONLY, strictly AFTER output_accel is final; its output is NEVER
#                      written to the wire -- divergence logging only (cloudlog 'stopping_service'
#                      events). Zero behavior change by construction. This is the one-flag REVERT
#                      target: flipping back here restores today's byte-identical legacy wire.
#   "LIVE_TERMINAL" -- stage 2 (wired 2026-07-02; now the FIRST REVERT TIER): the service OWNS the
#                      stopping-state wire for v <= 0.85 m/s (handback hysteresis 0.95), Santa Fe
#                      HEV fingerprint only, with a jerk-consistent takeover seeded from the live
#                      command. The service+context keep running the full stage-1 band (v < 2.5) in
#                      OBSERVATION, so the takeover context (a_coast EMA, gap filter, lead latch) is
#                      warm and the 0.85-2.5 divergence telemetry keeps flowing; the wire is written
#                      only in the own band. The legacy sub-0.30 over-brake cap family is bypassed on
#                      owned frames (flag-gated condition, code intact), and the close-gap creep
#                      feature is structurally dead there too (the service HOLD owns the standstill
#                      wire; plan §3: no post-stop motion lanes); the seg24 planner floor and the
#                      force-coast -0.32 standstill hold stay live (both deepen-only). A LIVE
#                      exception falls back to the legacy chain value for that frame and latches
#                      ownership OFF for the drive.
#                      GATES: the plan §6 stage-1 shadow-drive gate was WAIVED by the user
#                      (2026-07-01 decision: L2 system, driver supervises); the offline default-fail
#                      under-brake gate is RETAINED and must be green on the deploy SHA.
#   "LIVE"          -- stage 3 (CURRENT, wired 2026-07-02): the FULL stop-intent band. The service
#                      owns the wire on EVERY frame it reports active -- its own entry conditions
#                      (v < V_ENTER 2.5 AND (shouldStop OR lead-stopped latch with d_rem < 15)) and
#                      its own RELEASE/exit are the sole ownership authority, in BOTH the pid and
#                      stopping long-control states. This removes the stage-2 0.85 seam and its
#                      inherited-command strand: on the first live stage-2 drive (route 00001b72)
#                      the stopping state only engaged at v = 0.15, so the planner's one-frame
#                      aTarget slam (-0.32 -> -0.81 at v = 0.92, kinematic need only -0.33) reached
#                      the wire through the PID state (IMU jerk 8.3). While the service owns in the
#                      pid state the integrator is frozen and reseeded to the service command each
#                      owned frame (stepless handback, no windup) and the pid-band caps' pid.i
#                      side-effects are gated off. Everything else is stage-2 semantics unchanged
#                      (jerk-consistent takeover, cap bypass on owned frames, exception latch,
#                      seg24 floor + force-coast hold live). LIVE_TERMINAL and SHADOW remain as
#                      revert tiers -- revert is one word here.
SERVICE_MODE = "LIVE"

# cycle-32: Santa Fe HEV approach accel-tracking trim (longcontrol.py SANTA_FE_TRIM_* block). Bounded,
# deepen-only, dead-banded integral of the delay-compensated tracking error against the UNTRIMMED
# demand; learns only while the pid demand is the wire. False = byte-identical pre-cycle-32 wire.
SANTA_FE_ACCEL_TRACKING_TRIM = True

# universal stop program step 3 (docs/stopping/universal_stop_program.md): the service's APPROACH law.
# "legacy" = GLIDE/EASE with their patch lanes (normalization, late-entry corridor) -- byte-identical
# behaviour. "governor" = the one stateless law (governor_demand) + the 3.1 m barrier in the safety
# min; terminal descent, hold, release, monitor, a_kin, a_plan and the sole jerk limiter unchanged;
# no-lead stops keep the legacy law. Revert is one word.
SERVICE_APPROACH_LAW = "governor"   # FLIPPED 2026-08-29 (cycle 38): gate met -- 13/13 stopped-lead traces
                                    # consistent (governor deeper p50 0.89, divergence bounded), rests best yet
                                    # (median 4.13, zero long), the harsh cluster (ease->grab pump at walking
                                    # pace, both user bookmarks) is the legacy shape this law replaces. Revert
                                    # is this one word back to "legacy".
ATTRIBUTED_SAFETY = "live"          # "off" | "shadow" | "live" (2026-09-02/05, program step "attributed safety"): inside governor
                                    # ownership the SHADOW computes the candidate wire with the planner's comfort
                                    # output removed from the min (only attributed safety lanes may deepen: a_kin,
                                    # a_bar, a_mon, a_pred) and logs how often / how deep a_plan binds. Telemetry
                                    # only; the wire is untouched. "live" (2026-09-05, red-team 20260905-160050):
                                    # on eligible frames after 0.30 s of continuous eligibility the target becomes
                                    # min(candidate, last_cmd + 0.8 m/s^3 * dt) -- a_plan's excess is RELEASED at
                                    # <= 0.8 m/s^3; any ineligible frame re-admits a_plan at once. Revert = "shadow".
# cycle 53 (2026-09-06, route 00002086 s8 / s17): the governor's moving-lead reference. False = v_ref = max(v_lead,0) + q_ref(d)
# (a crawler's speed lifts the ego's speed ceiling; when the crawler stops the ego sheds crawler + closure speed inside d: the
# hot re-entry grabs -1.06 / -1.27). True = v_ref = q_ref(d): the margin profile IS the ego speed law, a moving lead only slows
# the closure of the reference (a_ff = -A_C (q_ref - max(v_lead,0)) / (q_ref + A_C TAU)). Byte-identical for v_lead <= 0; never
# shallower than False for v_lead > 0. Revert = False.
GOVERNOR_PROFILE_REFERENCE = True
# Limit brake release below the approach profile using the remaining margin; never deepen the previous command.
# Net reference bounded at GOV_A_C. Terminal descent, following/departure and safety lanes stay intact. Revert = False.
GOVERNOR_RECOVERY_BRAKE = True
# cycle 2026-10-04 (band-consistent governor): with the creep guard (FINAL_FLOOR) and FLAT_LANDING on, the governor profile
# closes at the levels the band executes -- GOV_A_C (= -A_GUARD) down to the descent capture speed, -A_FLOOR below it -- instead
# of the TAU fade, which asked -0.47..0 between 1.3 and 0.5 m/s where the guard holds -0.60 (an on-profile arrival rested ~0.9 m
# long; the 4.3-4.5 m rests came from hot arrivals and the entry bite). Same anchor, lag, forecast, pursuit and safety lanes.
# Scope (eb2 red-team): a stop whose service entry is hot under the band profile (it would ask more than
# stopping_service.GOV_BAND_ENTRY_TOL beyond the entry command) keeps the TAU law, so a hot arrival the stop line did not
# shape keeps today's landing. Derived: it flips and reverts with SANTA_FE_STOP_LINE (never set it on its own).
GOVERNOR_BAND_PROFILE = SANTA_FE_STOP_LINE
# cycle 53: the approach hands back exactly as today EXCEPT while the ego is still measurably CLOSING on a present lead inside
# the band (v - lv > MON_LEAD_RECEDE_MPS, d_rem < ENTRY_LEAD_D_REM_MAX, v < V_ENTER): a crawler at 0.3-0.6 m/s un-confirms the
# ENTRY latch and today's exit left the closure to the planner's trajectory lane (-0.24 at 1.02 m/s, 6 -> 5 m) until the
# re-entry landed hot. Following a crawler (v <= lv + 0.15) stays the planner's (braking-only contract), and the planner's go
# (a_target > 0.2) behind a lead outside the stopped window (> 0.3 m/s) ends the stay at once (review: a rolling ego stays
# faster than a slowly departing lead for seconds). Governor law only (the legacy law keeps its exit). Revert = False.
SERVICE_STAY_WHILE_CLOSING = True
WHOLE_APPROACH_GOVERNOR = "off"  # "off" | "shadow" only; OFF until the recorded-input gate passes (cycle 42)
IDENTIFICATION_HOOK = False   # TEMPORARY brake-response test mode (identification_hook.py). Master kill: False = nothing
                              # is constructed. True only in the test-session build: the hook starts OFF and only a long
                              # press of the wheel distance button arms it. True on the Santa Fe HEV with openpilot
                              # longitudinal also sets FrogPilot identification_mode for the whole drive (distance mappings
                              # NOTHING, physical button, Standard, no Traffic) without saved Params writes. Deleted with
                              # the module after the collection.

# cycle 52 (2026-09-05): the harsh no-lead FORCE-COAST stop is the force-coast profile itself (frogpilot/controls/lib/force_coast.py),
# held at -0.7 x strength into the wheel stop by all three consumers (the ACC zero-cruise cap bounded by the strength limiter,
# long control's no-target ramp, the planner's lower accel clip). The driver's contract: force coast ADDS braking the model would
# not, anything the car sees still wins, and it must never end in a bad stop. True = an absolute comfort tail bounds the profile
# on the shallow side below ~1.5 m/s ([gate 0.2, 0.5, 1.0, 1.5 m/s] -> [-0.5, -0.6, -0.8, -1.0], fading out by ~1.8 m/s; the
# strength profile is untouched above) and long control's force-coast command eases no faster than 0.8 m/s^3. False = today's
# profile. The floor contract in long control (deeper demand passes) is not behind this flag.
FORCE_COAST_TERMINAL_TAPER = True

# Stage 1, car-first stopping trial (2026-09-27, DECISION_v2 section 12). In 2nd gear the hybrid adds creep torque when the
# request sits at -0.5 or shallower between about 1.3 and 0.45 m/s, and cuts it at about 0.4 m/s before the 2->1 downshift:
# the car then brakes 0.2-0.5 deeper than asked (the late grab). FINAL_FLOOR is the creep guard: behind a confirmed stopped
# lead, below V_GUARD, the final request is at most A_GUARD (reached at J_GUARD from the request on the arming frame), easing
# to A_FLOOR across V_GUARD_EASE; deeper requests pass; it stays armed up to V_FLOOR. FLAT_LANDING lands at
# min(A_FLOOR, max(arrival, -0.70)) instead of the -0.70 descent. Revert = False.
FINAL_FLOOR = True
FLAT_LANDING = True
A_FLOOR = -0.50
V_FLOOR = 2.5
A_GUARD = -0.60
V_GUARD = 1.3
J_GUARD = 1.0
V_GUARD_EASE = (0.3, 0.5)
# Cycle 2026-10-12 WS REST (cycle_20261009/PLAN.md sections 54-57; ~/.route_sync/work/cyc_1013/rest/PROPOSAL.md). The creep guard
# arms at V_GUARD with no distance term and stops the car in ~1.3-1.9 m; the planner aims at 4.3 m (rest - distanceToStopTarget at
# the wheel stop, n=85), so a cold / on-profile arrival (gap 7.05 m at 1.3 m/s) rested 5-7.5 m (24 of 86 stops since 10-03). True =
# below V_GUARD the guard waits for the service's sticky terminal_ready latch: the remaining distance to the stop point (radar gap -
# anchor, or the planner's nearer target) is within the guard's own stopping distance at the current speed, v <= V_DESCENT_START,
# v rises while waiting, or a fail-safe line (legacy law, no lead / gap trust, lead change, uphill, monitor, a floor-defense lane
# binding). Until then the planner's own command stands. Brakes-less in the waiting window. Revert = False.
# Stall line (cycle 2026-10-13 WS TRIALFIX, 22a2_183.39): a wait above V_DESCENT_START with aEgo shallower than A_GAT_STALL for
# T_GAT_STALL is carried by the 2nd-gear creep: arm (ordinary J_GUARD ramp) before the creep feed-forward deepens the command and the
# creep cuts at the downshift.
# The stall line does not count in the first T_GAT_STALL_ENTRY_S after a service entry / re-assert (cycle 2026-10-16 WS TERMINAL,
# 22c6_362.58: aEgo still lagged the pre-entry -0.28 planner command, q 40, no creep -> stall arm at the entry, guard -0.60 from 1.17 m/s,
# rest 5.18; every real creep stall of 22a6..22c9 armed 1.2-4.2 s after the entry).
GAP_AWARE_TERMINAL = True
A_GAT_STALL = -0.15
T_GAT_STALL = 0.30
T_GAT_STALL_ENTRY_S = 0.5

# Cycle 2026-10-13 WS LANDING (~/.route_sync/work/cyc_1013/landing/PROPOSAL.md). The felt "stiction" at rest is the DELIVERED
# decel at the real wheel stop: swing = 1.71 |a_land| on 86 engaged stops; golden (stiction-free) share 0 % deeper than -0.5, 6 % at
# -0.5..-0.4, 62 % at -0.4..-0.3 (engaged + Radek's manual stops); today's landing delivers -0.52 (5 of 86 golden, all plant
# accidents). True = below V_LAND the FLAT_LANDING command eases at J_TERMINAL_DESCENT while the measured decel (carState aEgo,
# corrected for the command change still in flight over T_LAND_LAG) is deeper than A_LAND_DELIVERED; it freezes there, never
# shallower than A_LAND_CMD_MIN. Any hazard (lead identity change, a nearer target, a binding safety lane, the stall guard: aEgo
# shallower than A_LAND_STALL while vEgo > V_LAND_STALL, the monitor, dropout, an unconfirmed lead, a thin gap, uphill) latches the
# fail-safe: the ease re-deepens to today's landing at J_TERMINAL_DESCENT. The creep guard yields to the eased command below V_LAND.
# AEGO_LAND_BIAS: aEgo reads 0.05..0.08 shallower than the IMU in the last 0.35 s before the wheel stop (r 0.91-0.96, 86 stops,
# design/src/aego_track.py). Revert = False.
# J_LAND (cycle 2026-10-13 WS TRIALFIX): the ease grows at J_LAND, not J_TERMINAL_DESCENT. The plant delivers the eased command 0.3 s
# late with no bias (22a1/22a2: 12/12 samples within +-0.05); at a -0.6 approach the stop comes 0.66-0.71 s after V_LAND, so the 0.6
# ramp (-0.6 -> -0.32 in 0.47 s) finished at the wheel stop (golden 0/4). 1.2 ends it in 0.23 s = ~0.45 s before the stop. The
# fail-safe still re-deepens at J_TERMINAL_DESCENT.
DELIVERED_LANDING = True
J_LAND = 1.2
A_LAND_DELIVERED = -0.38
AEGO_LAND_BIAS = 0.06
A_LAND_CMD_MIN = -0.30
V_LAND = 0.45
T_LAND_LAG = 0.30
A_LAND_STALL = -0.20
V_LAND_STALL = 0.05
D_LAND_NEARER = 0.25

# Cycle 2026-10-16 WS CAPTURE (~/.route_sync/work/cyc_1016/capture/PROPOSAL.md). On hot arrivals the DELIVERED_LANDING fail-safe latched on
# the capture frame (54 of 146 recorded captures; 29 only because of the strict stopped-lead latch): the radar reads a stopped lead
# -0.15..-0.36 under -1.0..-1.5 braking (REL_SPEED 0.15 s late), the strict latch [-0.1, +0.3] un-confirms and re-confirms 0.03-0.3 s
# after the capture, and the car landed at FLAT -0.70 (22be_1088.74, 22c1_434.92). True = the landing alone also counts the lead as
# stopped while the entry latch holds, the identity is >= ATTR_TRUST_IN_S old and the raw vLead is within
# [-(K_LAND_LEAD_TOL |aEgo|max0.3s + V_LAND_LEAD_TOL), +V_LAND_LEAD_TOL] (stopped leads: 99.86 % of landing frames; a forward creep reads
# above +0.10 on 59 %). Every other fail-safe line stays. Revert = False.
LANDING_LEAD_TOLERANCE = True
K_LAND_LEAD_TOL = 0.15
V_LAND_LEAD_TOL = 0.10

# Cycle 2026-10-13 WS ENTRY (~/.route_sync/work/cyc_1013/entry/PROPOSAL_v2.md; Astra plan review PLAN_REVIEW_astra.md). A late in-band
# entry (the lead stops after the car is already slow: 22a1_1056.10 entered at 1.18 m/s with 1.05 m to the 4.3 m anchor) made the
# governor's catch-up step the wire -0.63 -> -1.13 (IMU -1.33). True = on an entry / re-assert below EC_V_MAX with an incoming command
# shallower than -EC_A, the phase demand is capped at -EC_A when the causal predictor (the commands in flight, then -EC_A held to
# V_DESCENT_START, then the census terminal travel) still rests at >= D_REST_MIN + EC_MARGIN. The floor lanes and a_plan bind through
# the min() as today; a hazard or a predicted rest below the threshold ends the cap and today's law resumes at J_DOWN. Revert = False.
ENTRY_COMFORT_CAP = True

# Cycle 2026-10-02 (cycle_20261002/PLAN.md). SANTA_FE_TRIM_HANDOFF: below 2.5 m/s the Santa Fe tracking trim is held while the
# planner still brakes behind a moving lead being closed on (on a downhill the trim is the grade compensation; its release at
# the 2.5 m/s gate made the 0000222e s4 approach arrive hot), the low-speed slew uses the wire without the trim it carried, and
# outside pid below 2.5 m/s the trim is never re-added (the stopping state integrated it). Revert = False.
SANTA_FE_TRIM_HANDOFF = True

# Cycle 2026-10-04 (cycle_20261003/PLAN.md sections 45, 54; route 00002232 at 4904.04). A RELEASE that ends at rest while the same
# lead still reads stopped (strict latch) re-holds (RAMP_TO_HOLD, hold anchor kept) instead of going INACTIVE: the next frame let
# LongControl enter `starting` (StopReq drop, 1st-gear creep toward the stopped lead) while the service re-entered the stop. The
# re-hold refuses the planner go while the lead stays latch-stopped and does not measurably creep away (Doppler > 0.15 m/s AND
# 0.3 m of fresh growth); a lead that creeps away is handed back as before. Revert = False.
RELEASE_END_STOPPED_LEAD_REHOLD = True

# Cycle 2026-10-10 C2 (syn_launch +1.2 s). HOLD -> RELEASE needed the planner go (a_target > 0.2, a 0.3-0.38 s pulse) in the SAME
# frame as the lead pull (lv - v > 0.5); a pulse that ends just before the pull is lost until the next pulse. True: a planner go seen
# in the last RELEASE_GO_KEEP_S still pairs with a later lead pull (Doppler with earned identity; not gap growth alone, which radar
# range drift arms at rest) while the planner does not ask to brake (a_target > 0) and the lead does not brake (aLeadK >= ATTR_LEAD_BRAKING;
# none = braking). The physical-go dwell is unchanged: it restarts
# on outward-held gap frames, and keeping it through them released 2235_s55/s56 toward a queue lead that rolled 1.7 m and stopped
# again (planner stop intent, 'starting' + re-entry, 24 s hold in pid). Revert = False.
RELEASE_GO_EVIDENCE_KEEP = True

# Cycle 2026-10-10 (WS-C3, cycle_20261009/PLAN.md sections 18, 23; q_2073_1098). While the service owns a rolling approach, shouldStop
# dropping as the lead departs takes LongControl through a one-frame 'starting' transit (stopping -> starting -> pid above
# vEgoStarting): the 'starting' branch's reset() ended the ownership, the frame sent the launch command (unslewed above 1.2 m/s) and
# the next owned frame reseeded the service's limiter on it (-0.8 -> +0.7 -> -0.02 in two frames; runner go_v8/v12_t2.2_a2 at 1.21
# m/s). Car drives 00002000..0000227e: no owned transit above 1.2 m/s, 14 at 0.1-1.2 m/s where the low-speed slew bounds the frame
# (steps <= 0.038). True = the transit frame stays owned and the reseed anchors on the previous wire: the service's own J_UP /
# RELEASE J_GO shape the hand-off. The same transit of the legacy chain (service not owning) is not in scope. Revert = False.
OWNED_STARTING_TRANSIT = True

# Cycle 2026-10-10 (WS-C4, cycle_20261009/PLAN.md sections 21, 23; car drives 00002000..0000227e: 8 holds in pid in 7 routes, e.g.
# 2234_701.09, 225d_1229.52, 225f_1110.20). A car that rests > 5 m (effective gap) behind a stopped lead meets the arbiter's
# far-stopped-lead release (v in [0, 0.55)), so the state machine stays in pid while the service holds at rest: no StopReq until vEgo
# dithers below 0 (1.9-10.9 s on the car). True = an owned service RAMP_TO_HOLD / HOLD enters stopping (the sender requests the SCC
# managed hold); the service's RELEASE then keeps stopping until its ramp ends, as on every other stop. These stopping frames are not
# stop intent: the arbiter (stop-intent timer, dropout holds) sees today's pid, so the launch after the hold is today's (a re-hold during
# a launch restarted the 1.0 s window that withholds the low-speed fast release: runner syn_launch 0.47 m/s slower). Revert = False.
SERVICE_HOLD_ENTERS_STOPPING = True

# Cycle 2026-10-05 (cycle_20261003/PLAN.md section 81; trial sim 00002086 1669.67 and 00002232 4958.48, car 00002231 1381.72).
# The Hyundai StopReq latch (carcontroller.py) sets on `stopping` and vEgo < 0.01, which the filtered vEgo reaches ~2.3 s after
# the wheel stop whatever the controller does: inside a service RELEASE ramp (or its hand-back frames) it sets StopReq 0.1-0.5 s
# before the launch clears it, and when a should_stop flicker re-enters `stopping` at rest after `starting` dropped it, it sets it
# again at once. With this flag a NEW set needs 1 s since the command last rose or LongControl was `starting` (a settled hold), or
# 1.5 s of waiting at rest (a release/re-hold cycle is not held on a shallow command without StopReq any longer); clears are
# unchanged, so StopReq is only ever removed and no launch waits on it. Revert = False.
STOPREQ_SETTLED_SET = True

# Cycle 2026-10-10 C6 (0000225d 1229.52, 0000203a 553.38: the HOLD kept -0.70 for 1.8 / 0.6 s after a confirmed departure while the
# lead left at 0.6-1.3 m/s, until the driver's gas). At rest in blended mode the planner's MPC follows the model's flat trajectory:
# aTarget stays 0..+0.2 with shouldStop set while the model itself says go (bit 0), so the planner go never comes. LongControl's own
# chain had already released the stop (departing-lead / far-stopped-lead release) and ran pid under the hold: the service hold alone
# held the car. The physical-go dwell meant for this is dead: it trusts measured-only gap frames and StopContext holds every outward
# radar step for T_PERSIST_OUT_S, so the 0.5 s dwell restarts at each step. A release alone would be a no-op dip: the next frame
# re-enters on the same standstill shouldStop. True, only while LongControl runs pid under the hold (its chain released the stop):
# a dwell of its own counts live geometry (outward holds are a lower bound on the gap) while the model's stop bit is clear, the planner
# does not brake (a_target > 0) and the lead does not brake (aLeadK >= ATTR_LEAD_BRAKING), and restarts when any of them fails (the
# measured-only dwell is today's); the RELEASE end hands the car to pid
# on the planner's target (no `starting` step), and the planner's shouldStop does not re-enter the service while the chain stays
# released in pid, the same lead keeps moving (> LEAD_STOPPED_V_MAX, earned identity), the model bit stays clear and v < V_ENTER;
# the exemption ends with the service's observation (out of band).
# Holds under `stopping` (StopReq), the stopped-lead latch entry and every safety lane are unchanged. With
# SERVICE_HOLD_ENTERS_STOPPING the pid test is its hold latch and its at-rest bridge must yield to this release. Revert = False.
HOLD_DEPARTURE_RELEASE = True

# Cycle 2026-10-18 WS RELEASE A (cyc_1018/release/PROPOSAL.md; review1018 Q1 / Q5 M1). Behind a hard-braking lead the legacy planner sits
# on its -2.25 cap; the service enters below 1.6 m/s with the wire near -2.0 and < 1 m to its anchor (22d4_4492.94, 22a6_1079.82, the two
# harshest of 61 stops). The law and the landing release, but the upward jerk limit J_UP 1.5 holds the wire 0.43-0.55 below every lane for
# the last 0.8-0.9 s and the wheel stop lands at -1.0..-1.3: the limiter's lag, not a lane's demand. True = in APPROACH_GLIDE (terminal
# descent and landing ease included) below V_REL, behind a lead the ENTRY stopped-lead latch confirms (live gap, identity >=
# ATTR_TRUST_IN_S), a wire deeper than the limiter's target (the min of the law and every lane) by more than D_REL rises at J_UP_FAST until
# it is within D_REL_OFF of the target or the target drops below it; once the wire has met its target the approach has no inherited depth
# left and J_UP applies to the end (one fast release per approach: a lane that deepens and releases later, the judges' noise / flap / move
# pumps, keeps today's rate). Only the upward rate changes: the target, the lanes, J_DOWN and J_SAFE are untouched, so the wire only
# reaches what the lanes already allow, sooner. Revert = False.
FAST_RELEASE_TO_LANES = False
V_REL = 1.6        # m/s; equal to the late in-band entry bound (ServiceParams.EC_V_MAX) by intent, kept separate so a change to the cap
                   # does not move the release scope
D_REL = 0.30       # m/s^2 the wire is deeper than the target when the fast release starts (no other wheel stop of 61 exceeds 0.20)
D_REL_OFF = 0.05   # m/s^2 band that ends it (hysteresis)
J_UP_FAST = 2.5    # m/s^3 the upward rate while it runs (J_UP 1.5 otherwise); = the runner's H6 release ceiling J_MAX (3.0 failed H6)

# Cycle 2026-10-18 WS RELEASE B (cyc_1018/release/PROPOSAL.md; review1018 Q1 side finding). At 2.4-3.2 m/s under a deep plan the arbiter's
# stop-target distance sits at the stop-target-mode activation limit, which has no hold hysteresis above 2.5 m/s: one frame without it ends
# 'stopping', CP.startingState enters 'starting' and the next frame is 'pid' (vEgo > vEgoStarting). That one frame sent startAccel +0.70:
# 53 such frames at >= 1.2 m/s on the cached drives 2226..22d4, 19 of them in a 50 Hz SCC12 send to the car. Below 1.2 m/s the low-speed
# output slew bounds the frame. True = a stopping -> starting frame at vEgo >= V_SLEW_MAX holds the previous wire; the state machine and
# every launch from rest are unchanged. Revert = False.
STARTING_BLIP_GUARD = False

# Cycle 2026-10-17 WS LAUNCH-C (cyc_1017/launchc; phase A = cyc_1013/launch). A clean car launch spends p50 0.60 s in the service's
# RELEASE ramp -0.70 -> 0 at J_GO under StopReq, then p50 0.30 s more in the arbiter's close-stopped-lead latch. True: inside an at-rest
# RELEASE, the first frame with the lead's earned Doppler departure (lv - v > the pull bar), a live gap, no dropout, the planner's go
# held RELEASE_STEP_GO_S without a stop request, a radar identity (never a vision-only lead), and a lead neither braking nor confirmed
# stopped (or measurably creeping away) steps the command to the release level (phase A). LongControl then counts the decision as the
# arbiter's departing-lead release while the same lead keeps moving (> LEAD_STOPPED_V_MAX), does not brake, still pulls away (LF bound >
# 0), the service stays out and the car has not launched (not pid): 'stopping' (StopReq) ends on the next frame, the planner's
# standstill shouldStop does not hold the car (as today's departing-lead release), and the service's departure mask
# (HOLD_DEPARTURE_RELEASE) keeps the planner's stop from re-entering it while the model's stop bit is clear. Losing the release while
# still 'starting' aborts into the service's hold; that hold waits for a re-earned departure only until it is secure or the lead is lost
# (then today's HOLD exits). Once the car rolls (above the step's at-rest bar) with the hand-over's departing-lead room, a lead that only
# falls under the bound's margin while it still moves off without braking keeps the release (no abort: the bound hands over and
# today's 'starting' / pid command goes on; fix round 2, 22d4). From the step until the hand-over, the
# command never exceeds the lead's pull-away
#   L = max(aLeadK, 0) + (vLead - vEgo - LF_V_MARGIN) - LF_LAG_S * |aLeadK|   (m/s^2; the velocity gain is 1 /s)
# applied as the FINAL deepen-only ceiling after the low-speed slew and every ordinary writer; the ceiling follows L down at once and
# rises at J_GO at most. The hand-over (the bound ends) needs the departing-lead room at the closing speed (the arbiter's moving table)
# AND a caught-up ego (vLead - vEgo <= LF_CAUGHT_UP_MPS); a replacement lead, a renewed rest in 'stopping' or a lead absent longer than
# the dropout window (T_DROPOUT_S) ends it too; an ended or lead-less bound relaxes at J_GO until it no longer binds. The driver's
# pedals, a disengage, a fault or force coast end it at once. Every other release keeps today's ramp and arbiter; every safety lane
# keeps its authority and rate. Revert = False.
LAUNCH_FOLLOW = True   # 2026-10-18 car trial (host decision; drive_report LAUNCH L1-L6 per drive; revert = False)
LF_V_MARGIN = 0.3        # m/s the ego stays below the lead's speed while bounded (covers 0.15 s * 1.5 m/s^2 of unseen own accel)
LF_LAG_S = 0.2           # s of radar lag (20 Hz sample-and-hold + Kalman): the lead's speed may have drifted this long at |aLeadK|
LF_CAUGHT_UP_MPS = 0.35  # m/s closing speed below which the ego has caught up with the departing lead (the hand-over)
