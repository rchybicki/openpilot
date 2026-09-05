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
ATTRIBUTED_SAFETY = "shadow"        # "off" | "shadow" | "live" (2026-09-02/05, program step "attributed safety"): inside governor
                                    # ownership the SHADOW computes the candidate wire with the planner's comfort
                                    # output removed from the min (only attributed safety lanes may deepen: a_kin,
                                    # a_bar, a_mon, a_pred) and logs how often / how deep a_plan binds. Telemetry
                                    # only; the wire is untouched. "live" (2026-09-05, red-team 20260905-160050):
                                    # on eligible frames after 0.30 s of continuous eligibility the target becomes
                                    # min(candidate, last_cmd + 0.8 m/s^3 * dt) -- a_plan's excess is RELEASED at
                                    # <= 0.8 m/s^3; any ineligible frame re-admits a_plan at once. Revert = "shadow".
WHOLE_APPROACH_GOVERNOR = "off"  # "off" | "shadow" only; OFF until the recorded-input gate passes (cycle 42)
IDENTIFICATION_HOOK = False   # TEMPORARY identification-drive step hook (identification_hook.py, protocol v2). Master
                              # kill: False = nothing is constructed. True only for the collection deploy; the hook
                              # still needs /data/identification_hook.arm at process start and NOTHING on both
                              # distance long-press mappings. Deleted with the module after the drive.

