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
