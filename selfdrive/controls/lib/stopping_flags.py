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

# KILL SWITCH: False restores the legacy radard publish-side dRel mutation
# (published dRel = true dRel - increasedStoppedDistance) and the exactly-cancelling
# compensation terms in long_mpc / stop_target_helpers / the planner Santa Fe caps.
# True publishes the TRUE lead distance and re-expresses ISD as the single
# "rest ISD meters farther back" term (§4.2.3). Flipped 2026-06-10 after the §4.2.5
# device read: IncreasedStoppedDistance == 0.0 on the target car (all weather variants 0),
# so the flip is bit-identical today; later ISD raises are compensated by design
# (single-point lead_d_rel_eff, rest-gap equality pinned for ISD in {0, 1.5, 3.0}).
PUBLISH_TRUE_LEAD_DISTANCE = True
