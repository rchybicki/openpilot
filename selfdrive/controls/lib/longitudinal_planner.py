#!/usr/bin/env python3
import math
import numpy as np

from cereal import log
import cereal.messaging as messaging
from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, update_should_stop_falling_edge_hold
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.lead_provenance import get_radar_only_min_acquire_d_rel
from openpilot.selfdrive.controls.lib.stop_context import StopContext
from openpilot.selfdrive.controls.lib.lead_provenance import StoppingLeadAuthority
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LEFTMOST_HIGHWAY_LEAD_EASING_SCALE, LongitudinalMpc, SOURCES
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import (
  LEAD_STOP_DISTANCE_TARGET,
  get_effective_lead_distance,
  get_published_lead_distance_compensation,
  get_stopped_lead_control_target,
)
from openpilot.selfdrive.controls.lib.stop_target_arbiter import should_enter_stop_target_mode, should_hold_stop_target_mode
from openpilot.selfdrive.modeld.constants import ModelConstants

from openpilot.frogpilot.common.frogpilot_utilities import has_adjacent_lane
from openpilot.frogpilot.common.frogpilot_variables import MINIMUM_LATERAL_ACCELERATION
from openpilot.frogpilot.controls.lib.force_coast import get_force_coast_target_from_toggles

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MAX_VALS = [2.0, 1.6, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5
EXPERIMENTAL_FREE_ROAD_LEAD_TIME = 1.4
EXPERIMENTAL_FREE_ROAD_LEAD_TIME_BP = [0.0, 15.0 * CV.KPH_TO_MS, 30.0 * CV.KPH_TO_MS, 50.0 * CV.KPH_TO_MS]
EXPERIMENTAL_FREE_ROAD_LEAD_TIME_VALS = [2.5, 2.3, 1.9, EXPERIMENTAL_FREE_ROAD_LEAD_TIME]
EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_MAX = 1.1
EXPERIMENTAL_FREE_ROAD_NO_LEAD_BOOST_MAX = 0.6
EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_GAIN_DEFAULT = 1.0
EXPERIMENTAL_FREE_ROAD_NO_LEAD_BOOST_GAIN_DEFAULT = 0.5
EXPERIMENTAL_FREE_ROAD_BRAKE_CUTOFF_DEFAULT = -0.2
EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_SCALE = 0.9
EXPERIMENTAL_FREE_ROAD_NO_LEAD_BOOST_SCALE = 1.7
EXPERIMENTAL_FREE_ROAD_NO_LEAD_SPEED_GATE_BP = [0.0, 0.5, 2.0]
EXPERIMENTAL_FREE_ROAD_NO_LEAD_SPEED_GATE_VALS = [0.0, 0.4, 1.0]
EXPERIMENTAL_FREE_ROAD_NO_LEAD_SPEED_GATE_STRENGTH = 1.5
EXPERIMENTAL_FREE_ROAD_NO_LEAD_MODEL_GATE_STRENGTH = 1.2
EXPERIMENTAL_FREE_ROAD_NATIVE_ACCEL_GATE_BP = [0.2, 0.6]
EXPERIMENTAL_FREE_ROAD_NATIVE_ACCEL_GATE_VALS = [1.0, 0.0]
EXPERIMENTAL_FREE_ROAD_LEAD_SPEED_GATE_BP = [0.0, 5.0 * CV.KPH_TO_MS, 10.0 * CV.KPH_TO_MS, 20.0 * CV.KPH_TO_MS, 35.0 * CV.KPH_TO_MS, 50.0 * CV.KPH_TO_MS]
EXPERIMENTAL_FREE_ROAD_LEAD_SPEED_GATE_VALS = [0.25, 0.3, 0.4, 0.55, 0.8, 1.0]
EXPERIMENTAL_FREE_ROAD_LEAD_STANDSTILL_GAP_BP = [0.0, 15.0 * CV.KPH_TO_MS, 30.0 * CV.KPH_TO_MS, 50.0 * CV.KPH_TO_MS]
EXPERIMENTAL_FREE_ROAD_LEAD_STANDSTILL_GAP_VALS = [4.0, 4.0, 2.0, 0.0]
EXPERIMENTAL_FREE_ROAD_LEAD_GAP_MARGIN_BP = [0.0, 1.0, 2.0, 4.0]
EXPERIMENTAL_FREE_ROAD_LEAD_GAP_MARGIN_VALS = [0.0, 0.55, 0.8, 1.0]
EXPERIMENTAL_FREE_ROAD_STANDARD_LEAD_GAP_SCALE = 0.7
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_BP = [0.0, 0.5, 1.5, 3.0]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_VALS = [0.0, 0.2, 0.6, 1.0]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_ACCEL_BP = [-0.2, 0.0, 0.3, 1.0]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_ACCEL_VALS = [0.0, 0.2, 0.5, 1.0]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_INFLUENCE_BP = [0.0, 10.0 * CV.KPH_TO_MS, 30.0 * CV.KPH_TO_MS, 50.0 * CV.KPH_TO_MS]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_INFLUENCE_VALS = [1.0, 1.0, 0.5, 0.0]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_GATE_STRENGTH = 0.5
EXPERIMENTAL_FREE_ROAD_BOOST_RAMP_UP = 0.05
EXPERIMENTAL_FREE_ROAD_BOOST_RAMP_DOWN = 0.08
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_MAX = 0.45
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_MAX_SPEED = 12.5
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_GAP_BP = [0.8, 1.1, 1.8, 2.4, 3.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_GAP_VALS = [1.0, 1.0, 0.75, 0.2, 0.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_BP = [4.5, 6.0, 8.0, 12.5]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_VALS = [0.35, 0.55, 0.85, 1.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_REQUEST_BP = [-3.0, -2.2, -1.5, -0.6, 0.2, 0.6]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_REQUEST_VALS = [0.35, 0.5, 0.75, 1.0, 0.55, 0.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_CLOSING_BP = [0.2, 0.8, 2.0, 4.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_CLOSING_VALS = [0.0, 0.35, 0.8, 1.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_TTC_BP = [1.0, 1.8, 2.6, 3.6, 5.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_TTC_VALS = [1.0, 1.0, 0.7, 0.35, 0.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_LEAD_SPEED_BP = [0.0, 0.4, 0.7, 1.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_LEAD_SPEED_VALS = [1.0, 1.0, 0.25, 0.0]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_MAX_SPEED = 16.0
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_GAP_BP = [1.55, 2.10, 2.70, 3.50]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_GAP_CAPS = [-0.72, -0.46, -0.18, 0.05]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_CLOSING_BP = [1.20, 2.00, 3.50, 5.00]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_CLOSING_TIGHTEN = [0.00, 0.04, 0.12, 0.18]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_DECEL_BP = [0.00, 0.40, 0.90, 1.50]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_DECEL_TIGHTEN = [0.00, 0.00, 0.05, 0.11]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_TTC_BP = [2.50, 4.00, 7.00, 10.00]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_TTC_TIGHTEN = [0.13, 0.08, 0.02, 0.00]
# 00001f70 seg46: sustained aLeadK was usable before the 1.20 m/s closing gate. A 1.20 gain
# modestly covers response lag while the -1.0 bound keeps this an early comfort cap.
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_GAIN = 1.20
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_MAX_DECEL = 1.0
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_MIN_LEAD_DECEL = 0.30
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_MIN_LEAD_SPEED = 2.0
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_MIN_MODEL_PROB = 0.5
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_SLEW_DOWN = 2.5   # m/s^3: end-review round 1 (medium) --
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_SLEW_UP = 1.5     # window eviction stepped the raw target
                                                               # ~0.45 in one frame and ineligibility
                                                               # released it instantly; the APPLIED
                                                               # authority is rate-bounded both ways
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_PERSIST_FRAMES = 6  # 0.3 s at 20 Hz on the SAME radar track
SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_SPEED_BP = [2.50, 5.00, 8.00, 12.50]
SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_MAX_DECEL = [1.05, 1.55, 2.05, 2.35]
SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_BUFFER_M = [0.35, 0.75, 1.15, 1.65]
SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_MIN_CLOSING = [0.55, 0.95, 1.45, 2.20]
SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_MIN_MEANINGFUL_DECEL = [0.55, 0.75, 1.00, 1.20]
# 2026-07-29 bookmarked harsh stop 00001f65 seg13 (t~3953, openpilot-completed, no takeover): the
# late path held a flat ~-1.5 constant-decel profile the whole approach (engaged marginally at
# 55.2 m vs its 55.5 m range at v=12.0) and arrived at 10 m still doing 4.2 m/s, forcing the
# stopping service to carry ~-1.5 into wheel-stop. Mid-band buffers raised (+0.6..+1.0) and the
# 11.5 m/s range kink filled (46->52) so the same approach sheds ~+0.1 m/s^2 more through the
# mid phase and arrives at 10 m around ~3 m/s -- shift the braking EARLIER, not deeper overall.
# Stopped-lead gates unchanged: this table still cannot touch moving-lead following.
SANTA_FE_STOPPED_LEAD_LATE_APPROACH_SPEED_BP = [6.00, 8.00, 10.00, 11.50, 12.50, 14.50, 16.00]
SANTA_FE_STOPPED_LEAD_LATE_APPROACH_BUFFER_M = [1.40, 3.00, 4.20, 5.00, 7.00, 11.00, 13.00]
SANTA_FE_STOPPED_LEAD_LATE_APPROACH_MAX_DECEL = [2.15, 2.55, 3.00, 3.10, 3.25, 3.25, 3.25]
SANTA_FE_STOPPED_LEAD_LATE_APPROACH_MIN_CLOSING = [4.50, 5.50, 6.50, 7.00, 7.50, 8.50, 9.50]
SANTA_FE_STOPPED_LEAD_LATE_APPROACH_MAX_TTC = [4.20, 4.50, 4.80, 5.00, 5.50, 6.40, 6.40]
SANTA_FE_STOPPED_LEAD_LATE_APPROACH_MAX_D_REL = [24.00, 32.00, 42.00, 52.00, 65.00, 100.00, 112.00]
# Firmness on the late-path required decel only (constant-decel geometry spends a buffer bump too
# thinly at 40+ m braking distances to be felt). 1.06 shifts ~0.1 m/s^2 into the mid phase of the
# 00001f65 approach -> arrives at 10 m roughly 0.6-0.8 m/s slower, so the terminal machinery
# inherits about half the kinetic energy. The single knob to crank if stops still arrive hot.
SANTA_FE_STOPPED_LEAD_LATE_APPROACH_FIRMNESS = 1.06
# Creep-to-stop extension (kill switch: stopping_flags.SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_EXTENSION).
# Extends the band down to a 0.55 m/s floor (the upper edge of the V2 terminal-hold / far-release /
# standstill-creep regime, which owns v_ego < 0.55) so the cap carries a confirmed stopping/
# creeping-then-stopping lead under a GENTLE controlled approach toward the 4.0 m hold gap, then
# hands off to low_speed_stopped_lead_glide_accel_cap (active 0.02-1.25 m/s, dToStop-aware) which
# finishes the brake. The anchor at 2.50 and a coincident 2.49 anchor make every interp value at
# and above 2.50 m/s byte-identical to the base tables, so the >= 2.50 m/s behavior is unchanged;
# only the new 0.55-2.50 band is added, with a deliberately gentle min-meaningful floor so the
# small-but-correct approach decel that lands the car at 4.0 m can pass instead of being gated out.
SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_SPEED_BP = [0.55, 1.60, 2.49, 2.50, 5.00, 8.00, 12.50]
SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_MAX_DECEL = [0.40, 0.75, 1.04, 1.05, 1.55, 2.05, 2.35]
SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_BUFFER_M = [0.28, 0.32, 0.35, 0.35, 0.75, 1.15, 1.65]
SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_MIN_CLOSING = [0.08, 0.30, 0.55, 0.55, 0.95, 1.45, 2.20]
SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_MIN_MEANINGFUL_DECEL = [0.05, 0.08, 0.10, 0.55, 0.75, 1.00, 1.20]
# santa_fe_stopping_lead_roll_in (kill switch: stopping_flags.SANTA_FE_STOPPING_LEAD_ROLL_IN).
# The MIRROR of the stopped-lead smooth-approach cap: a roll-in FLOOR (max()/RAISE) = "do not
# brake HARDER than needed to stop at the hold gap". It RAISES the MPC over-brake up to the same
# gentle stop-at-hold-gap decel the cap deepens DOWN to (both share
# get_santa_fe_stopped_lead_hold_gap_required_decel), so the command is clamped to exactly the
# decel that lands the car at the 4.0 m hold gap and rolls in continuously instead of
# near-stopping far back. Narrower, more conservative gates than the cap because raising brake
# is never strictly safe.
SANTA_FE_STOPPING_LEAD_ROLL_IN_V_EGO_MIN = 0.30          # below this -> low-speed glide owns the brake; floor off
SANTA_FE_STOPPING_LEAD_ROLL_IN_V_EGO_MAX = 2.50          # above this -> normal approach band; floor off
SANTA_FE_STOPPING_LEAD_ROLL_IN_LEAD_V_MAX = 0.55         # confirmed stopped/creeping lead only (upper creep edge)
SANTA_FE_STOPPING_LEAD_ROLL_IN_MIN_CLOSING = 0.20        # need a real closing approach to roll in
SANTA_FE_STOPPING_LEAD_ROLL_IN_MAX_CLOSING = 2.30        # closing-speed ceiling: a fast closure is not a gentle roll-in
SANTA_FE_STOPPING_LEAD_ROLL_IN_MIN_TTC_S = 4.0           # TTC floor: never raise brake when impact is < 4.0 s away
SANTA_FE_STOPPING_LEAD_ROLL_IN_GATE_OFF_MARGIN_M = 0.40  # gate off once remaining-to-hold-gap <= margin (hand off the finish)
SANTA_FE_STOPPING_LEAD_ROLL_IN_LEAD_DECEL_LATCH = 0.45   # lead hard-decel (m/s^2) that latches the floor OFF (sudden stop)
SANTA_FE_STOPPING_LEAD_ROLL_IN_LATCH_DWELL_S = 0.8       # hold the floor OFF this long after a latch trigger
SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_SPEED_BP = [2.50, 5.00, 8.00, 12.50, 15.00]
SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MIN_LEAD_DECEL = 0.75
SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_PROJECT_TIME = 2.0
SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MAX_STOP_TIME = [5.0, 6.5, 8.4, 10.0, 11.0]
SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MAX_PROJECTED_TTC = [4.5, 5.5, 7.0, 8.5, 8.5]
SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MIN_DECEL = [0.35, 0.50, 0.65, 0.80, 0.90]
SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_BUFFER_M = [0.35, 0.75, 1.15, 1.65, 2.00]
SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MAX_DECEL = [1.05, 1.55, 2.05, 2.35, 2.45]
SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MIN_CLOSING = [0.55, 0.95, 1.45, 2.20, 2.80]
SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_CONFIDENCE_MARGIN = [0.0, 1.0, 2.0]
SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_CONFIDENCE_VALS = [0.65, 0.85, 1.0]
SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_MAX_STOP_TIME = 4.2
SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_TTC_BP = [2.6, 3.6, 5.2, 6.4]
SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_TTC_DECEL = [0.34, 0.28, 0.12, 0.0]
SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_CLOSING_BP = [3.0, 4.5, 6.5, 8.5]
SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_CLOSING_DECEL = [0.0, 0.06, 0.16, 0.22]
SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_STOP_TIME_BP = [2.0, 3.0, SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_MAX_STOP_TIME]
SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_STOP_TIME_VALS = [1.0, 0.78, 0.15]
SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_MAX_DECEL = 0.42
SANTA_FE_DOWNHILL_QUEUE_COAST_ACCEL_MIN = -0.12
SANTA_FE_DOWNHILL_QUEUE_RELAX_CLIP_STEP = 0.12
SANTA_FE_DOWNHILL_STOPPED_LEAD_HIGH_SPEED_BP = [12.50, 16.00, 18.50]
SANTA_FE_DOWNHILL_STOPPED_LEAD_BUFFER_M = [1.65, 2.05, 2.35]
SANTA_FE_DOWNHILL_STOPPED_LEAD_MAX_DECEL = [2.35, 2.45, 2.55]
SANTA_FE_DOWNHILL_STOPPED_LEAD_MIN_DECEL = [1.20, 1.35, 1.45]
SANTA_FE_DOWNHILL_STOPPED_LEAD_MIN_CLOSING = [2.20, 2.75, 3.20]
SANTA_FE_DOWNHILL_STOPPED_LEAD_MAX_TTC = [7.00, 6.50, 6.00]
SANTA_FE_DOWNHILL_STOPPED_LEAD_MAX_D_REL = 110.0
# Stop-commitment necessity floor (kill switch: stopping_flags.SANTA_FE_STOP_COMMIT_ENVELOPE).
# Route 00001f47 seg6 (t~6331, driver brake takeover): approaching a lead that braked hard to a
# stop, the planner command RELAXED -1.93 -> -1.47 exactly as the decel required to rest at the
# 3.0 m floor blew through 2.2 -> 3.5 m/s^2; the car would have coasted to ~1.7 m. No writer in
# the stack escalates when kinematic necessity exceeds the comfort tables. This lane is the
# policy backstop: DEEPEN-ONLY min() to the decel that rests the car at the 3.0 m band floor
# (NOT the 4.0 m comfort target, so ordinary approaches that would land at 3.2-4.5 m never
# trigger it). Corpus-scanned before deploy (4.8 h engaged): with these gates it fires ~0.8/h,
# every fire a genuine below-floor terminal; the v_ego ceiling excludes highway braking-waves where a
# stop-behind-the-lead's-stop-point plan is the wrong frame (leads release long before resting).
SANTA_FE_STOP_COMMIT_REST_FLOOR_M = 3.0
SANTA_FE_STOP_COMMIT_LEAD_DECEL_MIN = 0.75   # below this a moving lead is not "braking to a stop"
SANTA_FE_STOP_COMMIT_LEAD_STOPPED_V = 0.5
SANTA_FE_STOP_COMMIT_MIN_BRAKE_DIST_M = 0.5
SANTA_FE_STOP_COMMIT_A_REQ_MIN = 1.5
SANTA_FE_STOP_COMMIT_ACTIVATE_MARGIN = 0.30  # Schmitt: engage only when required decel clearly exceeds the command
SANTA_FE_STOP_COMMIT_RELEASE_MARGIN = 0.10   # ... and release at a lower threshold so the floor cannot chatter
SANTA_FE_STOP_COMMIT_A_MAX = 3.25            # matches the deepest existing table authority (late-approach), below ACCEL_MIN
SANTA_FE_STOP_COMMIT_MAX_D_REL = 60.0
SANTA_FE_STOP_COMMIT_V_EGO_MIN = 0.5
SANTA_FE_STOP_COMMIT_V_EGO_MAX = 16.5
SANTA_FE_STOP_COMMIT_PERSIST_FRAMES = 10     # 0.5 s at 20 Hz on the SAME lead track (radar-glitch immunity, 00001b97 t~3926)
SANTA_FE_STOP_COMMIT_ALK_WINDOW = 6          # least-severe lead decel over 0.3 s: one aLeadK spike cannot shorten the lead's projected stop
SANTA_FE_STOP_COMMIT_ACTUATION_DELAY_S = 0.2  # necessity is computed on the gap after a response delay, not the instantaneous gap
SANTA_FE_STOP_COMMIT_VISION_PROB_MIN = 0.9   # vision-only leads share one sentinel track id; require high confidence on every frame
SANTA_FE_STOP_COMMIT_RADAR_CONFLICT_PROB_MIN = 0.9
SANTA_FE_STOP_COMMIT_RADAR_CONFLICT_GAP_M = 2.0

# Aim-commitment necessity floor (kill switch: stopping_flags.SANTA_FE_STOP_AIM_ENVELOPE).
# Route 00001f90 seg22 (bookmarked, first cycle-22 on-road data): approaching a lead that braked
# hard to a stop, the command EASED -2.06 -> -1.13 while constant-decel-to-rest-at-the-AIM
# (LEAD_STOP_DISTANCE_TARGET + increasedStoppedDistance = 4.3 true metres) required ~1.5
# sustained; the deficit was repaid at v~1 by the stopping service's floor lanes (wire -1.40 ->
# -2.46 in 0.75 s, felt jerk 5.29, rest 3.05). The cycle-13 floor above defends only the 3.0 m
# BAND FLOOR and correctly stayed out. This lane is the AIM's defender: once a stop is COMMITTED
# -- required decel to the aim in [ON, CAP] at onset, projected lead stop point within 35 m, the
# same certified-lead eligibility as the floor lane -- the command may not fall below the
# necessity, continuously (min(), no Schmitt-vs-command: measured on the recorded bookmark, a
# +0.30 command-relative margin left a ONE-FRAME engagement window before necessity blew through
# the cap). Onset REFUSES when a_req already exceeds CAP (a late-hot approach belongs to the
# floor/late-approach lanes; committing there steps the target 0 -> -2.25 in one frame, measured
# on 00001f90 seg21's earlier episode). The 35 m projected-stop gate excludes highway braking
# waves (leads projected to stop 80+ m out that release long before resting, f85 seg12 /
# f86 seg41). Corpus scan (3.37 h engaged, 304 segments): ~1.2 substantive episodes/h, every one
# a genuine hot approach of the bookmarked class. Below SANTA_FE_STOP_AIM_V_EGO_MIN the
# StoppingService owns the stop; this lane is the approach-band layer above it.
SANTA_FE_STOP_AIM_ON = 1.3           # commit onset: required decel to the aim reaches this...
SANTA_FE_STOP_AIM_CAP = 2.25         # ...but has not exceeded the comfort cap
SANTA_FE_STOP_AIM_OFF = 1.0          # release: requirement decayed (executed or lead moved off)
SANTA_FE_STOP_AIM_STOP_WITHIN_M = 35.0  # projected lead stop point: commit only to IMMINENT stops
SANTA_FE_STOP_AIM_V_EGO_MIN = 2.0    # below this the StoppingService owns the stop (V_ENTER 2.5)
SANTA_FE_STOP_AIM_RESLAM_V_MIN = 0.8   # cycle-28 (fd1 s4, felt 2.98, rest 2.8 = floor breach): a
SANTA_FE_STOP_AIM_RESLAM_LEAD_V_MIN = 0.3   # mid-queue re-slam -- the lead launches, ego follows
SANTA_FE_STOP_AIM_RESLAM_ALK_MAX = -0.5
SANTA_FE_STOP_AIM_RESLAM_EXIT_V = 0.3  # committed rides down to here (HOLD/RAMP owns below)     # at 1.7 m/s and 4.4 m, the lead slams back to zero.
                                     # In 0.8-2.0 m/s behind a MOVING lead nobody commits: the
                                     # aim lane's floor is 2.0, the service refuses moving leads
                                     # (cycle-25, correctly), the MPC ramps late. The lane now
                                     # extends down to 0.8 ONLY while the lead is decisively
                                     # decelerating yet still moving (windowed least-severe alk
                                     # <= -0.5 AND lv >= 0.3): launching/steady leads keep the
                                     # 2.0 floor -- the f85 s4 launch-fight exclusion stands.
SANTA_FE_STOP_AIM_ACTUATION_DELAY_S = 0.25  # plan red-team: the service MEASURED 0.25 s hydraulic
                                            # lag; the floor lane's 0.20 was 0.28 m optimistic at
                                            # 5.6 m/s. The floor lane keeps its own constant.
SANTA_FE_STOP_AIM_ROLLBACK_WINDOW = 10   # 0.5 s at 20 Hz: longer than the 0.36 s recorded
                                         # stopped-lead Doppler noise bursts (cycle-15 census)
SANTA_FE_STOP_AIM_ROLLBACK_HORIZON_S = 2.0  # cycle-27 (fc2 s6, felt 2.24): a lead ROLLING BACK
                                            # (-0.1..-0.2 m/s sustained, ~1 m over the approach)
                                            # recedes the stop point; the clamped-at-zero
                                            # necessity under-commits and the floor defence pays
                                            # at the end (-1.40 at gap 3.3). Project the rollback
                                            # over this horizon into the runway -- deepen-only.

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)

  if abs(a_y) > MINIMUM_LATERAL_ACCELERATION:
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
  else:
    a_x_allowed = a_target[1]

  return [a_target[0], min(a_target[1], a_x_allowed)]


def rate_limit_value(current_value, target_value, up_step, down_step):
  if target_value > current_value:
    return min(target_value, current_value + up_step)
  return max(target_value, current_value - down_step)


def get_experimental_boosted_accel(experimental_base_accel, acc_reference_accel, boost):
  boosted_accel = experimental_base_accel + max(boost, 0.0)

  # Never let the added boost pull Experimental below its own native request.
  # The ACC reference only caps the extra accel we added on top.
  return min(boosted_accel, max(experimental_base_accel, acc_reference_accel))


def apply_experimental_force_coast_cap(output_a_target, acc_reference_accel, force_coast):
  if not force_coast:
    return output_a_target

  return min(output_a_target, acc_reference_accel)


def should_allow_force_coast_stronger_lead_brake(v_ego, lead, output_should_stop):
  # output_should_stop is kept in the signature for call-site/test stability but is intentionally no
  # longer read: output_should_stop alone is NOT a license to brake harder than the gentle force-coast
  # target.
  # a distant, slow lead while we are ~stopped sets should_stop yet needs no hard brake. On route
  # 00001756 a 12.3 m lead closing at only 0.28 m/s while v_ego~0.05 drove a -1.70 m/s2 spike through
  # the old `if output_should_stop: return True` bypass (the harsh no-lead stop the driver bookmarked).
  # Allow the stronger lead-brake ONLY when the lead is genuinely close or closing -- this is NEVER
  # lead-blind, so a real close/closing lead always keeps full MPC brake authority (the P1
  # no-under-braking invariant); distant-slow and lead-free stops fall back to the force-coast cap.
  if not lead.status:
    return False

  d_rel = float(lead.dRel)
  if d_rel <= 0.0:
    return False

  v_rel = float(lead.vRel)
  v_lead = max(float(getattr(lead, "vLead", v_ego + v_rel)), 0.0)
  closing_speed = max(-v_rel, 0.0)
  time_gap = d_rel / max(v_ego, 1.0)
  ttc = d_rel / max(closing_speed, 0.1) if closing_speed > 0.1 else float("inf")

  stopped_lead_close = v_lead < 0.5 and d_rel < max(6.0, v_ego * 1.2)
  urgent_closing_lead = ttc < 4.0 and time_gap < 2.5
  return time_gap < 2.0 or urgent_closing_lead or stopped_lead_close


def apply_force_coast_strength_brake_limit(output_a_target, force_coast_target_accel, force_coast, v_ego, lead, output_should_stop, model_accel):
  if not force_coast or output_a_target >= force_coast_target_accel:
    return output_a_target
  if should_allow_force_coast_stronger_lead_brake(v_ego, lead, output_should_stop):
    return output_a_target

  brake_limit = force_coast_target_accel
  if model_accel is not None:
    brake_limit = min(brake_limit, float(model_accel))
  return max(output_a_target, brake_limit)


def get_active_long_distance_factor(lane_width_left, frogpilot_toggles):
  if has_adjacent_lane(lane_width_left, getattr(frogpilot_toggles, "lane_detection_width", 0.0)):
    return frogpilot_toggles.long_distance_factor
  return frogpilot_toggles.long_distance_factor * LEFTMOST_HIGHWAY_LEAD_EASING_SCALE


def get_experimental_free_road_boost_limits(lead, lead_boost_gain, no_lead_boost_gain):
  if lead.status:
    return EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_MAX, EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_SCALE, max(lead_boost_gain, 0.0)

  return EXPERIMENTAL_FREE_ROAD_NO_LEAD_BOOST_MAX, EXPERIMENTAL_FREE_ROAD_NO_LEAD_BOOST_SCALE, max(no_lead_boost_gain, 0.0)


def get_experimental_free_road_model_gate(e2e_accel, brake_cutoff):
  zero_boost_point = min(float(brake_cutoff), -0.02)
  mild_brake_point = zero_boost_point * 0.5
  coast_point = min(max(zero_boost_point * 0.1, -0.02), 0.0)

  return float(np.interp(e2e_accel, [zero_boost_point, mild_brake_point, coast_point, 0.2], [0.0, 0.25, 0.6, 1.0]))


def get_experimental_free_road_native_accel_gate(experimental_base_accel):
  return float(np.interp(experimental_base_accel, EXPERIMENTAL_FREE_ROAD_NATIVE_ACCEL_GATE_BP,
                         EXPERIMENTAL_FREE_ROAD_NATIVE_ACCEL_GATE_VALS))


def get_experimental_free_road_lead_time_threshold(v_ego):
  return float(np.interp(v_ego, EXPERIMENTAL_FREE_ROAD_LEAD_TIME_BP, EXPERIMENTAL_FREE_ROAD_LEAD_TIME_VALS))


def get_experimental_free_road_lead_speed_gate(v_ego):
  return float(np.interp(v_ego, EXPERIMENTAL_FREE_ROAD_LEAD_SPEED_GATE_BP, EXPERIMENTAL_FREE_ROAD_LEAD_SPEED_GATE_VALS))


def get_experimental_free_road_no_lead_speed_gate(speed_error):
  raw_gate = float(np.interp(speed_error, EXPERIMENTAL_FREE_ROAD_NO_LEAD_SPEED_GATE_BP,
                             EXPERIMENTAL_FREE_ROAD_NO_LEAD_SPEED_GATE_VALS))
  return min(1.0, raw_gate * EXPERIMENTAL_FREE_ROAD_NO_LEAD_SPEED_GATE_STRENGTH)


def get_experimental_free_road_lead_gap_gate(lead, v_ego, personality=log.LongitudinalPersonality.relaxed):
  standstill_gap = float(np.interp(v_ego, EXPERIMENTAL_FREE_ROAD_LEAD_STANDSTILL_GAP_BP,
                                   EXPERIMENTAL_FREE_ROAD_LEAD_STANDSTILL_GAP_VALS))
  desired_gap = standstill_gap + (v_ego * get_experimental_free_road_lead_time_threshold(v_ego))
  gap_scale = EXPERIMENTAL_FREE_ROAD_STANDARD_LEAD_GAP_SCALE if personality == log.LongitudinalPersonality.standard else 1.0
  # Scale both the opening point and the fully-open point, preserving the shape
  # of the existing lead-gap gate for each personality.
  gap_margin = (float(lead.dRel) / gap_scale) - desired_gap
  return float(np.interp(gap_margin, EXPERIMENTAL_FREE_ROAD_LEAD_GAP_MARGIN_BP, EXPERIMENTAL_FREE_ROAD_LEAD_GAP_MARGIN_VALS))


def get_experimental_free_road_lead_pullaway_gate(lead, v_ego):
  relative_speed = max(float(lead.vLead) - float(v_ego), 0.0)
  relative_speed_gate = float(np.interp(relative_speed, EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_BP, EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_VALS))
  lead_accel_gate = float(np.interp(float(lead.aLeadK), EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_ACCEL_BP, EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_ACCEL_VALS))

  # Fade the pull-away gating out by 50 kph so higher-speed following keeps the
  # existing lead behavior, while stop-and-go becomes much less eager.
  speed_influence = float(np.interp(v_ego, EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_INFLUENCE_BP,
                                    EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_INFLUENCE_VALS))
  pullaway_gate = relative_speed_gate * lead_accel_gate
  gated_pullaway = (1.0 - speed_influence) + (speed_influence * pullaway_gate)
  return 1.0 - (EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_GATE_STRENGTH * (1.0 - gated_pullaway))


def experimental_free_road_boost_allowed(mode, allow_throttle, should_stop, force_coast, lead, v_ego,
                                         personality=log.LongitudinalPersonality.relaxed):
  if mode != 'blended' or not allow_throttle or should_stop or force_coast:
    return False

  if lead.status and get_experimental_free_road_lead_gap_gate(lead, v_ego, personality) <= 0.0:
    return False

  return True


def get_experimental_free_road_boost_target(mode, allow_throttle, should_stop, force_coast, lead, v_ego, v_cruise,
                                            experimental_base_accel, acc_reference_accel, e2e_accel, lead_boost_gain, no_lead_boost_gain,
                                            brake_cutoff=EXPERIMENTAL_FREE_ROAD_BRAKE_CUTOFF_DEFAULT,
                                            personality=log.LongitudinalPersonality.relaxed):
  if not experimental_free_road_boost_allowed(mode, allow_throttle, should_stop, force_coast, lead, v_ego, personality):
    return 0.0

  accel_gap = max(acc_reference_accel - experimental_base_accel, 0.0)
  speed_error = max(v_cruise - v_ego, 0.0)
  if accel_gap <= 0.0 or speed_error <= 0.0:
    return 0.0

  # Allow a soft pull toward ACC while fading out once the model clearly
  # asks for braking or native Experimental acceleration is already strong. When a lead is already beyond the allowed time gap,
  # trust the ACC reference directly instead of suppressing the assist just
  # because cruise error is small. At stop-and-go speeds, still taper lead
  # boost down to avoid jumping at a moving lead and then braking again.
  model_gate = get_experimental_free_road_model_gate(e2e_accel, brake_cutoff)
  if not lead.status:
    model_gate = min(1.0, model_gate * EXPERIMENTAL_FREE_ROAD_NO_LEAD_MODEL_GATE_STRENGTH)
  native_accel_gate = get_experimental_free_road_native_accel_gate(experimental_base_accel)
  if lead.status:
    speed_gate = (get_experimental_free_road_lead_speed_gate(v_ego) *
                  get_experimental_free_road_lead_gap_gate(lead, v_ego, personality) *
                  get_experimental_free_road_lead_pullaway_gate(lead, v_ego))
  else:
    speed_gate = get_experimental_free_road_no_lead_speed_gate(speed_error)
  boost_max, boost_scale, boost_gain = get_experimental_free_road_boost_limits(lead, lead_boost_gain, no_lead_boost_gain)
  boost_cap = min(boost_max, boost_gain * boost_scale * accel_gap)
  return min(accel_gap, boost_cap * model_gate * native_accel_gate * speed_gate)


def update_experimental_free_road_boost(current_boost, mode, allow_throttle, should_stop, force_coast, lead, v_ego, v_cruise,
                                        experimental_base_accel, acc_reference_accel, e2e_accel, lead_boost_gain, no_lead_boost_gain,
                                        brake_cutoff=EXPERIMENTAL_FREE_ROAD_BRAKE_CUTOFF_DEFAULT,
                                        personality=log.LongitudinalPersonality.relaxed):
  boost_target = get_experimental_free_road_boost_target(mode, allow_throttle, should_stop, force_coast, lead, v_ego, v_cruise,
                                                         experimental_base_accel, acc_reference_accel, e2e_accel, lead_boost_gain,
                                                         no_lead_boost_gain, brake_cutoff, personality)
  if boost_target <= 0.0:
    return 0.0
  return rate_limit_value(current_boost, boost_target, EXPERIMENTAL_FREE_ROAD_BOOST_RAMP_UP, EXPERIMENTAL_FREE_ROAD_BOOST_RAMP_DOWN)


def is_santa_fe_hev_2022(cp):
  return getattr(cp, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022


def get_santa_fe_experimental_lead_caution_decel(v_ego, lead, output_a_target):
  if v_ego < SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_BP[0] or v_ego > SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_MAX_SPEED:
    return 0.0
  if not lead.status:
    return 0.0

  d_rel = float(lead.dRel)
  v_rel = float(lead.vRel)
  v_lead = max(float(getattr(lead, "vLead", v_ego + v_rel)), 0.0)
  if d_rel <= 0.0:
    return 0.0

  time_gap = d_rel / max(v_ego, 1.0)
  closing_speed = max(-v_rel, 0.0)
  ttc = d_rel / max(closing_speed, 0.1) if closing_speed > 0.1 else float("inf")

  gap_factor = float(np.interp(time_gap, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_GAP_BP, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_GAP_VALS))
  if gap_factor <= 0.0:
    return 0.0

  speed_factor = float(np.interp(v_ego, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_BP, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_VALS))
  request_factor = float(np.interp(output_a_target, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_REQUEST_BP, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_REQUEST_VALS))
  lead_stopped_factor = float(np.interp(v_lead, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_LEAD_SPEED_BP, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_LEAD_SPEED_VALS))
  if speed_factor <= 0.0 or request_factor <= 0.0 or lead_stopped_factor <= 0.0:
    return 0.0

  closing_factor = float(np.interp(closing_speed, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_CLOSING_BP, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_CLOSING_VALS))
  ttc_factor = 0.0 if not math.isfinite(ttc) else float(np.interp(ttc, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_TTC_BP, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_TTC_VALS))

  risk_factor = speed_factor * request_factor * gap_factor * lead_stopped_factor * closing_factor * ttc_factor
  return float(np.clip(SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_MAX * risk_factor, 0.0, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_MAX))


def santa_fe_experimental_decelerating_lead_feedforward_state_ok(v_ego, lead):
  if v_ego < SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_BP[0] or v_ego > SANTA_FE_EXPERIMENTAL_DECEL_LEAD_MAX_SPEED:
    return False
  if not lead.status:
    return False

  d_rel = float(lead.dRel)
  v_rel = float(lead.vRel)
  v_lead = max(float(getattr(lead, "vLead", v_ego + v_rel)), 0.0)
  if d_rel <= 0.0 or d_rel / max(v_ego, 1.0) > SANTA_FE_EXPERIMENTAL_DECEL_LEAD_GAP_BP[-1]:
    return False

  return (v_rel < 0.0
          and v_lead > SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_MIN_LEAD_SPEED
          and float(getattr(lead, "aLeadK", 0.0)) <= -SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_MIN_LEAD_DECEL
          and int(getattr(lead, "radarTrackId", -1)) >= 0
          and float(getattr(lead, "modelProb", 0.0)) >= SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_MIN_MODEL_PROB)


def update_santa_fe_experimental_decelerating_lead_persistence(track_id, alk_window, v_ego, lead):
  if not santa_fe_experimental_decelerating_lead_feedforward_state_ok(v_ego, lead):
    return None, []

  lead_track_id = int(lead.radarTrackId)
  a_lead_k = float(lead.aLeadK)
  if track_id != lead_track_id:
    return lead_track_id, [a_lead_k]
  return track_id, (alk_window + [a_lead_k])[-SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_PERSIST_FRAMES:]


def get_santa_fe_experimental_decelerating_lead_feedforward_target(v_ego, lead, a_lead_k_window):
  if (a_lead_k_window is None
      or len(a_lead_k_window) < SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_PERSIST_FRAMES
      or not santa_fe_experimental_decelerating_lead_feedforward_state_ok(v_ego, lead)):
    return None
  # Use the least-severe decel sustained across the dwell so one aLeadK spike cannot set the cap.
  sustained_a_lead_k = max(a_lead_k_window)
  if sustained_a_lead_k > -SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_MIN_LEAD_DECEL:
    return None
  return max(SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_GAIN * sustained_a_lead_k,
             -SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_MAX_DECEL)


def slew_santa_fe_experimental_decelerating_lead_feedforward_authority(authority, feedforward_target):
  # The RAW target is spike-proof (dwell window) but not continuous: the least-severe sample aging
  # out can step it ~0.45 in one frame, and a one-frame eligibility loss zeroes it. The APPLIED
  # authority slews toward the target so activation, eviction and release are all rate-bounded;
  # deeper native/MPC demands are unaffected (this lane only ever min()s on top).
  target = feedforward_target if feedforward_target is not None else 0.0
  return float(np.clip(target,
                       authority - SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_SLEW_DOWN * DT_MDL,
                       authority + SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_SLEW_UP * DT_MDL))


def advance_santa_fe_experimental_decelerating_lead_feedforward_lane(eligible, track_id, alk_window, authority, v_ego=None, lead=None):
  """THE single production path for the feedforward lane's state, both call sites (end-review
  round 3): eligible frames advance persistence and slew the authority toward the target;
  an ineligible frame (the mode edge) HARD-CLEARS everything -- authority AND window together,
  so re-entry must re-earn the full dwell and stale authority can never snap back onto the
  output. Mode exit is a caller-owned discontinuity, like the approach cap's own release."""
  if not eligible:
    return None, [], 0.0
  track_id, alk_window = update_santa_fe_experimental_decelerating_lead_persistence(track_id, alk_window, v_ego, lead)
  target = get_santa_fe_experimental_decelerating_lead_feedforward_target(v_ego, lead, alk_window)
  return track_id, alk_window, slew_santa_fe_experimental_decelerating_lead_feedforward_authority(authority, target)


def get_santa_fe_experimental_decelerating_lead_approach_cap(v_ego, lead):
  if v_ego < SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_BP[0] or v_ego > SANTA_FE_EXPERIMENTAL_DECEL_LEAD_MAX_SPEED:
    return None
  if not lead.status:
    return None

  d_rel = float(lead.dRel)
  v_rel = float(lead.vRel)
  if d_rel <= 0.0:
    return None

  closing_speed = max(-v_rel, 0.0)
  if closing_speed < SANTA_FE_EXPERIMENTAL_DECEL_LEAD_CLOSING_BP[0]:
    return None

  time_gap = d_rel / max(v_ego, 1.0)
  lead_decel = max(-float(getattr(lead, "aLeadK", 0.0)), 0.0)
  if time_gap > SANTA_FE_EXPERIMENTAL_DECEL_LEAD_GAP_BP[-1]:
    return None

  ttc = d_rel / max(closing_speed, 0.1)
  gap_cap = float(np.interp(time_gap, SANTA_FE_EXPERIMENTAL_DECEL_LEAD_GAP_BP, SANTA_FE_EXPERIMENTAL_DECEL_LEAD_GAP_CAPS))
  closing_tighten = float(np.interp(closing_speed, SANTA_FE_EXPERIMENTAL_DECEL_LEAD_CLOSING_BP,
                                    SANTA_FE_EXPERIMENTAL_DECEL_LEAD_CLOSING_TIGHTEN))
  lead_decel_tighten = float(np.interp(lead_decel, SANTA_FE_EXPERIMENTAL_DECEL_LEAD_DECEL_BP,
                                       SANTA_FE_EXPERIMENTAL_DECEL_LEAD_DECEL_TIGHTEN))
  ttc_tighten = float(np.interp(ttc, SANTA_FE_EXPERIMENTAL_DECEL_LEAD_TTC_BP, SANTA_FE_EXPERIMENTAL_DECEL_LEAD_TTC_TIGHTEN))
  return float(np.clip(gap_cap - closing_tighten - lead_decel_tighten - ttc_tighten, -0.85, 0.05))


def apply_santa_fe_experimental_decelerating_lead_approach_cap(output_a_target, v_ego, lead):
  cap = get_santa_fe_experimental_decelerating_lead_approach_cap(v_ego, lead)
  if cap is None:
    return output_a_target

  return min(output_a_target, cap)


def apply_santa_fe_experimental_lead_caution(output_a_target, v_ego, lead):
  extra_decel = get_santa_fe_experimental_lead_caution_decel(v_ego, lead, output_a_target)
  if extra_decel <= 0.0:
    return output_a_target

  return output_a_target - extra_decel


def get_santa_fe_stop_commit_required_decel(v_ego, d_rel, lead_v, lead_decel):
  """Constant decel that rests the ego SANTA_FE_STOP_COMMIT_REST_FLOOR_M behind the lead's
  projected stop point. lead_decel must already be the least-severe (windowed) estimate."""
  if lead_v < SANTA_FE_STOP_COMMIT_LEAD_STOPPED_V:
    lead_stop_dist = 0.0
  else:
    lead_stop_dist = (lead_v * lead_v) / (2.0 * max(lead_decel, SANTA_FE_STOP_COMMIT_LEAD_DECEL_MIN))
  d_stop = d_rel + lead_stop_dist - SANTA_FE_STOP_COMMIT_REST_FLOOR_M
  return (v_ego * v_ego) / (2.0 * max(d_stop, SANTA_FE_STOP_COMMIT_MIN_BRAKE_DIST_M))


def santa_fe_stop_commit_lead_state_ok(v_ego, lead):
  """Per-frame lead-state gate for the stop-commitment floor: an actionable stopping-or-stopped
  lead. Vision-only leads all carry the radarTrackId -1 sentinel, so same-track persistence proves
  nothing there -- require high vision confidence on every frame instead."""
  if not lead.status:
    return False
  d_rel = float(lead.dRel)
  if d_rel <= 0.0 or d_rel > SANTA_FE_STOP_COMMIT_MAX_D_REL:
    return False
  if int(getattr(lead, "radarTrackId", -1)) < 0 and float(getattr(lead, "modelProb", 1.0)) < SANTA_FE_STOP_COMMIT_VISION_PROB_MIN:
    return False
  lead_v = max(float(getattr(lead, "vLead", 0.0)), 0.0)
  a_lead_k = float(getattr(lead, "aLeadK", 0.0))
  return (a_lead_k <= -SANTA_FE_STOP_COMMIT_LEAD_DECEL_MIN and lead_v < float(v_ego)) or lead_v <= SANTA_FE_STOP_COMMIT_LEAD_STOPPED_V


def get_santa_fe_stop_commit_radar_min_acquire_d_rel(v_ego):
  """Minimum first-seen distance for radar-only custom authority. It covers the distance needed
  to stop at the floor with the lane's minimum useful decel, plus actuation and confirmation time."""
  return get_radar_only_min_acquire_d_rel(
    v_ego,
    rest_floor_m=SANTA_FE_STOP_COMMIT_REST_FLOOR_M,
    min_decel=SANTA_FE_STOP_COMMIT_A_REQ_MIN,
    actuation_delay_s=SANTA_FE_STOP_COMMIT_ACTUATION_DELAY_S,
    confirmation_time_s=SANTA_FE_STOP_COMMIT_PERSIST_FRAMES * DT_MDL,
  )


def update_santa_fe_stop_commit_track_certificate(previous_track_id, previous_certified, v_ego, lead, lead_state_ok):
  """Certify one selected lead track for custom deepening. A radar track is certified by any
  model association, or by being selected early enough to survive the normal persistence dwell.
  A close radar-only track cannot become certified merely because ego braking lowers the horizon."""
  if not lead_state_ok:
    return False

  track_id = int(getattr(lead, "radarTrackId", -1))
  model_prob = float(getattr(lead, "modelProb", 1.0))
  if track_id < 0:
    return model_prob >= SANTA_FE_STOP_COMMIT_VISION_PROB_MIN
  if model_prob > 0.0:
    return True
  if previous_track_id == track_id:
    return previous_certified
  return float(lead.dRel) >= get_santa_fe_stop_commit_radar_min_acquire_d_rel(v_ego)


def santa_fe_stop_commit_track_provenance_ok(lead, lead_two, track_certified):
  """Reject extra authority for an unconfirmed closer radar return that conflicts with a strongly
  model-confirmed farther lead. The normal radar/MPC path remains untouched."""
  if not track_certified:
    return False
  if int(getattr(lead, "radarTrackId", -1)) < 0 or float(getattr(lead, "modelProb", 1.0)) > 0.0:
    return True
  if not lead_two.status or float(getattr(lead_two, "modelProb", 0.0)) < SANTA_FE_STOP_COMMIT_RADAR_CONFLICT_PROB_MIN:
    return True
  return float(lead_two.dRel) - float(lead.dRel) < SANTA_FE_STOP_COMMIT_RADAR_CONFLICT_GAP_M


def update_santa_fe_stop_commit_persistence(track_id, frames, alk_window, lead_state_ok, lead_track_id, a_lead_k,
                                            vlead_window=None, v_lead=0.0):
  """Same-track persistence for the stop-commitment floor. A track switch (the 00001b97 2-frame
  vLead glitch class) or any gate-failing frame restarts the count. Vision-only leads share
  radarTrackId -1 and persist like any other track. Also carries the raw-vLead window for the
  aim lane's rollback qualification (cycle-27 end-review: one noisy -0.20 frame must not count
  as rollback -- the recorded stopped-lead Doppler bursts run up to 0.36 s, so the LEAST-negative
  sample over this longer window is the conservative rollback estimate: any clean frame inside
  the window zeroes it). Returns (track_id, frames, alk_window, vlead_window)."""
  if vlead_window is None:
    vlead_window = []
  if not lead_state_ok:
    return None, 0, [], []
  if track_id != lead_track_id:
    return lead_track_id, 1, [a_lead_k], [v_lead]
  return (track_id, frames + 1,
          (alk_window + [a_lead_k])[-SANTA_FE_STOP_COMMIT_ALK_WINDOW:],
          (vlead_window + [v_lead])[-SANTA_FE_STOP_AIM_ROLLBACK_WINDOW:])


def get_santa_fe_stop_commit_floor(v_ego, lead, output_a_target, alk_window, active_prev):
  """The necessity floor itself: (floor accel or None, active). DEEPEN-ONLY consumer contract:
  callers apply min(output_a_target, floor). Schmitt margins make activation/release hysteretic."""
  if v_ego <= SANTA_FE_STOP_COMMIT_V_EGO_MIN or v_ego > SANTA_FE_STOP_COMMIT_V_EGO_MAX:
    return None, False
  if not lead.status:
    return None, False
  d_rel = float(lead.dRel)
  if d_rel <= 0.0 or d_rel > SANTA_FE_STOP_COMMIT_MAX_D_REL:
    return None, False
  lead_v = max(float(getattr(lead, "vLead", 0.0)), 0.0)
  # least-severe decel over the window -> longest projected lead stop -> conservative necessity
  lead_decel = max(0.0, -max(alk_window)) if alk_window else 0.0
  # necessity on the gap AFTER the actuation response delay: the ego travels ~v*tau before a
  # deeper command physically arrives, so the instantaneous gap overstates what is available
  d_eff = max(d_rel - v_ego * SANTA_FE_STOP_COMMIT_ACTUATION_DELAY_S, 0.0)
  a_req = get_santa_fe_stop_commit_required_decel(v_ego, d_eff, lead_v, lead_decel)
  if a_req < SANTA_FE_STOP_COMMIT_A_REQ_MIN:
    return None, False
  margin = SANTA_FE_STOP_COMMIT_RELEASE_MARGIN if active_prev else SANTA_FE_STOP_COMMIT_ACTIVATE_MARGIN
  if a_req < -min(float(output_a_target), 0.0) + margin:
    return None, False
  return -min(a_req, SANTA_FE_STOP_COMMIT_A_MAX), True


def get_santa_fe_stop_aim_floor(v_ego, lead, output_a_target, alk_window, committed_prev, rest_aim,
                                vlead_window=None):
  """Aim-commitment necessity floor: (floor accel or None, committed). DEEPEN-ONLY consumer
  contract like the band floor: callers apply min(output_a_target, floor). Commitment is
  heat-gated at onset and hysteretic on release; while committed the floor is the CONTINUOUS
  necessity (no command-relative margin -- see the constants block)."""
  if v_ego > SANTA_FE_STOP_COMMIT_V_EGO_MAX:
    return None, False
  if v_ego <= SANTA_FE_STOP_AIM_V_EGO_MIN and not committed_prev:
    # re-slam extension: ENTRY below the normal floor only for a decelerating-but-moving lead.
    # An existing commitment rides through (committed_prev): the re-slam's natural end is the
    # lead reaching zero mid-commitment -- dropping the lane there re-creates the dead zone
    # (extension closed, stopped-lead latch not yet confirmed) at peak necessity.
    lead_moving = float(getattr(lead, "vLead", 0.0)) >= SANTA_FE_STOP_AIM_RESLAM_LEAD_V_MIN
    lead_braking = bool(alk_window) and max(alk_window) <= SANTA_FE_STOP_AIM_RESLAM_ALK_MAX
    if not (v_ego > SANTA_FE_STOP_AIM_RESLAM_V_MIN and lead_moving and lead_braking):
      return None, False
  if not lead.status:
    return None, False
  d_rel = float(lead.dRel)
  if d_rel <= 0.0 or d_rel > SANTA_FE_STOP_COMMIT_MAX_D_REL:
    return None, False
  lead_v = max(float(getattr(lead, "vLead", 0.0)), 0.0)
  lead_decel = max(0.0, -max(alk_window)) if alk_window else 0.0
  if lead_v < SANTA_FE_STOP_COMMIT_LEAD_STOPPED_V:
    lead_stop_dist = 0.0
  else:
    lead_stop_dist = (lead_v * lead_v) / (2.0 * max(lead_decel, SANTA_FE_STOP_COMMIT_LEAD_DECEL_MIN))
  d_eff = max(d_rel - v_ego * SANTA_FE_STOP_AIM_ACTUATION_DELAY_S, 0.0)
  a_req = (v_ego * v_ego) / (2.0 * max(d_eff + lead_stop_dist - rest_aim, SANTA_FE_STOP_COMMIT_MIN_BRAKE_DIST_M))
  # ROLLBACK PROJECTION (cycle-27, end-review shape): commitment is decided on the BASELINE
  # necessity above -- a projection-inflated a_req past CAP must never REFUSE the commitment it
  # exists to strengthen (the cap inversion). The projection deepens only the FLOOR VALUE,
  # cap-clipped. Rollback is the LEAST-negative raw vLead over the qualification window: the
  # recorded 0.36 s stopped-lead Doppler bursts always contain a clean frame, so noise projects
  # zero; genuine sustained rollback projects its shallowest observed magnitude.
  rollback = min(max(vlead_window), 0.0) if vlead_window else 0.0
  if rollback < 0.0:
    d_eff_rb = max(d_eff + rollback * SANTA_FE_STOP_AIM_ROLLBACK_HORIZON_S, 0.0)
    a_req_deep = (v_ego * v_ego) / (2.0 * max(d_eff_rb + lead_stop_dist - rest_aim,
                                              SANTA_FE_STOP_COMMIT_MIN_BRAKE_DIST_M))
  else:
    a_req_deep = a_req
  if committed_prev:
    # DEPARTING RELEASE: a lead accelerating away dissolves the stop (the launch case) -- the
    # baseline cannot decay quickly at tight gaps (the MIN_BRAKE_DIST clamp holds a_req ~ v^2),
    # so without this a stale commitment could suppress a launch until the gap opens. Departure
    # evidence is the LAST 0.1 s only (end-review: min over the full 6-frame window kept slam
    # frames in scope for 0.3 s after a rapid flip, commanding -1.3 against a +1.0 launch).
    lv_now = float(getattr(lead, "vLead", 0.0))
    alk_positive = len(alk_window) >= 2 and min(alk_window[-2:]) >= 0.15
    # launch-from-zero (round-3): a lead launching DURING commitment must release before it
    # reaches 0.5 m/s -- rising-speed evidence covers the low band (stopped-lead Doppler noise
    # is negative-side, so a +0.08 rise over 0.15 s with positive alk is a genuine launch)
    rising = (vlead_window is not None and len(vlead_window) >= 3
              and lv_now >= 0.15 and vlead_window[-1] - vlead_window[-3] >= 0.08)
    departing = alk_positive and (lv_now >= 0.5 or rising)
    # HOLD BAND (end-review: the runway clamp makes a_req <= v^2, so every commitment would
    # self-release below 1.0 m/s -- reopening the dead zone while the service latch is still
    # earning its dwell, and letting the planner EASE mid-landing, recorded cmd -0.33 at t-0.5).
    # Committed holds to RESLAM_EXIT_V; the baseline-decay release applies only at speed, where
    # decay means the stop genuinely dissolved (gap growth).
    if departing or v_ego <= SANTA_FE_STOP_AIM_RESLAM_EXIT_V:
      committed = False
    elif v_ego > 1.5:
      committed = a_req >= SANTA_FE_STOP_AIM_OFF
    else:
      committed = True
  else:
    committed = (SANTA_FE_STOP_AIM_ON <= a_req <= SANTA_FE_STOP_AIM_CAP
                 and d_rel + lead_stop_dist <= SANTA_FE_STOP_AIM_STOP_WITHIN_M)
  if not committed:
    return None, False
  return -min(a_req_deep, SANTA_FE_STOP_AIM_CAP), True


def get_santa_fe_stop_floor_demands(v_ego, lead, pre_lanes_a_target, alk_window,
                                    aim_committed_prev, floor_active_prev, rest_aim, aim_enabled,
                                    vlead_window=None):
  """Evaluate BOTH necessity floors against the SAME pre-lane command (end-review HIGH: applying
  the aim min() first fed the band floor's command-relative Schmitt a deeper command, raising its
  engage bar past a GENUINE floor requirement -- at aim-cap -2.25 a 2.40 floor need read as
  'within margin' and stayed masked, a 0.42 m floor deficit during a hardening-lead transition).
  The caller min-merges: output = min(output, every non-None floor)."""
  if aim_enabled:
    aim_floor, aim_committed = get_santa_fe_stop_aim_floor(
      v_ego, lead, pre_lanes_a_target, alk_window, aim_committed_prev, rest_aim,
      vlead_window=vlead_window)
  else:
    aim_floor, aim_committed = None, False
  commit_floor, floor_active = get_santa_fe_stop_commit_floor(
    v_ego, lead, pre_lanes_a_target, alk_window, floor_active_prev)
  return aim_floor, aim_committed, commit_floor, floor_active


# Cycle-31: REST-CLOSE REFERENCE FLOOR (design review E1-R). Routes 200a-2011: 4 of 12 rests
# landed 5.2-6.47 m behind CONFIRMED-STOPPED leads because the e2e model's comfort profile lets
# its reference velocity die at the walking-pace follow equilibrium (s22: the plan stalled at
# gap 6.5 = desired(v=1.5); the pure-creep cases settle wherever the queue equilibrium sat) --
# and the first v<0.05 frame latches the secure hold, so the 2 m closure to the 4.3 design rest
# never happens (post-rest re-close was REJECTED: post-stop motion near a lead is pinned off;
# prevention is strictly better). While armed, the MODEL REFERENCE (not any demand lane) is
# floored at the comfort closure curve toward rest: v_floor = min(v_cap, sqrt(2*0.5*d_eff)),
# raise-only, position re-integrated to match, never accelerating above the captured entry
# speed. Every braking lane (aim, re-slam, stop-commit, service) stays deepen-only ON TOP.
SANTA_FE_REST_CLOSE_ARM_V_MIN = 0.06
SANTA_FE_REST_CLOSE_ARM_V_MAX = 1.50
SANTA_FE_REST_CLOSE_CANCEL_V = 1.60
SANTA_FE_REST_CLOSE_V_CAP = 0.80
SANTA_FE_REST_CLOSE_DECEL = 0.50
SANTA_FE_REST_CLOSE_LAG_S = 0.25
SANTA_FE_REST_CLOSE_D_EFF_MIN = 0.50
SANTA_FE_REST_CLOSE_D_EFF_MAX = 2.50
SANTA_FE_REST_CLOSE_LEAD_V_MIN = -0.10
SANTA_FE_REST_CLOSE_EPOCH_RESET_V = 3.00  # leaving the stopping regime re-opens the one-shot


def apply_santa_fe_rest_close_reference_floor(x, v, a, v_floor_now, t_idxs):
  """Raise-only floor on the model reference: the closure curve from (t=0, v_floor_now) at
  constant SANTA_FE_REST_CLOSE_DECEL. Position is RE-INTEGRATED from the raised velocity (a
  velocity floor without a matching position curve would fight the MPC's x-cost). Returns
  (x, v, a, lifted)."""
  v_f = np.maximum(v_floor_now - SANTA_FE_REST_CLOSE_DECEL * t_idxs, 0.0)
  if not np.any(v_f > v + 1e-4):
    return x, v, a, False
  v_new = np.maximum(v, v_f)
  x_new = np.concatenate(([x[0]], x[0] + np.cumsum(0.5 * (v_new[1:] + v_new[:-1]) * np.diff(t_idxs))))
  a_new = np.gradient(v_new, t_idxs)
  return x_new, v_new, a_new, True


def get_santa_fe_rest_close_floor_v(gap, v_guard, rest_target, v_cap):
  """The closure-curve floor at the CURRENT gap, or None when outside the active window."""
  d_eff = gap - rest_target - SANTA_FE_REST_CLOSE_LAG_S * v_guard
  if not (SANTA_FE_REST_CLOSE_D_EFF_MIN <= d_eff <= SANTA_FE_REST_CLOSE_D_EFF_MAX):
    return None
  return min(v_cap, math.sqrt(2.0 * SANTA_FE_REST_CLOSE_DECEL * d_eff))


def update_santa_fe_rest_close_state(armed, spent, vcap, tid, rc_ok, v_ego, d_eff_arm, rc_tid,
                                     standstill=False):
  """Pure arm/cancel state machine for the E1-R lane: one arm per APPROACH -- spend on any
  disqualifier / speed escape / lead replacement holds for the rest of that approach, and the
  APPROACH-EPOCH boundary (R1 HIGH fix) re-opens it for the next one: a COMPLETED rest
  (standstill) or leaving the stopping regime entirely (v > EPOCH_RESET_V). Arming still
  requires v > ARM_V_MIN, so a cleared spend can never move a standing car -- the E3 rejection
  (no post-stop re-open of a latched rest) stands: the lane only ever raises a reference for a
  car already in motion. Returns (armed, spent, vcap, tid)."""
  if armed:
    if (not rc_ok or v_ego > SANTA_FE_REST_CLOSE_CANCEL_V
        or (tid is not None and rc_tid >= 0 and rc_tid != tid)):
      armed, spent = False, True
  elif (not spent and rc_ok
        and SANTA_FE_REST_CLOSE_ARM_V_MIN < v_ego <= SANTA_FE_REST_CLOSE_ARM_V_MAX
        and d_eff_arm is not None
        and SANTA_FE_REST_CLOSE_D_EFF_MIN <= d_eff_arm <= SANTA_FE_REST_CLOSE_D_EFF_MAX):
    armed, spent = True, False
    vcap = min(v_ego, SANTA_FE_REST_CLOSE_V_CAP)
    tid = rc_tid if rc_tid >= 0 else None
  if not armed and (standstill or v_ego > SANTA_FE_REST_CLOSE_EPOCH_RESET_V):
    spent = False
  return armed, spent, vcap, tid


def get_santa_fe_stopped_lead_late_approach_limits(v_ego, d_rel, closing_speed):
  if v_ego < SANTA_FE_STOPPED_LEAD_LATE_APPROACH_SPEED_BP[0] or v_ego > SANTA_FE_STOPPED_LEAD_LATE_APPROACH_SPEED_BP[-1]:
    return None

  min_closing = float(np.interp(v_ego, SANTA_FE_STOPPED_LEAD_LATE_APPROACH_SPEED_BP,
                                SANTA_FE_STOPPED_LEAD_LATE_APPROACH_MIN_CLOSING))
  if closing_speed < min_closing:
    return None

  max_d_rel = float(np.interp(v_ego, SANTA_FE_STOPPED_LEAD_LATE_APPROACH_SPEED_BP,
                              SANTA_FE_STOPPED_LEAD_LATE_APPROACH_MAX_D_REL))
  if d_rel > max_d_rel:
    return None

  projected_ttc = d_rel / max(closing_speed, 0.1)
  max_projected_ttc = float(np.interp(v_ego, SANTA_FE_STOPPED_LEAD_LATE_APPROACH_SPEED_BP,
                                      SANTA_FE_STOPPED_LEAD_LATE_APPROACH_MAX_TTC))
  if projected_ttc > max_projected_ttc:
    return None

  buffer_m = float(np.interp(v_ego, SANTA_FE_STOPPED_LEAD_LATE_APPROACH_SPEED_BP,
                             SANTA_FE_STOPPED_LEAD_LATE_APPROACH_BUFFER_M))
  max_decel = float(np.interp(v_ego, SANTA_FE_STOPPED_LEAD_LATE_APPROACH_SPEED_BP,
                              SANTA_FE_STOPPED_LEAD_LATE_APPROACH_MAX_DECEL))
  return buffer_m, max_decel


def get_santa_fe_stopped_lead_smooth_approach_cap(v_ego, lead, increased_stopped_distance=0.0, lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET):
  # Select the activation band. The creep extension lowers the floor to 0.55 m/s; every interp value
  # at and above 2.50 m/s is byte-identical to the base tables, so the >= 2.50 m/s behavior is unchanged.
  if stopping_flags.SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_EXTENSION:
    speed_bp = SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_SPEED_BP
    max_decel_v = SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_MAX_DECEL
    buffer_v = SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_BUFFER_M
    min_closing_v = SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_MIN_CLOSING
    min_meaningful_v = SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_MIN_MEANINGFUL_DECEL
  else:
    speed_bp = SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_SPEED_BP
    max_decel_v = SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_MAX_DECEL
    buffer_v = SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_BUFFER_M
    min_closing_v = SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_MIN_CLOSING
    min_meaningful_v = SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_MIN_MEANINGFUL_DECEL

  if v_ego < speed_bp[0] or v_ego > max(speed_bp[-1], SANTA_FE_STOPPED_LEAD_LATE_APPROACH_SPEED_BP[-1]):
    return None
  if not lead.status:
    return None

  d_rel = float(lead.dRel)
  if d_rel <= 0.0:
    return None

  v_rel = float(getattr(lead, "vRel", 0.0))
  lead_v = max(float(getattr(lead, "vLead", v_ego + v_rel)), 0.0)
  lead_v_limit = float(np.interp(v_ego, [2.50, 5.00, 8.00, 12.50], [0.55, 0.50, 0.45, 0.35]))
  if lead_v > lead_v_limit:
    return None

  closing_speed = max(v_ego - lead_v, 0.0)
  min_closing = float(np.interp(v_ego, speed_bp, min_closing_v))
  if closing_speed < min_closing:
    return None

  late_limits = get_santa_fe_stopped_lead_late_approach_limits(v_ego, d_rel, closing_speed)
  if v_ego > speed_bp[-1] and late_limits is None:
    return None

  # Source-pin (test_stop_target_helpers): the get_published_lead_distance_compensation call must
  # stay textually inside this function. The required-decel geometry below is shared with the
  # santa_fe_stopping_lead_roll_in FLOOR via get_santa_fe_stopped_lead_hold_gap_required_decel so
  # cap and floor converge on the SAME stop-at-hold-gap target.
  remaining_to_hold_gap = d_rel + get_published_lead_distance_compensation(increased_stopped_distance) - float(lead_stop_distance_target)
  if remaining_to_hold_gap <= 0.0:
    return None

  buffer_m = float(np.interp(v_ego, speed_bp, buffer_v))
  required_decel = get_santa_fe_stopped_lead_hold_gap_required_decel(v_ego, remaining_to_hold_gap, buffer_m)
  max_decel = float(np.interp(v_ego, speed_bp, max_decel_v))

  # If the stopped lead is acquired late, spend more speed early instead of saving it for the last
  # few meters. Normal stopped-lead approaches keep the gentler comfort table above.
  if late_limits is not None:
    late_buffer_m, late_max_decel = late_limits
    late_required = get_santa_fe_stopped_lead_hold_gap_required_decel(v_ego, remaining_to_hold_gap, late_buffer_m)
    required_decel = max(required_decel, late_required * SANTA_FE_STOPPED_LEAD_LATE_APPROACH_FIRMNESS)
    max_decel = max(max_decel, late_max_decel)

  min_meaningful_decel = float(np.interp(v_ego, speed_bp, min_meaningful_v))
  if required_decel < min_meaningful_decel:
    return None

  return -float(np.clip(required_decel, min_meaningful_decel, max_decel))


def get_santa_fe_stopped_lead_hold_gap_required_decel(v_ego, remaining_to_hold_gap, buffer_m):
  """Single source of truth for the stopped-lead hold-gap DECEL geometry shared by the
  smooth-approach CAP (min/deepen) and the santa_fe_stopping_lead_roll_in FLOOR (max/raise).
  Given the remaining distance to the 4.0 m + ISD hold gap (each caller computes this with the
  pinned get_published_lead_distance_compensation term so the ISD source pin holds) and the same
  buffer, returns the constant decel that brings v_ego to rest exactly at the hold gap. Because the
  floor and the cap consume the IDENTICAL required_decel, max() can never carry speed PAST the
  hold gap by construction."""
  braking_distance = max(remaining_to_hold_gap - buffer_m, 0.75)
  return (v_ego * v_ego) / (2.0 * braking_distance)


def apply_santa_fe_stopped_lead_smooth_approach_cap(output_a_target, v_ego, lead, increased_stopped_distance=0.0,
                                                    lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET):
  cap = get_santa_fe_stopped_lead_smooth_approach_cap(v_ego, lead, increased_stopped_distance, lead_stop_distance_target)
  if cap is None or output_a_target <= cap:
    return output_a_target

  return cap


def get_santa_fe_stopping_lead_roll_in(v_ego, lead, increased_stopped_distance=0.0,
                                       lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET):
  """The MIRROR of get_santa_fe_stopped_lead_smooth_approach_cap. Returns a FLOOR a_target
  (negative, the gentle stop-at-hold-gap decel) that the caller applies as max(output_a_target,
  floor) so the MPC cannot brake HARDER than needed to stop at the 4.0 m + ISD hold gap. Returns
  None outside its band / when a roll-in must not raise brake. The roll-in caller (not this
  function) owns the output_should_stop, force-coast, and latched-hard-stop gates."""
  if not stopping_flags.SANTA_FE_STOPPING_LEAD_ROLL_IN:
    return None

  if v_ego < SANTA_FE_STOPPING_LEAD_ROLL_IN_V_EGO_MIN or v_ego >= SANTA_FE_STOPPING_LEAD_ROLL_IN_V_EGO_MAX:
    return None
  if not lead.status:
    return None

  d_rel = float(lead.dRel)
  if d_rel <= 0.0:
    return None

  v_rel = float(getattr(lead, "vRel", 0.0))
  lead_v = max(float(getattr(lead, "vLead", v_ego + v_rel)), 0.0)
  if lead_v > SANTA_FE_STOPPING_LEAD_ROLL_IN_LEAD_V_MAX:
    return None

  closing_speed = max(v_ego - lead_v, 0.0)
  if closing_speed < SANTA_FE_STOPPING_LEAD_ROLL_IN_MIN_CLOSING:
    return None
  if closing_speed > SANTA_FE_STOPPING_LEAD_ROLL_IN_MAX_CLOSING:
    return None

  # Gate off once we are essentially at the hold gap -- hand the finish to the cap / low-speed
  # glide. Uses the SAME pinned ISD compensation term as the cap so floor and cap share one hold
  # gap (the get_published_lead_distance_compensation call also re-pins the ISD-helper source here).
  remaining_to_hold_gap = d_rel + get_published_lead_distance_compensation(increased_stopped_distance) - float(lead_stop_distance_target)
  if remaining_to_hold_gap <= SANTA_FE_STOPPING_LEAD_ROLL_IN_GATE_OFF_MARGIN_M:
    return None

  ttc = d_rel / max(closing_speed, 0.1)
  if ttc < SANTA_FE_STOPPING_LEAD_ROLL_IN_MIN_TTC_S:
    return None

  # Share the cap's hold-gap DECEL geometry exactly (same buffer at this v_ego, same required_decel).
  buffer_m = float(np.interp(v_ego, SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_SPEED_BP, SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_BUFFER_M))
  required_decel = get_santa_fe_stopped_lead_hold_gap_required_decel(v_ego, remaining_to_hold_gap, buffer_m)
  max_decel = float(np.interp(v_ego, SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_SPEED_BP, SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_MAX_DECEL))
  # Carry-past hardening: if even the kinematic stop-at-hold-gap decel is DEEPER than this gentle
  # floor's ceiling, the situation needs MORE brake than a gentle roll-in can give -> hand the brake
  # to the MPC (return None) instead of clipping to the shallow -max_decel, which would raise the
  # command shallower than required and carry speed PAST the hold gap.
  if required_decel > max_decel:
    return None
  return -float(np.clip(required_decel, 0.0, max_decel))


def santa_fe_stopping_lead_roll_in_latch_triggered(v_ego, lead):
  """A single Kalman-lagged lead-decel frame (aLeadK) is transient; the caller LATCHES this with a
  dwell so a real hard stop durably hands full brake authority to the MPC. Triggers when the lead
  is braking hard (aLeadK below -threshold) within the roll-in band."""
  if v_ego < SANTA_FE_STOPPING_LEAD_ROLL_IN_V_EGO_MIN or v_ego >= SANTA_FE_STOPPING_LEAD_ROLL_IN_V_EGO_MAX:
    return False
  if not lead.status:
    return False
  lead_decel = max(-float(getattr(lead, "aLeadK", 0.0)), 0.0)
  return lead_decel >= SANTA_FE_STOPPING_LEAD_ROLL_IN_LEAD_DECEL_LATCH


def apply_santa_fe_stopping_lead_roll_in(output_a_target, v_ego, lead, increased_stopped_distance=0.0,
                                         lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET):
  floor = get_santa_fe_stopping_lead_roll_in(v_ego, lead, increased_stopped_distance, lead_stop_distance_target)
  if floor is None or output_a_target >= floor:
    return output_a_target

  return floor


def get_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(v_ego, lead, accel_coast, increased_stopped_distance=0.0,
                                                                      lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET):
  if accel_coast < SANTA_FE_DOWNHILL_QUEUE_COAST_ACCEL_MIN:
    return None
  if v_ego <= SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_SPEED_BP[-1] or v_ego > SANTA_FE_DOWNHILL_STOPPED_LEAD_HIGH_SPEED_BP[-1]:
    return None
  if not lead.status:
    return None

  d_rel = float(lead.dRel)
  if d_rel <= 0.0 or d_rel > SANTA_FE_DOWNHILL_STOPPED_LEAD_MAX_D_REL:
    return None

  v_rel = float(getattr(lead, "vRel", 0.0))
  lead_v = max(float(getattr(lead, "vLead", v_ego + v_rel)), 0.0)
  lead_v_limit = float(np.interp(v_ego, [2.50, 5.00, 8.00, 12.50], [0.55, 0.50, 0.45, 0.35]))
  if lead_v > lead_v_limit:
    return None

  closing_speed = max(v_ego - lead_v, max(-v_rel, 0.0))
  min_closing = float(np.interp(v_ego, SANTA_FE_DOWNHILL_STOPPED_LEAD_HIGH_SPEED_BP, SANTA_FE_DOWNHILL_STOPPED_LEAD_MIN_CLOSING))
  if closing_speed < min_closing:
    return None

  projected_ttc = d_rel / max(closing_speed, 0.1)
  max_projected_ttc = float(np.interp(v_ego, SANTA_FE_DOWNHILL_STOPPED_LEAD_HIGH_SPEED_BP, SANTA_FE_DOWNHILL_STOPPED_LEAD_MAX_TTC))
  if projected_ttc > max_projected_ttc:
    return None

  remaining_to_hold_gap = d_rel + get_published_lead_distance_compensation(increased_stopped_distance) - float(lead_stop_distance_target)
  if remaining_to_hold_gap <= 0.0:
    return None

  buffer_m = float(np.interp(v_ego, SANTA_FE_DOWNHILL_STOPPED_LEAD_HIGH_SPEED_BP, SANTA_FE_DOWNHILL_STOPPED_LEAD_BUFFER_M))
  braking_distance = max(remaining_to_hold_gap - buffer_m, 0.75)
  required_decel = (v_ego * v_ego) / (2.0 * braking_distance)
  min_meaningful_decel = float(np.interp(v_ego, SANTA_FE_DOWNHILL_STOPPED_LEAD_HIGH_SPEED_BP, SANTA_FE_DOWNHILL_STOPPED_LEAD_MIN_DECEL))
  if required_decel < min_meaningful_decel:
    return None

  max_decel = float(np.interp(v_ego, SANTA_FE_DOWNHILL_STOPPED_LEAD_HIGH_SPEED_BP, SANTA_FE_DOWNHILL_STOPPED_LEAD_MAX_DECEL))
  return -float(np.clip(required_decel, min_meaningful_decel, max_decel))


def apply_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(output_a_target, v_ego, lead, accel_coast,
                                                                        increased_stopped_distance=0.0,
                                                                        lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET):
  cap = get_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(
    v_ego, lead, accel_coast, increased_stopped_distance, lead_stop_distance_target)
  if cap is None or output_a_target <= cap:
    return output_a_target

  return cap


def get_santa_fe_slowing_lead_queue_reserve_decel(projected_ttc, projected_closing_speed, lead_stop_time):
  if (
    not math.isfinite(projected_ttc)
    or not math.isfinite(projected_closing_speed)
    or not math.isfinite(lead_stop_time)
    or lead_stop_time > SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_MAX_STOP_TIME
  ):
    return 0.0

  ttc_reserve = float(np.interp(projected_ttc, SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_TTC_BP,
                                SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_TTC_DECEL))
  closing_reserve = float(np.interp(projected_closing_speed, SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_CLOSING_BP,
                                    SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_CLOSING_DECEL))
  stop_time_factor = float(np.interp(lead_stop_time, SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_STOP_TIME_BP,
                                     SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_STOP_TIME_VALS))
  return float(np.clip((ttc_reserve + closing_reserve) * stop_time_factor, 0.0, SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_MAX_DECEL))


def get_santa_fe_slowing_lead_smooth_approach_cap(v_ego, lead, increased_stopped_distance=0.0, lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET):
  if v_ego < SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_SPEED_BP[0] or v_ego > SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_SPEED_BP[-1]:
    return None
  if not lead.status:
    return None

  d_rel = float(lead.dRel)
  if d_rel <= 0.0:
    return None

  v_rel = float(getattr(lead, "vRel", 0.0))
  lead_v = max(float(getattr(lead, "vLead", v_ego + v_rel)), 0.0)
  stopped_lead_v_limit = float(np.interp(v_ego, [2.50, 5.00, 8.00, 12.50], [0.55, 0.50, 0.45, 0.35]))
  if lead_v <= stopped_lead_v_limit:
    return None

  lead_decel = max(-float(getattr(lead, "aLeadK", 0.0)), 0.0)
  if lead_decel < SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MIN_LEAD_DECEL:
    return None

  lead_stop_time = lead_v / max(lead_decel, 1e-3)
  max_stop_time = float(np.interp(v_ego, SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_SPEED_BP,
                                  SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MAX_STOP_TIME))
  if lead_stop_time > max_stop_time:
    return None

  closing_speed = max(v_ego - lead_v, 0.0)
  projected_closing_speed = closing_speed + (lead_decel * SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_PROJECT_TIME)
  min_closing = float(np.interp(v_ego, SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_SPEED_BP,
                                SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MIN_CLOSING))
  if projected_closing_speed < min_closing:
    return None
  projected_ttc = d_rel / max(projected_closing_speed, 0.1)
  max_projected_ttc = float(np.interp(v_ego, SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_SPEED_BP,
                                      SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MAX_PROJECTED_TTC))
  if projected_ttc > max_projected_ttc:
    return None

  lead_stop_distance = (lead_v * lead_v) / (2.0 * lead_decel)
  remaining_to_hold_gap = d_rel + lead_stop_distance + get_published_lead_distance_compensation(increased_stopped_distance) - float(lead_stop_distance_target)
  if remaining_to_hold_gap <= 0.0:
    return None

  buffer_m = float(np.interp(v_ego, SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_SPEED_BP,
                             SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_BUFFER_M))
  braking_distance = max(remaining_to_hold_gap - buffer_m, 0.75)
  required_decel = (v_ego * v_ego) / (2.0 * braking_distance)
  confidence = float(np.interp(max_stop_time - lead_stop_time, SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_CONFIDENCE_MARGIN,
                               SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_CONFIDENCE_VALS))
  required_decel *= confidence
  required_decel += get_santa_fe_slowing_lead_queue_reserve_decel(projected_ttc, projected_closing_speed, lead_stop_time)
  min_meaningful_decel = float(np.interp(v_ego, SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_SPEED_BP,
                                         SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MIN_DECEL))
  if required_decel < min_meaningful_decel:
    return None

  max_decel = float(np.interp(v_ego, SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_SPEED_BP,
                              SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MAX_DECEL))
  return -float(np.clip(required_decel, min_meaningful_decel, max_decel))


def apply_santa_fe_slowing_lead_smooth_approach_cap(output_a_target, v_ego, lead, increased_stopped_distance=0.0,
                                                    lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET):
  cap = get_santa_fe_slowing_lead_smooth_approach_cap(v_ego, lead, increased_stopped_distance, lead_stop_distance_target)
  if cap is None or output_a_target <= cap:
    return output_a_target

  return cap


def get_santa_fe_downhill_queue_min_accel_clip_step(v_ego, lead, accel_coast, output_a_target, prev_min_accel_clip):
  if output_a_target >= prev_min_accel_clip - 0.05:
    return 0.05
  if accel_coast < SANTA_FE_DOWNHILL_QUEUE_COAST_ACCEL_MIN:
    return 0.05
  if not lead.status:
    return 0.05

  d_rel = float(lead.dRel)
  if d_rel <= 0.0:
    return 0.05

  v_rel = float(getattr(lead, "vRel", 0.0))
  lead_v = max(float(getattr(lead, "vLead", v_ego + v_rel)), 0.0)
  lead_decel = max(-float(getattr(lead, "aLeadK", 0.0)), 0.0)
  closing_speed = max(float(v_ego) - lead_v, max(-v_rel, 0.0))
  stopped_lead_v_limit = float(np.interp(v_ego, [2.50, 5.00, 8.00, 12.50], [0.55, 0.50, 0.45, 0.35]))
  stopped_or_stopping_lead = lead_v <= stopped_lead_v_limit
  high_speed_stopped_queue = (
    stopped_or_stopping_lead
    and SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_SPEED_BP[-1] < v_ego <= SANTA_FE_DOWNHILL_STOPPED_LEAD_HIGH_SPEED_BP[-1]
  )
  max_d_rel = SANTA_FE_DOWNHILL_STOPPED_LEAD_MAX_D_REL if high_speed_stopped_queue else 55.0
  if d_rel > max_d_rel:
    return 0.05

  if high_speed_stopped_queue:
    min_closing = float(np.interp(v_ego, SANTA_FE_DOWNHILL_STOPPED_LEAD_HIGH_SPEED_BP, SANTA_FE_DOWNHILL_STOPPED_LEAD_MIN_CLOSING))
  else:
    min_closing = float(np.interp(v_ego, SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_SPEED_BP,
                                  SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MIN_CLOSING))
  if closing_speed < min_closing:
    return 0.05

  if not stopped_or_stopping_lead:
    if lead_decel < SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MIN_LEAD_DECEL:
      return 0.05
    lead_stop_time = lead_v / max(lead_decel, 1e-3)
    if lead_stop_time > SANTA_FE_SLOWING_LEAD_QUEUE_RESERVE_MAX_STOP_TIME:
      return 0.05

  projected_closing_speed = closing_speed + (lead_decel * SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_PROJECT_TIME)
  projected_ttc = d_rel / max(projected_closing_speed, 0.1)
  if high_speed_stopped_queue:
    max_projected_ttc = float(np.interp(v_ego, SANTA_FE_DOWNHILL_STOPPED_LEAD_HIGH_SPEED_BP, SANTA_FE_DOWNHILL_STOPPED_LEAD_MAX_TTC))
  else:
    max_projected_ttc = float(np.interp(v_ego, SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_SPEED_BP,
                                        SANTA_FE_SLOWING_LEAD_SMOOTH_APPROACH_MAX_PROJECTED_TTC))
  if projected_ttc > max_projected_ttc:
    return 0.05

  return SANTA_FE_DOWNHILL_QUEUE_RELAX_CLIP_STEP


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.acc_mpc = LongitudinalMpc(mode='acc', dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.acc_a_desired = init_a
    self.acc_v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_a_target_trajectory = 0.0
    self.output_should_stop = False
    self.should_stop_hold_timer_s = 0.0
    self.santa_fe_stopping_lead_roll_in_latch_s = 0.0
    self.decel_lead_feedforward_track_id = None
    self.decel_lead_feedforward_alk_window = []
    self.decel_lead_feedforward_authority = 0.0
    self.stop_commit_track_id = None
    self.stop_commit_lead_frames = 0
    self.stop_commit_alk_window = []
    self.stop_commit_vlead_window = []
    self.stop_commit_track_certified = False
    self.stop_commit_active = False
    self.stop_aim_committed = False
    self.rest_close_armed = False       # cycle-31: E1-R reference floor is live
    self.rest_close_spent = False       # ...and cannot re-arm this approach
    self.rest_close_vcap = 0.0
    self.rest_close_tid = None
    self._sf_stop_ctx = StopContext()   # the CONDITIONED stopped-lead classifier, shared CODE
    self._sf_lead_auth = StoppingLeadAuthority()  # R1 MEDIUM: same boundary longcontrol applies
                                        # with longcontrol's context (no raw-vLead gates)
    self.distance_to_stop_target_m = -1.0
    self.experimental_free_road_boost = 0.0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

  @staticmethod
  def parse_model(model_msg, v_ego, frogpilot_toggles):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))

    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0

    if frogpilot_toggles.taco_tune:
      max_lat_accel = np.interp(v_ego, [5, 10, 20], [1.5, 2.0, 3.0])
      curvatures = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z) / np.clip(v, 0.3, 100.0)
      max_v = np.sqrt(max_lat_accel / (np.abs(curvatures) + 1e-3)) - 2.0
      v = np.minimum(max_v, v)

    return x, v, a, j, throttle_prob

  def update(self, sm, frogpilot_toggles):
    mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'
    self.mpc.mode = mode

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise = sm['frogpilotPlan'].vCruise
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    decel_lead_feedforward_eligible = False  # set by the blended Santa Fe cap block; any other frame resets the lane state
    stop_commit_eligible = False  # set by the blended Santa Fe stop-commitment block; any other frame resets the lane state

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    acc_accel_clip = [sm['frogpilotPlan'].minAcceleration, sm['frogpilotPlan'].maxAcceleration]
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    if not sm['frogpilotPlan'].cscControllingSpeed:
      acc_accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, acc_accel_clip, self.CP)

    if mode == 'acc':
      accel_clip = acc_accel_clip.copy()
    else:
      accel_clip = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])
      self.acc_v_desired_filter.x = v_ego
      self.acc_a_desired = np.clip(sm['carState'].aEgo, acc_accel_clip[0], acc_accel_clip[1])
      self.experimental_free_road_boost = 0.0
      self.should_stop_hold_timer_s = 0.0
      self.santa_fe_stopping_lead_roll_in_latch_s = 0.0
      self.decel_lead_feedforward_track_id = None
      self.decel_lead_feedforward_alk_window = []
      self.decel_lead_feedforward_authority = 0.0
      self.stop_commit_track_id = None
      self.stop_commit_lead_frames = 0
      self.stop_commit_alk_window = []
      self.stop_commit_vlead_window = []
      self.stop_commit_track_certified = False
      self.stop_commit_active = False
      self.stop_aim_committed = False
      # rest-close (R1 HIGH): an engagement boundary is an approach epoch -- disarm and re-open
      # the one-shot so re-engagement captures a FRESH entry-speed cap. The _sf_stop_ctx OBJECT
      # stays warm, but its lead evidence is masked at the call site by (not reset_state and
      # rc_authorized) -- longcontrol's exact service_lead_status pattern -- so lead dwells are
      # engagement- and authority-scoped without touching the object here.
      self.rest_close_armed = False
      self.rest_close_spent = False
      self.rest_close_vcap = 0.0
      self.rest_close_tid = None

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'], v_ego, frogpilot_toggles)

    # -- cycle-31 REST-CLOSE reference floor (E1-R): see the constants block for the record ------
    if is_santa_fe_hev_2022(self.CP) and stopping_flags.SANTA_FE_REST_CLOSE_FLOOR:
      _rc_lead = sm['radarState'].leadOne
      rc_tid = int(getattr(_rc_lead, 'radarTrackId', -1))
      # R1/R2 MEDIUM: authority FIRST, then a MASKED lead into the classifier -- longcontrol's
      # exact pattern (service_lead_status, longcontrol.py): an unauthorized (road-furniture
      # class) return must not PRE-EARN the stopped dwell or gap trust while waiting for a
      # modelProb flip; every dwell starts earning only once the track is certified. The mask
      # also covers disengagement (certificate resets via lead_status), so lead evidence is
      # engagement-scoped exactly as longcontrol's service context.
      rc_authorized = self._sf_lead_auth.update(
        v_ego=v_ego, lead_status=bool(_rc_lead.status and not reset_state),
        lead_d_rel=float(_rc_lead.dRel), lead_track_id=rc_tid,
        model_prob=float(getattr(_rc_lead, 'modelProb', 0.0)))
      rc_lead_status = bool(_rc_lead.status and not reset_state and rc_authorized)
      rc_sig = self._sf_stop_ctx.update(
        v_ego=v_ego, a_ego=sm['carState'].aEgo,
        a_cmd=float(sm['carControl'].actuators.accel),
        lead_status=rc_lead_status, lead_v=float(_rc_lead.vLead),
        lead_d_rel=float(_rc_lead.dRel) if rc_lead_status else None,
        lead_track_id=rc_tid if rc_lead_status else None,
        standstill=bool(sm['carState'].standstill), dt=DT_MDL)
      rc_gap_live = (not rc_sig.dropout_active
                     and (rc_sig.gap_source == "measured"
                          or (rc_sig.gap_source == "held" and rc_sig.gap_hold_outward)))
      rc_deeper_lane = self.stop_aim_committed or self.stop_commit_active
      rc_ok = (mode == 'blended' and not reset_state and rc_authorized
               and not sm['frogpilotCarState'].forceCoast
               and rc_sig.lead_confirmed_stopped and rc_sig.lead_motion_earned
               and rc_gap_live and rc_sig.d_gap is not None
               and float(_rc_lead.vLead) >= SANTA_FE_REST_CLOSE_LEAD_V_MIN
               and not sm['carState'].standstill and not rc_deeper_lane)
      d_eff_arm = None
      if rc_sig.d_gap is not None:
        d_eff_arm = (rc_sig.d_gap
                     - (LEAD_STOP_DISTANCE_TARGET + float(sm['frogpilotPlan'].increasedStoppedDistance))
                     - SANTA_FE_REST_CLOSE_LAG_S * v_ego)
      # one arm per approach; permanent spend on any disqualifier -- the standstill cancel means
      # an already-latched rest is never re-opened (E3 rejected: no post-stop motion)
      self.rest_close_armed, self.rest_close_spent, self.rest_close_vcap, self.rest_close_tid = \
        update_santa_fe_rest_close_state(
          self.rest_close_armed, self.rest_close_spent, self.rest_close_vcap, self.rest_close_tid,
          rc_ok, v_ego, d_eff_arm, rc_tid, standstill=bool(sm['carState'].standstill))
      if self.rest_close_armed:
        v_guard = max(v_ego, v_ego - float(_rc_lead.vLead), 0.0)
        rc_floor = get_santa_fe_rest_close_floor_v(
          rc_sig.d_gap,
          v_guard,
          LEAD_STOP_DISTANCE_TARGET + float(sm['frogpilotPlan'].increasedStoppedDistance),
          self.rest_close_vcap)
        if rc_floor is not None:
          x, v, a, _rc_lifted = apply_santa_fe_rest_close_reference_floor(
            x, v, a, rc_floor, np.array(T_IDXS_MPC))
    elif is_santa_fe_hev_2022(self.CP):
      self.rest_close_armed, self.rest_close_spent = False, False
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED * 2],
                                             [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    active_long_distance_factor = get_active_long_distance_factor(sm['frogpilotPlan'].laneWidthLeft, frogpilot_toggles)

    self.mpc.set_weights(
      sm['frogpilotPlan'].accelerationJerk,
      sm['frogpilotPlan'].dangerJerk,
      sm['frogpilotPlan'].speedJerk,
      prev_accel_constraint,
      personality=sm['selfdriveState'].personality,
    )
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(
      v_cruise,
      sm['modelV2'],
      sm['radarState'],
      x,
      v,
      a,
      j,
      sm['frogpilotPlan'].dangerFactor,
      sm['frogpilotPlan'].tFollow,
      accel_clip[0],
      accel_clip[1],
      frogpilot_toggles,
      personality=sm['selfdriveState'].personality,
      short_distance_factor=frogpilot_toggles.short_distance_factor,
      long_distance_factor=active_long_distance_factor,
      increased_stopped_distance=sm['frogpilotPlan'].increasedStoppedDistance,
    )

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)
    self.distance_to_stop_target_m = float(getattr(self.mpc, "distance_to_stop_target_m", -1.0))

    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t = frogpilot_toggles.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(
      self.v_desired_trajectory,
      self.a_desired_trajectory,
      CONTROL_N_T_IDX,
      action_t=action_t,
      vEgoStopping=frogpilot_toggles.vEgoStopping,
    )
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if mode == 'acc':
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc
      self.experimental_free_road_boost = 0.0
      self.acc_v_desired_filter.x = self.v_desired_filter.x
      self.acc_a_desired = self.a_desired
    else:
      self.acc_mpc.set_weights(
        sm['frogpilotPlan'].accelerationJerk,
        sm['frogpilotPlan'].dangerJerk,
        sm['frogpilotPlan'].speedJerk,
        prev_accel_constraint,
        personality=sm['selfdriveState'].personality,
      )
      self.acc_v_desired_filter.x = max(0.0, self.acc_v_desired_filter.update(v_ego))
      self.acc_mpc.set_cur_state(self.acc_v_desired_filter.x, self.acc_a_desired)
      self.acc_mpc.update(
        v_cruise,
        sm['modelV2'],
        sm['radarState'],
        x,
        v,
        a,
        j,
        sm['frogpilotPlan'].dangerFactor,
        sm['frogpilotPlan'].tFollow,
        acc_accel_clip[0],
        acc_accel_clip[1],
        frogpilot_toggles,
        personality=sm['selfdriveState'].personality,
        short_distance_factor=frogpilot_toggles.short_distance_factor,
        long_distance_factor=active_long_distance_factor,
        increased_stopped_distance=sm['frogpilotPlan'].increasedStoppedDistance,
      )
      acc_v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.acc_mpc.v_solution)
      acc_a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.acc_mpc.a_solution)
      acc_a_prev = self.acc_a_desired
      self.acc_a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, acc_a_desired_trajectory))
      self.acc_v_desired_filter.x = self.acc_v_desired_filter.x + self.dt * (self.acc_a_desired + acc_a_prev) / 2.0
      output_a_target_acc, _ = get_accel_from_plan(
        acc_v_desired_trajectory,
        acc_a_desired_trajectory,
        CONTROL_N_T_IDX,
        action_t=action_t,
        vEgoStopping=frogpilot_toggles.vEgoStopping,
      )
      output_a_target_acc = float(np.clip(output_a_target_acc, acc_accel_clip[0], acc_accel_clip[1]))
      experimental_base_a_target = min(output_a_target_mpc, output_a_target_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      self.experimental_free_road_boost = update_experimental_free_road_boost(
        self.experimental_free_road_boost,
        mode,
        self.allow_throttle,
        self.output_should_stop,
        sm['frogpilotCarState'].forceCoast,
        sm['radarState'].leadOne,
        v_ego,
        v_cruise,
        experimental_base_a_target,
        output_a_target_acc,
        output_a_target_e2e,
        getattr(frogpilot_toggles, "experimental_lead_boost_gain", EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_GAIN_DEFAULT),
        getattr(frogpilot_toggles, "experimental_no_lead_boost_gain", EXPERIMENTAL_FREE_ROAD_NO_LEAD_BOOST_GAIN_DEFAULT),
        brake_cutoff=getattr(frogpilot_toggles, "experimental_boost_brake_cutoff", EXPERIMENTAL_FREE_ROAD_BRAKE_CUTOFF_DEFAULT),
        personality=sm['selfdriveState'].personality,
      )
      output_a_target = get_experimental_boosted_accel(experimental_base_a_target, output_a_target_acc, self.experimental_free_road_boost)
      output_a_target = apply_experimental_force_coast_cap(output_a_target, output_a_target_acc, sm['frogpilotCarState'].forceCoast)
      if is_santa_fe_hev_2022(self.CP):
        decel_lead_feedforward_eligible = not reset_state
        decel_lead = sm['radarState'].leadOne
        self.decel_lead_feedforward_track_id, self.decel_lead_feedforward_alk_window, self.decel_lead_feedforward_authority = \
          advance_santa_fe_experimental_decelerating_lead_feedforward_lane(
            decel_lead_feedforward_eligible, self.decel_lead_feedforward_track_id,
            self.decel_lead_feedforward_alk_window, self.decel_lead_feedforward_authority, v_ego, decel_lead)
        if self.decel_lead_feedforward_authority < -1e-3:
          output_a_target = min(output_a_target, self.decel_lead_feedforward_authority)
        output_a_target = apply_santa_fe_experimental_decelerating_lead_approach_cap(output_a_target, v_ego, decel_lead)
        output_a_target = apply_santa_fe_experimental_lead_caution(output_a_target, v_ego, sm['radarState'].leadOne)
        output_a_target = apply_santa_fe_slowing_lead_smooth_approach_cap(
          output_a_target,
          v_ego,
          sm['radarState'].leadOne,
          increased_stopped_distance=sm['frogpilotPlan'].increasedStoppedDistance,
        )
        output_a_target = apply_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(
          output_a_target,
          v_ego,
          sm['radarState'].leadOne,
          accel_coast,
          increased_stopped_distance=sm['frogpilotPlan'].increasedStoppedDistance,
        )
        output_a_target = apply_santa_fe_stopped_lead_smooth_approach_cap(
          output_a_target,
          v_ego,
          sm['radarState'].leadOne,
          increased_stopped_distance=sm['frogpilotPlan'].increasedStoppedDistance,
        )
        # santa_fe_stopping_lead_roll_in (FLOOR / max-raise) -- the MIRROR of the smooth-approach
        # cap above, and the LAST Santa Fe quirk so it sees the fully-capped command. Latch the
        # floor OFF on a lead hard-stop (single aLeadK frame is transient -> dwell timer on self),
        # then gate the floor OFF on force-coast, during the latch dwell, and whenever longcontrol
        # is (or is about to be) in the stopping state -- where the seg24 anti-collision net reuses
        # aTarget as min(output_accel, a_target). output_should_stop ALONE is insufficient: longcontrol
        # enters stopping via (should_stop OR should_enter_stop_target_mode(v_ego, a_target, dts)), so
        # there is a window where the seg24 net is LIVE but output_should_stop is still False. Gate on
        # longcontrol's EXACT stopping-entry condition, evaluated on the PRE-floor output_a_target (the
        # a_target longcontrol will actually test), so a raised aTarget can never weaken the committed
        # stop -- the floor acts ONLY during the rolling approach.
        if santa_fe_stopping_lead_roll_in_latch_triggered(v_ego, sm['radarState'].leadOne):
          self.santa_fe_stopping_lead_roll_in_latch_s = SANTA_FE_STOPPING_LEAD_ROLL_IN_LATCH_DWELL_S
        else:
          self.santa_fe_stopping_lead_roll_in_latch_s = max(0.0, self.santa_fe_stopping_lead_roll_in_latch_s - self.dt)
        # Gate the floor OFF whenever longcontrol is (or could be) in the stopping state, so a raised
        # aTarget can never weaken the seg24 anti-collision net. The floor's far-approach band is
        # disjoint from longcontrol's stopping band -- it acts only in pid-mode follow, before any stop
        # commitment -- and this gate enforces that boundary by mirroring longcontrol's FULL stopping
        # condition (longcontrol.py:485-487): should_stop OR should_enter OR should_hold (the
        # persistence term), evaluated on the PRE-floor aTarget. It also mirrors the arbiter's synthetic
        # stopped-lead control target (stop_target_arbiter.py:519-537): when active the arbiter min-merges
        # it AND drives the stopped-lead stop, so longcontrol enters stopping on a target the planner's
        # raw distance_to_stop_target_m does not see -- defer the close stopped-lead closure to the
        # arbiter; the floor owns only the far approach beyond it. The synthetic check uses the SAME
        # ISD-effective gap longcontrol feeds the arbiter (lead_d_rel_eff, longcontrol.py:712,729) so the
        # gate is convention-exact (the arbiter's synthetic activates at raw_dRel <= trigger_gap + ISD).
        roll_in_lead = sm['radarState'].leadOne
        synthetic_stopped_lead_stop_active = False
        if roll_in_lead.status:
          roll_in_lead_v = max(float(getattr(roll_in_lead, "vLead", v_ego + float(getattr(roll_in_lead, "vRel", 0.0)))), 0.0)
          roll_in_lead_d_rel_eff = get_effective_lead_distance(float(roll_in_lead.dRel), float(sm['frogpilotPlan'].increasedStoppedDistance))
          synthetic_stopped_lead_stop_active = get_stopped_lead_control_target(v_ego, roll_in_lead_v, roll_in_lead_d_rel_eff) is not None
        longcontrol_entering_stop = (self.output_should_stop
                                     or synthetic_stopped_lead_stop_active
                                     or should_enter_stop_target_mode(v_ego, output_a_target, self.distance_to_stop_target_m)
                                     or should_hold_stop_target_mode(v_ego, output_a_target, self.distance_to_stop_target_m))
        if (not longcontrol_entering_stop and not sm['frogpilotCarState'].forceCoast
            and self.santa_fe_stopping_lead_roll_in_latch_s <= 0.0):
          output_a_target = apply_santa_fe_stopping_lead_roll_in(
            output_a_target,
            v_ego,
            sm['radarState'].leadOne,
            increased_stopped_distance=sm['frogpilotPlan'].increasedStoppedDistance,
          )
        # Stop-commitment necessity floor (00001f47 seg6 takeover) -- LAST writer so it sees the
        # fully-capped command and can only DEEPEN it. Persistence runs only on frames where the
        # lane is fully eligible (blended Santa Fe, flag on, no force-coast); any other frame
        # clears ALL lane state via the common-path reset below, so confirmation is always a
        # fresh contiguous 0.5 s. The floor itself is Schmitt-gated inside get_santa_fe_stop_commit_floor.
        stop_commit_lead = sm['radarState'].leadOne
        if stopping_flags.SANTA_FE_STOP_COMMIT_ENVELOPE and not sm['frogpilotCarState'].forceCoast:
          stop_commit_eligible = True
          stop_commit_lead_state_ok = santa_fe_stop_commit_lead_state_ok(v_ego, stop_commit_lead)
          self.stop_commit_track_certified = update_santa_fe_stop_commit_track_certificate(
            self.stop_commit_track_id,
            self.stop_commit_track_certified,
            v_ego,
            stop_commit_lead,
            stop_commit_lead_state_ok,
          )
          self.stop_commit_track_id, self.stop_commit_lead_frames, self.stop_commit_alk_window, self.stop_commit_vlead_window = \
            update_santa_fe_stop_commit_persistence(
              self.stop_commit_track_id,
              self.stop_commit_lead_frames,
              self.stop_commit_alk_window,
              stop_commit_lead_state_ok,
              int(getattr(stop_commit_lead, "radarTrackId", -1)),
              float(getattr(stop_commit_lead, "aLeadK", 0.0)),
              vlead_window=self.stop_commit_vlead_window,
              v_lead=float(getattr(stop_commit_lead, "vLead", 0.0)),
            )
          stop_commit_provenance_ok = santa_fe_stop_commit_track_provenance_ok(
            stop_commit_lead, sm['radarState'].leadTwo, self.stop_commit_track_certified)
          if self.stop_commit_lead_frames >= SANTA_FE_STOP_COMMIT_PERSIST_FRAMES and stop_commit_provenance_ok:
            stop_aim_floor, self.stop_aim_committed, stop_commit_floor, self.stop_commit_active = \
              get_santa_fe_stop_floor_demands(
                v_ego, stop_commit_lead, output_a_target, self.stop_commit_alk_window,
                self.stop_aim_committed, self.stop_commit_active,
                LEAD_STOP_DISTANCE_TARGET + float(sm['frogpilotPlan'].increasedStoppedDistance),
                stopping_flags.SANTA_FE_STOP_AIM_ENVELOPE,
                vlead_window=self.stop_commit_vlead_window)
            for lane_floor in (stop_aim_floor, stop_commit_floor):
              if lane_floor is not None:
                output_a_target = min(output_a_target, lane_floor)
          else:
            self.stop_commit_active = False
            self.stop_aim_committed = False
      if experimental_base_a_target < output_a_target_mpc and output_a_target <= experimental_base_a_target:
        self.mpc.source = SOURCES[3]

    if not decel_lead_feedforward_eligible:
      # HARD-CLEAR at the mode edge (end-review rounds 2+3): the shared lane-advance function is
      # the single production path -- see advance_santa_fe_experimental_decelerating_lead_feedforward_lane.
      self.decel_lead_feedforward_track_id, self.decel_lead_feedforward_alk_window, self.decel_lead_feedforward_authority = \
        advance_santa_fe_experimental_decelerating_lead_feedforward_lane(
          False, self.decel_lead_feedforward_track_id, self.decel_lead_feedforward_alk_window,
          self.decel_lead_feedforward_authority)

    if sm['frogpilotCarState'].forceCoast:
      force_coast_target_accel = get_force_coast_target_from_toggles(v_ego, frogpilot_toggles)
      output_a_target = apply_force_coast_strength_brake_limit(output_a_target, force_coast_target_accel, True, v_ego,
                                                               sm['radarState'].leadOne, self.output_should_stop,
                                                               output_a_target_e2e)

    if sm['frogpilotCarState'].forceCoast and sm['carState'].standstill:
      self.output_should_stop = True
      output_a_target = min(output_a_target, 0.0)

    if not stop_commit_eligible:
      # Lane not eligible this frame (acc mode, non-Santa-Fe, kill switch off, or force-coast):
      # drop all confirmation state so a later eligible frame starts a fresh contiguous 0.5 s.
      self.stop_commit_track_id = None
      self.stop_commit_lead_frames = 0
      self.stop_commit_alk_window = []
      self.stop_commit_vlead_window = []
      self.stop_commit_track_certified = False
      self.stop_commit_active = False
      self.stop_aim_committed = False

    min_accel_clip_step = 0.05
    if is_santa_fe_hev_2022(self.CP):
      min_accel_clip_step = get_santa_fe_downhill_queue_min_accel_clip_step(
        v_ego, sm['radarState'].leadOne, accel_coast, output_a_target, self.prev_accel_clip[0])
    accel_clip[0] = np.clip(accel_clip[0], self.prev_accel_clip[0] - min_accel_clip_step, self.prev_accel_clip[0] + 0.05)
    accel_clip[1] = np.clip(accel_clip[1], self.prev_accel_clip[1] - 0.05, self.prev_accel_clip[1] + 0.05)
    self.output_a_target_trajectory = float(np.clip(output_a_target_mpc, accel_clip[0], accel_clip[1]))
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

    # shouldStop falling-edge hold (§4.1) -- runs after the force-coast standstill override
    # above, so a forced stop always asserts through the hold (the hold is strictly additive
    # on the deassert side and cannot create stops).
    raw_should_stop = bool(self.output_should_stop)
    if stopping_flags.SHOULD_STOP_LOOKAHEAD_S > 0.0:
      lookahead_v = float(np.interp(action_t + stopping_flags.SHOULD_STOP_LOOKAHEAD_S, CONTROL_N_T_IDX, self.v_desired_trajectory))
      raw_should_stop = raw_should_stop or lookahead_v < frogpilot_toggles.vEgoStopping
    self.output_should_stop, self.should_stop_hold_timer_s = update_should_stop_falling_edge_hold(
      raw_should_stop,
      float(self.v_desired_trajectory[0]),
      float(self.output_a_target),
      frogpilot_toggles.vEgoStopping,
      self.should_stop_hold_timer_s,
      stopping_flags.SHOULD_STOP_FALLING_EDGE_HOLD_S,
      self.dt,
    )

  def publish(self, sm, pm, frogpilot_toggles):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['modelV2'].leadsV3[0].prob > frogpilot_toggles.lead_detection_probability
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.leadTrajectoryX0 = self.mpc.lead_xv_0[:, 0].tolist()
    longitudinalPlan.leadTrajectoryV0 = self.mpc.lead_xv_0[:, 1].tolist()
    longitudinalPlan.leadTrajectoryX1 = self.mpc.lead_xv_1[:, 0].tolist()
    longitudinalPlan.leadTrajectoryV1 = self.mpc.lead_xv_1[:, 1].tolist()

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.aTargetTrajectory = float(self.output_a_target_trajectory)
    longitudinalPlan.aTargetTrajectoryValid = True
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)
    longitudinalPlan.distanceToStopTarget = float(self.distance_to_stop_target_m)

    pm.send('longitudinalPlan', plan_send)
