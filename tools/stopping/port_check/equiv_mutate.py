#!/usr/bin/env python3
"""equiv_replay.py with one mutation applied, to prove the comparison sees a change in each stage. argv: mutation out.pkl tip_tree segs...
  mpc_floor   planner: the MPC lower bound = ACCEL_MIN instead of the plan minimum accel_clip[0] (upstream's solver floor)
  no_gat      LongControl: stopping_flags.GAP_AWARE_TERMINAL = False
  radar_delay radard: RadarD delay 0.15 s instead of the Hyundai CP.radarDelay
  new_eco_accel  FrogPilot planner: the new base's ECO acceleration table instead of ours
  no_lane_centering  controlsd: lane centering gain 0 (the stage needs EQUIV_STORE_JSON)
  v_cruise_from_ego  card: the first engagement ignores InitialSetSpeed (the stage needs EQUIV_STORE_JSON)
  aol_cruise_enabled  card: always on lateral follows cruise engaged instead of the main switch (the stage needs EQUIV_STORE_JSON)
"""
import sys
from types import SimpleNamespace

import equiv_replay


def apply(mutation):
  if mutation == 'mpc_floor':
    from opendbc.car.interfaces import ACCEL_MIN
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
    orig = LongitudinalMpc.update

    def update(self, *args, **kw):
      return orig(self, *args[:9], ACCEL_MIN, *args[10:], **kw)
    LongitudinalMpc.update = update
  elif mutation == 'no_gat':
    from openpilot.selfdrive.controls.lib import stopping_flags
    stopping_flags.GAP_AWARE_TERMINAL = False
  elif mutation == 'radar_delay':
    from openpilot.tools.stopping.sim import radar_replay
    radar_replay.radar_delay = lambda fingerprint: 0.15
  elif mutation == 'new_eco_accel':
    from openpilot.frogpilot.controls.lib import frogpilot_acceleration
    frogpilot_acceleration.A_CRUISE_MAX_VALS_ECO = [1.4, 1.3, 1.0, 0.7, 0.6, 0.5, 0.4]
  elif mutation == 'no_lane_centering':
    from openpilot.selfdrive.controls.lib import lane_centering
    lane_centering.GAIN = 0.0
  elif mutation == 'v_cruise_from_ego':
    from openpilot.selfdrive.car.cruise import VCruiseHelper
    initialize = VCruiseHelper.initialize_v_cruise

    def initialize_v_cruise(self, CS, experimental_mode, resume_prev_button, frogpilot_toggles):
      initialize(self, CS, experimental_mode, resume_prev_button, SimpleNamespace(**{**vars(frogpilot_toggles), 'initial_set_speed': 0}))
    VCruiseHelper.initialize_v_cruise = initialize_v_cruise
  elif mutation == 'aol_cruise_enabled':
    from openpilot.frogpilot.controls.frogpilot_card import FrogPilotCard
    card_update = FrogPilotCard.update

    def update(self, carState, frogpilotCarState, sm, frogpilot_toggles):
      return card_update(self, carState, frogpilotCarState, sm, SimpleNamespace(**{**vars(frogpilot_toggles), 'always_on_lateral_main': False}))
    FrogPilotCard.update = update
  else:
    raise ValueError(mutation)


if __name__ == "__main__":
  apply(sys.argv[1])
  equiv_replay.main(sys.argv[2], sys.argv[3], sys.argv[4:])
