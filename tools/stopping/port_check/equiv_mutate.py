#!/usr/bin/env python3
"""equiv_replay.py with one mutation applied, to prove the comparison sees a change in each stage. argv: mutation out.pkl tip_tree segs...
  mpc_floor   planner: the MPC lower bound = ACCEL_MIN instead of the plan minimum accel_clip[0] (upstream's solver floor)
  no_gat      LongControl: stopping_flags.GAP_AWARE_TERMINAL = False
  radar_delay radard: RadarD delay 0.15 s instead of the Hyundai CP.radarDelay
  new_eco_accel  FrogPilot planner: the new base's ECO acceleration table instead of ours
  no_lane_centering  controlsd: lane centering gain 0 (the stage needs EQUIV_STORE_JSON)
"""
import sys

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
  else:
    raise ValueError(mutation)


if __name__ == "__main__":
  apply(sys.argv[1])
  equiv_replay.main(sys.argv[2], sys.argv[3], sys.argv[4:])
