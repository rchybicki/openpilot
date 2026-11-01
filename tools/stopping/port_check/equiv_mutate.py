#!/usr/bin/env python3
"""equiv_replay.py with one mutation applied, to prove the comparison sees a change in each stage. argv: mutation out.pkl tip_tree segs...
  mpc_floor   planner: the MPC lower bound = ACCEL_MIN instead of the plan minimum accel_clip[0] (upstream's solver floor)
  no_gat      LongControl: stopping_flags.GAP_AWARE_TERMINAL = False
  radar_delay radard: RadarD delay 0.15 s instead of the Hyundai CP.radarDelay
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
  else:
    raise ValueError(mutation)


if __name__ == "__main__":
  apply(sys.argv[1])
  equiv_replay.main(sys.argv[2], sys.argv[3], sys.argv[4:])
