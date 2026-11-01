import numpy as np
import pytest

import cereal.messaging as messaging
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.car.interfaces import ACCEL_MIN
from openpilot.frogpilot.common.frogpilot_variables import get_frogpilot_toggles
from openpilot.selfdrive.controls.lib import longitudinal_planner
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc
from openpilot.selfdrive.modeld.constants import ModelConstants

PLAN_MIN_ACCEL = -1.0


class FakeSM(dict):
  def __init__(self, msgs):
    super().__init__({k: getattr(m, k) for k, m in msgs.items()})
    self.updated = dict.fromkeys(msgs, True)
    self.valid = dict.fromkeys(msgs, True)
    self.alive = dict.fromkeys(msgs, True)
    self.logMonoTime = dict.fromkeys(msgs, 0)
    self.frame = 0

  def all_checks(self, service_list=None):
    return True


def planner_inputs(v_ego=15.0, lead_d_rel=40.0, lead_v=14.0):
  msgs = {s: messaging.new_message(s) for s in ('carState', 'controlsState', 'selfdriveState', 'radarState', 'modelV2', 'carControl',
                                                'liveParameters', 'frogpilotPlan', 'frogpilotCarState')}
  msgs['carState'].carState.vEgo = v_ego
  msgs['carState'].carState.vCruise = 100.
  msgs['selfdriveState'].selfdriveState.enabled = True
  msgs['carControl'].carControl.longActive = True
  model = msgs['modelV2'].modelV2
  t = np.array(ModelConstants.T_IDXS)
  model.position.x = (v_ego * t).tolist()
  model.velocity.x = [v_ego] * ModelConstants.IDX_N
  model.acceleration.x = [0.0] * ModelConstants.IDX_N
  for model_lead in model.init('leadsV3', 3):
    model_lead.prob = 1.0
    model_lead.x = [lead_d_rel + lead_v * lt for lt in ModelConstants.LEAD_T_IDXS]
    model_lead.v = [lead_v] * len(ModelConstants.LEAD_T_IDXS)
    model_lead.a = [0.0] * len(ModelConstants.LEAD_T_IDXS)
  lead = msgs['radarState'].radarState.leadOne
  lead.status, lead.dRel, lead.vLead, lead.vLeadK, lead.modelProb, lead.radarTrackId = True, lead_d_rel, lead_v, lead_v, 1.0, 7
  plan = msgs['frogpilotPlan'].frogpilotPlan
  plan.minAcceleration, plan.maxAcceleration, plan.tFollow, plan.dangerFactor = PLAN_MIN_ACCEL, 2.0, 1.45, long_mpc.LEAD_DANGER_FACTOR
  plan.accelerationJerk, plan.dangerJerk, plan.speedJerk = long_mpc.A_CHANGE_COST, long_mpc.DANGER_ZONE_COST, long_mpc.J_EGO_COST
  plan.vCruise = 100. / 3.6
  return FakeSM(msgs)


def test_cruise_maximum_is_todays():
  assert long_mpc.CRUISE_MAX_ACCEL == 2.0
  assert longitudinal_planner.A_CRUISE_MAX_VALS == [2.0, 1.6, 0.8, 0.6]
  assert longitudinal_planner.A_CRUISE_MAX_BP == [0., 10.0, 25., 40.]
  assert [longitudinal_planner.get_max_accel(v) for v in (0., 10., 25., 40.)] == [2.0, 1.6, 0.8, 0.6]


@pytest.mark.parametrize("lead", [False, True])
def test_mpc_lower_bound_is_the_plan_minimum_not_accel_min(lead):
  CP = CarInterface.get_non_essential_params(CAR.HYUNDAI_SANTA_FE_HEV_2022)
  planner = longitudinal_planner.LongitudinalPlanner(CP, init_v=15.0)
  sm = planner_inputs()
  sm['radarState'].leadOne.status = lead

  planner.update(sm, get_frogpilot_toggles())

  assert planner.mpc.params[:, 0] == pytest.approx(PLAN_MIN_ACCEL)
  assert PLAN_MIN_ACCEL != ACCEL_MIN


@pytest.mark.parametrize("standstill", [False, True])
def test_upstream_force_stop_cannot_take_planner_ownership(standstill):
  CP = CarInterface.get_non_essential_params(CAR.HYUNDAI_SANTA_FE_HEV_2022)
  outputs = []
  for forcing_stop in (False, True):
    planner = longitudinal_planner.LongitudinalPlanner(CP, init_v=15.0)
    sm = planner_inputs()
    sm['carState'].standstill = standstill
    sm['frogpilotPlan'].forcingStop = forcing_stop
    sm['frogpilotPlan'].forcingStopLength = 5.0
    for _ in range(3):
      planner.update(sm, get_frogpilot_toggles())
    outputs.append((planner.output_a_target, planner.output_should_stop, planner.v_desired_trajectory.tolist(),
                    planner.a_desired_trajectory.tolist(), planner.mpc.params[:, 0].tolist()))
  assert outputs[0] == outputs[1]
