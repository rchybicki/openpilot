from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx")

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LEAD_ACCEL_TAU, LEAD_T_IDXS_MODEL, LongitudinalMpc, T_IDXS


def make_mpc(v_ego):
  mpc = LongitudinalMpc.__new__(LongitudinalMpc)
  mpc.x0 = np.zeros(3)
  mpc.x0[1] = v_ego
  return mpc


def radar_lead(status=True, d_rel=40.0, v_lead=15.0, a_lead=-1.0, tau=LEAD_ACCEL_TAU):
  return SimpleNamespace(status=status, dRel=d_rel, vLead=v_lead, aLeadK=a_lead, aLeadTau=tau)


def model_lead(prob=0.9, v0=15.0, drop=4.0):
  v = [v0 - drop * i / (len(LEAD_T_IDXS_MODEL) - 1) for i in range(len(LEAD_T_IDXS_MODEL))]
  x = list(np.cumsum([0.0] + [v[i] * 2.0 for i in range(len(v) - 1)]))
  return SimpleNamespace(prob=prob, x=x, v=v)


def test_default_path_extrapolates_radar_with_decaying_acceleration():
  mpc = make_mpc(20.0)
  toggles = SimpleNamespace(human_following=False, lead_detection_probability=0.35)
  xv = mpc.process_lead(model_lead(), radar_lead(v_lead=15.0, a_lead=-1.0), toggles)
  assert xv.shape == (len(T_IDXS), 2)
  assert xv[0, 0] == pytest.approx(40.0)
  assert xv[0, 1] == pytest.approx(15.0)
  # decelerating lead: speed falls, then the decay flattens it; never negative
  assert xv[5, 1] < 15.0
  assert np.all(xv[:, 1] >= 0.0)
  assert np.all(np.diff(xv[:, 0]) >= 0.0)


def test_default_path_ignores_model_trajectory():
  mpc = make_mpc(20.0)
  toggles = SimpleNamespace(human_following=False, lead_detection_probability=0.35)
  steady = mpc.process_lead(model_lead(drop=0.0), radar_lead(a_lead=0.0), toggles)
  braking_model = mpc.process_lead(model_lead(drop=10.0), radar_lead(a_lead=0.0), toggles)
  assert np.allclose(steady, braking_model)


def test_human_following_uses_model_trajectory_anchored_to_radar():
  mpc = make_mpc(20.0)
  toggles = SimpleNamespace(human_following=True, lead_detection_probability=0.35)
  xv = mpc.process_lead(model_lead(v0=15.0, drop=4.0), radar_lead(v_lead=15.0, a_lead=0.0), toggles)
  assert xv[0, 0] == pytest.approx(40.0)
  assert xv[0, 1] == pytest.approx(15.0)
  assert xv[-1, 1] == pytest.approx(11.0, abs=0.5)


def test_traffic_mode_forces_model_trajectory():
  mpc = make_mpc(20.0)
  toggles = SimpleNamespace(human_following=False, lead_detection_probability=0.35)
  radar_only = mpc.process_lead(model_lead(drop=4.0), radar_lead(a_lead=0.0), toggles)
  traffic = mpc.process_lead(model_lead(drop=4.0), radar_lead(a_lead=0.0), toggles, traffic_mode_active=True)
  assert not np.allclose(radar_only, traffic)
  assert traffic[-1, 1] < radar_only[-1, 1]


def test_no_lead_fakes_a_fast_lead_on_both_paths():
  mpc = make_mpc(20.0)
  for human_following in (False, True):
    toggles = SimpleNamespace(human_following=human_following, lead_detection_probability=0.35)
    xv = mpc.process_lead(model_lead(prob=0.0), radar_lead(status=False), toggles)
    assert xv[0, 0] == pytest.approx(50.0)
    assert np.all(xv[:, 1] == pytest.approx(30.0))


def test_immediate_crash_distance_is_clipped():
  mpc = make_mpc(30.0)
  toggles = SimpleNamespace(human_following=False, lead_detection_probability=0.35)
  xv = mpc.process_lead(model_lead(), radar_lead(d_rel=1.0, v_lead=0.0, a_lead=0.0), toggles)
  # ((30+0)/2)*(30-0)/(3.5*2) = 64.3 m
  assert xv[0, 0] == pytest.approx(64.29, abs=0.1)
