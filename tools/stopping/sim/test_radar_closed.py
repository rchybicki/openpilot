"""Closed-loop radar path (T4) and the F1 following matrix (T5): the closed loop runs the tree's production RadarD publication path
with the tree's CP.radarDelay, and F1 scores physical truth. Needs the SIM_HOME case cache (the synthetic donor is case s20). The sim
checks run in a subprocess without the conftest OPENPILOT_PREFIX (the macOS ZMQ backend does not support it)."""
import os
import subprocess
import sys

import numpy as np
import pytest

from openpilot.tools.stopping.sim import SIM_HOME
from openpilot.tools.stopping.sim import gates as G

needs_cache = pytest.mark.skipif(not (SIM_HOME / 'cases' / 's20.pkl').is_file(), reason='needs SIM_HOME case cache (README.md)')


def _sub(check):
  env = {k: v for k, v in os.environ.items() if k != 'OPENPILOT_PREFIX'}
  r = subprocess.run([sys.executable, '-c', f'from openpilot.tools.stopping.sim import test_radar_closed as T; T.{check}()'], env=env,
                     capture_output=True, text=True)
  assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-3000:]


def _run(case, variant=None, **opts):
  from openpilot.tools.stopping.sim import rharness as R
  from openpilot.tools.stopping.review.kcs_plant import Cell
  return R.run(case, variant=variant, cell=Cell(trigger='level', threshold=-0.42, gain_delta=-0.035), start='auto', frac=(0.5, 0.3),
               creep=dict(off_grade=0.0), cruise_standstill='car', standstill='gate', **opts)


def _bump_published_vlead():
  """A publication-only radard change: RadarD.update publishes leadOne.vLead + 0.5 (tracks, filters and selection unchanged)."""
  import contextlib
  from openpilot.selfdrive.controls import radard
  from openpilot.tools.stopping.sim import harness as H
  base = radard.RadarD.update

  def update(self, sm, rr):
    base(self, sm, rr)
    if self.radar_state.leadOne.status:
      self.radar_state.leadOne.vLead += 0.5

  @contextlib.contextmanager
  def ctx():
    with H.patched((radard.RadarD, 'update', update)):
      yield
  return ctx


@needs_cache
def test_publication_only_radard_change_moves_the_published_speed_and_the_command():
  _sub('check_publication_only_radard_change_moves_the_published_speed_and_the_command')


def check_publication_only_radard_change_moves_the_published_speed_and_the_command():
  from openpilot.tools.stopping.sim import f1 as F1
  base = _run(F1.case('f1hb_v25_a3', 'K'))
  bump = _run(F1.case('f1hb_v25_a3', 'K'), variant=_bump_published_vlead())
  a, b = base['trace'], bump['trace']
  on = a['lead_status'].astype(bool) & (a['t'] > 1.0)
  assert np.allclose((b['lead_v'] - a['lead_v'])[on & (np.abs(b['lead_v'] - a['lead_v']) < 1.0)], 0.5, atol=0.15)   # the closed loop moves both
  assert np.nanmax(np.abs(b['wire'] - a['wire'])) > 0.1                 # the planner / LongControl see it: the command changes
  assert np.allclose(np.nan_to_num(b['vl_true'][:200]), np.nan_to_num(a['vl_true'][:200]))   # truth is not the published speed


@needs_cache
def test_f1_runs_the_production_frogpilot_following():
  _sub('check_f1_runs_the_production_frogpilot_following')


def check_f1_runs_the_production_frogpilot_following():
  """Astra tooling review finding 1: F1 runs the tree's FrogPilotPlanner on the simulated state, so a patched FrogPilotFollowing.update
  (t_follow 0.1) is called every model tick and changes the command and the physical gap."""
  import contextlib
  from openpilot.frogpilot.controls.lib.frogpilot_following import FrogPilotFollowing
  from openpilot.tools.stopping.sim import f1 as F1
  from openpilot.tools.stopping.sim import harness as H
  base_update, calls = FrogPilotFollowing.update, [0]

  def update(self, *a, **k):
    base_update(self, *a, **k)
    calls[0] += 1
    self.t_follow = 0.1

  @contextlib.contextmanager
  def short_follow():
    with H.patched((FrogPilotFollowing, 'update', update)):
      yield
  a = _run(F1.case('f1hb_v25_a3', 'K'))['trace']
  b = _run(F1.case('f1hb_v25_a3', 'K'), variant=short_follow)['trace']
  assert calls[0] > 200                                       # 20 Hz over the 16 s case
  assert np.nanmax(np.abs(b['wire'] - a['wire'])) > 0.3
  assert np.nanmin(b['gap']) < np.nanmin(a['gap']) - 1.0     # the short follow time closes in on the braking lead


@needs_cache
def test_radar_delay_comes_from_the_tree_and_changes_the_published_speed():
  _sub('check_radar_delay_comes_from_the_tree_and_changes_the_published_speed')


def check_radar_delay_comes_from_the_tree_and_changes_the_published_speed():
  """CP.radarDelay is read from the tree's CarInterface (a candidate that sets it is seen without a harness option); the delay-only
  publication reads a braking lead faster than HEAD (vLead = vRel + the vEgo of 0.15 s ago)."""
  import contextlib
  from opendbc.car.hyundai.interface import CarInterface
  from openpilot.tools.stopping.sim import f1 as F1
  from openpilot.tools.stopping.sim import harness as H
  from openpilot.tools.stopping.sim import rharness as R
  base = _run(F1.case('f1hb_v25_a4', 'K'))
  assert base['info']['radar_delay'] == R.tree_radar_delay(R._donor()['case']['cp']) == 0.0
  orig = CarInterface.get_non_essential_params.__func__

  def gnep(cls, candidate):
    ret = orig(cls, candidate)
    ret.radarDelay = 0.15
    return ret

  @contextlib.contextmanager
  def delay():
    with H.patched((CarInterface, 'get_non_essential_params', classmethod(gnep))):
      yield
  cand = _run(F1.case('f1hb_v25_a4', 'K'), variant=delay)
  assert abs(cand['info']['radar_delay'] - 0.15) < 1e-6
  a, b = base['trace'], cand['trace']
  brake = (a['t'] > 3.3) & (a['a_ego'] < -2.0)   # the lead brakes at -4 from 3.0 s; the error moves by -0.15 x aEgo while the ego brakes
  assert brake.sum() > 100
  err_a, err_b = (a['lead_v'] - a['vl_true'])[brake], (b['lead_v'] - b['vl_true'])[brake]
  assert np.nanmean(err_b) > np.nanmean(err_a) + 0.3


@needs_cache
def test_recorded_case_reruns_radard_on_the_logged_inputs_before_the_takeover():
  _sub('check_recorded_case_reruns_radard_on_the_logged_inputs_before_the_takeover')


def check_recorded_case_reruns_radard_on_the_logged_inputs_before_the_takeover():
  """Before the takeover the re-run (production RadarD on the logged liveTracks / carState / modelV2 / frogpilotPlan, picked by the
  exact replay's rule, warm from radar_replay.RADAR_WARM s before the window) publishes the logged leadOne: same status and track on
  >= 98 % of the radar frames, the same vLead (vRel + vEgo) and, warm, the same Kalman state (vLeadK; cold it was 3.3 m/s off)."""
  from openpilot.tools.stopping.sim import radar_replay as RR
  r = _run('2235_s71')
  f = r['info']['radar_fid']
  assert r['info']['radar_warm'] >= RR.RADAR_WARM
  assert f['n'] > 1000
  assert f['status'] >= 0.98 * f['n'] and f['track'] >= 0.98 * f['lead'] > 0
  assert f['vl_max'] < 1e-3 and f['vlk_max'] < 1e-3


def test_v_mean_removes_the_pulse_ripple_and_keeps_the_speed():
  from openpilot.tools.stopping.sim import rharness as R
  t = np.arange(0, 5, 0.01)
  v = 6.0 - 0.8 * t
  m = R.v_mean(t, v + np.where(np.arange(len(t)) % 2, 0.3, -0.3))   # +-0.3 m/s alternating every 10 ms (the 2086_s17 pulses)
  assert np.max(np.abs(m - v)[5:-5]) < 0.03   # the 11-frame mean leaves 0.3 / 11 of an odd-frame ripple
  assert np.allclose(R.v_mean(t, v)[5:-5], v[5:-5])   # a ramp is unchanged inside the window


@needs_cache
def test_recorded_track_shift_does_not_carry_the_pulse_ripple():
  _sub('check_recorded_track_shift_does_not_carry_the_pulse_ripple')


def check_recorded_track_shift_does_not_carry_the_pulse_ripple():
  """After the takeover the closed loop moves the logged tracks by plant speed - the 0.1 s mean of the pulse truth: on 2086_s17 the
  published leadOne speed no longer steps by the ripple (raw truth: max 0.255 m/s per frame, p99 0.101; smoothed 0.149 / 0.072)."""
  r = _run('2086_s17')
  tr = r['trace']
  on = tr['closed'].astype(bool) & tr['lead_status'].astype(bool)
  step = np.abs(np.diff(tr['lead_v'][on]))
  assert on.sum() > 1000 and step.max() < 0.2 and np.percentile(step, 99) < 0.085


@needs_cache
def test_recorded_case_radard_warms_up_before_a_late_cached_segment_list():
  _sub('check_recorded_case_radard_warms_up_before_a_late_cached_segment_list')


def check_recorded_case_radard_warms_up_before_a_late_cached_segment_list():
  """s5's cached case paths start one segment after radard's warm-up: the stream reads the route's segments of its whole range."""
  from openpilot.tools.stopping.sim import harness as H
  from openpilot.tools.stopping.sim import radar_replay as RR
  from openpilot.tools.stopping.sim import rharness as R
  c = H.case('s5')
  assert c['paths'][0].endswith('--4/rlog.zst')   # the cache this test guards against (window from 241 s, warm-up from 228 s)
  E = R._stream(c)
  assert c['lo'] - E['t_rs'][0] >= RR.RADAR_WARM


@needs_cache
def test_synthetic_radar_reads_obs_scale_like_vego():
  _sub('check_synthetic_radar_reads_obs_scale_like_vego')


def check_synthetic_radar_reads_obs_scale_like_vego():
  """The synthetic radar's range and Doppler read OBS_SCALE x the pulse truth (10-02 verify_impact: 1.010), as carState vEgo does: the
  published vLead (vRel + vEgo) of a steady lead reads OBS_SCALE x its speed and dRel OBS_SCALE x the gap (0.1 m steps)."""
  from openpilot.tools.stopping.sim import f1 as F1
  from openpilot.tools.stopping.sim import rharness as R
  tr = _run(F1.case('f1hb_v25_a3', 'K'))['trace']
  steady = tr['lead_status'].astype(bool) & (tr['t'] > 1.0) & (tr['t'] < 2.9)   # the lead brakes from 3.0 s
  assert steady.sum() > 100
  assert abs(np.median(tr['lead_v'][steady] / tr['vl_true'][steady]) - R.OBS_SCALE) < 0.002
  assert abs(np.median(tr['gap_meas'][steady] - R.OBS_SCALE * tr['gap_true'][steady])) < 0.1


@needs_cache
def test_f1_measures_use_the_physical_gap_and_the_sent_demand():
  _sub('check_f1_measures_use_the_physical_gap_and_the_sent_demand')


def check_f1_measures_use_the_physical_gap_and_the_sent_demand():
  from openpilot.tools.stopping.sim import f1 as F1
  c = F1.case('f1hb_v20_a3_stop', 'A')
  r = _run(c)
  m = F1.measures(r['trace'], c)
  assert not m['collision'] and m['min_gap'] > 0.5   # HEAD with the production FrogPilot jerk costs: ~1.3 m (ACC, -3 to rest from 20 m/s)
  assert m['onset'] is not None and 0.0 <= m['onset'] < 1.5
  assert m['min_sent'] < -2.5


def _f1row(case, cell, gap, ttc, onset, sent, release=None, closing=3.0):
  t = np.arange(0, 10, 0.02, dtype=np.float32)
  return dict(case=case, cell=cell, start='auto', mode='drv', f1=dict(min_gap=gap, min_ttc=ttc, max_closing=closing, collision=gap <= 0, onset=onset,
              release=release, min_sent=float(np.min(sent)), t=t, sent=np.asarray(sent, dtype=np.float32)))


def test_f1_gate_absolute_floor_and_relative_limits():
  sent_h = np.where(np.arange(500) > 50, -3.0, 0.0)
  h = _f1row('f1hb_v25_a3', 'F1K', 12.0, 4.0, 1.0, sent_h, release=5.0)
  same = G.f1_pair(h, _f1row('f1hb_v25_a3', 'F1K', 12.0, 4.0, 1.0, sent_h, release=5.0))
  assert same['fails'] == []
  later = G.f1_pair(h, _f1row('f1hb_v25_a3', 'F1K', 12.0, 4.0, 1.0 + G.F1_ONSET + 0.05, sent_h, release=5.0))
  assert 'onset' in later['fails']
  closer = G.f1_pair(h, _f1row('f1hb_v25_a3', 'F1K', 12.0 - max(G.F1_GAP_M, G.F1_GAP_FRAC * 12.0) - 0.1, 4.0, 1.0, sent_h, release=5.0))
  assert 'min_gap' in closer['fails']
  less = G.f1_pair(h, _f1row('f1hb_v25_a3', 'F1K', 12.0, 4.0, 1.0, np.where(np.arange(500) > 50, -2.8, 0.0), release=5.0))
  assert 'deficit' in less['fails']   # 0.2 m/s^2 less for 9 s = 1.8 m/s of braking missing
  floor = G.f1_pair(_f1row('x', 'F1K', 3.0, 4.0, 1.0, sent_h), _f1row('x', 'F1K', G.F1_FLOOR_GAP - 0.1, 4.0, 1.0, sent_h))
  assert 'floor_gap' in floor['fails']
  crash = G.f1_pair(_f1row('x', 'F1K', 1.0, 0.8, 1.0, sent_h), _f1row('x', 'F1K', -0.1, 0.5, 1.0, sent_h))
  assert 'collision' in crash['fails']
