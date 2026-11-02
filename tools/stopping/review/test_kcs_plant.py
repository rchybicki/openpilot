import math

import numpy as np
import pytest

from openpilot.tools.stopping.review.kcs_plant import Cell, Plant, cells, fit_gain
from openpilot.tools.stopping.review.plant_sim import metrics, reversals, summarize_metrics


def test_engaged_step_delay_lag_and_slew():
  p = Plant(5., 0., 0.)
  for _ in range(12):
    assert p.step(-1.) == 0.
  assert -.05 < p.step(-1.) < 0.
  for _ in range(90):
    p.step(-1.)
  assert p.a == pytest.approx(-.985, abs=.02)
  assert not p.off
  before = p.r
  p.step(-.5, upper=1.)
  assert p.r - before == pytest.approx(.01)
  before = p.r
  p.step(-.5, upper=3.)
  assert p.r - before == pytest.approx(.03)


def test_release_loss_and_rebuild():
  p = Plant(2.4, -.95, -1., Cell(trigger='level'))
  for _ in range(150):
    p.step(-.3)
  assert p.off and p.loss > .08
  loss = p.loss
  for _ in range(100):
    p.step(-.8)
  assert not p.off and p.loss < loss / 3


def test_grade_applies_engaged_and_off():
  flat = Plant(2., -.285, -.3, Cell(trigger='level'))
  hill = Plant(2., -.285, -.3, Cell(trigger='level', grade=1.))
  for _ in range(80):
    flat.step(-.3)
    hill.step(-.3)
  assert flat.off and hill.a < flat.a - .08
  engaged = [Plant(3., -.5, -.5, Cell(grade=g)) for g in (0., 1.)]
  for _ in range(40):
    for p in engaged:
      p.step(-.5)
  assert engaged[0].v - engaged[1].v == pytest.approx(.0981 * .4, abs=.002)


def test_observation_stop_delay_and_abs_tail():
  p = Plant(.3, -.7, -.7)
  while p.v > 0:
    p.step(-.7)
  assert not p.standstill and p.v_ego > .05
  for _ in range(21):
    p.step(-.7)
  assert not p.standstill
  p.step(-.7)
  assert p.standstill
  for _ in range(50):
    p.step(-.7)
  assert .02 < p.raw < .06
  assert p.v == 0 and not p.hold_unknown


def test_input_validation_and_grid():
  with pytest.raises(ValueError):
    Plant(float('nan'), 0, 0)
  with pytest.raises(ValueError):
    Plant(1, 0, 0).step(float('nan'))
  assert len(cells(True)) == 385
  assert len(set(cells(True))) == 385
  assert cells(True)['history_p0.35_off0.3_on0_g-0.07_s-1_d0.12'].grade == -1


def test_fit_has_no_heldout_dependency():
  rows = [dict(role='train', release=False, u=u, band=b, a_pul=u * .95, rep='21ef_A1_1')
          for u in (-1., -.8, -.5) for b in ('5.5-4', '4-2.5', '2.5-1.5', '1.5-0.5', '<0.5')]
  a, provenance = fit_gain(rows)
  b, _ = fit_gain(rows + [dict(r, role='test', a_pul=999., rep='21f0_A5_1') for r in rows])
  assert a == b and len(provenance) == 15
  with pytest.raises(ValueError, match='held-out'):
    fit_gain([dict(r, rep='21f0_A5_1') for r in rows])


def test_reversals_endpoints_plateau_and_noise():
  u = np.array([-.8, -.7, -.6, -.6, -.7, -.8, -.7, -.8])
  events = reversals(np.arange(len(u)) * .01, u)
  assert len(events) == 2 and events[0]['release'] == pytest.approx(.2)
  assert not reversals([0, 1, 2], [-.8, -.71, -.8])
  with pytest.raises(ValueError):
    reversals([0], [math.nan])


def test_metrics_retains_incomplete_and_creep():
  trace = dict(t=np.arange(6) * .01, v=[2.5, 1, .1, 0, .1, .6], a=[-.5] * 6,
               u=[-.8, -.6, -.8, -.7, -.7, -.7], gap=[5, 4.9, 4.8, 4.7, 4.7, 4.6], off=[0, 1, 1, 0, 0, 0])
  m = metrics(trace)
  assert m['creep'] and m['relaunch'] and not m['incomplete']
  assert m['jerk_proxy'] == 2.5 and m['rest_gap'] == 4.7
  assert m['command_reversals'] == 0  # rebuild is below .15; excluded
  trace['v'] = [2.5, 1, .5, .2, .2, .2]
  incomplete = metrics(trace)
  assert incomplete['incomplete'] and incomplete['rest_gap'] is None and incomplete['a_stop'] is None
  summary = summarize_metrics([m, incomplete])
  assert summary['n'] == 2 and summary['incomplete'] == 1 and summary['completed'] == 1


def test_missing_lead_gap_is_visible_not_an_incomplete_stop():
  row = metrics(dict(t=[0., .01, .02], v=[.1, 0., 0.], a=[-.5, 0., 0.],
                     u=[-.7] * 3, gap=[math.nan] * 3, off=[False] * 3))
  assert row['rest_gap'] is None and row['min_gap'] is None and not row['incomplete']
  summary = summarize_metrics([row])
  assert summary['completed'] == 1 and summary['rest_gap_count'] == 0


def test_onset_remembers_release_state():
  p = Plant(1.5, -.8, -.8, Cell(onset=.8))
  for _ in range(25):
    p.step(-.3, upper=1.)
  assert p.off and p.onset == .8 and p.loss == 0.
  for _ in range(20):
    p.step(-.3, upper=3.)
  assert p.loss == 0.  # changing state cannot retroactively remove the onset delay


def test_output_boundary():
  from openpilot.tools.stopping.review.plant_data import OUTPUT, output_path
  with pytest.raises(ValueError, match='run outputs'):
    output_path(OUTPUT.parent / 'outside.json')


def test_unknown_shallow_hold_does_not_force_zero_creep():
  p = Plant(0., 0., -.3, Cell(trigger='level', p_max=.6))
  for _ in range(200):
    p.step(-.3)
  assert p.hold_unknown and p.v > .05
  held = Plant(0., 0., -.7, Cell(trigger='level', p_max=.6))
  for _ in range(200):
    held.step(-.7)
  assert held.v == 0 and not held.hold_unknown


def test_stopreq_and_deep_command_do_not_latch_acceleration():
  # Deliberately extreme downhill tests absence of a latch, not calibrated breakaway.
  p = Plant(0., 0., -.7, grade=-10.)
  for _ in range(100):
    p.step(-.7, stop_req=True)
  assert p.v > 0 and p.a > 0
  row = metrics(dict(t=[0., .01], v=[.1, 0.], a=[-.7, 0.], u=[-.7, -.7], gap=[4., 4.], off=[False, False]))
  assert row['stationary_validation'].startswith('UNVALIDATED')


def test_free_warmup_retains_regime_and_motion_without_crossing_reseed():
  from openpilot.tools.stopping.review.plant_sim import free_roll
  t = np.arange(0., 5., .01)
  u = np.where(t < .5, -.8, -.3)
  e = dict(id='warmup', split='test', truth='synthetic', t=t, v=np.full(len(t), 2.5), a=np.full(len(t), -.8),
           send_t=t, send_u=u, jerk_t=t, jerk_up=np.full(len(t), 3.), jerk_lo=np.full(len(t), 5.),
           stop_req=np.zeros(len(t)), cross=2.1, rest=4., grade=0.)
  trace = {}
  free_roll(e, Cell(), Plant(1., 0., 0.).gain, trace)
  p = Plant(2.5, -.8, -.8)
  for i in range(211):
    p.step(float(u[i]))
  assert trace['v'][210] == pytest.approx(p.v)
  assert trace['off'][210] == p.off and p.off
  assert abs(trace['v'][210] - 2.5) > .3
  with pytest.raises(ValueError, match='warm-up'):
    free_roll(dict(e, cross=1.99), Cell(), p.gain)


def test_pulse_cache_edge_uses_available_interval():
  from openpilot.tools.stopping.review.plant_data import attach_pulse_truth
  t = np.arange(0., 4., .01)
  # Encoded wheel counters advance .5 per .01 s, wrap every 128 decoded units.
  pulses = np.column_stack([t] + [(.5 * np.arange(len(t))) % 128] * 4)
  e = dict(id='edge', t=t, cross=2.1)
  inputs = {'edge': dict(pulses=pulses.tolist(), frames=[dict(t=3., cs=dict(standstill=True))])}
  scale = attach_pulse_truth([e], inputs)
  assert e['v'][0] == pytest.approx(scale / .01)
  assert e['v'][0] == pytest.approx(e['v'][100])


def test_output_rejects_first_run_names():
  from openpilot.tools.stopping.review.plant_data import OUTPUT, output_path
  with pytest.raises(ValueError, match='v2_'):
    output_path(OUTPUT / 'all_nominal.json')


def test_census_sorts_publications_and_retains_rejected_stops(monkeypatch):
  from pathlib import Path
  import zstandard
  from cereal import log
  from opendbc.can import CANPacker
  from openpilot.tools.stopping.review.plant_data import census_route

  packer = CANPacker('hyundai_kia_generic')
  events = []
  for i in range(502):
    t = i * .01
    for kind in ('carState', 'carControl', 'radarState', 'sendcan'):
      e = log.Event.new_message(logMonoTime=int((10 + t) * 1e9), valid=True)
      if kind == 'carState':
        c = e.init(kind)
        c.vEgo, c.standstill, c.canValid = (3. if i < 220 else max(0., (500 - i) * .008)), i >= 500, True
      elif kind == 'carControl':
        c = e.init(kind)
        c.enabled = c.longActive = True
      elif kind == 'radarState':
        l = e.init(kind).leadOne
        l.status, l.vLead, l.dRel = True, 0., 8.
      else:
        msg = packer.make_can_msg('SCC12', 0, dict(aReqValue=-.5, ACCMode=1))
        c = e.init(kind, 1)[0]
        c.address, c.dat, c.src = msg
      events.append(e.to_bytes())
  # Reverse each batch: carState is stored before/after unrelated future publications.
  shuffled = [e for i in range(0, len(events), 40) for e in reversed(events[i:i + 40])]
  raw = zstandard.ZstdCompressor().compress(b''.join(shuffled))
  monkeypatch.setattr(Path, 'read_bytes', lambda self: raw)
  result = census_route(['/local/00002031--test--0/rlog.zst'])
  assert result['eligible'] == 1 and not result['failures']
  # Invalid compressed input is visible, never counted as a route with no stops.
  monkeypatch.setattr(Path, 'read_bytes', lambda self: b'not zstd')
  result = census_route(['/local/00002031--test--0/rlog.zst'])
  assert len(result['failures']) == 1 and result['eligible'] == 0


def test_twin_event_counts_include_release_without_rebuild():
  release = reversals([0., .1, .2], [-.8, -.6, -.6], include_open=True)
  cycle = reversals([0., .1, .2], [-.8, -.6, -.8], include_open=True)
  assert len(release) == len(cycle) == 1
  assert release[0]['rebuild'] is None and cycle[0]['rebuild'] == .2
