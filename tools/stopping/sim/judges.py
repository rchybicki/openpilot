"""The judges' closed-loop grids as INFO stages of run_gates (never gating; README "Judges' grids"): the weak-plant stress grid
(cycle 2026-10-09 verify3-safety jstress.py, 288 runs), the adversarial grids with the SCC12 StopReq (adv.py / adv_lib.py: cutin, move,
noise, far, depart, sgs, flap, regap, push, crawl, spike, fault, gas) and Astra's fixed-time cut-in pair (astra_code
reproduce_cutin_fixed_time.py). run() is adv_lib.run; jstress is its grid 'stress' (no sender, 20 s, 1.5 s of rest), so a HEAD row
reproduces the judges' recorded numbers.

  STOP_SIM_TREE=<arm manifest.json> python -m openpilot.tools.stopping.sim.judges ARM_DIR JOBS.json

One job = one grid; rows go to ARM_DIR/judges.pkl (finished grids are reused). The arm key holds this engine's sha (this file + the
worker's common files), so a change here never touches the closed / replay rows. Plant (jstress / adv_lib): LongControl on a Santa Fe
HEV, 0.20 s pure delay + 0.25 s first-order lag, gain g, constant push p (grade / creep), a 2nd-gear creep push (+0.25 while 0.45 < v <
1.75 and the delayed wire > -0.50), radar at 20 Hz with 0.1 m gap quantisation and the stopped-lead Doppler bias 0.089 * a_ego, plus
optional radar latency. Event injection (cfg keys; trigger = the first frame v drops below the first element, or the frame of time
cfg['t_<event>'] when given):
  spike   = (v_trigger, lead_v_reading, n_radar_frames): the radar reports this lead speed for n frames
  fault   = v_trigger: the service raises from the trigger on (LongControl's fault path)
  gas     = (v_trigger, seconds, accel): driver gas override (freeze_integrator + gasPressed), plant follows accel, wire ignored
  regap   = (v_trigger, dgap, seconds): measured gap offset (a farther / closer target) for a window
  push    = (v_trigger, dp, seconds): extra plant push (grade) for a window
  cutin   = (v_trigger, dgap): a stopped car appears dgap closer (true geometry); cutin_new_track: the radar track id changes with it
  depart  = (mode, x, accel, vmax[, stop_after_s, decel]): lead departs ('stopped_s': x s after the ego stops; 'v': when ego v < x;
            't': at time x), accelerates at accel to vmax; optional: stops again stop_after_s later at decel (stop-go-stop)
  crawl   = lead_v constant from t=0 (lead_v may change at 'crawl_stop' = (t, decel)); lead = 'reverse': -0.3 m/s for 0.9 m from 1 s
  flap    = (v_trigger, lead_v_reading, n_frames, period): the lead-speed reading flaps; anoise = (sigma, phi, seed): AR(1) a_ego noise
Planner proxies: blind (aTarget -0.05, no stop), gentle (rest-3.5 kinematic demand capped at -0.35), kin (rest-4.3 kinematic demand
capped at -1.5, shouldStop), kin_go (kin + go: aTarget +0.8 and shouldStop False once the measured lead reads > 0.5 m/s and the gap
exceeds 5 m). StopReq from the real Hyundai CarController (SCC12) on the SENDER grids."""
import hashlib
import itertools
import json
import math
import multiprocessing as mp
import os
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from types import SimpleNamespace

from openpilot.tools.stopping.sim import loader

loader.install_from_env()
import numpy as np

from openpilot.tools.stopping.sim import worker as W

FILES = [Path(__file__).resolve()] + W.COMMON_FILES
ENGINE_SHA = W.engine_sha(FILES)[0]
DT = 0.01
NORMAL, WEAK = (0.9, 0.0), (0.7, 0.3)
ADV = ('sgs', 'move', 'far', 'crawl', 'flap', 'depart', 'noise', 'fault', 'spike', 'regap', 'cutin', 'gas', 'push')   # longest first (2 workers)
STAGES = {'stress': ('stress',), 'adv': ADV + ('cutfix',)}   # run_gates --stages name -> grids
SENDER = ('depart', 'far', 'sgs', 'crawl', 'move', 'flap', 'cutfix')   # grids with the Hyundai sender (StopReq), as adv.py
EVENTS = ('spike', 'fault', 'gas', 'regap', 'push', 'cutin')


def grid(name):
  """The configurations of one grid (adv.py / jstress.py order)."""
  G = []
  P = itertools.product
  if name == 'stress':   # jstress.py: 288 runs
    for v0, gap0, w0, plant, pl, lead in P((1.5, 2.45), (4.5, 5.5, 6.5, 8.0, 11.0, 15.0), (-0.1, -0.8), (NORMAL, WEAK), ('blind', 'gentle', 'kin'),
                                          ('stopped', 'reverse')):
      G.append(dict(v0=v0, gap0=gap0, wire0=w0, planner=pl, plant=plant, lead=lead, T=20.0, rest_frames=150))
  elif name == 'cutfix':   # Astra (2026-10-10): the stopped target appears 3 m nearer at the fixed time 1.32 s; same and new track id
    for new in (False, True):
      G.append(dict(v0=2.3, gap0=10.0, wire0=-0.4, planner='gentle', plant=NORMAL, cutin=(1.8, 3.0), t_cutin=1.32, cutin_new_track=new))
  elif name == 'spike':
    for v0, gap0, w0, pl, vt, plant in P((1.9, 2.3), (7.0, 8.0, 9.0, 10.0, 11.0), (-0.4, -0.7), ('gentle', 'kin'), (None, 1.75, 1.5, 1.0, 0.6),
                                         (NORMAL, WEAK)):
      c = dict(v0=v0, gap0=gap0, wire0=w0, planner=pl, plant=plant)
      if vt is not None:
        c['spike'] = (vt, 0.35, 1)
      G.append(c)
  elif name == 'fault':
    for v0, gap0, w0, pl, vt, plant in P((1.9, 2.3), (6.0, 7.0, 8.0, 9.0, 10.0, 11.0), (-0.4, -0.7), ('gentle', 'kin'), (1.75, 1.55, 1.35, 1.1, 0.6),
                                         (NORMAL, WEAK)):
      G.append(dict(v0=v0, gap0=gap0, wire0=w0, planner=pl, plant=plant, fault=vt))
  elif name == 'gas':
    for v0, gap0, w0, pl, ev, plant in P((1.9, 2.3), (6.0, 7.0, 8.0, 9.0, 10.0), (-0.4, -0.7), ('gentle', 'kin'),
                                         ((1.8, 0.4, 0.2), (1.5, 0.6, 0.0), (1.2, 0.5, 0.3), (0.8, 0.4, 0.0)), (NORMAL, WEAK)):
      G.append(dict(v0=v0, gap0=gap0, wire0=w0, planner=pl, plant=plant, gas=ev))
  elif name == 'depart':   # ego stops behind a stopped lead; the lead departs some time after the ego stops (launch timing)
    for v0, gap0, w0, (acc, vmax), after, plant in P((1.9, 2.3), (6.5, 8.0, 9.5, 11.0), (-0.4, -0.8), ((1.0, 3.0), (0.4, 1.2), (0.25, 0.6)), (0.5, 2.0),
                                                     (NORMAL, WEAK)):
      G.append(dict(v0=v0, gap0=gap0, wire0=w0, planner='kin_go', plant=plant, depart=('stopped_s', after, acc, vmax), T=30.0))
  elif name == 'sgs':   # the lead departs while the ego is still approaching, then stops again (stop-go-stop)
    for v0, gap0, w0, vt, (acc, vmax, st, dec), pl, plant in P((1.9, 2.3), (6.5, 8.0, 9.5), (-0.4, -0.8), (1.0, 0.6, 0.4),
                                                               ((1.0, 2.0, 1.5, 2.5), (0.5, 1.0, 1.0, 1.5), (0.8, 1.5, 2.5, 3.0)), ('kin_go', 'kin'),
                                                               (NORMAL, WEAK)):
      G.append(dict(v0=v0, gap0=gap0, wire0=w0, planner=pl, plant=plant, depart=('v', vt, acc, vmax, st, dec), T=30.0))
  elif name == 'regap':
    for v0, gap0, w0, ev, pl, plant in P((1.9, 2.3), (7.0, 8.0, 9.0, 10.0), (-0.4, -0.7),
                                         ((1.7, 5.0, 0.6), (1.4, 5.0, 1.5), (1.0, 4.0, 0.8), (0.7, 6.0, 1.0), (1.6, -1.5, 0.3), (1.0, -1.0, 0.3)),
                                         ('gentle', 'kin'), (NORMAL, WEAK)):
      G.append(dict(v0=v0, gap0=gap0, wire0=w0, planner=pl, plant=plant, regap=ev))
  elif name == 'far':
    for v0, gap0, w0, pl in P((1.8, 2.1, 2.45), tuple(np.arange(8.0, 15.01, 0.5)), (-0.3, -0.65, -1.0), ('blind', 'gentle', 'kin')):
      G.append(dict(v0=v0, gap0=float(gap0), wire0=w0, planner=pl, plant=NORMAL, rest_frames=500))
  elif name == 'crawl':
    for v0, gap0, w0, cv, pl, cst in P((1.5, 2.0, 2.4), (6.0, 8.0, 10.0), (-0.3, -0.8), (0.35, 0.45, 0.6, 0.9), ('gentle', 'kin'), (None, (3.0, 1.5))):
      c = dict(v0=v0, gap0=gap0, wire0=w0, planner=pl, plant=NORMAL, crawl=cv, T=20.0)
      if cst is not None:
        c['crawl_stop'] = cst
      G.append(c)
  elif name == 'push':   # a transient push (bump / grade / creep surge), and a long downhill push that ends ownership by speed
    for v0, gap0, w0, ev, pl, plant in P((1.9, 2.3), (6.5, 8.0, 9.5), (-0.4, -0.7),
                                         ((1.2, 0.3, 0.3), (0.9, 0.5, 0.3), (0.6, 0.4, 0.4), (1.6, 1.4, 1.5), (1.0, 1.2, 2.0)), ('gentle', 'kin'),
                                         (NORMAL, WEAK)):
      G.append(dict(v0=v0, gap0=gap0, wire0=w0, planner=pl, plant=plant, push=ev))
  elif name == 'cutin':   # a stopped car appears closer (lead change to a nearer stopped target, true geometry)
    for v0, gap0, w0, vt, off, pl, plant in P((1.9, 2.3), (8.0, 10.0, 12.0), (-0.4, -0.7), (1.8, 1.4, 1.0, 0.6), (1.5, 3.0), ('gentle', 'kin'),
                                              (NORMAL, WEAK)):
      G.append(dict(v0=v0, gap0=gap0, wire0=w0, planner=pl, plant=plant, cutin=(vt, off)))
  elif name == 'move':   # the stopped lead starts to move while the ego is still approaching
    for v0, gap0, w0, vt, (acc, vmax), pl, plant in P((1.9, 2.3), (6.5, 8.0, 9.5), (-0.4, -0.8), (1.75, 1.55, 1.35), ((0.3, 1.0), (0.6, 2.0), (1.0, 4.0)),
                                                      ('kin_go', 'kin', 'gentle'), (NORMAL, WEAK)):
      G.append(dict(v0=v0, gap0=gap0, wire0=w0, planner=pl, plant=plant, depart=('v', vt, acc, vmax), T=20.0))
  elif name == 'noise':   # a_ego noise (AR(1) phi 0.95; recorded sigma 0.023-0.034) on weak plants
    for i, (v0, gap0, w0, plant, pl, sig) in enumerate(P((1.5, 2.0, 2.45), (5.5, 6.5, 8.0), (-0.1, -0.8), ((0.6, 0.3), (0.7, 0.4), (0.8, 0.45), NORMAL),
                                                         ('blind', 'gentle', 'kin'), (0.03, 0.06, 0.10))):
      G.append(dict(v0=v0, gap0=gap0, wire0=w0, planner=pl, plant=plant, lead='stopped', radar_lag=0.15, anoise=(sig, 0.95, 1001 + i)))
  elif name == 'flap':   # the entry latch flaps (lead-speed reading +0.35 for n radar frames every period) on a stopped lead
    for v0, gap0, w0, (n_f, per), pl, plant in P((1.9, 2.3), (6.5, 7.5, 8.5, 10.0), (-0.4, -0.8), ((1, 0.4), (1, 0.7), (2, 0.6), (3, 1.0), (4, 0.9)),
                                                 ('gentle', 'kin'), (NORMAL, WEAK)):
      G.append(dict(v0=v0, gap0=gap0, wire0=w0, planner=pl, plant=plant, flap=(1.85, 0.35, n_f, per)))
  else:
    raise ValueError(f'unknown judge grid {name!r}')
  return G


def run(cfg, sender=True):
  """One closed-loop run (adv_lib.run) -> metrics + 'wire_sha' (sha1 of the 100 Hz command rounded to 1e-9)."""
  from openpilot.common.swaglog import cloudlog
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
  from opendbc.car.hyundai.values import CAR
  cloudlog.disabled = True
  v0, gap0, wire0 = cfg['v0'], cfg['gap0'], cfg['wire0']
  g, p = cfg.get('plant', NORMAL)
  planner = cfg.get('planner', 'kin')
  rlag = int(round(cfg.get('radar_lag', 0.0) / DT))
  # the judges' DummyCarParams / DummyFrogPilotToggles (test_longcontrol_fast_release.py), inline: no test-file import across arms
  lc = LongControl(SimpleNamespace(longitudinalTuning=SimpleNamespace(kpBP=[0.0], kpV=[1.0], kiBP=[0.0], kiV=[0.0]), stopAccel=-1.0,
                                   carFingerprint=CAR.HYUNDAI_SANTA_FE_HEV_2022, enableGasInterceptorDEPRECATED=False, startingState=False))
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  svc = lc._service_shadow_svc
  toggles = SimpleNamespace(vEgoStarting=0.1, vEgoStopping=0.2, force_coast_strength=1.0, human_acceleration=False, startAccel=1.0)
  if sender:
    from opendbc.car.hyundai.tests.test_can_bounds_fork import get_signal, make_controller, run_frame
    snd, _ = make_controller()
  v, x, a = v0, 0.0, g * wire0 + p
  xl, vl = gap0, cfg.get('crawl', 0.0)
  lc.last_output_accel = wire0
  buf = [wire0] * 20
  hist = []   # (gap, vl, a) per frame for the radar latency
  meas_gap, meas_vl, pl_gap = gap0, vl, gap0
  stopped_k = dep_k = flap_k0 = None
  trig, ev, box = {}, {}, {}
  sr = spike_left = 0
  rows = []
  orig_update = svc.update

  def cap(**kw):
    box['r'] = orig_update(**kw)
    return box['r']
  svc.update = cap
  dep = cfg.get('depart')
  T = cfg.get('T', 25.0)
  back = n_state = 0.0
  an = cfg.get('anoise')
  if an is not None:
    eps = np.random.default_rng(int(an[2])).standard_normal(int(T / DT) + 1) * an[0] * math.sqrt(1.0 - an[1] ** 2)
  flap = cfg.get('flap')
  for k in range(int(T / DT)):
    t = k * DT
    # --- triggers: the first frame the ego speed drops below the event's first element, or the frame of cfg['t_<event>'] ---
    for name in EVENTS:
      c = cfg.get(name)
      if c is None or name in trig or k == 0:
        continue
      at, thr = cfg.get('t_' + name), (c if name == 'fault' else c[0])
      if (k >= round(at / DT)) if at is not None else (thr is not None and v < thr):
        trig[name] = k
    if trig.get('fault') == k:
      def fail(**kw):
        raise RuntimeError('injected service fault')
      svc.update = fail
    # --- lead motion ---
    if cfg.get('lead') == 'reverse':
      if t >= 1.0 and back < 0.9:
        vl = max(vl - DT, -0.3)
      elif back >= 0.9:
        vl = min(vl + DT, 0.0)
    if dep is not None and dep_k is None:
      mode, xx = dep[0], dep[1]
      if (mode == 'stopped_s' and stopped_k is not None and (k - stopped_k) * DT >= xx) or (mode == 'v' and v < xx and k > 0) or \
         (mode == 't' and t >= xx):
        dep_k = k
    if dep_k is not None:
      if len(dep) > 4 and (k - dep_k) * DT >= dep[4]:
        vl = max(vl - dep[5] * DT, 0.0)
      else:
        vl = min(vl + dep[2] * DT, dep[3])
    cs_ = cfg.get('crawl_stop')
    if cs_ is not None and t >= cs_[0]:
      vl = max(vl - cs_[1] * DT, 0.0)
    if 'cutin' in trig and 'cutin_done' not in ev:
      xl -= cfg['cutin'][1]
      ev['cutin_done'] = k
    xl += vl * DT
    back = gap0 - xl
    gap = xl - x
    hist.append((gap, vl, a))
    if k % 5 == 0:
      hg, hv, ha = hist[max(len(hist) - 1 - rlag, 0)]
      dg = cfg['regap'][1] if 'regap' in trig and (k - trig['regap']) * DT < cfg['regap'][2] else 0.0
      meas_gap = round((hg + dg) / 0.1) * 0.1
      meas_vl = hv + 0.089 * ha
      if 'spike' in trig and 'spike_done' not in ev:
        if spike_left == 0 and 'spike_started' not in ev:
          spike_left = cfg['spike'][2]
          ev['spike_started'] = k
        if spike_left > 0:
          meas_vl = cfg['spike'][1]
          spike_left -= 1
          if spike_left == 0:
            ev['spike_done'] = k
    if flap is not None and k % 5 == 0:
      if flap_k0 is None and v < flap[0] and k > 0:
        flap_k0 = k
      if flap_k0 is not None and dep_k is None and ((k - flap_k0) // 5) % max(int(round(flap[3] / (5 * DT))), 1) < flap[2]:
        meas_vl = flap[1]
    # --- planner proxy ---
    if meas_gap == meas_gap:
      pl_gap = meas_gap
    if planner == 'blind':
      tgt, ss, dts = -0.05, False, -1.0
    elif planner == 'gentle':
      tgt, ss, dts = max(-v * v / (2 * max(pl_gap - 3.5, 0.3)), -0.35), False, -1.0
    else:
      tgt, ss, dts = -min(v * v / (2 * max(pl_gap - 4.3, 0.3)), 1.5), True, max(pl_gap - 4.3, 0.0)
      if planner == 'kin_go' and meas_vl > 0.5 and pl_gap > 5.0:
        tgt, ss, dts = 0.8, False, -1.0
    gas_on = 'gas' in trig and (k - trig['gas']) * DT < cfg['gas'][1]
    a_meas = a
    if an is not None:
      n_state = an[1] * n_state + eps[k]
      a_meas = a + n_state
    cs = SimpleNamespace(vEgo=v, aEgo=a_meas, brakePressed=False, standstill=v < 0.005, cruiseState=SimpleNamespace(standstill=False),
                         gasPressed=gas_on, canValid=True, canTimeout=False)
    prev_state = lc.long_control_state
    wire = lc.update(True, cs, tgt, ss, dts, (-3.5, 2), toggles, experimental_mode=True, lead_status=True, lead_v=meas_vl, lead_d_rel=meas_gap,
                     lead_a=0, lead_track_id=8 if 'cutin_done' in ev and cfg.get('cutin_new_track') else 7, lead_model_prob=0.99,
                     increased_stopped_distance=0.3, a_target_trajectory=tgt, freeze_integrator=gas_on)
    if sender:
      _, msgs = run_frame(snd, dict(accel=wire, state=prev_state, v_ego=v, a_ego=a, long_active=True, gas_pressed=gas_on))
      if 0x421 in msgs:
        sr = int(bool(get_signal('SCC12', 'StopReq', msgs[0x421])))
    buf.append(wire)
    wd = buf.pop(0)
    creep = 0.25 if (0.45 < v < 1.75 and wd > -0.50) else 0.0
    pp = p + (cfg['push'][1] if 'push' in trig and (k - trig['push']) * DT < cfg['push'][2] else 0.0)
    target_a = cfg['gas'][2] if gas_on else g * wd + pp + creep
    a += (target_a - a) * -math.expm1(-DT / 0.25)
    nv = max(v + a * DT, 0.0)
    if v == 0.0 and a <= 0.0:
      nv = 0.0
    x += 0.5 * (v + nv) * DT
    v = nv
    r_ = box.pop('r', None)
    dbg = r_.debug if (r_ is not None and r_.debug) else {}
    sig = lc._service_signals
    rows.append((t, v, xl - x, wire, int(svc.phase), int(lc.long_control_state), sr, creep > 0, vl, int(bool(dbg.get('taper', False))),
                 int(bool(sig.lead_stopped_for_entry)) if sig is not None else -1))
    if v == 0.0 and stopped_k is None:
      stopped_k = k
    if dep is None and stopped_k is not None and k - stopped_k > cfg.get('rest_frames', 300):
      break
    if dep is not None and dep_k is not None and len(dep) <= 4 and (k - dep_k) * DT > 8.0:
      break
    a = a if v > 0 else min(a, 0.0)
  svc.update = orig_update
  m = metrics(np.array(rows, dtype=float), stopped_k, dep_k, trig)
  if not sender:   # no StopReq in the run: the rest-without-StopReq measure does not apply
    m.pop('rest_nosr_s')
  return m


def metrics(R, stopped_k, dep_k, trig):
  """adv_lib.metrics on the trace columns (t, v, gap, wire, phase, lcs, stopreq, creep, vl_true, taper, entry latch)."""
  from openpilot.selfdrive.controls.lib.stopping_service import Phase
  t, v, gap, wire, ph, lcs, sr, creep, vl, tap, latch = R.T
  n = len(t)
  d1 = np.r_[0.0, np.diff(wire)]
  w5 = np.minimum(wire, 0.0)
  r5 = np.r_[np.zeros(5), w5[5:] - w5[:-5]]
  out = dict(min_gap=round(float(gap.min()), 3), min_wire=round(float(wire.min()), 3), n=n,
             wire_sha=hashlib.sha1(np.round(wire, 9).astype(np.float64).tobytes()).hexdigest()[:12])
  out['rest'] = round(float(gap[stopped_k]), 3) if stopped_k is not None else None
  out['t_stop'] = round(float(t[stopped_k]), 2) if stopped_k is not None else None
  kend = stopped_k if stopped_k is not None else (dep_k if dep_k is not None else n)
  out['creep_s'] = round(float(creep[:kend].sum() * DT), 2)
  out['taper_frames'] = int(tap.sum())
  out['latch_off_frames'] = int(((latch == 0) & (ph != 0)).sum())
  # re-grab after an ease while 0.45 <= v (jstress) from the deepest wire in the first 2.5 s of service ownership
  ko = np.flatnonzero(ph != 0)
  rg = 0.0
  if len(ko):
    km = ko[0] + int(np.argmin(wire[ko[0]:ko[0] + 250]))
    rm = -9.0
    for i in range(km, n):
      if v[i] < 0.45:
        break
      rm = max(rm, wire[i])
      rg = min(rg, wire[i] - rm)
  out['regrab'] = round(float(rg), 3)
  # releases (braking component): whole run and after each event (1 frame, 5 frames = 0.05 s)
  out['max_rise_1f'] = round(float(d1.max()), 3)
  out['max_rise_5f'] = round(float(r5.max()), 3)
  for name, k in list(trig.items()) + ([('depart', dep_k)] if dep_k is not None else []):
    kk = slice(k, min(k + 300, n))
    out[f'{name}_rise_1f'] = round(float(d1[kk].max()), 3)
    out[f'{name}_rise_5f'] = round(float(r5[kk].max()), 3)
    out[f'{name}_wire0'] = round(float(wire[k - 1]), 3)
    out[f'{name}_v'] = round(float(v[k]), 3)
  if 'cutin' in trig:   # the cut-in instant (Astra's pair): time and the true gap before / after
    k = trig['cutin']
    out.update(cutin_t=round(float(t[k]), 2), cutin_gap=round(float(gap[k]), 3), cutin_prior_gap=round(float(gap[k - 1]), 3))
  # service hold at rest (RAMP_TO_HOLD / HOLD) while LongControl is in pid; rest without StopReq (before a departure)
  rest_m = v <= 0.0
  if dep_k is not None:
    rest_m &= np.arange(n) < dep_k
  out['pid_hold_s'] = round(float((np.isin(ph, (int(Phase.RAMP_TO_HOLD), int(Phase.HOLD))) & (lcs == 1) & rest_m).sum() * DT), 2)
  out['rest_nosr_s'] = round(float((rest_m & (sr == 0)).sum() * DT), 2)
  out['rest_s'] = round(float(rest_m.sum() * DT), 2)
  # launch: first motion (v > 0.05) after the departure once the ego had stopped before it
  if dep_k is not None and stopped_k is not None and stopped_k < dep_k:
    km = np.flatnonzero((np.arange(n) > dep_k) & (v > 0.05))
    out['launch_dt'] = round(float(t[km[0]] - t[dep_k]), 2) if len(km) else None
    out['gap_at_launch'] = round(float(gap[km[0]]), 2) if len(km) else None
  if dep_k is not None:
    kd4 = min(dep_k + 400, n - 1)
    out.update(gap_dep4=round(float(gap[kd4]), 3), v_dep4=round(float(v[kd4]), 3), v_min_after_dep=round(float(v[dep_k:].min()), 3),
               stop_after_dep=bool(stopped_k is not None and stopped_k > dep_k), min_gap_after_dep=round(float(gap[dep_k:].min()), 3),
               v_max_after_dep=round(float(v[dep_k:].max()), 3), vl_end=round(float(vl[-1]), 2))
  return out


def check():
  """The worker's per-job assertions for this engine: mapped modules from their snapshot (the arm's flag file included, by sha1),
  this engine's files unchanged and no executed tools/stopping or test file outside them, every module-level flag as the arm's
  manifest records it (+ the overrides)."""
  assert os.environ.get(loader.ENV) and loader.MANIFEST, 'no source tree installed (STOP_SIM_TREE)'
  loader.check()
  assert W.engine_sha(FILES)[0] == ENGINE_SHA, 'judge engine files changed during the run'
  bad = W.unhashed(FILES)
  assert not bad, f'executed files outside the judge engine fingerprint (add them to judges.FILES): {bad}'
  from openpilot.selfdrive.controls.lib import stopping_flags as SF
  ent = loader.MANIFEST['mapped'].get(loader.FLAGS_MODULE)
  assert ent and os.path.realpath(SF.__file__) == os.path.realpath(ent['file']), ('flags not from the arm flag file', SF.__file__)
  have = {k: repr(v) for k, v in vars(SF).items() if k.isupper() and not k.startswith('_')}
  assert have == loader.MANIFEST['flag_values'], 'module flags differ from the arm flag file'
  assert all(have[k] == repr(v) for k, v in (loader.MANIFEST.get('overrides') or {}).items()), 'override not in effect'


def job(j):
  t0 = time.monotonic()
  try:
    check()
    rows = []
    for i, c in enumerate(grid(j['grid'])):
      try:
        r = run(c, sender=j['grid'] in SENDER)
      except Exception as exc:   # a run the tree under test cannot complete is a row (listed), not a lost grid
        r = dict(err=f'{type(exc).__name__}: {exc}')
      rows.append(dict(r, i=i, cfg=c))
    check()
    return dict(j, err=None, rows=rows, engine=ENGINE_SHA, secs=round(time.monotonic() - t0, 1))
  except Exception as exc:   # an engine / environment failure: the grid has no rows (reported, never gating)
    return dict(j, err=f'{type(exc).__name__}: {exc}\n{traceback.format_exc()[-1500:]}')


def main(argv):
  arm_dir, jobs_path = Path(argv[1]), Path(argv[2])
  jobs = json.loads(jobs_path.read_text())
  out = arm_dir / 'judges.pkl'
  have = {r['grid'] for r in W.load_rows(out) if not r.get('err')}
  todo = [j for j in jobs if j['grid'] not in have]
  print(f'arm {arm_dir.name} judges: {len(todo)}/{len(jobs)} grids, engine {ENGINE_SHA}', flush=True)
  err_path = arm_dir / f'errors_{jobs_path.stem}.json'
  err_path.unlink(missing_ok=True)
  if not todo:
    return 0
  procs = max(1, min(int(os.environ.get('SIM_PROCS', '2')), 2))
  errs = []

  def done(r):
    if r.get('err'):
      errs.append(dict(key=[r['grid']], err=r['err'][-1500:]))
      print('ERR', r['grid'], r['err'][-600:], flush=True)
      return
    W.save_rows(out, [x for x in W.load_rows(out) if not x.get('err') and x['grid'] != r['grid']] + [r])
    print(f"{r['grid']}: {len(r['rows'])} runs {r['secs']:.0f} s, run errors {sum('err' in x for x in r['rows'])}", flush=True)

  try:
    if procs > 1:
      # a worker that dies (a signal, os._exit) breaks the pool: every unfinished grid becomes a reported error and the gate run goes
      # on (a multiprocessing.Pool replaced the dead process and waited forever for its lost task: Astra tooling review P2)
      with ProcessPoolExecutor(procs, mp_context=mp.get_context('spawn'), max_tasks_per_child=4) as ex:
        futs = {ex.submit(job, j): j for j in todo}
        for f in as_completed(futs):
          try:
            r = f.result()
          except Exception as exc:   # BrokenProcessPool or a lost result: the grid is an error row, never a hang
            r = dict(futs[f], err=f'{type(exc).__name__}: {exc}')
          done(r)
    else:
      for j in todo:
        done(job(j))
  finally:
    if errs:
      err_path.write_text(json.dumps(errs, indent=1))
  return 3 if errs else 0


if __name__ == '__main__':
  sys.exit(main(sys.argv))
