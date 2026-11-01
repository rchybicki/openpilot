# ruff: noqa: RUF100, ISC002, E501, C420, F401, UP034  (copied engine code kept verbatim; README.md)
"""Logged transmission gear and powertrain torque per case (harness_g, GEAR.md).

CAN bus 0 (src 0): 0x240 (576) byte 4 low nibble = gear (1..6; 0 at rest in P/N), 0x472 (1138) bytes 0-1 = signed 13-bit
powertrain torque q (negative = regen, positive = creep), as ~/.route_sync/corpus/kcs2_drive_20260927/deep/host_creep/extract.py.
One npz per case under gear/ (absolute logMonoTime seconds, the case clock): t_gear, gear, t_q, q.
"""
import sys
from pathlib import Path

import numpy as np

from openpilot.tools.stopping.sim import SIM_HOME
from openpilot.tools.stopping.sim import harness as H

GEAR_DIR = SIM_HOME / 'gear'


def _read(c):
  lo, hi = c['lo'] - 10.0, c['hi'] + 5.0
  g, q = [], []
  for p in c['paths']:
    for e in H._events(p):
      if e.which() != 'can':
        continue
      t = e.logMonoTime * 1e-9
      if not lo <= t <= hi:
        continue
      for m in e.can:
        if m.src != 0:
          continue
        if m.address == 576 and len(m.dat) > 4:
          g.append((t, m.dat[4] & 0x0F))
        elif m.address == 1138 and len(m.dat) > 1:
          d = m.dat
          x = d[0] | ((d[1] & 0x1F) << 8)
          q.append((t, x - 0x2000 if x & 0x1000 else x))
  g = np.array(g, dtype=float).reshape(-1, 2)
  q = np.array(q, dtype=float).reshape(-1, 2)
  g, q = g[np.argsort(g[:, 0], kind='stable')], q[np.argsort(q[:, 0], kind='stable')]
  return dict(t_gear=g[:, 0], gear=g[:, 1], t_q=q[:, 0], q=q[:, 1])


def load(cid, build=True):
  """{t_gear, gear, t_q, q} for a case id (None: synthetic / no rlog CAN). Cached in gear/<cid>.npz."""
  path = GEAR_DIR / f'{cid}.npz'
  if path.is_file():
    z = np.load(path)
    return {k: z[k] for k in z.files} if len(z['gear']) else None
  if not build:
    return None
  c = H.case(cid)
  if not c.get('paths'):
    return None
  d = _read(c)
  GEAR_DIR.mkdir(exist_ok=True)
  np.savez_compressed(path, **d)
  return d if len(d['gear']) else None


def _build_one(cid):
  try:
    from openpilot.tools.stopping.sim import rharness as R   # registers the R.NEW / history ids in H.CASES
    R._register(cid)
    d = load(cid)
    return 'ok' if d is not None else 'no gear'
  except Exception as exc:  # noqa: BLE001 -- report the failing case
    return f'{type(exc).__name__}: {exc}'


def build(ids, processes=3):
  with H._pool(processes) as pool:
    return dict(zip(ids, pool.map(_build_one, ids, chunksize=1), strict=True))


if __name__ == '__main__':
  from openpilot.tools.stopping.sim import rharness as R
  ids = list(R.CORE + R.GOOD + ('2086_s8', '2086_s17')) + list(R.NEW) + R.history_cases('aim') + R.history_cases('nobite')
  ids = [i for i in ids if not (GEAR_DIR / f'{i}.npz').is_file()]
  print(len(ids), 'to build', flush=True)
  print(build(ids, processes=int(sys.argv[1]) if sys.argv[1:] else 3))


# ---- the gear model (GEAR.md section 2; all values MEASURED on the 93 recorded cases unless marked) -------------------------
V21 = 0.29    # 2->1 downshift, true (pulse) speed: median of 80 logged shifts (p10 0.27, p90 0.32, range 0.10-0.39)
V31 = 1.60    # 3->1 hot-arrival downshift: median of 13 (1.54-1.70) = the KCS plant's own 1st-gear shift speed (1.6)
V12 = 4.1     # 1->2 upshift at a launch (2 logged; only used after a closed-loop stop)
# 2nd-gear creep (0x472 q > 0 in gear >= 2): engages once the 0.3 s-lagged command has sat >= THR for DWELL s at v <= V_HI and v is
# in [V_LO, V_HI]; ends when the lagged command goes below THR or v < V_CUT (the cut); torque target QC (1 + VSLOPE (1 - v)) q units,
# first-order rise TAU_R / decay TAU_D. Fit on the logged command / speed / q (gear_scratch/onset.py, qfit.py): stops with creep
# predicted / logged 22 / 22 (21 both) of 93; q rms 50 over the 2nd-gear 0.40-1.7 m/s frames of the 23 creep stops.
# In 2nd gear the plant's brake-off push (the KCS 'loss' target, applied only in the plant's brake-off regime) is
# max(k_q x q_model, p2_hi at v >= v_p2 else p2_lo). k_q: open-loop plant replays on the logged SCC12 of the 23 creep stops
# (gear_scratch/kq*.py): k_q 0.0012 -> 2232_s40 distance error +0.02 m (0 -> -2.51, 0.0018 -> +1.15); group |dx| mean 1.06 m
# (legacy plant 1.21). p2_hi = the KCS 'P=.12 above shift'; p2_lo 0: no 2nd-gear push without creep below v_p2 (KCS2 J3_5).
CREEP = dict(thr=-0.505, lag=0.30, v_hi=1.6, v_lo=0.45, v_cut=0.40, dwell=0.5, qc=200.0, vslope=0.75, tau_r=1.2, tau_d=0.3,
             k_q=0.0012, p2_hi=0.12, v_p2=1.3, p2_lo=0.0,
             off_grade=1.0)
# off_grade: with the true gear the brake-off regime has no 1st-gear push on 2nd-gear stops; what moved the car with the brake
# released (2235_s55 on a -4.1 % grade: logged +0.05..+0.21 m/s^2 at a -0.3..-0.5 wire, q <= 44) is the grade, which a released
# brake does not compensate. The 2nd-gear push therefore adds the grade share the case fraction leaves out (1 - 0.3 below 2.5 m/s),
# through the KCS push dynamics (onset 0.6 s, lag 0.5 s) and within the KCS cap p_max. 1st gear is the legacy plant (KCS law, case
# fraction). Applying the full grade instantly while off (the RHARNESS 'off1' frac) was tried: in 2nd gear only, the s55 landing
# stalls creeping for 4 s; in all gears the s55 plant re-accelerates to 0.64 m/s in 1st gear and stops 3.9 s late (GEAR.md 2.4).


class GearState:
  """The plant's gear per 10 ms frame: 1 or 2 (2 = any gear above 1st; the plant only distinguishes 1st).

  mode 'log'      (default for rlog cases): before the takeover the logged gear by time; from the takeover the logged shifts into
                  / out of 1st replayed by speed (the plant shifts when its true speed crosses the speed of the logged 2->1 shift;
                  a logged 3->1 hot arrival shifts at V31, the KCS plant's identified shift speed: the logged 1.54-1.70 put the
                  shift dip on the brake-off hair trigger, 2235_s47 at its logged 1.54 m/s stalls creeping and never stops),
                  then the model rule (V21) if no logged downshift is left.
  mode 'log_time' : the logged gear by time throughout (sensitivity: the closed loop arrives at another time).
  mode 'model'    : 2nd until v < V21; a case that logged a 3->1 hot arrival in the approach shifts at V31 instead.
                  (default for synthetic cases)
  mode '1st'      : 2nd until v < V31 (a hot 1st-gear arrival; the synthetic 1st-gear option).
  float x         : 2nd until v < x.
  Upshift after a closed-loop stop: v > V12."""
  def __init__(self, mode, c=None, t0=None):
    self.mode = mode
    d = load(c['id']) if c is not None and c.get('kind') != 'synthetic' and c.get('paths') else None
    if mode in ('log', 'log_time') and d is None:
      mode = self.mode = 'model'
    self.d = d
    self.v_down = V21 if mode in ('model', 'log', 'log_time') else V31 if mode == '1st' else float(mode)
    self.events, self.k = [], 0
    self.state = None
    if d is not None and c is not None:
      ft = np.array([f['t'] for f in c['frames']])
      t_lo = t0 if t0 is not None else ft[0]
      t_stop = c.get('t_stop') or c['hi']
      g = np.where(d['gear'] >= 2, 2, np.where(d['gear'] == 1, 1, 0))
      self.tg, self.g = d['t_gear'], g
      ch = [k for k in range(1, len(g)) if g[k] and g[k - 1] and g[k] != g[k - 1] and t_lo < d['t_gear'][k] <= c['hi']]
      hot = lambda k: d['gear'][k] == 1 and d['gear'][k - 1] >= 3  # noqa: E731   3->1 hot arrival
      self.events = [(int(g[k]), V31 if hot(k) else float(np.interp(d['t_gear'][k], ft, c['v_true']))) for k in ch]
      if mode == 'model' and any(hot(k) for k in range(1, len(g)) if t_lo - 1.0 <= d['t_gear'][k] <= t_stop + 0.5):
        self.v_down = V31

  def logged(self, now):
    i = int(np.searchsorted(self.tg, now, side='right')) - 1
    while i > 0 and self.g[i] == 0:   # 0 = P/N: keep the last driving gear
      i -= 1
    return int(self.g[max(i, 0)]) or 1

  def __call__(self, now, v, loop):
    if self.d is not None and (not loop or self.mode == 'log_time'):
      self.state = self.logged(now)
      return self.state
    if self.state is None:
      self.state = 1 if v < self.v_down else 2
    if self.mode == 'log' and self.k < len(self.events):
      g_to, v_k = self.events[self.k]
      if (g_to == 1 and v <= v_k) or (g_to == 2 and v >= v_k):
        self.state, self.k = g_to, self.k + 1
      return self.state
    if self.state == 2 and v < self.v_down:
      self.state = 1
    elif self.state == 1 and v > V12:
      self.state = 2
    return self.state
