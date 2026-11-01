"""Offline KCS1 wheel plant; no runtime imports or per-episode residual correction.

Source numbers refer to the plant reader in readers_and_designs.json, items 1-13.
Only reps 1-4 inform the gain fit. Other nominal values are the training-only
plant3 spec; scenario endpoints are hypotheses, never fitted to held-out stops.
"""
from collections import deque
from dataclasses import asdict, dataclass, replace
from itertools import product
import math

import numpy as np

DT = .01
# #4: pulse training medians, +/- .07 between-maneuver sensitivity. Rows -1,-.8,-.5;
# columns >=4, >=2.5, >=1.5, >=.5, <.5 m/s. Shallow/deep extrapolation is UNMEASURED.
GAIN = ((.985, .941, .906, .897, .947), (.935, .941, .864, 1.027, 1.020), (.964, .968, .908, .953, .919))


@dataclass(frozen=True)
class Cell:
  trigger: str = 'history'  # #7: hypothesis, alternative level thresholds -.38/-.42/-.47
  threshold: float = -.42
  p_max: float = .45       # #7: unknown below .7 m/s; .35-.6 sensitivity, not fit
  tau_off: float = .3      # #7: training distance .3, E rebuild shape .6 s
  onset: float = .6        # #7: stopping-state only; training indistinguishable .4-.8 s
  gain_delta: float = 0.0  # #4: fixed per run, +/- .07; never fit per stop
  grade: float = 0.0       # #10: percent, added to measured grade in both regimes; +/-1
  delay: float = .12      # #2: wheel fit .12 [.10,.16]; IMU .19 sensitivity


def cells(full=False):
  """Stable named cells; full=True returns the Cartesian 384-cell robustness box."""
  out = {'nominal': Cell()}
  if full:
    for trigger, p, tau, onset, gain, grade, delay in product(
        ('level38', 'level42', 'level47', 'history'), (.35, .6), (.3, .6), (0., .8), (-.07, .07), (-1., 0., 1.), (.12, .19)):
      name = f'{trigger}_p{p:g}_off{tau:g}_on{onset:g}_g{gain:+g}_s{grade:+g}_d{delay:g}'
      out[name] = Cell('history' if trigger == 'history' else 'level',
                       -.42 if trigger == 'history' else -int(trigger[5:]) / 100, p, tau, onset, gain, grade, delay)
  else:
    for threshold in (-.38, -.42, -.47):
      out[f'level{abs(threshold):g}'] = Cell(trigger='level', threshold=threshold)
    for key, values in dict(p_max=(.35, .6), tau_off=(.6,), onset=(0., .8),
                            gain_delta=(-.07, .07), grade=(-1., 1.), delay=(.19,)).items():
      for value in values:
        out[f'{key}={value:g}'] = replace(Cell(), **{key: value})
  return out


def held(t, x, query):
  return np.asarray(x)[np.clip(np.searchsorted(t, query, side='right') - 1, 0, len(t) - 1)]


def fit_gain(rows):
  """Reuse pulse-window estimates; explicitly reject held-out rows before aggregation."""
  training = [r for r in rows if r['role'] == 'train' and not r['release']]
  table, provenance = [], []
  for u in (-1., -.8, -.5):
    gains = []
    for band in ('5.5-4', '4-2.5', '2.5-1.5', '1.5-0.5', '<0.5'):
      matches = [r for r in training if abs(r['u'] - u) < .006 and r['band'] == band]
      if not matches:
        raise ValueError(f'missing training gain {u}/{band}')
      if any(int(r['rep'].split('_')[1][1:]) > 4 for r in matches):
        raise ValueError('held-out repetition mislabeled as training')
      values = [r['a_pul'] / r['u'] for r in matches]
      gains.append(float(np.median(values)))
      provenance.append(dict(command=u, band=band, n=len(values), median=gains[-1],
                             min=min(values), max=max(values), reps=sorted({r['rep'] for r in matches})))
    table.append(gains)
  return table, provenance


class Plant:
  """100 Hz: sent SCC12 -> SCC14 slew -> delay -> gain/lag + release loss.

  #3 tau=.10 [.079,.108] s. #7 v_off=2.6, release=.10, rebuild=.08
  are hypothesis thresholds, not precisely identified. #7 P=.12 above shift;
  below: .30+.42*(1.27-v); #8 shift=1.6 (uphill 1.07-1.32 unmodelled),
  dip=.40 half-sine over .85 s (shape assumed), tau_L=.5 [.3,.5] s.
  #2 extra rebuild delay=.017 s (D all) vs .010 owned; quantized to .02 s.
  #13 observation lag .05 [.03,.07], .09 below .5; standstill+.22 [.182,.263].
  ABS tail below .15: .10/(1+t/.38) from flag; extrapolation before flag,
  one wheel LSB .0087. aEgo uses the native KF recurrence (not a fitted sensor).
  #12 stationary behaviour is UNVALIDATED for ALL commands. Forward-only
  integration omits rollback/static friction; StopReq and deep wire never force
  zero acceleration. Positive gain and push at rest are unmeasured.
  """
  def __init__(self, v, a, wire, cell=None, gain=GAIN, grade=0.):
    cell = Cell() if cell is None else cell
    if not all(math.isfinite(x) for x in (v, a, wire, grade)) or v < 0:
      raise ValueError('invalid seed')
    if cell.trigger not in ('history', 'level') or cell.delay < 0 or cell.tau_off <= 0:
      raise ValueError('invalid plant cell')
    self.cell, self.gain, self.grade = cell, gain, grade + cell.grade
    self.v, self.a, self.x, self.t = v, a, 0., 0.
    self.r, self.ref, self.brake, self.loss = wire, wire, a, 0.
    self.off, self.gear1, self.dip_t = False, v < 1.6, None
    self.off_t, self.reengage_until, self.onset = 0., 0., 0.
    self.queue = deque([wire] * round(cell.delay / DT))
    self.speed_history = deque([v] * 10, maxlen=10)
    self.v_ego, self.a_ego, self.raw = v, a, v
    self.standstill, self.stop_t, self.tail_t = False, None, None
    self.hold_unknown = False

  def step(self, wire, upper=3., lower=5., stop_req=False):
    if not all(math.isfinite(x) for x in (wire, upper, lower)) or upper < 0 or lower < 0:
      raise ValueError('nonfinite command or invalid SCC14 limit')
    c = self.cell
    self.r += max(-lower * DT, min(upper * DT, wire - self.r))
    self.queue.append(self.r)
    r = self.queue.popleft()
    band = 0 if self.v >= 4 else 1 if self.v >= 2.5 else 2 if self.v >= 1.5 else 3 if self.v >= .5 else 4
    g0, g1, g2 = (row[band] for row in self.gain)
    g = g0 if r <= -1 else g0 + (g1 - g0) * (r + 1) / .2 if r < -.8 else g1 + (g2 - g1) * (r + .8) / .3 if r < -.5 else g2
    was_off = self.off
    if c.trigger == 'history':
      if self.off:
        self.ref = max(self.ref, r)
        if r <= self.ref - .08:
          self.off, self.ref = False, r
      else:
        self.ref = min(self.ref, r)
        if r - self.ref >= .10 and self.v < 2.6:
          self.off, self.ref = True, r
    else:
      self.off = r > c.threshold and self.v < 2.6
    if self.off and not was_off:
      self.off_t = self.t
      self.onset = c.onset if upper < 2 else 0.
    if was_off and not self.off:
      self.reengage_until = self.t + .017
    if self.t >= self.reengage_until:
      self.brake += ((g + c.gain_delta) * r - self.brake) * -math.expm1(-DT / .10)
    if not self.gear1 and self.v < 1.6:
      self.gear1 = True
      if self.off:
        self.dip_t = self.t
    push = min(c.p_max, max(0., .30 + .42 * (1.27 - self.v))) if self.gear1 else .12
    target_loss = push if self.off and self.t - self.off_t >= self.onset else 0.
    self.loss += (target_loss - self.loss) * -math.expm1(-DT / (.5 if self.off else c.tau_off))
    dip = 0.
    if self.off and self.dip_t is not None and 0 <= self.t - self.dip_t < .85:
      dip = -.40 * math.sin(math.pi * (self.t - self.dip_t) / .85)
    self.a = self.brake + self.loss + dip - 9.81 * self.grade / 100
    self.hold_unknown |= self.v == 0 and wire > -.70 + 1e-9
    # No StopReq/deep-wire latch. The nonnegative-speed plant cannot prove rollback
    # or static friction: all stationary outcomes are explicitly UNVALIDATED.
    next_v = max(0., self.v + self.a * DT)
    self.x += (self.v + next_v) * DT / 2
    if next_v == 0 and self.v > 0:
      self.stop_t = self.t + DT
    elif next_v > 0 and self.v == 0:
      self.stop_t, self.tail_t = None, None
    self.v = next_v
    self.t += DT
    self.speed_history.append(self.v)
    delay_frames = 9 if self.v < .5 else 5
    raw = self.speed_history[-delay_frames - 1]
    if self.v < .15 and self.tail_t is None:
      self.tail_t = self.t
    if self.tail_t is not None:
      since = max(0., self.t - (self.stop_t + .22 if self.stop_t is not None else self.tail_t + .22))
      raw = max(raw, .10 / (1 + since / .38), .0087)
    # Native wheel KF, held at the CAN wheel cadence (50 Hz).
    if round(self.t / DT) % 2 == 0:
      self.raw = raw
    error = self.raw - self.v_ego
    self.v_ego += .01 * self.a_ego + .17406038913518396 * error
    self.a_ego += 1.6592563982783999 * error
    self.standstill = self.stop_t is not None and self.t - self.stop_t >= .22 - 1e-9
    return self.a


def parameter_manifest():
  return dict(nominal=asdict(Cell()), gain=GAIN, sources=Plant.__doc__,
              fit='gain rows restricted to role=train AND device rep <=4; no natural/held-out fitting',
              stationary_validation='UNVALIDATED: forward-only plant; no rollback/static-friction identification',
              unknown=['level vs history', 'deep command gain below -1', 'push below .7 m/s',
                       'static hold shallower than -.70', 'positive acceleration/relaunch', 'uphill dip trigger'])
