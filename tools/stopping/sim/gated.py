# ruff: noqa: RUF100, ISC002, E501, C420, F401, UP034  (copied engine code kept verbatim; README.md)
"""Standstill/launch gate for the harness_g plant (module SSG, cyc_1004/plant). Wraps any kcs_plant.Plant-like object.

MEASURED basis (logged CAN, cyc_1004/plant; Santa Fe HEV, 1st gear at rest):
- At rest with StopReq=1 the car does not move whatever the command: 0/29 logged intervals with the command at -0.30..-0.05 for
  0.6-4.0 s moved more than 0.011 m (sr1_replay.json); 0x472 torque stays 0 under a latched StopReq (q_rest 0 in 90/90 releases).
- Natural release (StopReq latched, n=90): 0x472 torque appears 0.254 s after the StopReq drop (p10 0.244, p90 0.266).
- StopReq=0 at rest (KCS2 fast-cycle holds, n=35): no torque and no motion while the command is <= 0 (ramps -0.70 -> 0 over
  0.58 s); the torque appears 0.146 s after the sent command first exceeds 0 (p10 0.084, p90 0.246).
- The gate opens at max(StopReq drop + L_SR, command > C_GO + L_CMD). While it is closed the plant speed is held at 0 (no creep,
  no rollback; rollback and breakaway on grades are UNMEASURED). Once open, the base plant integrates freely (its own brake-off
  onset/lag then sets the motion time: open-loop launch error med -0.02 s, p10 -0.11, p90 +0.15 s on the 90 + 41 logged releases).
  A stop (base speed reaches 0) with StopReq=1 or a command <= C_GO closes the gate again.
- UNMEASURED (the gate assumes 'held'): StopReq=0 at rest with the command in (-0.5, 0] for longer than 0.6 s; commands in
  (0, +0.03] (the exact C_GO); rest on a grade steeper than the logged releases.
The base plant's filters (SCC14 slew, delay, brake lag, brake-off push) keep running while held, so the release starts from the
filtered command state."""

_OWN = frozenset(('inner', 'held', 't', 't_sr0', 't_cmd', 't_open', 'L_SR', 'L_CMD', 'C_GO'))


class Gated:
  L_SR = 0.25    # s, StopReq drop -> torque (n=90: median 0.254)
  L_CMD = 0.15   # s, command > C_GO -> torque with StopReq already 0 (n=35: median 0.146)
  C_GO = 0.0     # m/s^2, the sent command must exceed this at rest (KCS2: no torque while <= 0)

  def __init__(self, inner, **kw):
    object.__setattr__(self, 'inner', inner)
    for k, v in kw.items():
      object.__setattr__(self, k, v)
    object.__setattr__(self, 'held', inner.v <= 0.0)
    object.__setattr__(self, 't', 0.0)
    object.__setattr__(self, 't_sr0', None)
    object.__setattr__(self, 't_cmd', None)
    object.__setattr__(self, 't_open', None)

  def __getattr__(self, k):
    return getattr(object.__getattribute__(self, 'inner'), k)

  def __setattr__(self, k, v):
    if k in _OWN:
      object.__setattr__(self, k, v)
    else:
      setattr(self.inner, k, v)

  def _set(self, **kw):
    for k, v in kw.items():
      object.__setattr__(self, k, v)

  def step(self, wire, upper=3., lower=5., stop_req=False):
    p = self.inner
    self._set(t=self.t + 0.01)
    if self.held and p.v > 0.0:     # motion seeded from outside (harness takeover reseed): not at rest
      self._set(held=False)
    if self.held:
      t_sr0 = None if stop_req else (self.t if self.t_sr0 is None else self.t_sr0)
      t_cmd = (self.t if self.t_cmd is None else self.t_cmd) if wire > self.C_GO else None
      self._set(t_sr0=t_sr0, t_cmd=t_cmd)
      if t_sr0 is not None and t_cmd is not None and self.t >= max(t_sr0 + self.L_SR, t_cmd + self.L_CMD) - 1e-9:
        self._set(held=False, t_open=self.t)
    if not self.held:
      a = p.step(wire, upper, lower, stop_req)
      if p.v <= 0.0 and (stop_req or wire <= self.C_GO):
        self._set(held=True, t_sr0=None, t_cmd=None)
      return a
    snap = (p.x, p.stop_t, p.tail_t, p.__dict__.get('_floor'), p.v_ego, p.a_ego, p.raw, p.standstill)
    a = p.step(wire, upper, lower, stop_req)
    if p.v > 0.0:
      p.v = 0.0
      p.x, p.stop_t, p.tail_t, fl, p.v_ego, p.a_ego, p.raw, p.standstill = snap
      if fl is not None:
        p._floor = fl
      p.speed_history[-1] = 0.0
      p.a = min(p.a, 0.0)
    return min(a, 0.0)


# ---- optional re-stop sensitivity term (cell 'gate+creep1') ------------------------------------------------------------------
import math  # noqa: E402

K_Q = 0.0012     # m/s^2 per 0x472 unit (gear.CREEP k_q, fitted on 2nd-gear creep; reproduces free 1st-gear creep within 0.03-0.08)
FRAC_LO = 0.3    # the harness grade fraction below 2.5 m/s (the term is injected through the plant's own grade input)


def q1(v):
  """median 0x472 of engaged 1st-gear frames with the brake held (-0.75..-0.47, 0.1-2.0 m/s, wframes): ~120-156 to 1.0 m/s,
  84-122 at 1.0-1.6, 16-60 at 1.6-2.0 -> 150 up to 1.0 m/s, linear to 30 at 1.8 m/s."""
  return 150.0 if v <= 1.0 else max(30.0, 150.0 - 150.0 * (v - 1.0))


class Creep1:
  """1st-gear creep push while the plant brake is ON (the harness applies its 1st-gear push only in the brake-off regime):
  k_q x q1(v) below 1.8 m/s, first-order lag 0.3 s, through the plant's grade input. MEASURED need: 1st-gear brake-on frames deliver
  +0.10..+0.36 m/s^2 less decel than commanded (2nd gear within 0.05); not fitted to stop distances."""
  def __init__(self, inner, k_q=K_Q, tau=0.3):
    object.__setattr__(self, 'inner', inner)
    object.__setattr__(self, 'base_grade', inner.grade)
    object.__setattr__(self, 'k_q', k_q)
    object.__setattr__(self, 'tau', tau)
    object.__setattr__(self, 'e', 0.0)

  def __getattr__(self, k):
    return getattr(object.__getattribute__(self, 'inner'), k)

  def __setattr__(self, k, v):
    if k == 'grade':
      object.__setattr__(self, 'base_grade', v)
    elif k in ('inner', 'base_grade', 'k_q', 'tau', 'e'):
      object.__setattr__(self, k, v)
    else:
      setattr(self.inner, k, v)

  def step(self, wire, upper=3., lower=5., stop_req=False):
    p = self.inner
    on = getattr(p, 'gear_now', 2) == 1 and not p.off and 0.0 < p.v < 1.8
    tgt = self.k_q * q1(p.v) if on else 0.0
    object.__setattr__(self, 'e', self.e + (tgt - self.e) * -math.expm1(-0.01 / self.tau))
    p.grade = self.base_grade - self.e * 100.0 / (9.81 * FRAC_LO)
    return p.step(wire, upper, lower, stop_req)
