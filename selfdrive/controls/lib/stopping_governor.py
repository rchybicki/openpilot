"""Pure whole-approach stopping profile and comfort-rate limiter."""

from __future__ import annotations

import math


A_TERMINAL = 0.60  # m/s^2 terminal comfort deceleration
A_RANGE = 0.60     # m/s^2 added head-band deceleration
Q_KNEE = 2.5       # m/s closure-speed transition
REST_BASE = 4.0    # m base stopped-lead rest gap; ISD is added
LAG = 0.45         # s actuation-lag distance allowance
TAU = 0.80         # s pursuit response time
A_MIN = -2.5       # m/s^2 maximum whole-approach braking demand
A_MAX = 0.5        # m/s^2 maximum whole-approach acceleration demand
J_DOWN = 0.60      # m/s^3 comfort deepening rate
J_UP = 0.80        # m/s^3 comfort release rate


def _finite(*values) -> bool:
  try:
    return all(value is not None and math.isfinite(value) for value in values)
  except TypeError:
    return False


def profile_decel(q: float) -> float | None:
  """Positive comfort-deceleration magnitude A(q)."""
  if not _finite(q) or q < 0.0:
    return None
  q2 = float(q) * float(q)
  if not math.isfinite(q2):
    return None
  return A_TERMINAL + A_RANGE * q2 / (q2 + Q_KNEE ** 2)


def gap_ref(q: float, isd: float) -> float | None:
  """Reference gap D(q), including the stopped-distance setting."""
  if not _finite(q, isd) or q < 0.0 or REST_BASE + isd <= 0.0:
    return None
  q2 = float(q) * float(q)
  if not math.isfinite(q2):
    return None
  k2 = Q_KNEE ** 2
  integral = (5.0 / 12.0) * q2 + (5.0 * k2 / 24.0) * math.log1p(2.0 * q2 / k2)
  return REST_BASE + float(isd) + (LAG + TAU) * float(q) + integral


def capture_reserve(q: float) -> float | None:
  """Extra distance needed to capture the comfort curve at J_DOWN."""
  a_profile = profile_decel(q)
  if a_profile is None:
    return None
  reserve = float(q) * a_profile / (2.0 * J_DOWN)
  return reserve if math.isfinite(reserve) else None


def comfort_slew(prev: float, target: float, dt: float, j_down: float = J_DOWN, j_up: float = J_UP) -> float | None:
  """Rate-limit comfort demand; safety lanes may bypass this limiter."""
  if not _finite(prev, target, dt, j_down, j_up) or dt <= 0.0 or j_down < 0.0 or j_up < 0.0:
    return None
  down_step, up_step = float(j_down) * float(dt), float(j_up) * float(dt)
  if not _finite(down_step, up_step):
    return None
  return min(max(float(target), float(prev) - down_step), float(prev) + up_step)


def whole_approach_demand(v: float, v_lead: float, gap: float, isd: float) -> tuple[float, float, float, float] | None:
  """Return (a_raw, q_ref, v_ref, d_ref), or None for unusable inputs."""
  if not _finite(v, v_lead, gap, isd) or v < 0.0 or gap <= 0.0:
    return None
  d_zero = gap_ref(0.0, isd)
  if d_zero is None:
    return None
  if gap <= d_zero:
    q_ref = 0.0
  else:
    lo, hi = 0.0, 1.0
    for _ in range(16):
      d_hi = gap_ref(hi, isd)
      if d_hi is None:
        return None
      if d_hi >= gap:
        break
      hi *= 2.0
    else:
      return None
    for _ in range(48):
      mid = (lo + hi) / 2.0
      d_mid = gap_ref(mid, isd)
      if d_mid is None:
        return None
      if d_mid < gap:
        lo = mid
      else:
        hi = mid
    q_ref = (lo + hi) / 2.0
  a_profile = profile_decel(q_ref)
  d_ref = gap_ref(q_ref, isd)
  if a_profile is None or d_ref is None:
    return None
  v_ref = float(v_lead) + q_ref
  a_ff = -q_ref / (TAU + q_ref / a_profile)
  a_raw = min(max(a_ff + (v_ref - float(v)) / TAU, A_MIN), A_MAX)
  return (a_raw, q_ref, v_ref, d_ref) if _finite(a_raw, q_ref, v_ref, d_ref) else None
