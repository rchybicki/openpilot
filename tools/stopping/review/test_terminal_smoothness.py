"""felt_jerk / felt_pair minimum span (cyc_1012 landing): a one-sample aEgo step over a 4 ms carState gap at the window start
read 3.76 and 4.66 on two smooth stops (route 0000228d 1681.56, 2682.57)."""
import numpy as np

from openpilot.tools.stopping.review import terminal_smoothness as ts


def _series(step=0.02):
  """100 Hz aEgo ramp of 0.5 m/s^3 whose second sample is 4 ms after the first and `step` m/s^2 off (5 m/s^3 as a pair)."""
  T = np.concatenate([[0.0, 0.004], 0.01 * np.arange(1, 121)])
  A = -0.5 + 0.5 * T
  A[1] += step
  return list(T), list(A)


def test_a_4ms_one_sample_step_does_not_dominate():
  T, A = _series()
  value, j, k = ts.felt_pair(T, A, 0, len(T) - 1)
  assert abs(value - 0.5) < 0.1, value
  assert T[k] - T[j] >= ts.FELT_MIN_SPAN_S
  assert ts.felt_jerk(T, A, 0, len(T) - 1) == round(value, 3)


def test_a_short_pair_after_a_carstate_gap_does_not_dominate():
  T, A = _series(step=0.0)
  T = T[:60] + [t + 0.5 for t in T[60:]]   # a 0.51 s gap: j restarts at the first sample after it
  A = A[:60] + [A[60]] + [a + 0.02 for a in A[61:]]   # and the next sample, 10 ms later, steps 0.02
  value, j, k = ts.felt_pair(T, A, 0, len(T) - 1)
  assert value < 0.7 and T[k] - T[j] >= ts.FELT_MIN_SPAN_S


def test_a_real_step_keeps_its_value():
  T = list(0.01 * np.arange(200))
  A = [-0.5 if t < 1.0 else -0.2 for t in T]   # 0.3 m/s^2 step: |dA| / ~0.3 s
  assert 0.99 < ts.felt_jerk(T, A, 0, len(T) - 1) < 1.2


def test_a_window_shorter_than_the_span_scores_no_pair():
  T = list(0.01 * np.arange(20))
  assert ts.felt_pair(T, [0.1 * t for t in T], 0, len(T) - 1) == (0.0, None, None)
