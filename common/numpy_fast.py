from __future__ import annotations


def clip(x, lo, hi):
  return max(lo, min(hi, x))


def interp(x, xp, fp):
  n = len(xp)

  def get_interp(xv):
    hi = 0
    while hi < n and xv > xp[hi]:
      hi += 1
    low = hi - 1
    if hi == n and xv > xp[low]:
      return fp[-1]
    if hi == 0:
      return fp[0]
    return (xv - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low]

  if hasattr(x, "__iter__"):
    return [get_interp(v) for v in x]
  return get_interp(x)


def mean(x):
  return sum(x) / len(x)
