"""Identified stopping-plant model with dt re-discretization (stopping redesign spec, section 5.1).

The fitted AR(1) at Delta = dt_fit_s (7-feature schema preserved verbatim from
tools/stopping/stopping_model.py FEATURE_NAMES):

  a[k+1] = c0 + phi*a[k] + c2*u_d[k] + c3*v[k] + c4*relu(u_d[k] - u_rel) + c5*ls(v) + c6*u_d[k]*ls(v)
  ls(v)  = clip((low_speed_ref - v) / low_speed_ref, 0, 1);  u_d[k] = u[k - delay_frames]

This is the ZOH discretization of a first-order lag with speed-scheduled gain and dead time.
`PlantModel` re-discretizes to the controller dt: phi' = phi**(dt/dt_fit_s); every input
coefficient (intercept, accel_cmd_delayed, v_ego, relief, low_speed, cmd_x_low_speed) scales
by (1 - phi')/(1 - phi); dead time rounds to ceil(delay_s/dt) frames. One implementation
serves 100 Hz runtime and 10/20 Hz replay -- no dt_scale machinery anywhere.

Documented physics (unit-test-pinned in test_stopping_plant.py): with the archived 20260514
coefficients, tau = -0.1/ln(0.8715) ~ 0.727 s, and gain_dc(v) COLLAPSES AND INVERTS SIGN near
v ~ 0.21 m/s (gain_dc(0.3) ~ +0.26, gain_dc(0.2) ~ -0.03). HARD RULE: NO CODE MAY EVER DIVIDE
BY gain_dc -- it exists for documentation, telemetry, and eval only (spec 5.7; the guard
survives into Phase 2). The fitted DC gain is an in-sample artifact under feedback, not
steady-state truth.
"""

from __future__ import annotations

import json
import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.controls.lib.stopping_params import STOPPING_PARAMS

PLANT_COEF_KEYS = ("intercept", "a_ego_prev", "accel_cmd_delayed", "v_ego", "relief", "low_speed", "cmd_x_low_speed")
_INPUT_COEF_KEYS = ("intercept", "accel_cmd_delayed", "v_ego", "relief", "low_speed", "cmd_x_low_speed")


@dataclass(frozen=True)
class PlantParams:
  dt_fit_s: float               # dt the coefficients were fitted at (0.10 for the archived fits)
  delay_s: float                # input dead time L (s)
  coef: dict[str, float]        # keys exactly PLANT_COEF_KEYS
  relief_cmd_threshold: float   # -0.25
  low_speed_ref: float          # 1.20

  def __post_init__(self) -> None:
    if set(self.coef) != set(PLANT_COEF_KEYS):
      missing = set(PLANT_COEF_KEYS) - set(self.coef)
      extra = set(self.coef) - set(PLANT_COEF_KEYS)
      raise ValueError(f"PlantParams.coef keys mismatch: missing={sorted(missing)} extra={sorted(extra)}")
    if self.dt_fit_s <= 0.0:
      raise ValueError(f"dt_fit_s must be > 0, got {self.dt_fit_s}")
    if self.delay_s < 0.0:
      raise ValueError(f"delay_s must be >= 0, got {self.delay_s}")
    phi = self.coef["a_ego_prev"]
    # The AR pole must be in (0, 1): re-discretization phi**(dt/dt_fit_s) needs a positive
    # stable pole, and the identified plant physics (first-order lag) guarantees one.
    if not (0.0 < phi < 1.0):
      raise ValueError(f"AR pole a_ego_prev must be in (0, 1), got {phi}")
    if self.low_speed_ref <= 0.0:
      raise ValueError(f"low_speed_ref must be > 0, got {self.low_speed_ref}")


# Archived 20260514 fit (862 rows) -- params #3-5 in stopping_params; the single reference
# plant until a spec-7.5 refit passes its acceptance gate.
PLANT_PARAMS_REF = PlantParams(
  dt_fit_s=STOPPING_PARAMS.PLANT_MODEL_DT_FIT_S,
  delay_s=STOPPING_PARAMS.PLANT_MODEL_DELAY_S,
  coef=dict(STOPPING_PARAMS.PLANT_MODEL_REF),
  relief_cmd_threshold=STOPPING_PARAMS.RELIEF_CMD_THRESHOLD,
  low_speed_ref=STOPPING_PARAMS.LOW_SPEED_AUTHORITY_REF,
)


class PlantModel:
  """Re-discretized 7-feature actuator model. One code path for runtime predictor and offline sim."""

  def __init__(self, p: PlantParams, dt: float) -> None:
    if not (dt > 0.0 and math.isfinite(dt)):
      raise ValueError(f"dt must be a positive finite float, got {dt}")
    self.params = p
    self.dt = float(dt)
    phi_fit = p.coef["a_ego_prev"]
    self.phi = phi_fit ** (self.dt / p.dt_fit_s)
    scale = (1.0 - self.phi) / (1.0 - phi_fit)
    self.coef = {key: p.coef[key] * scale for key in _INPUT_COEF_KEYS}
    # ceil with a tiny epsilon so float jitter in delay_s/dt (e.g. 0.10000465/0.1) does not
    # bump the dead time by a whole frame.
    self.delay_frames = max(0, math.ceil(p.delay_s / self.dt - 1e-9))
    self.relief_cmd_threshold = p.relief_cmd_threshold
    self.low_speed_ref = p.low_speed_ref

  @property
  def time_constant_s(self) -> float:
    """First-order lag time constant tau = -dt/ln(phi); ~0.727 s with the archived 20260514 fit."""
    return -self.dt / math.log(self.phi)

  def low_speed_factor(self, v_ego: float) -> float:
    return min(max((self.low_speed_ref - v_ego) / max(self.low_speed_ref, 1e-6), 0.0), 1.0)

  def predict_next(self, a_ego_prev: float, accel_cmd_delayed: float, v_ego: float) -> float:
    """One-step prediction (FittedStoppingModel-compatible signature)."""
    relief = max(0.0, accel_cmd_delayed - self.relief_cmd_threshold)
    ls = self.low_speed_factor(v_ego)
    c = self.coef
    return (c["intercept"]
            + self.phi * a_ego_prev
            + c["accel_cmd_delayed"] * accel_cmd_delayed
            + c["v_ego"] * v_ego
            + c["relief"] * relief
            + c["low_speed"] * ls
            + c["cmd_x_low_speed"] * accel_cmd_delayed * ls)

  def rollforward(self, a0: float, v0: float, u_history: Sequence[float]) -> float:
    """Forward sim over already-sent commands (Phase-2 Smith core; telemetry-only in V1).

    Steps the plant through the dead-time pipeline: the last `delay_frames` entries of
    `u_history` (oldest first) are the in-flight commands that will still act on the plant.
    Returns the predicted accel once the pipeline empties. With delay_frames == 0 (or an
    empty history) this is the identity on a0.
    """
    a = float(a0)
    v = float(v0)
    if self.delay_frames <= 0:
      return a
    for u in list(u_history)[-self.delay_frames:]:
      a = self.predict_next(a, float(u), v)
      v = max(0.0, v + a * self.dt)
    return a

  def gain_dc(self, v: float) -> float:
    """Fitted steady-state command gain K(v) = (c_cmd + c_cxl*ls(v)) / (1 - phi).

    DOCUMENTED HARD GUARD: with the archived 20260514 coefficients K(v) collapses and
    INVERTS SIGN near v ~ 0.21 m/s (K(0.3) ~ +0.26, K(0.2) ~ -0.03). NO CODE MAY EVER
    DIVIDE BY gain_dc (spec 5.7) -- it exists for documentation, telemetry, and eval only.
    The value is invariant under re-discretization (input scale and 1 - phi' cancel),
    which test_stopping_plant.py pins.
    """
    ls = self.low_speed_factor(v)
    return (self.coef["accel_cmd_delayed"] + self.coef["cmd_x_low_speed"] * ls) / (1.0 - self.phi)


def plant_params_from_legacy_json(data: dict[str, Any]) -> PlantParams:
  """Build PlantParams from a legacy FittedStoppingModel JSON payload.

  Accepts both the bare `FittedStoppingModel.as_json()` payload and the fit-tool wrapper
  (`{"model": {...}, ...}`, as written by fit_stopping_model.py / fit_plant_model.py).
  The base linear `coefficients` are always used; band/blend heads are not modeled
  (PlantModel is the single linear AR(1) -- for the archived 20260514 low_speed_blend fit
  the head equals the base coefficients). dt_s is snapped to 1 ms so the median-dt jitter
  recorded by the fitters (e.g. 0.10000465) does not inflate ceil(delay_s/dt) by a frame.
  """
  model = data.get("model", data)
  raw_coef = model["coefficients"]
  coef = {key: float(raw_coef.get(key, 0.0)) for key in PLANT_COEF_KEYS}
  dt_fit_s = round(float(model["dt_s"]), 3)
  return PlantParams(
    dt_fit_s=dt_fit_s,
    delay_s=int(model["delay_frames"]) * dt_fit_s,
    coef=coef,
    relief_cmd_threshold=float(model["relief_cmd_threshold"]),
    low_speed_ref=float(model["low_speed_ref"]),
  )


def load_legacy_model_json(path: str) -> PlantParams:
  with open(path) as f:
    return plant_params_from_legacy_json(json.load(f))
