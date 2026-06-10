"""Tests for the re-discretizing PlantModel (spec sections 5.1, 8 / WP3).

Pins:
- re-discretization invariants (dt-chain: 0.1 s fit stepped at 0.01 s x 10 == one 0.1 s step),
- the gain_dc sign inversion in (0.15, 0.3) m/s -- the single most important physics finding,
- legacy FittedStoppingModel JSON loading (incl. the archived repo fits),
- the AST guard: no division by gain_dc anywhere in selfdrive/.

Pure python; import-clean without scons artifacts.
"""

import ast
import math
import pathlib

import pytest

from openpilot.selfdrive.controls.lib.stopping_plant import (
  PLANT_PARAMS_REF,
  PlantModel,
  PlantParams,
  load_legacy_model_json,
  plant_params_from_legacy_json,
)

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
ARCHIVE_DIR = REPO_ROOT / "docs" / "stopping" / "archive"


def _ref_model(dt: float) -> PlantModel:
  return PlantModel(PLANT_PARAMS_REF, dt)


class TestRediscretization:
  def test_identity_at_fit_dt(self):
    m = _ref_model(0.10)
    assert m.phi == pytest.approx(PLANT_PARAMS_REF.coef["a_ego_prev"], abs=1e-12)
    for key in ("intercept", "accel_cmd_delayed", "v_ego", "relief", "low_speed", "cmd_x_low_speed"):
      assert m.coef[key] == pytest.approx(PLANT_PARAMS_REF.coef[key], abs=1e-12)
    assert m.delay_frames == 1

  def test_dt_chain_10x_substeps_equal_one_fit_step(self):
    """0.1 s fit stepped at 0.01 s x 10 must equal one 0.1 s step (held inputs)."""
    coarse = _ref_model(0.10)
    fine = _ref_model(0.01)
    for a0, u, v in [(-0.3, -0.4, 0.5), (0.0, -0.1, 1.5), (-0.8, -0.9, 0.1), (-0.15, -0.25, 0.02)]:
      expected = coarse.predict_next(a0, u, v)
      a = a0
      for _ in range(10):
        a = fine.predict_next(a, u, v)
      assert a == pytest.approx(expected, abs=1e-9), f"dt-chain mismatch at (a0={a0}, u={u}, v={v})"

  def test_dt_chain_20hz_variant(self):
    coarse = _ref_model(0.10)
    fine = _ref_model(0.05)
    a = -0.3
    a = fine.predict_next(a, -0.4, 0.5)
    a = fine.predict_next(a, -0.4, 0.5)
    assert a == pytest.approx(coarse.predict_next(-0.3, -0.4, 0.5), abs=1e-9)

  def test_dead_time_rounds_up(self):
    p = PLANT_PARAMS_REF
    assert PlantModel(p, 0.10).delay_frames == 1
    assert PlantModel(p, 0.01).delay_frames == 10
    assert PlantModel(p, 0.05).delay_frames == 2
    # non-divisible dt rounds UP: 0.1 s of dead time at 0.04 s frames -> 3 frames
    assert PlantModel(p, 0.04).delay_frames == 3

  def test_dead_time_float_jitter_does_not_add_a_frame(self):
    jittery = PlantParams(dt_fit_s=0.1, delay_s=0.1 + 1e-12, coef=dict(PLANT_PARAMS_REF.coef),
                          relief_cmd_threshold=-0.25, low_speed_ref=1.20)
    assert PlantModel(jittery, 0.1).delay_frames == 1

  def test_time_constant_documented_physics(self):
    # tau = -0.1/ln(0.8715) ~ 0.727 s; invariant under re-discretization
    assert _ref_model(0.10).time_constant_s == pytest.approx(0.727, abs=0.01)
    assert _ref_model(0.01).time_constant_s == pytest.approx(_ref_model(0.10).time_constant_s, abs=1e-9)


class TestGainDc:
  """The authority-collapse curve. NO CODE MAY EVER DIVIDE BY gain_dc (spec 5.7)."""

  def test_sign_inversion_pinned_in_band(self):
    m = _ref_model(0.10)
    assert m.gain_dc(0.3) > 0.0
    assert m.gain_dc(0.15) < 0.0
    # documented approximate magnitudes (spec 5.1)
    assert m.gain_dc(0.3) == pytest.approx(0.26, abs=0.02)
    assert m.gain_dc(0.2) == pytest.approx(-0.03, abs=0.02)

  def test_inversion_point_location(self):
    m = _ref_model(0.10)
    lo, hi = 0.15, 0.30
    for _ in range(60):
      mid = 0.5 * (lo + hi)
      if m.gain_dc(mid) < 0.0:
        lo = mid
      else:
        hi = mid
    v_star = 0.5 * (lo + hi)
    assert 0.15 < v_star < 0.30
    assert v_star == pytest.approx(0.212, abs=0.005)

  def test_invariant_under_rediscretization(self):
    for v in (0.0, 0.1, 0.21, 0.3, 0.8, 1.5):
      assert _ref_model(0.01).gain_dc(v) == pytest.approx(_ref_model(0.10).gain_dc(v), abs=1e-9)

  def test_no_division_by_gain_dc_anywhere_in_selfdrive(self):
    """AST guard (spec 5.7 / 8): scan every BinOp/AugAssign division under selfdrive/ for a
    denominator whose source mentions gain_dc. The guard survives into Phase 2."""
    offenders = []
    for path in sorted((REPO_ROOT / "selfdrive").rglob("*.py")):
      try:
        source = path.read_text()
      except UnicodeDecodeError:
        continue
      if "gain_dc" not in source:
        continue
      tree = ast.parse(source, filename=str(path))
      for node in ast.walk(tree):
        denominator = None
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div | ast.FloorDiv):
          denominator = node.right
        elif isinstance(node, ast.AugAssign) and isinstance(node.op, ast.Div | ast.FloorDiv):
          denominator = node.value
        if denominator is not None:
          segment = ast.get_source_segment(source, denominator) or ""
          if "gain_dc" in segment:
            offenders.append(f"{path}:{denominator.lineno}: / {segment}")
    assert not offenders, "division by gain_dc is FORBIDDEN (the gain inverts sign at ~0.21 m/s):\n" + "\n".join(offenders)


class TestPredictAndRollforward:
  def test_predict_next_matches_legacy_formula(self):
    """Golden check against the legacy FittedStoppingModel feature construction at the fit dt."""
    m = _ref_model(0.10)
    c = PLANT_PARAMS_REF.coef
    for a_prev, u, v in [(-0.3, -0.4, 0.5), (-0.1, -0.2, 1.4), (0.05, -0.9, 0.0)]:
      relief = max(0.0, u - (-0.25))
      ls = min(max((1.20 - v) / 1.20, 0.0), 1.0)
      expected = (c["intercept"] + c["a_ego_prev"] * a_prev + c["accel_cmd_delayed"] * u
                  + c["v_ego"] * v + c["relief"] * relief + c["low_speed"] * ls + c["cmd_x_low_speed"] * u * ls)
      assert m.predict_next(a_prev, u, v) == pytest.approx(expected, abs=1e-12)

  def test_rollforward_single_delay_step(self):
    m = _ref_model(0.10)
    assert m.delay_frames == 1
    assert m.rollforward(-0.3, 0.8, [-0.5]) == pytest.approx(m.predict_next(-0.3, -0.5, 0.8))
    # only the last delay_frames commands are in flight
    assert m.rollforward(-0.3, 0.8, [-9.9, -0.5]) == pytest.approx(m.predict_next(-0.3, -0.5, 0.8))

  def test_rollforward_empty_history_is_identity(self):
    m = _ref_model(0.10)
    assert m.rollforward(-0.3, 0.8, []) == -0.3

  def test_rollforward_converges_toward_braking(self):
    m = _ref_model(0.01)
    assert m.delay_frames == 10
    a_end = m.rollforward(0.0, 1.5, [-0.8] * 10)
    assert a_end < 0.0


class TestValidationAndLegacyLoad:
  def test_coef_key_mismatch_raises(self):
    bad = dict(PLANT_PARAMS_REF.coef)
    bad.pop("relief")
    with pytest.raises(ValueError, match="coef keys mismatch"):
      PlantParams(dt_fit_s=0.1, delay_s=0.1, coef=bad, relief_cmd_threshold=-0.25, low_speed_ref=1.2)

  def test_unstable_pole_raises(self):
    bad = dict(PLANT_PARAMS_REF.coef)
    bad["a_ego_prev"] = 1.05
    with pytest.raises(ValueError, match="AR pole"):
      PlantParams(dt_fit_s=0.1, delay_s=0.1, coef=bad, relief_cmd_threshold=-0.25, low_speed_ref=1.2)

  def test_nonpositive_dt_raises(self):
    with pytest.raises(ValueError, match="dt must be"):
      PlantModel(PLANT_PARAMS_REF, 0.0)

  def test_bare_legacy_payload_loads(self):
    payload = {
      "delay_frames": 1,
      "coefficients": dict(PLANT_PARAMS_REF.coef),
      "rmse": 0.066, "mae": 0.042, "r2": 0.944, "sample_count": 862,
      "dt_s": 0.10000465200027975,
      "relief_cmd_threshold": -0.25, "low_speed_ref": 1.2,
      "model_kind": "low_speed_blend_linear",
    }
    p = plant_params_from_legacy_json(payload)
    assert p.dt_fit_s == 0.1
    assert p.delay_s == pytest.approx(0.1)
    assert PlantModel(p, 0.1).delay_frames == 1

  def test_archived_20260514_fit_loads_and_matches_params_ref(self):
    p = load_legacy_model_json(str(ARCHIVE_DIR / "plant_model_20260514_full_new_low_speed_blend.json"))
    for key, value in PLANT_PARAMS_REF.coef.items():
      assert p.coef[key] == pytest.approx(value, abs=1e-12), f"archived fit diverges from PLANT_MODEL_REF on {key}"
    assert p.delay_s == pytest.approx(PLANT_PARAMS_REF.delay_s)
    assert p.dt_fit_s == pytest.approx(PLANT_PARAMS_REF.dt_fit_s)

  def test_archived_20260531_fit_loads(self):
    p = load_legacy_model_json(str(ARCHIVE_DIR / "plant_model_20260531T075153Z_all.json"))
    assert p.dt_fit_s == 0.1
    assert p.delay_s == 0.0
    m = PlantModel(p, 0.1)
    assert m.delay_frames == 0
    assert math.isfinite(m.predict_next(-0.3, -0.4, 0.5))
