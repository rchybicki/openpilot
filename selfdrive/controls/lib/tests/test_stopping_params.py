"""Tests for the frozen stopping parameter registry (spec section 3 / WP3).

Pure python + numpy-free; import-clean without scons artifacts.
"""

import dataclasses
import pathlib

import pytest

from openpilot.selfdrive.controls.lib.stopping_params import STOPPING_PARAMS, StoppingParams, render_parameters_doc

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
DOC_PATH = REPO_ROOT / "docs" / "stopping" / "parameters.md"

# rows 1-39 = spec section-3 table; row 40 = hold-acquisition soften (driveway route
# 00001702--dcdc5c3eea--0); row 41 = cranked comfort requirement (2026-06-13 user-felt-forces iteration)
SPEC_ROWS = set(range(1, 42))


def _table_fields():
  return [f for f in dataclasses.fields(StoppingParams) if f.metadata["kind"] == "table"]


def test_doc_matches_dataclass():
  assert DOC_PATH.is_file(), f"missing generated doc {DOC_PATH}; regenerate with python3 -m openpilot.selfdrive.controls.lib.stopping_params"
  assert DOC_PATH.read_text() == render_parameters_doc(STOPPING_PARAMS), (
    "docs/stopping/parameters.md is stale; regenerate with python3 -m openpilot.selfdrive.controls.lib.stopping_params"
  )


def test_every_field_has_complete_metadata():
  for f in dataclasses.fields(StoppingParams):
    for key in ("row", "unit", "provenance", "kind"):
      assert key in f.metadata, f"{f.name} missing metadata key {key}"
    assert f.metadata["row"] in SPEC_ROWS, f"{f.name} row {f.metadata['row']} outside the spec parameter table"
    assert f.metadata["provenance"], f"{f.name} has empty provenance"


def test_all_spec_rows_covered():
  rows = {f.metadata["row"] for f in dataclasses.fields(StoppingParams)}
  assert rows == SPEC_ROWS, f"spec rows without a dataclass field: {sorted(SPEC_ROWS - rows)}"


def test_tables_monotone_in_bp_axis():
  for f in _table_fields():
    bp, v = getattr(STOPPING_PARAMS, f.name)
    assert len(bp) == len(v), f"{f.name}: bp/v length mismatch"
    assert len(bp) >= 2, f"{f.name}: degenerate table"
    assert all(b1 > b0 for b0, b1 in zip(bp, bp[1:], strict=False)), f"{f.name}: bp axis not strictly increasing: {bp}"


def test_expected_accel_envelope_consistent():
  p = STOPPING_PARAMS
  assert len(p.EXPECTED_ACCEL_V_BP) == len(p.EXPECTED_ACCEL_MAX) == len(p.EXPECTED_ACCEL_MIN)
  assert all(b1 > b0 for b0, b1 in zip(p.EXPECTED_ACCEL_V_BP, p.EXPECTED_ACCEL_V_BP[1:], strict=False))
  # max (least braking) must stay above min (deepest braking) everywhere
  assert all(hi > lo for hi, lo in zip(p.EXPECTED_ACCEL_MAX, p.EXPECTED_ACCEL_MIN, strict=True))


def test_dataclass_is_frozen():
  with pytest.raises(dataclasses.FrozenInstanceError):
    STOPPING_PARAMS.V_SETTLE = 0.0  # type: ignore[misc]


def test_plant_ref_coefficient_keys():
  names = [name for name, _ in STOPPING_PARAMS.PLANT_MODEL_REF]
  assert names == ["intercept", "a_ego_prev", "accel_cmd_delayed", "v_ego", "relief", "low_speed", "cmd_x_low_speed"]


def test_golden_values_pinned_to_legacy_source():
  """Spot-pins for load-bearing values; each must match the cited legacy source verbatim."""
  p = STOPPING_PARAMS
  # G1: stopping_shadow.py:25-35 (archived 20260514 fit)
  coef = dict(p.PLANT_MODEL_REF)
  assert coef["a_ego_prev"] == pytest.approx(0.8715303653619791)
  assert coef["accel_cmd_delayed"] == pytest.approx(0.37169541723475924)
  assert coef["cmd_x_low_speed"] == pytest.approx(-0.45138720666502313)
  # G7: stopping_controller.py:2055 -- the binding terminal ceiling over the FULL 0-0.60 domain
  assert p.A_END_STOP_TABLE == ((0.00, 0.10, 0.15, 0.25, 0.60), (-0.255, -0.255, -0.30, -0.42, -0.68))
  # G8: stopping_controller.py:2021 / :2047 -- arrest depth + rate (red-team F27)
  assert p.A_ARREST_MAX_TABLE[1][0] == -1.40
  assert p.J_ARREST_TABLE == ((0.00, 0.08), (4.0, 2.2))
  # G4: stopping_controller.py:1766 / :1775 (red-team F35)
  assert p.OVERBRAKE_RELEASE_FLOOR_TABLE[1] == (1.00, 1.15, 1.35, 1.60, 1.80)
  assert p.A_DISTURBANCE_FLOOR_TABLE[1] == (-0.34, -0.31, -0.26, -0.18, -0.11)
  # G7: stopping_controller.py:2392 (red-team F30)
  assert p.J_END_STOP_RELEASE_TABLE == ((0.00, 0.60), (0.90, 0.45))
  # G11: longcontrol.py:46-48 + stop_target_helpers.py:14-16
  assert (p.HOLD_GAP_M, p.TARGET_HOLD_GAP_M, p.FAR_CRAWL_GAP_M, p.LEAD_STOP_TARGET_M) == (2.75, 3.75, 5.0, 4.0)
  # dropout-hold windows + escapes (longcontrol.py:471-475)
  assert (p.T_STOP_INTENT_HOLD_S, p.T_STOP_INTENT_HOLD_STANDSTILL_S) == (0.4, 1.4)
  assert (p.HOLD_ESCAPE_A_TARGET, p.HOLD_ESCAPE_LAST_OUTPUT) == (0.12, -0.08)


def test_dist_lpf_tau_default_is_kill_switch_bypass():
  # Gate precondition: the estimator-equivalence Tier-1 row passes only at tau=0.0 (legacy G4
  # single-frame semantics). Reverting to a nonzero default silently invalidates the recorded
  # gate state without failing any other test, so the shipped default is pinned here.
  assert STOPPING_PARAMS.DIST_LPF_TAU_S == 0.0
