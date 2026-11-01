"""Forward-protection pins for the verbatim predicate ports in stop_target_arbiter.py.

WHY THIS FILE EXISTS (the gap it closes): the Commit B equivalence gates and the legacy oracle in
test_stop_target_arbiter.py import the predicate helpers FROM the live arbiter module, so an edit
to a predicate body mutates the oracle and the implementation TOGETHER and every equivalence test
keeps passing. The longcontrol-vs-arbiter AST-equality tests skip once longcontrol.py no longer
defines the function (which is the case post Commit B). This file is the remaining backstop: it
pins a SHA-256 of a normalized AST fingerprint (ast.dump with line/col attributes stripped) of
each ported predicate's source, recorded as golden hashes below, so ANY body edit fails loudly
here regardless of what the oracle says.

Golden-hash provenance: every fingerprint below was verified AST-identical to the corresponding
definition in longcontrol.py at baseline commit 3be25f5240 (the verbatim-port source of truth) on
2026-06-10. There are currently NO deliberate divergences from that baseline. (The 2026-06-10
go-signal investigation -- TestHoldVsPlannerGoSemantics in test_stop_target_arbiter.py -- ended
with "no production code change", so the predicates remain baseline-verbatim.) If a future
intentional change lands, annotate the affected entry in-place: deliberate change, date, reason.

Fingerprints are pure source-level: this file parses stop_target_arbiter.py with ast and never
imports it, so it runs anywhere python + pytest run. Hashes were generated under Python 3.11
(the repo's pinned version). ast.dump formatting can change across Python minor versions, so a
SIMULTANEOUS failure of every pin after a Python upgrade means "regenerate all hashes" (run this
file directly: ``python selfdrive/controls/lib/tests/test_arbiter_predicate_pins.py`` prints the
current fingerprint table), while a failure of ONE pin means a real edit to that definition.
"""

import ast
import hashlib
import pathlib

import pytest

ARBITER_PATH = pathlib.Path(__file__).resolve().parent.parent / "stop_target_arbiter.py"
BASELINE_COMMIT = "3be25f5240"
WORKLOG_PATH = "docs/stopping/worklog.md"

# --- golden hashes ---------------------------------------------------------------------------------
# All entries verified AST-identical to longcontrol.py@3be25f5240 on 2026-06-10; no deliberate
# divergences recorded. On an intentional change, update the hash AND annotate the entry here
# (deliberate change, date, reason) AND log the divergence from baseline 3be25f5240 in
# docs/stopping/worklog.md.

FUNCTION_GOLDEN_HASHES = {
  "has_explicit_stop_target":
    "1cb298ced0476287481d3807f487ef29ca6b4bcce2af4ae64baa270c804bb4d9",
  "should_enter_stop_target_mode":
    "736d15e8dc7989b8a2290c8877dea15f717725476a3e20d4d56621e8e7a63f2f",
  "should_hold_stop_target_mode":
    "6d926f4441bbf91f8462666436497b59c4929a829e6fdde207ecc6a6396536a5",
  "should_apply_stop_target_approach_mode":
    "fdd1a43aa19e54f12e7724eb68e5b48b9790f5b7554c45d098d07111ca124c61",
  "should_apply_stop_target_carry_mode":
    "748c71fdf2f8db63e32e60749c4f74d4478e289d51f711332f0ae616004155f9",
  "should_release_far_stopped_lead_gap":
    "3c1e379ee854dda53535a4377abf31ee1ac9ebb7b2c0e0fc7c4d0552c0d2ee9d",
  "should_hold_recent_close_stopped_lead_dropout":
    "cbf98ce5abd28a9050014c41d909a444b94f9c23bc717cd6727248f7b97575c9",
  "should_apply_low_speed_stopped_lead_glide_accel_cap":
    "d4a01c577fb0bb9dc87b1d576233c3aa42a62fd9fab29335d57f03fb6cb9a413",
  "should_apply_stop_entry_handoff_soften":
    "906ede05c38253a8bfdb82244248c7f26686b1da3c68a7aa98025e1dd95666a1",
  "stop_entry_handoff_accel_cap":
    "7871e211013f56871b6ece0d044a931d96019a7997ae5624a42e70099901198e",
  "should_hold_stop_target_dropout":
    "53bfe9386dd2d36cc9ce57cd806e2c8eb497f5df270a759824e244b4972d6c2b",
  "should_hold_no_target_standstill_dropout":
    "65683ac6f30cc09953d56f8ef41f43c4bd2fe4a4707b5eee482ecb69671c3f17",
  "should_hold_low_speed_stop_target_release":
    "a26b4aa68e765be4ec911dc3c881127a6792530e27e8a5936e3b0ccbb16b210f",
}

# Ported constants: the verbatim block (longcontrol.py:39-49 at 3be25f5240) plus the legacy timer
# cap (the 10.0 literal at longcontrol.py:891/:897, named _LEGACY_TIMER_CAP_S in the arbiter).
# The consolidated-hold constants (CONSOLIDATED_*, RELEASE_HOLD_*) and the LONG_CTRL_STATE_* capnp
# mirrors are NOT baseline-derived and are deliberately not pinned here.
CONSTANT_GOLDEN_HASHES = {
  "MIN_STOP_TARGET_MODE_DISTANCE_M":
    "752ff52bb3a807ac6090033c2cac2db0ded5eb5c24626854865c31ab3aa7aa45",
  "MAX_STOP_TARGET_MODE_DISTANCE_M":
    "4aa9f44e66f60215c793de1a53ea7c76e071eeedd18cf35cc1f93acf51f5018a",
  "LEAD_FOLLOW_TARGET_HOLD_GAP_M":
    "0c9e5da81c00e2b1d3fede109f2a2b8ca73894d0d7efab6ef68b3364684b4cab",
  "FAR_STOPPED_LEAD_CRAWL_GAP_M":
    "1589e2452fc5649582604b32fb38d7582cb084b1e21eab74fad05ed7cd919a5b",
  "FAR_STOPPED_LEAD_CLOSE_TARGET_HOLD_M":
    "c22408292d071cdf3de16a8312e4b508fa5976c1eb4c5619d3ab02fc826c9698",
  "_LEGACY_TIMER_CAP_S":
    "411ff698628562e19880dd78b0c424d6f5737d84428911e54e34b3747b892718",
}


# --- fingerprinting --------------------------------------------------------------------------------


def _fingerprint(node: ast.AST) -> str:
  """SHA-256 of the normalized AST dump (include_attributes=False strips line/col info)."""
  return hashlib.sha256(ast.dump(node, include_attributes=False).encode("utf-8")).hexdigest()


def _arbiter_tree() -> ast.Module:
  return ast.parse(ARBITER_PATH.read_text())


def _top_level_functions(tree: ast.Module) -> dict[str, ast.FunctionDef]:
  return {node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)}


def _top_level_constant_assigns(tree: ast.Module) -> dict[str, ast.Assign]:
  out: dict[str, ast.Assign] = {}
  for node in tree.body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
      out[node.targets[0].id] = node
  return out


def _pin_failure_message(kind: str, name: str, expected: str, actual: str) -> str:
  this_file = pathlib.Path(__file__).name
  return f"""{kind} '{name}' in {ARBITER_PATH.name} no longer matches its pinned golden AST fingerprint.
  expected sha256: {expected}
  actual   sha256: {actual}
This pin is the forward-protection backstop: the equivalence gates import the arbiter
predicates from the live module, so a body edit mutates oracle and implementation together
and the gates cannot detect it. If this is an INTENTIONAL behavior change you must BOTH:
  1. update the golden hash in {this_file} (run
     `python selfdrive/controls/lib/tests/{this_file}` to print current
     fingerprints) and annotate the entry (deliberate change, date, reason); AND
  2. note the divergence from baseline {BASELINE_COMMIT} in {WORKLOG_PATH}
     (what changed, why, and the date).
If you did NOT intend to change arbiter stopping behavior, revert the edit instead."""


# --- pins ------------------------------------------------------------------------------------------


class TestArbiterPredicatePins:
  @pytest.mark.parametrize("name", sorted(FUNCTION_GOLDEN_HASHES))
  def test_predicate_body_pinned(self, name):
    funcs = _top_level_functions(_arbiter_tree())
    assert name in funcs, (
      f"ported predicate '{name}' is no longer defined at top level in {ARBITER_PATH.name}; " +
      "renaming or removing a verbatim-ported predicate is a behavior-relevant change -- see the " +
      f"golden-hash header in this file for the required worklog/hash procedure (baseline {BASELINE_COMMIT})"
    )
    actual = _fingerprint(funcs[name])
    expected = FUNCTION_GOLDEN_HASHES[name]
    assert actual == expected, _pin_failure_message("ported predicate", name, expected, actual)

  @pytest.mark.parametrize("name", sorted(CONSTANT_GOLDEN_HASHES))
  def test_constant_pinned(self, name):
    assigns = _top_level_constant_assigns(_arbiter_tree())
    assert name in assigns, (
      f"ported constant '{name}' is no longer assigned at top level in {ARBITER_PATH.name}; " +
      f"see the golden-hash header in this file for the required worklog/hash procedure (baseline {BASELINE_COMMIT})"
    )
    actual = _fingerprint(assigns[name])
    expected = CONSTANT_GOLDEN_HASHES[name]
    assert actual == expected, _pin_failure_message("ported constant", name, expected, actual)

  def test_every_top_level_function_is_pinned(self):
    # Rename/split protection: today the arbiter's only top-level functions ARE the 13 verbatim
    # ports. A new or renamed top-level function must either get a golden hash here (if it carries
    # ported/stop-decision logic) or be consciously exempted by updating this test.
    funcs = set(_top_level_functions(_arbiter_tree()))
    pinned = set(FUNCTION_GOLDEN_HASHES)
    assert funcs == pinned, (
      f"top-level functions in {ARBITER_PATH.name} diverge from the pinned set: " +
      f"unpinned={sorted(funcs - pinned)} missing={sorted(pinned - funcs)}. " +
      "Add golden hashes for new predicate-bearing functions (and log the divergence from " +
      f"baseline {BASELINE_COMMIT} in {WORKLOG_PATH}) or update this exemption consciously."
    )


if __name__ == "__main__":
  # regeneration helper: prints the current fingerprint table in copy-paste form
  tree = _arbiter_tree()
  funcs = _top_level_functions(tree)
  assigns = _top_level_constant_assigns(tree)
  print("FUNCTION_GOLDEN_HASHES = {")
  for fname in FUNCTION_GOLDEN_HASHES:
    print(f'  "{fname}":\n    "{_fingerprint(funcs[fname])}",')
  print("}")
  print()
  print("CONSTANT_GOLDEN_HASHES = {")
  for cname in CONSTANT_GOLDEN_HASHES:
    print(f'  "{cname}":\n    "{_fingerprint(assigns[cname])}",')
  print("}")
