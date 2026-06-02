import ast
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]


def _calls_named(path: str, function_name: str) -> list[ast.Call]:
  tree = ast.parse((REPO_ROOT / path).read_text())
  calls = []
  for node in ast.walk(tree):
    if not isinstance(node, ast.Call):
      continue
    func = node.func
    if isinstance(func, ast.Name) and func.id == function_name:
      calls.append(node)
    elif isinstance(func, ast.Attribute) and func.attr == function_name:
      calls.append(node)
  return calls


def _has_keyword(call: ast.Call, keyword_name: str) -> bool:
  return any(keyword.arg == keyword_name for keyword in call.keywords)


def test_frogpilot_following_paths_use_explicit_stopped_lead_target() -> None:
  for path in (
    "frogpilot/controls/lib/frogpilot_following.py",
    "frogpilot/controls/lib/frogpilot_traffic.py",
  ):
    calls = _calls_named(path, "desired_follow_distance")
    assert calls
    assert all(_has_keyword(call, "lead_stop_distance_target") for call in calls)


def test_long_mpc_stopped_lead_obstacles_use_explicit_target() -> None:
  calls = _calls_named("selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py", "get_stopped_equivalence_factor")
  assert calls
  assert all(_has_keyword(call, "lead_stop_distance_target") for call in calls)
