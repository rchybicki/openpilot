import json

import pytest

from cereal import messaging
from openpilot.tools.stopping import drive_replay as DR
from openpilot.tools.stopping.sim import recorded

PLAN = dict(tFollow=1.45, increasedStoppedDistance=2.5, dangerFactor=0.9)


def new_base_log():
  """A log of this branch: the toggles ride in frogpilotProcessState, frogpilotPlan has none."""
  plan = messaging.new_message("frogpilotPlan")
  for k, v in PLAN.items():
    setattr(plan.frogpilotPlan, k, v)
  state = messaging.new_message("frogpilotProcessState")
  state.frogpilotProcessState.frogpilotToggles = json.dumps({"increase_stopped_distance": 1.25})
  return b"".join(m.to_bytes() for m in (plan, state))


def old_base_log():
  """A log of the old branch (the recorded schema): the toggles ride in frogpilotPlan, whose fields the new base renumbered."""
  ev = recorded.log().Event.new_message()
  plan = ev.init("frogpilotPlan")
  for k, v in PLAN.items():
    setattr(plan, k, v)
  plan.frogpilotToggles = json.dumps({"increase_stopped_distance": 2.5})
  return ev.to_bytes()


# triage_one.read_events leaves the rlog file to the garbage collector (unchanged here); its ResourceWarning is not this test's subject
@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
@pytest.mark.parametrize("log, stopped_distance", [(new_base_log, 1.25), (old_base_log, 2.5)])
def test_b12_planner_events_read_the_logged_toggles_and_inputs(tmp_path, monkeypatch, log, stopped_distance):
  rlog = tmp_path / "rlog"
  rlog.write_bytes(log())
  monkeypatch.setattr(DR, "seg_paths", lambda route, lo, hi: [rlog])

  events = DR.planner_events("route", 0.0, 1.0)

  assert events["toggles"].increase_stopped_distance == stopped_distance
  plans = events["by"]["frogpilotPlan"][1]
  assert len(plans) == 1
  # the planner input in this tree's schema, as the planner reads it
  for k, v in PLAN.items():
    assert getattr(plans[0].frogpilotPlan, k) == pytest.approx(v), k
  assert plans[0].frogpilotPlan.speedJerk == 0.0
