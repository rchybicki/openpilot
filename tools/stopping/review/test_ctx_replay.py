"""The replay tool's eligibility classification must evaluate every veto on every frame class (review R1)."""
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.stopping_service import ATTR_TRUST_IN_S, ATTR_LEAD_BRAKING
from openpilot.tools.stopping.review.ctx_replay import reason_deployed, reason_gap_live


def _sig(gap_source="measured", hold_outward=False, dropout=False, age=ATTR_TRUST_IN_S + 1.0, earned=True):
  return SimpleNamespace(gap_source=gap_source, gap_hold_outward=hold_outward, dropout_active=dropout,
                         track_age_s=age, lead_motion_earned=earned)


def test_measured_mature_frame_is_eligible_under_both_rules():
  assert reason_deployed(_sig(), False, 0.0) is None and reason_gap_live(_sig(), False, 0.0) is None


def test_outward_hold_is_gap_under_the_deployed_rule_and_eligible_under_gap_live():
  s = _sig("held", hold_outward=True)
  assert reason_deployed(s, False, 0.0) == "gap" and reason_gap_live(s, False, 0.0) is None


def test_inward_or_invalid_holds_and_dropout_stay_gap_under_both_rules():
  for s in (_sig("held", hold_outward=False), _sig("decay", dropout=True), _sig("none", dropout=True)):
    assert reason_deployed(s, False, 0.0) == "gap" and reason_gap_live(s, False, 0.0) == "gap"


def test_outward_hold_never_hides_the_other_vetoes():
  s = _sig("held", hold_outward=True, age=0.1)
  assert reason_gap_live(s, False, 0.0) == "identity"
  s = _sig("held", hold_outward=True, earned=False)
  assert reason_gap_live(s, False, 0.0) == "identity"
  assert reason_gap_live(_sig("held", hold_outward=True), True, 0.0) == "fcw"
  assert reason_gap_live(_sig("held", hold_outward=True), False, ATTR_LEAD_BRAKING - 0.1) == "lead_braking"
  # and on measured frames the same vetoes apply under both rules
  assert reason_deployed(_sig(age=0.1), False, 0.0) == "identity" and reason_deployed(_sig(), True, 0.0) == "fcw"
  assert reason_deployed(_sig(), False, ATTR_LEAD_BRAKING - 0.1) == "lead_braking"
