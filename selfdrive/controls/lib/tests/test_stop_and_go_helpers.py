from openpilot.selfdrive.controls.lib.stop_and_go_helpers import should_release_stop_hold_for_departing_lead


def test_departing_lead_can_release_stop_hold_near_standstill() -> None:
  should_release = should_release_stop_hold_for_departing_lead(
    human_acceleration=True,
    output_should_stop=True,
    force_coast=False,
    standstill=True,
    v_ego=0.0,
    v_ego_starting=0.1,
    lead_status=True,
    lead_v=1.0,
    lead_d_rel=5.0,
  )

  assert should_release is True


def test_departing_lead_waits_for_more_gap_when_lead_only_creeps() -> None:
  should_release = should_release_stop_hold_for_departing_lead(
    human_acceleration=True,
    output_should_stop=True,
    force_coast=False,
    standstill=True,
    v_ego=0.0,
    v_ego_starting=0.1,
    lead_status=True,
    lead_v=0.36,
    lead_d_rel=3.49,
  )

  assert should_release is False


def test_departing_lead_releases_after_close_gap_opens_with_clear_departure() -> None:
  should_release = should_release_stop_hold_for_departing_lead(
    human_acceleration=True,
    output_should_stop=True,
    force_coast=False,
    standstill=True,
    v_ego=0.0,
    v_ego_starting=0.1,
    lead_status=True,
    lead_v=1.36,
    lead_d_rel=4.40,
  )

  assert should_release is True


def test_force_coast_keeps_stop_hold_latched_at_standstill() -> None:
  should_release = should_release_stop_hold_for_departing_lead(
    human_acceleration=True,
    output_should_stop=True,
    force_coast=True,
    standstill=True,
    v_ego=0.0,
    v_ego_starting=0.1,
    lead_status=True,
    lead_v=1.0,
    lead_d_rel=5.0,
  )

  assert should_release is False
