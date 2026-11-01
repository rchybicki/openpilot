from types import SimpleNamespace

import pytest

from cereal import messaging
from openpilot.common.constants import CV
from openpilot.frogpilot.controls.lib.speed_limit_controller import MAX_NON_POLAND_OFFSET_KPH, SpeedLimitController

WARSAW = {"latitude": 52.2297, "longitude": 21.0122, "bearing": 0.0}
BERLIN = {"latitude": 52.5200, "longitude": 13.4050, "bearing": 0.0}
KPH = CV.KPH_TO_MS


def make_slc(gps_position=WARSAW, **toggles):
  stored = {"MapBoxRequests": {}, "MapboxSecretKey": "", "PreviousSpeedLimit": 0.0}
  planner = SimpleNamespace(params=SimpleNamespace(get=stored.__getitem__), gps_valid=gps_position is not None, gps_position=gps_position)
  slc = SpeedLimitController(SimpleNamespace(frogpilot_planner=planner))
  slc.frogpilot_toggles = SimpleNamespace(**{"map_speed_lookahead_higher": 0.0, "map_speed_lookahead_lower": 0.0,
                                             "speed_limit_controller_override_manual": False,
                                             "speed_limit_controller_override_set_speed": False, **toggles})
  return slc


def sm_with(gas_pressed=False, enabled=True, speed_limit=0.0, next_speed_limit=0.0, next_distance=0.0):
  msgs = {s: messaging.new_message(s) for s in ("carState", "selfdriveState", "mapdOut")}
  msgs["carState"].carState.gasPressed = gas_pressed
  msgs["selfdriveState"].selfdriveState.enabled = enabled
  mapd = msgs["mapdOut"].mapdOut
  mapd.speedLimit, mapd.nextSpeedLimit, mapd.nextSpeedLimitDistance = speed_limit, next_speed_limit, next_distance
  return {s: getattr(m, s) for s, m in msgs.items()}


@pytest.mark.parametrize("limit_kph, supported_kph", [(0, 0), (50, 50), (150, 150), (160, 150), (250, 150)])
def test_every_speed_limit_source_is_capped_at_150_kph(limit_kph, supported_kph):
  slc = make_slc()
  assert slc.supported_speed_limit(limit_kph * KPH) == pytest.approx(supported_kph * KPH)

  slc.update_map_speed_limit(20.0, sm_with(speed_limit=limit_kph * KPH, next_speed_limit=limit_kph * KPH))
  assert slc.map_speed_limit == pytest.approx(supported_kph * KPH)
  assert slc.next_speed_limit == pytest.approx(supported_kph * KPH)


@pytest.mark.parametrize("limit_kph, offset_kph", [(10, 0), (15, 5), (42, 10), (50, 10), (90, 15), (120, 20), (140, 5), (150, 0)])
def test_offset_follows_the_limit_table_in_poland(limit_kph, offset_kph):
  assert make_slc(WARSAW).get_offset(limit_kph * KPH) == pytest.approx(offset_kph * KPH)


@pytest.mark.parametrize("limit_kph", [90, 120])
def test_offset_outside_poland_is_capped_at_10_kph(limit_kph):
  slc = make_slc(BERLIN)
  assert not slc.in_poland
  assert slc.get_offset(limit_kph * KPH) == pytest.approx(MAX_NON_POLAND_OFFSET_KPH * KPH)


def test_unknown_position_counts_as_poland():
  slc = make_slc(None)
  assert slc.in_poland
  assert slc.get_offset(120 * KPH) == pytest.approx(20 * KPH)


@pytest.mark.parametrize("v_ego_kph, overriding", [(125, False), (139, False), (141, True)])
def test_gas_overrides_only_above_the_limit_plus_offset(v_ego_kph, overriding):
  slc = make_slc(WARSAW, speed_limit_controller_override_manual=True)
  slc.target = 120 * KPH
  slc.update_override(130 * KPH, 0.0, v_ego_kph * KPH, 0.0, sm_with(gas_pressed=True))
  assert slc.override_slc == overriding


def test_gas_override_speed_stays_between_limit_plus_offset_and_set_speed():
  slc = make_slc(WARSAW, speed_limit_controller_override_manual=True)
  slc.target = 50 * KPH
  slc.update_override(80 * KPH, 0.0, 95 * KPH, 0.0, sm_with(gas_pressed=True))
  assert slc.overridden_speed == pytest.approx(80 * KPH)

  slc.update_override(80 * KPH, 0.0, 70 * KPH, 0.0, sm_with(gas_pressed=False))
  assert slc.overridden_speed == pytest.approx(80 * KPH)


def test_no_override_while_disengaged():
  slc = make_slc(WARSAW, speed_limit_controller_override_manual=True)
  slc.target = 50 * KPH
  slc.update_override(80 * KPH, 0.0, 95 * KPH, 0.0, sm_with(gas_pressed=True, enabled=False))
  assert not slc.override_slc
  assert slc.overridden_speed == 0


@pytest.mark.parametrize("position, override_kph", [(WARSAW, 130), (BERLIN, 60)])
def test_set_speed_override_is_capped_outside_poland(position, override_kph):
  slc = make_slc(position, speed_limit_controller_override_set_speed=True)
  slc.target = 50 * KPH
  slc.update_override(130 * KPH, 0.0, 95 * KPH, 0.0, sm_with(gas_pressed=True))
  assert slc.override_slc
  assert slc.overridden_speed == pytest.approx(override_kph * KPH)


@pytest.mark.parametrize("distance, switched", [(400.0, False), (150.0, True)])
def test_lower_next_limit_is_taken_within_its_braking_distance(distance, switched):
  slc = make_slc(WARSAW, map_speed_lookahead_lower=1.0)
  v_ego = 100 * KPH
  slc.update_map_speed_limit(v_ego, sm_with(speed_limit=100 * KPH, next_speed_limit=50 * KPH, next_distance=distance))
  assert slc.calculate_change_distance(v_ego, 50 * KPH) > v_ego * 1.0
  assert slc.map_speed_limit == pytest.approx((50 if switched else 100) * KPH)
