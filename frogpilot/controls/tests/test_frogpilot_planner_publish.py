from types import SimpleNamespace

import pytest

from cereal import messaging
from openpilot.frogpilot.controls.frogpilot_planner import FrogPilotPlanner
from openpilot.selfdrive.selfdrived.events import Events


class Messages(dict):
  def all_checks(self, service_list=None):
    return True


def planner_state(tracking_lead):
  slc = SimpleNamespace(experimental_mode=False, map_speed_limit=0.0, mapbox_limit=0.0, next_speed_limit=0.0, overridden_speed=0.0,
                        source="None", speed_limit_changed_timer=0, unconfirmed_speed_limit=0.0)
  return SimpleNamespace(
    frogpilot_following=SimpleNamespace(acceleration_jerk=0.0, danger_factor=0.0, danger_jerk=0.0, speed_jerk=0.0, t_follow=1.45,
                                        desired_follow_distance=0),
    frogpilot_cem=SimpleNamespace(ce_status=0, experimental_mode=False, stop_light_detected=False),
    frogpilot_vcruise=SimpleNamespace(csc_controlling_speed=False, csc_target=0.0, csc=SimpleNamespace(enable_training=False), slc=slc,
                                      forcing_stop=False, tracked_model_length=0.0, slc_target=0.0, slc_offset=0.0),
    frogpilot_events=SimpleNamespace(events=Events(frogpilot=True), wheel_image_update_count=0),
    frogpilot_weather=SimpleNamespace(weather_id=0, increase_stopped_distance=0.0, is_daytime=True),
    frogpilot_acceleration=SimpleNamespace(max_accel=0.0, min_accel=0.0),
    gps_position={"bearing": 0.0}, gps_valid=False, lane_width_left=0.0, lane_width_right=0.0, lateral_check=False,
    road_curvature=0.0, v_cruise=0.0, tracking_lead=tracking_lead,
  )


@pytest.mark.parametrize("tracking_lead", [False, True])
def test_b11_frogpilot_plan_publishes_tracking_lead(tracking_lead):
  sent = []
  sm = Messages(frogpilotCarState=messaging.new_message("frogpilotCarState").frogpilotCarState)
  pm = SimpleNamespace(send=lambda service, msg: sent.append(msg))

  FrogPilotPlanner.publish(planner_state(tracking_lead), 0, sm, pm, SimpleNamespace(increase_stopped_distance=0.0))

  assert sent[0].frogpilotPlan.trackingLead is tracking_lead
