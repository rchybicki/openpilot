from types import SimpleNamespace

import pytest

from cereal import messaging
from openpilot.frogpilot.controls.lib.frogpilot_events import FrogPilotEvents
from openpilot.selfdrive.selfdrived.events import FrogPilotEventName

SERVICES = ("carControl", "carState", "frogpilotCarState", "frogpilotModelV2", "frogpilotSelfdriveState", "modelV2", "selfdriveState")


def make_events():
  planner = SimpleNamespace(frogpilot_cem=SimpleNamespace(stop_light_detected=False),
                            frogpilot_vcruise=SimpleNamespace(forcing_stop=False, slc=SimpleNamespace(speed_limit_changed_timer=0)),
                            lateral_acceleration=0.0, lead_one=SimpleNamespace(dRel=0.0, status=False, vLead=0.0),
                            lead_relevant=False, model_stopped=True, params=SimpleNamespace(get=lambda key: None))
  theme_manager = SimpleNamespace(update_wheel_image=lambda *args, **kwargs: None, restore_wheel_image=lambda: False)
  return FrogPilotEvents(planner, theme_manager), planner


def stopped_sm(model_speed=0.0):
  sm = {s: getattr(messaging.new_message(s), s) for s in SERVICES}
  sm["carState"].standstill = True
  sm["carState"].gearShifter = "drive"
  sm["modelV2"].velocity.t = [0.0, 6.0, 10.0]
  sm["modelV2"].velocity.x = [0.0, model_speed, model_speed]
  return sm


def toggles(alerts):
  return SimpleNamespace(always_on_lateral=False, current_holiday_theme="stock", green_light_alert=alerts, is_metric=True,
                         lead_departing_alert=alerts, nnff=False, random_events=False, speed_limit_changed_alert=False,
                         startup_alert_bottom="", startup_alert_top="")


def fired(events, name):
  return name in events.events.names


@pytest.mark.parametrize("alerts", [False, True])
def test_b06_lead_first_seen_at_standstill_departs(alerts):
  # the lead appears only after the car has stopped: the old branch sets the departure baseline at standstill
  events, planner = make_events()
  planner.lead_one = SimpleNamespace(dRel=5.0, status=True, vLead=0.0)
  events.update(False, stopped_sm(), toggles(alerts))
  assert not fired(events, FrogPilotEventName.leadDeparting)

  planner.lead_one = SimpleNamespace(dRel=7.0, status=True, vLead=2.0)
  events.update(False, stopped_sm(), toggles(alerts))
  assert fired(events, FrogPilotEventName.leadDeparting) == alerts   # the owner's stored LeadDepartingAlert 0: never

