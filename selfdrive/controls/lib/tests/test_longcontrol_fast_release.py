import pytest

from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
from openpilot.selfdrive.controls.lib import longcontrol as longcontrol_module
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import (
  LongControl,
  LongCtrlState,
  apply_experimental_close_lead_accel_cap,
  experimental_close_lead_accel_cap,
  far_stopped_lead_brake_floor,
  far_stopped_lead_crawl_accel_cap,
  far_stopped_lead_settle_accel_cap,
  force_coast_no_target_pid_brake_cap,
  low_speed_close_lead_brake_step,
  low_speed_close_lead_accel_cap,
  low_speed_stopped_lead_glide_accel_cap,
  pid_slowing_lead_approach_accel_cap,
  pid_stopped_lead_approach_accel_cap,
  pid_stopped_lead_approach_brake_step,
  should_apply_pid_stopped_lead_approach_accel_cap,
  should_apply_stopping_planner_floor,
  stopping_planner_floor_active,
  STOPPING_PLANNER_FLOOR_V_EGO_MIN,
  STOPPING_PLANNER_FLOOR_GAP_MAX_M,
  STOPPING_PLANNER_FLOOR_A_TARGET_MAX,
  should_hold_recent_close_stopped_lead_dropout,
  should_observe_pid_stopping_shadow,
  should_apply_stop_entry_handoff_soften,
  should_apply_stop_target_approach_mode,
  should_apply_stop_target_carry_mode,
  should_enter_stop_target_mode,
  should_hold_low_speed_stop_target_release,
  should_hold_stop_target_mode,
  should_release_far_stopped_lead_gap,
  stop_entry_handoff_accel_cap,
  stop_target_approach_accel_cap,
  stop_target_carry_accel_floor,
)
from openpilot.frogpilot.controls.lib.force_coast import FORCE_COAST_RAMP_IN_S, get_force_coast_ramped_accel, get_force_coast_target_from_toggles


@pytest.fixture(autouse=True)
def _pin_legacy_service_mode(monkeypatch):
  # Stage-2 LIVE_TERMINAL (stopping_flags.SERVICE_MODE, plan §6) hands the sub-0.85 stopping wire to
  # the V3 service. Every wire pin in this file targets the LEGACY writer chain, which stays fully
  # computed on every frame and is exactly the SERVICE_MODE == "SHADOW" behavior (the byte-identical
  # one-flag revert path) -- so pin the mode to SHADOW here. LIVE_TERMINAL wire behavior is covered
  # by test_longcontrol_live_terminal.py.
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "SHADOW")


class DummyCruiseState:
  def __init__(self, standstill: bool = False) -> None:
    self.standstill = standstill


class DummyCarState:
  def __init__(
    self,
    v_ego: float,
    a_ego: float,
    brake_pressed: bool = False,
    standstill: bool = False,
    cruise_standstill: bool = False,
  ) -> None:
    self.vEgo = v_ego
    self.aEgo = a_ego
    self.brakePressed = brake_pressed
    self.standstill = standstill
    self.cruiseState = DummyCruiseState(standstill=cruise_standstill)


class DummyLongitudinalTuning:
  def __init__(self) -> None:
    self.kpBP = [0.0]
    self.kpV = [1.0]
    self.kiBP = [0.0]
    self.kiV = [0.0]


class DummyCarParams:
  def __init__(self, car_fingerprint=HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022) -> None:
    self.longitudinalTuning = DummyLongitudinalTuning()
    self.carFingerprint = car_fingerprint
    self.enableGasInterceptor = False
    self.startingState = False
    self.stopAccel = -1.0


class DummyFrogPilotToggles:
  def __init__(self) -> None:
    self.vEgoStarting = 0.1
    self.vEgoStopping = 0.2
    self.force_coast_strength = 1.0
    self.human_acceleration = False
    self.startAccel = 1.0


class SpyStoppingController:
  def __init__(self) -> None:
    self.distance_to_stop_target_m = None
    self.raw_should_stop = None

  def reset(self) -> None:
    return None

  def update(self, **kwargs):
    self.distance_to_stop_target_m = kwargs.get("distance_to_stop_target_m")
    self.raw_should_stop = kwargs.get("raw_should_stop")
    return type("StopResult", (), {"output_accel": kwargs["output_accel"], "release_lock_active": False})()


class ResetTrackingStoppingController:
  def __init__(self) -> None:
    self.reset_calls = 0
    self.update_calls = 0

  def reset(self) -> None:
    self.reset_calls += 1

  def update(self, **kwargs):
    self.update_calls += 1
    return type("StopResult", (), {"output_accel": kwargs["output_accel"], "release_lock_active": False})()


class FixedStoppingController:
  def __init__(self, output_accel: float) -> None:
    self.output_accel = output_accel

  def reset(self) -> None:
    return None

  def update(self, **kwargs):
    return type("StopResult", (), {"output_accel": self.output_accel, "release_lock_active": False})()


@pytest.mark.parametrize("freeze_integrator", [False, True])
def test_longcontrol_forwards_external_integrator_freeze(monkeypatch, freeze_integrator) -> None:
  cp = DummyCarParams(car_fingerprint=HYUNDAI_CAR.HYUNDAI_ELANTRA_2021)
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  forwarded = []

  def pid_update(error, error_rate=0.0, speed=0.0, feedforward=0.0, freeze_integrator=False):
    forwarded.append(freeze_integrator)
    return feedforward

  monkeypatch.setattr(lc.pid, "update", pid_update)
  lc.update(
    active=True,
    CS=DummyCarState(v_ego=10.0, a_ego=0.0),
    a_target=0.2,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    freeze_integrator=freeze_integrator,
  )

  assert forwarded == [freeze_integrator]


def test_longcontrol_blocks_fast_release_without_standstill_when_stop_intent_recent() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)

  cs = DummyCarState(v_ego=0.2, a_ego=-0.2, standstill=False, cruise_standstill=False)
  accel_limits = (-3.0, 2.0)

  lc.last_output_accel = -0.20
  lc.update(active=True, CS=cs, a_target=-0.2, should_stop=True, distance_to_stop_target_m=-1.0, accel_limits=accel_limits, frogpilot_toggles=toggles)

  lc.last_output_accel = -0.20
  out = lc.update(active=True, CS=cs, a_target=1.0, should_stop=False, distance_to_stop_target_m=-1.0, accel_limits=accel_limits, frogpilot_toggles=toggles)

  # Without the stop-intent guard, allow_fast_release would be active (resume intent) and the low-speed slew would permit a +0.026 step -> -0.174.
  # With the guard active, we use the default low-speed release step (0.004 at v=0.2) -> -0.196.
  assert out == pytest.approx(-0.196, abs=1e-12)


def test_longcontrol_disables_human_acceleration_takeoff_in_experimental_mode() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  toggles.human_acceleration = True
  toggles.startAccel = 0.6
  toggles.vEgoStarting = 2.0
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.starting

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=1.3, a_ego=0.0, standstill=False, cruise_standstill=False),
    a_target=1.2,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=True,
  )

  assert out == pytest.approx(0.6, abs=1e-12)
  assert lc.long_control_state == LongCtrlState.starting


def test_longcontrol_releases_standstill_hold_for_departing_lead_even_if_should_stop_is_still_latched() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  toggles.human_acceleration = True
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-1.06)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -1.06
  lc.arbiter.time_since_standstill_s = 0.0
  lc.arbiter.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.01, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.0,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=True,
    lead_status=True,
    lead_v=0.8,
    lead_d_rel=5.8,
    force_coast=False,
  )

  assert lc.long_control_state == LongCtrlState.starting
  assert out == pytest.approx(-1.0302, abs=1e-12)
  assert out > -1.06


def test_longcontrol_holds_bookmarked_tight_departing_lead_seed() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  toggles.human_acceleration = True
  lc = LongControl(cp)
  tracker = ResetTrackingStoppingController()
  lc.stopping_controller = tracker
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.456
  lc.arbiter.time_since_standstill_s = 0.0
  lc.arbiter.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.0, a_ego=-0.006, standstill=True, cruise_standstill=False),
    a_target=-0.001,
    should_stop=True,
    distance_to_stop_target_m=0.376,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=True,
    lead_status=True,
    lead_v=0.36,
    lead_d_rel=3.49,
    force_coast=False,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert tracker.update_calls == 1
  assert tracker.reset_calls == 0
  assert out == pytest.approx(-0.456, abs=1e-12)


def test_longcontrol_holds_stopping_for_creeping_departing_lead_seed() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  toggles.human_acceleration = True
  lc = LongControl(cp)
  tracker = ResetTrackingStoppingController()
  lc.stopping_controller = tracker
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.12
  lc.arbiter.time_since_standstill_s = 0.0
  lc.arbiter.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.0,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=True,
    lead_status=True,
    lead_v=0.56,
    lead_d_rel=4.90,
    force_coast=False,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert tracker.update_calls == 1
  assert tracker.reset_calls == 0
  assert out == pytest.approx(-0.12, abs=1e-12)


def test_longcontrol_departing_lead_launch_overrides_lingering_negative_start_target() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  toggles.human_acceleration = True
  toggles.startAccel = 0.6
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.starting
  lc.last_output_accel = -0.32
  lc.arbiter.time_since_standstill_s = 0.0
  lc.arbiter.time_since_stop_intent_s = 0.4

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=-0.35,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=False,
    lead_status=True,
    lead_v=3.49,
    lead_d_rel=5.40,
    force_coast=False,
  )

  assert lc.long_control_state == LongCtrlState.starting
  assert out == pytest.approx(-0.29, abs=1e-12)
  assert out > -0.32


def test_longcontrol_force_coast_blocks_departing_lead_launch_floor() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  toggles.human_acceleration = True
  toggles.startAccel = 0.6
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.starting
  lc.last_output_accel = -0.32
  lc.arbiter.time_since_standstill_s = 0.0
  lc.arbiter.time_since_stop_intent_s = 0.4

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=-0.35,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=False,
    lead_status=True,
    lead_v=3.49,
    lead_d_rel=5.40,
    force_coast=True,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out == pytest.approx(-0.32, abs=1e-12)


def test_longcontrol_forwards_distance_to_stop_target_into_stopping_controller() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  spy = SpyStoppingController()
  lc.stopping_controller = spy
  lc.long_control_state = LongCtrlState.stopping

  cs = DummyCarState(v_ego=0.35, a_ego=-0.25, standstill=False, cruise_standstill=False)
  accel_limits = (-3.0, 2.0)

  lc.last_output_accel = -0.20
  lc.update(
    active=True,
    CS=cs,
    a_target=-0.2,
    should_stop=True,
    distance_to_stop_target_m=1.75,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert spy.distance_to_stop_target_m == pytest.approx(1.75, abs=1e-12)


def test_longcontrol_forwards_raw_should_stop_false_when_stop_target_enters_stopping() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  spy = SpyStoppingController()
  lc.stopping_controller = spy
  lc.long_control_state = LongCtrlState.pid

  cs = DummyCarState(v_ego=2.2, a_ego=-0.15, standstill=False, cruise_standstill=False)
  accel_limits = (-3.0, 2.0)

  lc.last_output_accel = -0.18
  lc.update(
    active=True,
    CS=cs,
    a_target=-0.25,
    should_stop=False,
    distance_to_stop_target_m=0.9,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert spy.raw_should_stop is False


def test_should_enter_stop_target_mode_only_when_close_and_braking() -> None:
  assert should_enter_stop_target_mode(v_ego=2.2, a_target=-0.20, distance_to_stop_target_m=0.9)
  assert not should_enter_stop_target_mode(v_ego=2.2, a_target=-0.05, distance_to_stop_target_m=0.9)
  assert not should_enter_stop_target_mode(v_ego=2.2, a_target=-0.20, distance_to_stop_target_m=3.5)
  assert not should_enter_stop_target_mode(v_ego=2.2, a_target=-0.20, distance_to_stop_target_m=0.18)


def test_should_apply_stop_target_approach_mode_only_in_midband() -> None:
  assert should_apply_stop_target_approach_mode(v_ego=3.2, a_target=-0.10, distance_to_stop_target_m=1.8)
  assert not should_apply_stop_target_approach_mode(v_ego=3.2, a_target=-0.03, distance_to_stop_target_m=1.8)
  assert not should_apply_stop_target_approach_mode(v_ego=3.2, a_target=-0.10, distance_to_stop_target_m=4.0)
  assert not should_apply_stop_target_approach_mode(v_ego=3.2, a_target=-0.20, distance_to_stop_target_m=0.8)


def test_should_apply_stop_target_carry_mode_for_low_speed_far_target() -> None:
  assert should_apply_stop_target_carry_mode(v_ego=1.232, a_target=-0.6166, distance_to_stop_target_m=4.472)
  assert not should_apply_stop_target_carry_mode(v_ego=1.232, a_target=-0.04, distance_to_stop_target_m=4.472)
  assert not should_apply_stop_target_carry_mode(v_ego=1.232, a_target=-0.6166, distance_to_stop_target_m=0.9)
  assert not should_apply_stop_target_carry_mode(v_ego=2.2, a_target=-0.58, distance_to_stop_target_m=4.47)


def test_stop_target_approach_mode_can_own_tiny_meaningful_target_before_full_stop_mode() -> None:
  assert should_apply_stop_target_approach_mode(v_ego=4.2, a_target=-1.20, distance_to_stop_target_m=0.15)
  assert not should_enter_stop_target_mode(v_ego=4.2, a_target=-1.20, distance_to_stop_target_m=0.15)
  assert should_enter_stop_target_mode(v_ego=4.2, a_target=-1.20, distance_to_stop_target_m=0.23)
  assert should_enter_stop_target_mode(v_ego=3.95, a_target=-1.27, distance_to_stop_target_m=0.61)


def test_stop_target_mode_waits_for_more_than_tiny_target_when_nonurgent() -> None:
  assert not should_enter_stop_target_mode(v_ego=2.30, a_target=-0.335, distance_to_stop_target_m=0.28)
  assert should_apply_stop_target_approach_mode(v_ego=2.30, a_target=-0.335, distance_to_stop_target_m=0.28)


def test_stop_target_mode_holds_once_target_is_stable_and_braking_persists() -> None:
  assert should_hold_stop_target_mode(v_ego=2.13, a_target=-0.283, distance_to_stop_target_m=1.68)
  assert not should_hold_stop_target_mode(v_ego=2.13, a_target=-0.05, distance_to_stop_target_m=1.68)
  assert not should_hold_stop_target_mode(v_ego=2.13, a_target=-0.283, distance_to_stop_target_m=2.20)


def test_stop_target_approach_accel_cap_prefers_more_brake_when_closer_or_faster() -> None:
  far_cap = stop_target_approach_accel_cap(v_ego=3.0, distance_to_stop_target_m=2.8)
  close_cap = stop_target_approach_accel_cap(v_ego=3.0, distance_to_stop_target_m=1.2)
  fast_cap = stop_target_approach_accel_cap(v_ego=5.0, distance_to_stop_target_m=2.8)

  assert close_cap < far_cap
  assert fast_cap < far_cap


def test_stop_target_carry_accel_floor_relaxes_more_when_target_is_farther() -> None:
  close_floor = stop_target_carry_accel_floor(v_ego=1.0, distance_to_stop_target_m=1.8)
  far_floor = stop_target_carry_accel_floor(v_ego=1.0, distance_to_stop_target_m=4.5)

  assert far_floor > close_floor


def test_should_apply_stop_entry_handoff_soften_only_for_non_urgent_deep_inherited_brake() -> None:
  assert should_apply_stop_entry_handoff_soften(
    v_ego=1.0,
    a_ego=-0.75,
    a_target=-0.30,
    last_output_accel=-0.70,
    distance_to_stop_target_m=0.9,
  )
  assert not should_apply_stop_entry_handoff_soften(
    v_ego=1.0,
    a_ego=-0.75,
    a_target=-0.70,
    last_output_accel=-0.70,
    distance_to_stop_target_m=0.9,
  )
  assert not should_apply_stop_entry_handoff_soften(
    v_ego=1.0,
    a_ego=-0.75,
    a_target=-0.30,
    last_output_accel=-0.70,
    distance_to_stop_target_m=0.10,
  )


def test_should_apply_stop_entry_handoff_soften_extends_to_moderate_speed_non_urgent_handoff() -> None:
  assert should_apply_stop_entry_handoff_soften(
    v_ego=1.85,
    a_ego=-0.72,
    a_target=-0.42,
    last_output_accel=-0.74,
    distance_to_stop_target_m=1.10,
  )
  assert not should_apply_stop_entry_handoff_soften(
    v_ego=1.85,
    a_ego=-0.72,
    a_target=-0.68,
    last_output_accel=-0.74,
    distance_to_stop_target_m=1.10,
  )


def test_stop_entry_handoff_accel_cap_prefers_gentler_handoff_when_target_is_farther() -> None:
  near_cap = stop_entry_handoff_accel_cap(v_ego=1.0, distance_to_stop_target_m=0.25)
  far_cap = stop_entry_handoff_accel_cap(v_ego=1.0, distance_to_stop_target_m=1.2)
  assert far_cap > near_cap


def test_stop_entry_handoff_accel_cap_allows_moderate_speed_soften_without_forcing_urgent_close_stop_shape() -> None:
  moderate_speed_far = stop_entry_handoff_accel_cap(v_ego=1.85, distance_to_stop_target_m=1.10)
  moderate_speed_close = stop_entry_handoff_accel_cap(v_ego=1.85, distance_to_stop_target_m=0.30)
  assert moderate_speed_far > moderate_speed_close
  assert moderate_speed_far > -0.70


def test_should_hold_low_speed_stop_target_release_for_route_shaped_stop_go_case() -> None:
  assert should_hold_low_speed_stop_target_release(
    v_ego=0.037,
    a_target=0.11,
    distance_to_stop_target_m=0.593,
    last_distance_to_stop_target_m=0.694,
    last_output_accel=-0.42,
    time_since_stop_intent_s=0.0,
  )
  assert should_hold_low_speed_stop_target_release(
    v_ego=0.149,
    a_target=-0.417,
    distance_to_stop_target_m=1.633,
    last_distance_to_stop_target_m=1.633,
    last_output_accel=-0.523,
    time_since_stop_intent_s=0.0,
  )
  assert not should_hold_low_speed_stop_target_release(
    v_ego=0.037,
    a_target=0.11,
    distance_to_stop_target_m=0.18,
    last_distance_to_stop_target_m=0.694,
    last_output_accel=-0.42,
    time_since_stop_intent_s=0.0,
  )
  assert not should_hold_low_speed_stop_target_release(
    v_ego=0.037,
    a_target=0.11,
    distance_to_stop_target_m=0.88,
    last_distance_to_stop_target_m=0.694,
    last_output_accel=-0.42,
    time_since_stop_intent_s=0.0,
  )


def test_longcontrol_enters_stopping_early_for_close_stopped_lead_target() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  spy = SpyStoppingController()
  lc.stopping_controller = spy
  lc.long_control_state = LongCtrlState.pid

  cs = DummyCarState(v_ego=2.2, a_ego=-0.15, standstill=False, cruise_standstill=False)
  accel_limits = (-3.0, 2.0)

  lc.last_output_accel = -0.18
  lc.update(
    active=True,
    CS=cs,
    a_target=-0.25,
    should_stop=False,
    distance_to_stop_target_m=0.9,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert spy.distance_to_stop_target_m == pytest.approx(0.9, abs=1e-12)


def test_longcontrol_stays_in_pid_when_stopped_lead_target_is_still_far() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid

  cs = DummyCarState(v_ego=2.2, a_ego=-0.15, standstill=False, cruise_standstill=False)
  accel_limits = (-3.0, 2.0)

  lc.last_output_accel = -0.18
  lc.update(
    active=True,
    CS=cs,
    a_target=-0.25,
    should_stop=False,
    distance_to_stop_target_m=3.5,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.pid


def test_longcontrol_plain_pid_braking_stays_close_to_model_request() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid

  cs = DummyCarState(v_ego=2.0, a_ego=-0.3, standstill=False, cruise_standstill=False)
  accel_limits = (-3.0, 2.0)

  out = lc.update(
    active=True,
    CS=cs,
    a_target=-0.8,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  # Raw PID would request -1.3 here; keep the plain braking path within a small fixed margin of the model request instead.
  assert out == pytest.approx(-0.88, abs=1e-12)
  assert out > -1.0
  assert lc.pid.i == pytest.approx(0.0, abs=1e-12)


def test_longcontrol_plain_pid_braking_alignment_is_santa_fe_only() -> None:
  cp = DummyCarParams(car_fingerprint=HYUNDAI_CAR.HYUNDAI_ELANTRA_2021)
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=2.0, a_ego=-0.3, standstill=False, cruise_standstill=False),
    a_target=-0.8,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
  )

  assert out == pytest.approx(-1.3, abs=1e-12)
  assert lc.pid.i == pytest.approx(0.0, abs=1e-12)


def test_experimental_close_lead_accel_cap_route_535_pre_stop_seed() -> None:
  cap = experimental_close_lead_accel_cap(
    v_ego=12.72,
    lead_v=13.64,
    lead_d_rel=25.89,
  )

  assert cap == pytest.approx(0.115, abs=0.002)


def test_experimental_close_lead_accel_cap_ignores_far_or_departing_lead() -> None:
  assert experimental_close_lead_accel_cap(v_ego=13.4, lead_v=18.4, lead_d_rel=67.0) is None
  assert experimental_close_lead_accel_cap(v_ego=13.4, lead_v=18.0, lead_d_rel=28.0) is None


def test_low_speed_close_lead_accel_cap_bookmarked_too_close_seed() -> None:
  cap = low_speed_close_lead_accel_cap(
    v_ego=0.45,
    lead_v=-0.06,
    lead_d_rel=2.40,
  )

  assert cap <= -0.78
  assert low_speed_close_lead_accel_cap(v_ego=0.45, lead_v=0.50, lead_d_rel=2.40) is None
  assert low_speed_close_lead_accel_cap(v_ego=0.45, lead_v=-0.06, lead_d_rel=3.90) is None


def test_low_speed_close_lead_accel_cap_activates_before_min_hold_gap_seed_1628() -> None:
  cap = low_speed_close_lead_accel_cap(
    v_ego=0.47,
    lead_v=0.0,
    lead_d_rel=2.90,
  )

  assert cap is not None
  assert cap <= -0.60
  assert low_speed_close_lead_brake_step(0.47, 2.90) < 0.006


def test_low_speed_close_lead_accel_cap_activates_at_new_comfort_gap() -> None:
  cap = low_speed_close_lead_accel_cap(
    v_ego=0.47,
    lead_v=0.0,
    lead_d_rel=3.40,
  )

  assert cap is not None
  assert cap <= -0.55
  assert low_speed_close_lead_brake_step(0.47, 3.40) <= 0.006


def test_low_speed_stopped_lead_glide_accel_cap_bookmarked_far_gap_seed() -> None:
  cap = low_speed_stopped_lead_glide_accel_cap(
    v_ego=0.83,
    lead_v=-0.09,
    lead_d_rel=7.40,
    distance_to_stop_target_m=4.40,
  )

  assert cap == pytest.approx(-0.48, abs=0.02)
  assert low_speed_stopped_lead_glide_accel_cap(v_ego=0.83, lead_v=0.55, lead_d_rel=7.40, distance_to_stop_target_m=4.40) is None
  assert low_speed_stopped_lead_glide_accel_cap(v_ego=0.83, lead_v=-0.09, lead_d_rel=9.00, distance_to_stop_target_m=4.40) is None


def test_low_speed_stopped_lead_glide_cap_protects_min_hold_gap_seed_1628() -> None:
  cap = low_speed_stopped_lead_glide_accel_cap(
    v_ego=0.388,
    lead_v=0.0,
    lead_d_rel=2.80,
    distance_to_stop_target_m=-1.0,
  )

  assert cap is not None
  assert cap <= -0.55


def test_low_speed_stopped_lead_glide_cap_catches_new_comfort_gap() -> None:
  cap = low_speed_stopped_lead_glide_accel_cap(
    v_ego=0.47,
    lead_v=0.0,
    lead_d_rel=3.40,
    distance_to_stop_target_m=-1.0,
  )

  assert cap is not None
  assert cap <= -0.54


def test_low_speed_stopped_lead_glide_accel_cap_blocks_route_90b_pre_stop_release() -> None:
  cap = low_speed_stopped_lead_glide_accel_cap(
    v_ego=1.17,
    lead_v=-0.02,
    lead_d_rel=5.80,
    distance_to_stop_target_m=4.40,
  )

  assert cap == pytest.approx(-0.56, abs=0.03)


def test_low_speed_stopped_lead_glide_accel_cap_blocks_route_90b_near_standstill_positive_release() -> None:
  cap = low_speed_stopped_lead_glide_accel_cap(
    v_ego=0.02,
    lead_v=0.00,
    lead_d_rel=5.10,
    distance_to_stop_target_m=0.0,
  )

  assert cap == pytest.approx(-0.20, abs=0.02)
  assert low_speed_stopped_lead_glide_accel_cap(v_ego=0.02, lead_v=0.30, lead_d_rel=5.10, distance_to_stop_target_m=0.0) == pytest.approx(-0.20, abs=0.02)
  assert low_speed_stopped_lead_glide_accel_cap(v_ego=0.02, lead_v=1.20, lead_d_rel=5.10, distance_to_stop_target_m=0.0) is None
  assert low_speed_stopped_lead_glide_accel_cap(v_ego=0.02, lead_v=0.00, lead_d_rel=7.20, distance_to_stop_target_m=0.0) is None


def test_far_stopped_lead_gap_release_triggers_until_explicit_target_is_close() -> None:
  assert should_release_far_stopped_lead_gap(v_ego=0.05, lead_status=True, lead_v=0.0, lead_d_rel=8.99, distance_to_stop_target_m=-1.0)
  assert should_release_far_stopped_lead_gap(v_ego=0.05, lead_status=True, lead_v=0.0, lead_d_rel=8.99, distance_to_stop_target_m=None)

  assert should_release_far_stopped_lead_gap(v_ego=0.05, lead_status=True, lead_v=0.0, lead_d_rel=5.10, distance_to_stop_target_m=-1.0)
  assert should_release_far_stopped_lead_gap(v_ego=0.05, lead_status=True, lead_v=0.0, lead_d_rel=8.99, distance_to_stop_target_m=4.40)
  assert not should_release_far_stopped_lead_gap(v_ego=0.05, lead_status=True, lead_v=0.0, lead_d_rel=4.90, distance_to_stop_target_m=-1.0)
  assert not should_release_far_stopped_lead_gap(v_ego=0.05, lead_status=True, lead_v=0.0, lead_d_rel=8.99, distance_to_stop_target_m=1.20)
  assert not should_release_far_stopped_lead_gap(v_ego=0.05, lead_status=True, lead_v=0.8, lead_d_rel=8.99, distance_to_stop_target_m=-1.0)
  assert not should_release_far_stopped_lead_gap(v_ego=0.90, lead_status=True, lead_v=0.0, lead_d_rel=8.99, distance_to_stop_target_m=-1.0)


def test_far_stopped_lead_brake_floor_softens_as_gap_grows() -> None:
  assert far_stopped_lead_brake_floor(v_ego=0.05, lead_d_rel=8.99) == pytest.approx(-0.08, abs=0.01)
  assert far_stopped_lead_brake_floor(v_ego=0.35, lead_d_rel=5.10) == pytest.approx(-0.154, abs=0.01)


def test_far_stopped_lead_settle_accel_cap_guards_explicit_target_release() -> None:
  assert far_stopped_lead_settle_accel_cap(v_ego=0.215, lead_d_rel=6.90, distance_to_stop_target_m=3.61) == pytest.approx(-0.257, abs=0.01)
  assert far_stopped_lead_settle_accel_cap(v_ego=0.215, lead_d_rel=6.90, distance_to_stop_target_m=-1.0) is None
  assert far_stopped_lead_settle_accel_cap(v_ego=0.215, lead_d_rel=6.90, distance_to_stop_target_m=1.20) is None
  assert far_stopped_lead_settle_accel_cap(v_ego=0.60, lead_d_rel=6.90, distance_to_stop_target_m=3.61) is None


def test_low_speed_stopped_lead_glide_accel_cap_ignores_no_target_far_gap() -> None:
  assert low_speed_stopped_lead_glide_accel_cap(v_ego=0.05, lead_v=0.0, lead_d_rel=8.99, distance_to_stop_target_m=-1.0) is None
  assert low_speed_stopped_lead_glide_accel_cap(v_ego=0.05, lead_v=0.0, lead_d_rel=8.99, distance_to_stop_target_m=None) is None
  assert low_speed_stopped_lead_glide_accel_cap(v_ego=0.05, lead_v=0.0, lead_d_rel=5.10, distance_to_stop_target_m=-1.0) is None
  assert low_speed_stopped_lead_glide_accel_cap(v_ego=0.05, lead_v=0.0, lead_d_rel=4.90, distance_to_stop_target_m=-1.0) is not None
  assert low_speed_stopped_lead_glide_accel_cap(v_ego=0.83, lead_v=-0.09, lead_d_rel=7.40, distance_to_stop_target_m=4.40) is not None


def test_pid_stopped_lead_approach_accel_cap_route_1755_seed() -> None:
  early_cap = pid_stopped_lead_approach_accel_cap(
    v_ego=17.62,
    lead_v=7.18,
    lead_d_rel=70.47,
  )
  mid_cap = pid_stopped_lead_approach_accel_cap(
    v_ego=16.61,
    lead_v=5.87,
    lead_d_rel=59.07,
  )
  late_cap = pid_stopped_lead_approach_accel_cap(
    v_ego=14.99,
    lead_v=5.27,
    lead_d_rel=51.11,
  )

  assert early_cap == pytest.approx(-0.99, abs=0.03)
  assert mid_cap == pytest.approx(-1.18, abs=0.03)
  assert late_cap == pytest.approx(-1.25, abs=0.03)
  assert early_cap > mid_cap > late_cap


def test_pid_stopped_lead_approach_accel_cap_ignores_normal_following() -> None:
  assert pid_stopped_lead_approach_accel_cap(v_ego=17.62, lead_v=12.80, lead_d_rel=70.47) is None
  assert pid_stopped_lead_approach_accel_cap(v_ego=17.62, lead_v=9.20, lead_d_rel=70.47) is None
  assert pid_stopped_lead_approach_accel_cap(v_ego=17.62, lead_v=7.18, lead_d_rel=90.00) is None
  assert pid_stopped_lead_approach_accel_cap(v_ego=5.90, lead_v=0.20, lead_d_rel=22.00) is None


def test_pid_slowing_lead_approach_cap_route_1b09_seed() -> None:
  cap = pid_slowing_lead_approach_accel_cap(
    v_ego=11.18,
    lead_v=8.79,
    lead_d_rel=15.30,
    lead_a=-1.46,
  )

  assert cap == pytest.approx(-1.70, abs=0.05)


def test_pid_slowing_lead_approach_cap_ignores_normal_following() -> None:
  assert pid_slowing_lead_approach_accel_cap(v_ego=11.76, lead_v=9.60, lead_d_rel=13.60, lead_a=-0.30) is None
  assert pid_slowing_lead_approach_accel_cap(v_ego=11.76, lead_v=11.00, lead_d_rel=35.00, lead_a=-1.20) is None
  assert pid_slowing_lead_approach_accel_cap(v_ego=17.50, lead_v=16.20, lead_d_rel=27.00, lead_a=-0.35) is None


def test_longcontrol_caps_experimental_close_lead_accel_chase_for_santa_fe() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=12.72, a_ego=0.43, standstill=False, cruise_standstill=False),
    a_target=0.567,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=True,
    lead_status=True,
    lead_v=13.64,
    lead_d_rel=25.89,
  )

  close_lead_cap = experimental_close_lead_accel_cap(12.72, 13.64, 25.89)
  assert close_lead_cap is not None
  assert out == pytest.approx(apply_experimental_close_lead_accel_cap(0.704, close_lead_cap), abs=1e-12)
  assert close_lead_cap < out < 0.45


def test_longcontrol_zero_ki_close_lead_cap_does_not_poison_later_boost() -> None:
  cp = DummyCarParams()
  cp.longitudinalTuning.kpV = [0.0]
  cp.longitudinalTuning.kiV = [0.0]
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid

  close_out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=12.72, a_ego=0.43, standstill=False, cruise_standstill=False),
    a_target=0.704,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=True,
    lead_status=True,
    lead_v=13.64,
    lead_d_rel=25.89,
  )

  assert close_out < 0.45
  assert lc.pid.i == pytest.approx(0.0, abs=1e-12)

  far_out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=12.07, a_ego=0.02, standstill=False, cruise_standstill=False),
    a_target=0.739,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=True,
    lead_status=True,
    lead_v=14.80,
    lead_d_rel=69.90,
  )

  assert far_out == pytest.approx(0.739, abs=1e-12)
  assert lc.pid.i == pytest.approx(0.0, abs=1e-12)


def test_longcontrol_pid_stopped_lead_approach_adds_early_brake_for_santa_fe() -> None:
  cp = DummyCarParams()
  cp.longitudinalTuning.kpV = [0.0]
  cp.longitudinalTuning.kiV = [0.0]
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.41

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=17.62, a_ego=-0.25, standstill=False, cruise_standstill=False),
    a_target=-0.41,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=False,
    lead_status=True,
    lead_v=7.18,
    lead_d_rel=70.47,
  )

  cap = pid_stopped_lead_approach_accel_cap(17.62, 7.18, 70.47)
  assert cap is not None
  assert out == pytest.approx(-0.41 - pid_stopped_lead_approach_brake_step(17.62), abs=1e-12)
  assert cap < out < -0.41
  assert lc.pid.i == pytest.approx(0.0, abs=1e-12)


def test_longcontrol_pid_stopped_lead_approach_does_not_weaken_hard_planner_brake() -> None:
  cp = DummyCarParams()
  cp.longitudinalTuning.kpV = [0.0]
  cp.longitudinalTuning.kiV = [0.0]
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -1.30

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=17.62, a_ego=-1.20, standstill=False, cruise_standstill=False),
    a_target=-1.30,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=False,
    lead_status=True,
    lead_v=7.18,
    lead_d_rel=70.47,
  )

  assert pid_stopped_lead_approach_accel_cap(17.62, 7.18, 70.47) > out
  assert out == pytest.approx(-1.30, abs=1e-12)


def test_longcontrol_pid_slowing_lead_approach_adds_early_brake_for_santa_fe() -> None:
  cp = DummyCarParams()
  cp.longitudinalTuning.kpV = [0.0]
  cp.longitudinalTuning.kiV = [0.0]
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -1.62

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=11.18, a_ego=-1.31, standstill=False, cruise_standstill=False),
    a_target=-1.62,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=False,
    lead_status=True,
    lead_v=8.79,
    lead_d_rel=15.30,
    lead_a=-1.46,
  )

  assert pid_slowing_lead_approach_accel_cap(11.18, 8.79, 15.30, -1.46) < out < -1.62
  assert out == pytest.approx(-1.62 - pid_stopped_lead_approach_brake_step(11.18), abs=1e-12)
  assert lc.pid.i == pytest.approx(0.0, abs=1e-12)


def test_longcontrol_pid_slowing_lead_approach_does_not_weaken_hard_planner_brake() -> None:
  cp = DummyCarParams()
  cp.longitudinalTuning.kpV = [0.0]
  cp.longitudinalTuning.kiV = [0.0]
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -2.30

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=7.28, a_ego=-2.25, standstill=False, cruise_standstill=False),
    a_target=-2.34,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=False,
    lead_status=True,
    lead_v=2.20,
    lead_d_rel=13.45,
    lead_a=-1.60,
  )

  assert out == pytest.approx(-2.34, abs=1e-12)


def test_longcontrol_close_lead_accel_cap_is_santa_fe_experimental_only() -> None:
  toggles = DummyFrogPilotToggles()

  non_experimental = LongControl(DummyCarParams())
  non_experimental.long_control_state = LongCtrlState.pid
  out_non_experimental = non_experimental.update(
    active=True,
    CS=DummyCarState(v_ego=12.72, a_ego=0.43, standstill=False, cruise_standstill=False),
    a_target=0.567,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=False,
    lead_status=True,
    lead_v=13.64,
    lead_d_rel=25.89,
  )

  other_car = LongControl(DummyCarParams(car_fingerprint=HYUNDAI_CAR.HYUNDAI_ELANTRA_2021))
  other_car.long_control_state = LongCtrlState.pid
  out_other_car = other_car.update(
    active=True,
    CS=DummyCarState(v_ego=12.72, a_ego=0.43, standstill=False, cruise_standstill=False),
    a_target=0.567,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=True,
    lead_status=True,
    lead_v=13.64,
    lead_d_rel=25.89,
  )

  assert out_non_experimental == pytest.approx(0.704, abs=1e-12)
  assert out_other_car == pytest.approx(0.704, abs=1e-12)


def test_longcontrol_pid_stopped_lead_approach_cap_is_santa_fe_only() -> None:
  cp = DummyCarParams(car_fingerprint=HYUNDAI_CAR.HYUNDAI_ELANTRA_2021)
  cp.longitudinalTuning.kpV = [0.0]
  cp.longitudinalTuning.kiV = [0.0]
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.41

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=17.62, a_ego=-0.25, standstill=False, cruise_standstill=False),
    a_target=-0.41,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=False,
    lead_status=True,
    lead_v=7.18,
    lead_d_rel=70.47,
  )

  assert out == pytest.approx(-0.41, abs=1e-12)
  assert should_apply_pid_stopped_lead_approach_accel_cap(DummyCarParams())
  assert not should_apply_pid_stopped_lead_approach_accel_cap(cp)


def test_longcontrol_caps_low_speed_close_lead_stop_unwind_for_santa_fe(monkeypatch) -> None:
  # KILL SWITCH OFF: this exercises the legacy low_speed_close_lead_accel_cap over-brake, which the
  # terminal-glide profile (default ON) retires (the corrected target + jerk-limited tracker own the
  # terminal approach). Pin the legacy patchwork behavior with the flag off.
  monkeypatch.setattr(stopping_flags, "SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED", False)
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-0.42)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.42

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.30, a_ego=-0.08, standstill=False, cruise_standstill=False),
    a_target=-0.11,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=-0.01,
    lead_d_rel=1.50,
  )

  assert out < -0.43
  assert out > low_speed_close_lead_accel_cap(0.30, -0.01, 1.50)


def test_longcontrol_low_speed_close_lead_stop_cap_is_santa_fe_only() -> None:
  toggles = DummyFrogPilotToggles()
  lc = LongControl(DummyCarParams(car_fingerprint=HYUNDAI_CAR.HYUNDAI_ELANTRA_2021))
  lc.stopping_controller = FixedStoppingController(output_accel=-0.42)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.42

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.30, a_ego=-0.08, standstill=False, cruise_standstill=False),
    a_target=-0.11,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=-0.01,
    lead_d_rel=1.50,
  )

  assert out == pytest.approx(-0.42, abs=1e-12)


def test_longcontrol_limits_far_gap_stopped_lead_glide_unwind_for_santa_fe(monkeypatch) -> None:
  # KILL SWITCH OFF: legacy low_speed_stopped_lead_glide_accel_cap over-brake (the binding leapfrog
  # cap the terminal-glide profile retires). Pin the legacy patchwork with the flag off.
  monkeypatch.setattr(stopping_flags, "SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED", False)
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.407

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.83, a_ego=-0.49, standstill=False, cruise_standstill=False),
    a_target=-0.50,
    should_stop=False,
    distance_to_stop_target_m=4.40,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=-0.09,
    lead_d_rel=7.40,
  )

  assert out < -0.41
  assert out > low_speed_stopped_lead_glide_accel_cap(0.83, -0.09, 7.40, 4.40)


def test_longcontrol_blocks_positive_release_while_stopped_lead_should_stop_remains_true(monkeypatch) -> None:
  # KILL SWITCH OFF: this asserts the legacy glide-cap value clamps a positive controller release.
  # The terminal-glide profile (default ON) retires the glide cap; the stop-intent state pin
  # (long_control_state == stopping) is preserved regardless and is covered by the flag-ON variant
  # below. Pin the legacy glide-cap clamp with the flag off.
  monkeypatch.setattr(stopping_flags, "SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED", False)
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=0.70)
  lc.long_control_state = LongCtrlState.starting
  lc.last_output_accel = -0.269

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.02, a_ego=-0.01, standstill=True, cruise_standstill=False),
    a_target=0.007,
    should_stop=True,
    # 0.05 is the closest runtime-reachable explicit target (STOP_TARGET_CLOSE_HOLD_REMAINING_M);
    # the planner never publishes exactly 0.0 (sentinel domain is -1.0 / strictly positive — the
    # stopped-lead fade tail can emit arbitrarily small positives; the synthetic stopped-lead
    # control target is additionally >= 0.05)
    distance_to_stop_target_m=0.05,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.00,
    lead_d_rel=5.10,
  )

  assert out == pytest.approx(low_speed_stopped_lead_glide_accel_cap(0.02, 0.00, 5.10, 0.05), abs=1e-12)
  assert out < -0.18
  assert lc.long_control_state == LongCtrlState.stopping


def test_longcontrol_terminal_glide_keeps_stop_intent_without_glide_overbrake() -> None:
  # TERMINAL-GLIDE PROFILE (flag ON, default): with the glide over-brake cap retired, the command is
  # no longer pulled to the legacy glide-cap value -- but the stop INTENT is unchanged. A STOPPED
  # lead at 5.10 m with should_stop True still pins the state machine in stopping (the seg24-class
  # anti-collision net stays live). The output here passes through the FixedStoppingController double
  # (the real V2 firm hold owns the terminal command; not modeled by the double), so this test pins
  # only the load-bearing state-machine invariant, not the double's passthrough value.
  assert stopping_flags.SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-0.30)
  lc.long_control_state = LongCtrlState.starting
  lc.last_output_accel = -0.269

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.02, a_ego=-0.01, standstill=True, cruise_standstill=False),
    a_target=0.007,
    should_stop=True,
    distance_to_stop_target_m=0.05,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.00,
    lead_d_rel=5.10,
  )

  # Stop intent preserved (the seg24-class net is intact); the glide-cap over-brake is bypassed, so
  # the controller's own command passes through untouched. The legacy glide cap (~-0.22 here) would
  # have RAISED the -0.30 command up to ~-0.22 (max(cap, ...)); with the cap retired the deeper -0.30
  # stands -- i.e. the command stays below the cap, proving the cap no longer rewrites it.
  assert lc.long_control_state == LongCtrlState.stopping
  assert out == pytest.approx(-0.30, abs=1e-12)
  assert out < low_speed_stopped_lead_glide_accel_cap(0.02, 0.00, 5.10, 0.05)


def test_longcontrol_releases_far_no_target_stopped_lead_gap_instead_of_hard_holding() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-1.05)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.44
  lc.arbiter.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.05, a_ego=-0.05, standstill=False, cruise_standstill=False),
    a_target=0.0,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=8.99,
  )

  assert lc.long_control_state == LongCtrlState.pid
  assert out == pytest.approx(-0.413, abs=1e-12)
  assert out > -0.44
  assert out <= far_stopped_lead_crawl_accel_cap(0.05, 8.99)
  assert out > -0.90


def test_longcontrol_force_coast_blocks_far_no_target_stopped_lead_release() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-0.50)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.50
  lc.arbiter.time_since_stop_intent_s = 2.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.009, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.0,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=24.26,
    force_coast=True,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out == pytest.approx(-0.50, abs=1e-12)


def test_longcontrol_force_coast_recovers_starting_state_at_standstill() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-0.30)
  lc.long_control_state = LongCtrlState.starting
  lc.last_output_accel = 0.40

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.009, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.4,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    force_coast=True,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out <= 0.0


def test_longcontrol_far_no_target_stopped_lead_release_is_santa_fe_only() -> None:
  toggles = DummyFrogPilotToggles()
  lc = LongControl(DummyCarParams(car_fingerprint=HYUNDAI_CAR.HYUNDAI_ELANTRA_2021))
  lc.stopping_controller = FixedStoppingController(output_accel=-1.05)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.44
  lc.arbiter.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.05, a_ego=-0.05, standstill=False, cruise_standstill=False),
    a_target=0.0,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=8.99,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out == pytest.approx(-1.05, abs=1e-12)


def test_longcontrol_releases_far_stopped_lead_until_explicit_target_is_close() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-1.05)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.44
  lc.arbiter.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.05, a_ego=-0.05, standstill=False, cruise_standstill=False),
    a_target=0.0,
    should_stop=True,
    distance_to_stop_target_m=4.40,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=8.99,
  )

  assert lc.long_control_state == LongCtrlState.pid
  assert out > -0.44
  assert out <= far_stopped_lead_crawl_accel_cap(0.05, 8.99)


def test_longcontrol_far_explicit_target_release_keeps_brake_settled() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-1.05)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.352
  lc.arbiter.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.215, a_ego=-0.602, standstill=False, cruise_standstill=False),
    a_target=-0.145,
    should_stop=True,
    distance_to_stop_target_m=3.608,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=6.90,
  )

  assert lc.long_control_state == LongCtrlState.pid
  assert out <= -0.24
  assert out > -0.40


def test_longcontrol_keeps_stopping_once_stopped_lead_gap_is_inside_band() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-1.05)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.44
  lc.arbiter.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.05, a_ego=-0.05, standstill=False, cruise_standstill=False),
    a_target=0.0,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=4.80,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out == pytest.approx(-1.05, abs=1e-12)


def test_longcontrol_stopped_lead_glide_cap_is_santa_fe_only() -> None:
  toggles = DummyFrogPilotToggles()
  lc = LongControl(DummyCarParams(car_fingerprint=HYUNDAI_CAR.HYUNDAI_ELANTRA_2021))
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.407

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.83, a_ego=-0.49, standstill=False, cruise_standstill=False),
    a_target=-0.50,
    should_stop=False,
    distance_to_stop_target_m=4.40,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=-0.09,
    lead_d_rel=7.40,
  )

  assert out > -0.40


def test_longcontrol_softly_brakes_in_stopped_lead_approach_band_before_stop_mode() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=3.2, a_ego=-0.12, standstill=False, cruise_standstill=False)

  baseline = LongControl(cp)
  baseline.long_control_state = LongCtrlState.pid
  baseline.last_output_accel = -0.05
  out_without = baseline.update(
    active=True,
    CS=cs,
    a_target=-0.08,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  with_target = LongControl(cp)
  with_target.long_control_state = LongCtrlState.pid
  with_target.last_output_accel = -0.05
  out_with = with_target.update(
    active=True,
    CS=cs,
    a_target=-0.08,
    should_stop=False,
    distance_to_stop_target_m=1.8,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert with_target.long_control_state == LongCtrlState.pid
  assert out_with < out_without - 1e-6
  assert out_with == pytest.approx(stop_target_approach_accel_cap(cs.vEgo, 1.8), abs=1e-12)


def test_longcontrol_relaxes_low_speed_brake_when_stop_target_is_still_far() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=1.232, a_ego=-0.55, standstill=False, cruise_standstill=False)

  baseline = LongControl(cp)
  baseline.long_control_state = LongCtrlState.pid
  baseline.last_output_accel = -0.59
  out_without = baseline.update(
    active=True,
    CS=cs,
    a_target=-0.6166,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  with_target = LongControl(cp)
  with_target.long_control_state = LongCtrlState.pid
  with_target.last_output_accel = -0.59
  out_with = with_target.update(
    active=True,
    CS=cs,
    a_target=-0.6166,
    should_stop=False,
    distance_to_stop_target_m=4.472,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert with_target.long_control_state == LongCtrlState.pid
  assert out_with > out_without + 1e-6
  assert out_with > -0.35


def test_longcontrol_softens_deep_inherited_brake_on_first_stop_handoff() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=1.03, a_ego=-0.79, standstill=False, cruise_standstill=False)

  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.77
  out = lc.update(
    active=True,
    CS=cs,
    a_target=-0.35,
    should_stop=True,
    distance_to_stop_target_m=0.9,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out > -0.70


def test_longcontrol_does_not_soften_urgent_close_stop_handoff() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=0.55, a_ego=-0.73, standstill=False, cruise_standstill=False)

  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.77
  out = lc.update(
    active=True,
    CS=cs,
    a_target=-0.30,
    should_stop=True,
    distance_to_stop_target_m=0.10,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out < -0.74


def test_longcontrol_softens_moderately_fast_non_urgent_stop_handoff() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=1.85, a_ego=-0.72, standstill=False, cruise_standstill=False)

  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.74
  out = lc.update(
    active=True,
    CS=cs,
    a_target=-0.42,
    should_stop=True,
    distance_to_stop_target_m=1.10,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out > -0.70


def test_longcontrol_enters_stop_mode_early_for_short_gap_lead_seed() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=4.18, a_ego=-1.16, standstill=False, cruise_standstill=False)

  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -1.22
  out = lc.update(
    active=True,
    CS=cs,
    a_target=-1.23,
    should_stop=False,
    distance_to_stop_target_m=0.23,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out > -1.10


def test_longcontrol_keeps_tiny_nonurgent_stop_target_in_approach_band() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=2.30, a_ego=-0.21, standstill=False, cruise_standstill=False)

  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.39
  out = lc.update(
    active=True,
    CS=cs,
    a_target=-0.335,
    should_stop=False,
    distance_to_stop_target_m=0.28,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.pid
  assert out > -0.55


def test_longcontrol_keeps_stopping_while_stop_target_is_still_stable() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=2.13, a_ego=-0.47, standstill=False, cruise_standstill=False)

  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.50
  lc.update(
    active=True,
    CS=cs,
    a_target=-0.283,
    should_stop=False,
    distance_to_stop_target_m=1.68,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping


def test_longcontrol_preserves_stopping_controller_state_while_stop_target_intent_remains_active() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=2.13, a_ego=-0.47, standstill=False, cruise_standstill=False)

  spy = ResetTrackingStoppingController()
  lc = LongControl(cp)
  lc.stopping_controller = spy
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.50

  lc.update(
    active=True,
    CS=cs,
    a_target=-0.283,
    should_stop=False,
    distance_to_stop_target_m=1.68,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert spy.update_calls == 1
  assert spy.reset_calls == 0


def test_longcontrol_keeps_stopping_across_low_speed_stop_target_dropout() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.359

  lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.189, a_ego=-0.328, standstill=False, cruise_standstill=False),
    a_target=-0.197,
    should_stop=True,
    distance_to_stop_target_m=0.934,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.160, a_ego=-0.313, standstill=False, cruise_standstill=False),
    a_target=0.20,
    should_stop=False,
    distance_to_stop_target_m=0.865,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out < -0.10


def test_longcontrol_holds_low_speed_stop_target_release_in_stopping_state() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  tracker = ResetTrackingStoppingController()
  lc.stopping_controller = tracker
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.42
  lc.arbiter._last_target_distance_m = 0.694
  lc.arbiter.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.037, a_ego=-0.08, standstill=False, cruise_standstill=False),
    a_target=0.11,
    should_stop=False,
    distance_to_stop_target_m=0.593,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert tracker.reset_calls == 0
  assert tracker.update_calls == 1
  assert out == pytest.approx(-0.42, abs=1e-12)


def test_longcontrol_holds_wide_low_speed_stop_target_dropout_in_stopping_state() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  tracker = ResetTrackingStoppingController()
  lc.stopping_controller = tracker
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.523
  lc.arbiter._last_target_distance_m = 1.633
  lc.arbiter.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.149, a_ego=-0.660, standstill=False, cruise_standstill=False),
    a_target=-0.417,
    should_stop=False,
    distance_to_stop_target_m=1.633,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert tracker.reset_calls == 0
  assert tracker.update_calls == 1
  assert out == pytest.approx(-0.523, abs=1e-12)


def test_longcontrol_allows_starting_when_low_speed_stop_target_moves_away() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  tracker = ResetTrackingStoppingController()
  lc.stopping_controller = tracker
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.42
  lc.arbiter._last_target_distance_m = 0.694
  lc.arbiter.time_since_stop_intent_s = 0.0

  lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.037, a_ego=-0.08, standstill=False, cruise_standstill=False),
    a_target=0.11,
    should_stop=False,
    distance_to_stop_target_m=1.30,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.starting
  assert tracker.reset_calls >= 1
  assert tracker.update_calls == 0


def test_longcontrol_keeps_stopping_path_unclamped_when_stop_request_is_active() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.stopping_controller = FixedStoppingController(output_accel=-1.05)
  lc.last_output_accel = -0.30

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.8, a_ego=-0.2, standstill=False, cruise_standstill=False),
    a_target=-0.40,
    should_stop=True,
    distance_to_stop_target_m=0.9,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out == pytest.approx(-1.05, abs=1e-12)


def test_longcontrol_allows_resume_when_low_speed_stop_target_moves_away() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.359

  lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.189, a_ego=-0.328, standstill=False, cruise_standstill=False),
    a_target=-0.197,
    should_stop=True,
    distance_to_stop_target_m=0.934,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.160, a_ego=-0.313, standstill=False, cruise_standstill=False),
    a_target=0.20,
    should_stop=False,
    distance_to_stop_target_m=1.30,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.pid


def test_longcontrol_holds_no_target_standstill_dropout_in_stopping_state() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  tracker = ResetTrackingStoppingController()
  lc = LongControl(cp)
  lc.stopping_controller = tracker
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.72
  lc.arbiter.time_since_stop_intent_s = 0.25

  lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.012, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.0,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert tracker.reset_calls == 0
  assert tracker.update_calls == 1


def test_close_stopped_lead_dropout_hold_matches_live_green_light_bookmark() -> None:
  assert should_hold_recent_close_stopped_lead_dropout(
    v_ego=0.0,
    v_ego_starting=0.1,
    standstill=True,
    time_since_standstill_s=0.0,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=3.90,
    distance_to_stop_target_m=-1.0,
    force_coast=False,
  )


def test_close_stopped_lead_dropout_hold_matches_close_target_floor_green_light_bookmark() -> None:
  assert should_hold_recent_close_stopped_lead_dropout(
    v_ego=0.0,
    v_ego_starting=0.1,
    standstill=True,
    time_since_standstill_s=0.0,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=3.30,
    distance_to_stop_target_m=0.05,
    force_coast=False,
  )


def test_close_stopped_lead_dropout_hold_covers_live_false_start_tail_seed() -> None:
  assert should_hold_recent_close_stopped_lead_dropout(
    v_ego=0.98,
    v_ego_starting=0.1,
    standstill=False,
    time_since_standstill_s=0.75,
    lead_status=True,
    lead_v=0.16,
    lead_d_rel=3.40,
    distance_to_stop_target_m=-1.0,
    force_coast=False,
  )


def test_close_stopped_lead_dropout_hold_releases_moving_departed_lead() -> None:
  assert not should_hold_recent_close_stopped_lead_dropout(
    v_ego=0.98,
    v_ego_starting=0.1,
    standstill=False,
    time_since_standstill_s=0.75,
    lead_status=True,
    lead_v=1.10,
    lead_d_rel=3.40,
    distance_to_stop_target_m=-1.0,
    force_coast=False,
  )


def test_longcontrol_clamps_stale_far_target_to_close_stopped_lead_seed() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  tracker = SpyStoppingController()
  lc = LongControl(cp)
  lc.stopping_controller = tracker
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.60

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=1.20, a_ego=-0.92, standstill=False, cruise_standstill=False),
    a_target=-0.98,
    should_stop=False,
    distance_to_stop_target_m=2.70,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=3.20,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  # The far-target -> close-stopped-lead clamp (synthetic min-merge) is still live under the
  # terminal-glide profile; only the synthetic rest gap moved (4.0 m vs the legacy 2.75 m), so the
  # seeded synthetic target the controller receives is the 0.05 m close-hold floor (lead_d_rel 3.20
  # is inside the 4.0 m rest) instead of the legacy 0.45 m. Min-merged against the planner's 2.70 m.
  if stopping_flags.SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED:
    assert tracker.distance_to_stop_target_m == pytest.approx(0.05, abs=1e-12)
  else:
    assert tracker.distance_to_stop_target_m == pytest.approx(0.45, abs=1e-12)
  assert tracker.raw_should_stop is False
  # This seed (v=1.2, close STOPPED lead at 3.2 m, planner demanding -0.98) is the incident-0000173c
  # regime: the stopping controller alone commands only -0.60, which would coast toward the lead. The
  # stopping-phase planner-aTarget floor (2026-06-18) deepens the command to the planner's -0.98 so
  # the car does not under-brake into the close stopped lead -- a one-way deepen, the safe direction,
  # and independent of the synthetic rest-gap value above.
  assert out == pytest.approx(-0.98, abs=1e-12)


def test_longcontrol_holds_close_stopped_lead_after_green_light_dropout_seed() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  tracker = ResetTrackingStoppingController()
  lc = LongControl(cp)
  lc.stopping_controller = tracker
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.12
  lc.arbiter.time_since_standstill_s = 0.0
  lc.arbiter.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.11,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=3.90,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert tracker.reset_calls == 0
  assert tracker.update_calls == 1
  assert out == pytest.approx(-0.12, abs=1e-12)


def test_longcontrol_holds_close_stopped_lead_after_close_target_floor_green_light_dropout_seed() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  tracker = ResetTrackingStoppingController()
  lc = LongControl(cp)
  lc.stopping_controller = tracker
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.12
  lc.arbiter.time_since_standstill_s = 0.0
  lc.arbiter.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.68,
    should_stop=False,
    distance_to_stop_target_m=0.05,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=3.30,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert tracker.reset_calls == 0
  assert tracker.update_calls == 1
  assert out == pytest.approx(-0.12, abs=1e-12)


def test_longcontrol_reenters_stopping_for_recent_standstill_close_stopped_lead_dropout_seed() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.starting
  lc.last_output_accel = 0.09
  lc.arbiter.time_since_standstill_s = 0.25
  lc.arbiter.time_since_stop_intent_s = 0.80

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.27, a_ego=0.89, standstill=False, cruise_standstill=False),
    a_target=0.26,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.15,
    lead_d_rel=3.80,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out < 0.0


def test_longcontrol_allows_departing_lead_after_green_light_dropout() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.12
  lc.arbiter.time_since_standstill_s = 0.0
  lc.arbiter.time_since_stop_intent_s = 0.0

  lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.30,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=1.36,
    lead_d_rel=4.40,
  )

  assert lc.long_control_state == LongCtrlState.starting


def test_longcontrol_force_coast_holds_no_target_standstill_dropout_past_normal_timeout() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  tracker = ResetTrackingStoppingController()
  lc = LongControl(cp)
  lc.stopping_controller = tracker
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.05
  lc.arbiter.time_since_stop_intent_s = 2.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.009, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.0,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    force_coast=True,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert tracker.reset_calls == 0
  assert tracker.update_calls == 1
  assert out < -0.05


def _force_coast_frames(a_target, strength=1.0, v_ego=4.66, n=None):
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  toggles.force_coast_strength = strength
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  ramp_frames = int(round(FORCE_COAST_RAMP_IN_S / longcontrol_module.DT_CTRL))
  outputs = [lc.update(
    active=True,
    CS=DummyCarState(v_ego=v_ego, a_ego=0.0, standstill=False, cruise_standstill=False),
    a_target=a_target,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    force_coast=True,
  ) for _ in range(ramp_frames + 1 if n is None else n)]
  return outputs, ramp_frames, toggles


def test_longcontrol_force_coast_ramps_no_target_pid_braking_over_one_and_a_half_seconds() -> None:
  # the planner asks less than the force-coast profile: the wire starts from the planner's own demand (no step) and the
  # ramp deepens it to the profile over 1.5 s (2026-09-05 driver's contract: force coast is a FLOOR of braking)
  outputs, ramp_frames, toggles = _force_coast_frames(a_target=-0.3)
  target = get_force_coast_target_from_toggles(4.66, toggles)
  assert outputs[0] == pytest.approx(0.0, abs=1e-12)     # the ramp starts from the command already on the wire
  assert outputs[-1] == pytest.approx(target, abs=1e-12)
  assert all(outputs[idx] >= outputs[idx + 1] - 1e-12 for idx in range(len(outputs) - 1))


def test_longcontrol_force_coast_never_caps_a_deeper_demand() -> None:
  # the model's own braking (or a close lead) passed the planner deeper than the profile: it reaches the wire at once, exactly
  outputs, _, toggles = _force_coast_frames(a_target=-1.44)
  assert get_force_coast_target_from_toggles(4.66, toggles) == pytest.approx(-1.2)
  assert all(-1.44 * 1.2 <= out <= -1.44 for out in outputs)          # the dummy PID's ~10% gain, never the -1.2 profile


def test_longcontrol_force_coast_synthetic_demand_at_the_profile_still_ramps_in() -> None:
  # review 20260905-212027 [high]: the planner's ACC zero-cruise demand arrives limited to the FULL profile (-1.2 here); it is
  # force coast's own request, not a hazard -- the wire must ramp in from 0 over 1.5 s, not step to -1.2 on the first frame
  outputs, ramp_frames, toggles = _force_coast_frames(a_target=-1.2)   # exactly the profile (the dummy gains add ~10%, inside the margin)
  assert outputs[0] == pytest.approx(0.0, abs=1e-9)
  assert outputs[ramp_frames // 2] == pytest.approx(-0.6, abs=0.02)
  assert outputs[-1] == pytest.approx(-1.2, abs=1e-9)


def test_longcontrol_force_coast_demand_lift_mid_ramp_eases_at_the_release_limit(monkeypatch) -> None:
  # review 20260905-212027 [medium]: a hazard demand (-2.0) that lifts to -0.3 halfway through the ramp must not step the wire
  # up to the ramp value in one frame; the branch output eases at FORCE_COAST_RELEASE_J
  from openpilot.selfdrive.controls.lib import stopping_flags as flags
  monkeypatch.setattr(flags, "FORCE_COAST_TERMINAL_TAPER", True)
  toggles = DummyFrogPilotToggles()
  toggles.force_coast_strength = 1.4
  lc = LongControl(DummyCarParams())
  lc.long_control_state = LongCtrlState.pid
  outs = []
  for i in range(150):
    a_t = -2.0 if i < 75 else -0.3
    outs.append(float(lc.update(active=True, CS=DummyCarState(v_ego=5.0, a_ego=-1.5, standstill=False, cruise_standstill=False),
                                a_target=a_t, should_stop=False, distance_to_stop_target_m=-1.0, accel_limits=(-3.5, 2.0),
                                frogpilot_toggles=toggles, experimental_mode=True, lead_status=False, lead_v=0.0, lead_d_rel=0.0,
                                force_coast=True)))
  assert outs[74] <= -1.9
  assert max(outs[k + 1] - outs[k] for k in range(74, 149)) <= 0.8 * 0.01 + 1e-6
  assert outs[-1] <= get_force_coast_target_from_toggles(5.0, toggles) + 0.02   # and settles no shallower than the floor


def test_longcontrol_force_coast_adds_no_target_braking_when_pid_would_coast() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid

  first = lc.update(
    active=True,
    CS=DummyCarState(v_ego=4.66, a_ego=0.0, standstill=False, cruise_standstill=False),
    a_target=0.0,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    force_coast=True,
  )

  second = lc.update(
    active=True,
    CS=DummyCarState(v_ego=4.66, a_ego=0.0, standstill=False, cruise_standstill=False),
    a_target=0.0,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    force_coast=True,
  )

  target = get_force_coast_target_from_toggles(4.66, toggles)
  assert first == pytest.approx(0.0, abs=1e-12)
  assert second == pytest.approx(get_force_coast_ramped_accel(0.0, target, longcontrol_module.DT_CTRL), abs=1e-12)
  assert second < first


def test_longcontrol_force_coast_strength_sets_the_no_target_floor() -> None:
  outputs, _, toggles = _force_coast_frames(a_target=-0.3, strength=1.5)
  assert outputs[-1] == pytest.approx(get_force_coast_target_from_toggles(4.66, toggles), abs=1e-12)
  deeper, _, _ = _force_coast_frames(a_target=-2.4, strength=1.5)
  assert all(out == pytest.approx(-2.4, abs=0.3) for out in deeper)   # the floor never caps a deeper demand, nor deepens it


def test_longcontrol_force_coast_weak_strength_is_a_weak_floor() -> None:
  outputs, _, toggles = _force_coast_frames(a_target=-0.3, strength=0.5)
  force_coast_target = get_force_coast_target_from_toggles(4.66, toggles)
  assert outputs[-1] == pytest.approx(force_coast_target, abs=1e-12)
  assert outputs[-1] > force_coast_no_target_pid_brake_cap(4.66)
  deeper, _, _ = _force_coast_frames(a_target=-2.4, strength=0.5)
  assert all(out == pytest.approx(-2.4, abs=0.3) for out in deeper)


def test_longcontrol_force_coast_disables_fast_low_speed_release() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.20

  first = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.2, a_ego=-0.2, standstill=False, cruise_standstill=False),
    a_target=1.0,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    force_coast=True,
  )

  second = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.2, a_ego=-0.2, standstill=False, cruise_standstill=False),
    a_target=1.0,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    force_coast=True,
  )

  target = get_force_coast_target_from_toggles(0.2, toggles)
  assert first == pytest.approx(-0.20, abs=1e-12)
  assert second == pytest.approx(get_force_coast_ramped_accel(-0.20, target, longcontrol_module.DT_CTRL), abs=1e-12)
  assert second < first


def test_longcontrol_force_coast_pid_brake_cap_is_santa_fe_only() -> None:
  toggles = DummyFrogPilotToggles()
  lc = LongControl(DummyCarParams(car_fingerprint=HYUNDAI_CAR.HYUNDAI_ELANTRA_2021))
  lc.long_control_state = LongCtrlState.pid

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=4.66, a_ego=0.0, standstill=False, cruise_standstill=False),
    a_target=-1.44,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    force_coast=True,
  )

  assert out < -2.0


def test_longcontrol_does_not_log_pid_stopping_shadow_for_normal_cruise(monkeypatch) -> None:
  events: list[tuple[str, dict[str, object]]] = []

  def capture_event(event: str, *args, **kwargs) -> None:
    events.append((event, kwargs))

  monkeypatch.setattr(longcontrol_module.cloudlog, "event", capture_event)

  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid

  for _ in range(20):
    lc.update(
      active=True,
      CS=DummyCarState(v_ego=1.20, a_ego=0.0, standstill=False, cruise_standstill=False),
      a_target=0.0,
      should_stop=False,
      distance_to_stop_target_m=-1.0,
      accel_limits=(-3.0, 2.0),
      frogpilot_toggles=toggles,
    )

  assert [event for event, _payload in events if event == "stopping_shadow"] == []


def test_should_observe_pid_stopping_shadow_includes_low_speed_stop_like_windows() -> None:
  assert should_observe_pid_stopping_shadow(
    v_ego=1.0,
    a_target=0.0,
    output_accel=-0.04,
    distance_to_stop_target_m=-1.0,
    force_coast=True,
    lead_status=False,
    lead_v=0.0,
    lead_d_rel=0.0,
    stop_request_active=False,
    stop_target_approach_active=False,
    stop_target_carry_active=False,
  )
  assert not should_observe_pid_stopping_shadow(
    v_ego=3.0,
    a_target=-0.5,
    output_accel=-0.5,
    distance_to_stop_target_m=1.0,
    force_coast=False,
    lead_status=False,
    lead_v=0.0,
    lead_d_rel=0.0,
    stop_request_active=False,
    stop_target_approach_active=True,
    stop_target_carry_active=False,
  )


# --- stopping redesign WP7: dark V2 dispatch, §6.4 slew restructure, F15 emission gate -------------


def test_v2_dispatch_passes_longcontrol_arbiter_decision() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  from openpilot.selfdrive.controls.lib.stopping_controller_v2 import StoppingControllerV2
  assert isinstance(lc.stopping_controller, StoppingControllerV2)

  captured: dict[str, object] = {}
  real_update = lc.stopping_controller.update

  def spy(*args, **kwargs):
    captured["decision"] = kwargs.get("decision")
    return real_update(*args, **kwargs)

  lc.stopping_controller.update = spy
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.30
  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.4, a_ego=-0.2, standstill=False, cruise_standstill=False),
    a_target=-0.3,
    should_stop=True,
    distance_to_stop_target_m=1.2,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
  )
  decision = captured.get("decision")
  assert decision is not None, "V2 dispatch must pass the longcontrol-arbiter StopDecision (F2)"
  assert decision.stop_request_active
  assert out <= -0.05


def _drive_stopping_without_stop_request(lc, toggles):
  # no-target standstill dropout: state stays pinned in stopping (state_dropout_hold) while
  # stop_request_active is False -- the only legacy frame class where the global guard slew
  # applied inside stopping state.
  return lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.0,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
  )


def test_global_slew_exempts_whole_stopping_state_under_v2(monkeypatch) -> None:
  # §6.4: under V2 one jerk limiter (the tracker's) owns every stopping frame.
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.30
  lc.arbiter.time_since_stop_intent_s = 0.0

  calls: list[bool] = []
  original = longcontrol_module.apply_low_speed_output_slew

  def spy(**kwargs):
    calls.append(True)
    return original(**kwargs)

  monkeypatch.setattr(longcontrol_module, "apply_low_speed_output_slew", spy)
  _drive_stopping_without_stop_request(lc, toggles)
  assert lc.long_control_state == LongCtrlState.stopping
  assert not calls, "V2 topology: the guard slew must be exempted for the whole stopping state"


def test_lead_d_rel_eff_invariant_across_publish_flag_for_same_true_gap(monkeypatch) -> None:
  # §4.2.4 / F4: for the same TRUE gap, the stopping layer behaves identically in both flag
  # states -- flag off sees the mutated published gap, flag on sees true gap minus ISD.
  from openpilot.selfdrive.controls.lib import stopping_flags

  isd = 1.5
  true_gap = 4.6

  def drive(published_gap: float, flag: bool) -> tuple[object, float]:
    monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", flag)
    cp = DummyCarParams()
    toggles = DummyFrogPilotToggles()
    lc = LongControl(cp)
    lc.stopping_controller = FixedStoppingController(output_accel=-0.30)
    lc.long_control_state = LongCtrlState.stopping
    lc.last_output_accel = -0.30
    out = lc.update(
      active=True,
      CS=DummyCarState(v_ego=0.5, a_ego=-0.1, standstill=False, cruise_standstill=False),
      a_target=-0.2,
      should_stop=True,
      distance_to_stop_target_m=-1.0,
      accel_limits=(-3.0, 2.0),
      frogpilot_toggles=toggles,
      lead_status=True,
      lead_v=0.0,
      lead_d_rel=published_gap,
      increased_stopped_distance=isd,
    )
    return lc.long_control_state, out

  state_off, out_off = drive(true_gap - isd, flag=False)  # today: radard publishes true - ISD
  state_on, out_on = drive(true_gap, flag=True)           # post-flip: radard publishes true
  assert state_on == state_off
  assert out_on == out_off


def test_v2_stopping_shadow_emission_gate(monkeypatch) -> None:
  # F15: the v2 debug dict has no shadow_profile key; the rewritten gate must still emit on the
  # existing stopping_shadow channel (passthrough payload + ground truth + §3.2 row 1 counters).
  events: list[tuple[str, dict[str, object]]] = []

  def capture_event(event: str, *args, **kwargs) -> None:
    events.append((event, kwargs))

  monkeypatch.setattr(longcontrol_module.cloudlog, "event", capture_event)

  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.50

  for _ in range(20):
    lc.update(
      active=True,
      CS=DummyCarState(v_ego=0.50, a_ego=-0.05, standstill=False, cruise_standstill=False),
      a_target=-0.20,
      should_stop=True,
      distance_to_stop_target_m=0.40,
      accel_limits=(-3.0, 2.0),
      frogpilot_toggles=toggles,
      lead_status=True,
      lead_v=0.0,
      lead_d_rel=4.2,
    )

  payloads = [payload for event, payload in events if event == "stopping_shadow"]
  assert len(payloads) == 1, "one (phase, source) change-triggered event expected; steady frames suppressed"
  payload = payloads[0]
  # facade debug-dict passthrough (spec section 2 telemetry contract)
  assert str(payload["version"]).startswith("v2_")
  for key in ("phase", "a_ref", "disturbance", "rollout_m", "remaining_m", "release_inhibit_active",
              "recovery_i", "settled_time_s", "source", "triggers"):
    assert key in payload, f"v2 payload missing {key}"
  assert "shadow_profile" not in payload  # F36: the oracle keys are retired with the oracle
  # ground-truth fields
  for key in ("v_ego", "a_ego", "output_accel", "lead_status", "lead_v", "lead_d_rel"):
    assert key in payload, f"v2 payload missing ground-truth {key}"
  assert payload["v_ego"] == 0.50
  assert payload["lead_d_rel"] == 4.2
  # §3.2 row 1 retirement counters (Commit C instrumentation)
  for key in ("legacy_hold_fired", "single_hold_covered", "hold_divergence",
              "consolidated_hold_active", "consolidated_hold_source", "consolidated_target_m"):
    assert key in payload, f"v2 payload missing arbiter hold telemetry {key}"


def test_v2_stopping_shadow_emits_again_on_phase_change(monkeypatch) -> None:
  events: list[tuple[str, dict[str, object]]] = []

  def capture_event(event: str, *args, **kwargs) -> None:
    events.append((event, kwargs))

  monkeypatch.setattr(longcontrol_module.cloudlog, "event", capture_event)

  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.50

  # TRACK-phase frames, then SETTLE-phase frames: the (phase, source) cadence trigger must fire
  # again on the phase change even though the 2 s periodic floor has not elapsed.
  for v_ego in [1.5] * 10 + [0.03] * 10:
    lc.update(
      active=True,
      CS=DummyCarState(v_ego=v_ego, a_ego=-0.10, standstill=v_ego < 0.05, cruise_standstill=False),
      a_target=-0.20,
      should_stop=True,
      distance_to_stop_target_m=2.0,
      accel_limits=(-3.0, 2.0),
      frogpilot_toggles=toggles,
    )

  payloads = [payload for event, payload in events if event == "stopping_shadow"]
  assert len(payloads) == 2
  assert payloads[0]["phase"] != payloads[1]["phase"]


def test_legacy_stopping_shadow_payload_carries_hold_telemetry(monkeypatch) -> None:
  # Commit C: the §3.2 row 1 counters ride the LEGACY shadow payload too, so divergence evidence
  # accumulates during the pre-flip soak as well.
  events: list[tuple[str, dict[str, object]]] = []

  def capture_event(event: str, *args, **kwargs) -> None:
    events.append((event, kwargs))

  monkeypatch.setattr(longcontrol_module.cloudlog, "event", capture_event)

  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.50

  for _ in range(10):
    lc.update(
      active=True,
      CS=DummyCarState(v_ego=0.50, a_ego=-0.05, standstill=False, cruise_standstill=False),
      a_target=-0.20,
      should_stop=True,
      distance_to_stop_target_m=0.40,
      accel_limits=(-3.0, 2.0),
      frogpilot_toggles=toggles,
    )

  payloads = [payload for event, payload in events if event == "stopping_shadow"]
  assert len(payloads) == 1
  for key in ("legacy_hold_fired", "single_hold_covered", "hold_divergence",
              "consolidated_hold_active", "consolidated_hold_source", "consolidated_target_m"):
    assert key in payloads[0], f"legacy payload missing arbiter hold telemetry {key}"


def test_v2_reset_on_user_disable_clears_tracker_state() -> None:
  # F5: a driver brake tap disengages (USER_DISABLE) -> longActive False -> off state -> full
  # reset; re-engage starts with fresh tracker state (no stale d_hat / recovery_i / rollout).
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.50

  for _ in range(30):
    lc.update(
      active=True,
      CS=DummyCarState(v_ego=0.30, a_ego=0.10, standstill=False, cruise_standstill=False),
      a_target=-0.20,
      should_stop=True,
      distance_to_stop_target_m=0.8,
      accel_limits=(-3.0, 2.0),
      frogpilot_toggles=toggles,
    )
  tracker = lc.stopping_controller.tracker
  assert tracker.rollout_m > 0.0  # accumulated state to clear

  out = lc.update(
    active=False,  # USER_DISABLE: longActive drops
    CS=DummyCarState(v_ego=0.30, a_ego=0.0, brake_pressed=True, standstill=False, cruise_standstill=False),
    a_target=0.0,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
  )
  assert lc.long_control_state == LongCtrlState.off
  assert out == 0.0
  assert tracker.rollout_m == 0.0
  assert tracker.d_hat == 0.0
  assert tracker.recovery_i == 0.0


# --- Stopping-phase planner-aTarget safety floor (incident 0000173c seg24, 2026-06-18) ----------
# The floor is a one-way DEEPEN: in the stopping state, while a close lead is present and the planner
# demands decel deeper than the stopping controller's command, honor the planner so the car cannot
# coast through its stop point into the lead (the bookmark: planner -0.42, legacy -0.12 on a downhill).


def test_stopping_planner_floor_gate_arms_only_in_closing_approach() -> None:
  # ARMED: rolling above the standstill band, close lead present, planner demanding decel deeper
  # than the (shallow, floored) controller command -- the exact bookmark regime.
  assert stopping_planner_floor_active(
    v_ego=1.5, lead_status=True, lead_v=0.0, lead_d_rel=3.5, a_target=-0.42, output_accel=-0.12)


def test_stopping_planner_floor_gate_off_below_standstill_band() -> None:
  # Below the v floor the terminal settle/hold lanes own the command -- never fight the gentle hold.
  assert not stopping_planner_floor_active(
    v_ego=STOPPING_PLANNER_FLOOR_V_EGO_MIN - 0.01, lead_status=True, lead_v=0.0,
    lead_d_rel=3.5, a_target=-0.42, output_accel=-0.12)


def test_stopping_planner_floor_gate_off_without_lead() -> None:
  assert not stopping_planner_floor_active(
    v_ego=1.5, lead_status=False, lead_v=0.0, lead_d_rel=3.5, a_target=-0.42, output_accel=-0.12)
  assert not stopping_planner_floor_active(
    v_ego=1.5, lead_status=True, lead_v=0.0, lead_d_rel=None, a_target=-0.42, output_accel=-0.12)


def test_stopping_planner_floor_gate_off_for_far_lead() -> None:
  # A lead beyond the stop-relevant gap does not arm the floor (lead-free / following stops untouched).
  assert not stopping_planner_floor_active(
    v_ego=1.5, lead_status=True, lead_v=0.0, lead_d_rel=STOPPING_PLANNER_FLOOR_GAP_MAX_M + 0.5,
    a_target=-0.42, output_accel=-0.12)


def test_stopping_planner_floor_gate_off_for_shallow_or_positive_a_target() -> None:
  # The planner must be demanding meaningful decel; near-zero/positive aTarget never arms the floor.
  assert not stopping_planner_floor_active(
    v_ego=1.5, lead_status=True, lead_v=0.0, lead_d_rel=3.5,
    a_target=STOPPING_PLANNER_FLOOR_A_TARGET_MAX + 0.01, output_accel=-0.12)
  assert not stopping_planner_floor_active(
    v_ego=1.5, lead_status=True, lead_v=0.0, lead_d_rel=3.5, a_target=0.2, output_accel=-0.12)


def test_stopping_planner_floor_gate_off_when_command_already_deeper() -> None:
  # One-way deepen: when the controller is ALREADY braking at least as deep as the planner, the floor
  # does not arm (it would not change the command -- and must never make it shallower).
  assert not stopping_planner_floor_active(
    v_ego=1.5, lead_status=True, lead_v=0.0, lead_d_rel=3.5, a_target=-0.42, output_accel=-0.50)
  assert not stopping_planner_floor_active(
    v_ego=1.5, lead_status=True, lead_v=0.0, lead_d_rel=3.5, a_target=-0.42, output_accel=-0.42)


def test_stopping_planner_floor_applies_in_longcontrol_incident_regime(monkeypatch) -> None:
  # Integrated: in the stopping state with a close lead, a shallow floored controller command
  # (-0.12, the bookmark's coast-in command) is deepened to the planner aTarget (-0.42).
  monkeypatch.setattr(longcontrol_module, "STOPPING_PLANNER_FLOOR_ENABLED", True)
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-0.12)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.12

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=1.5, a_ego=-0.10, standstill=False, cruise_standstill=False),
    a_target=-0.42,
    should_stop=True,
    distance_to_stop_target_m=0.05,  # collapsed-to-pin stop target (the real incident value)
    accel_limits=(-3.5, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=3.5,
  )
  # command is now at least as deep as the planner demand -- never the shallow -0.12 coast-in
  assert out <= -0.42 + 1e-9


def test_stopping_planner_floor_one_way_deepen_never_shallower(monkeypatch) -> None:
  # When the controller is already braking DEEPER than the planner, the floor must NOT raise it.
  monkeypatch.setattr(longcontrol_module, "STOPPING_PLANNER_FLOOR_ENABLED", True)
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-0.60)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.60

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=1.5, a_ego=-0.40, standstill=False, cruise_standstill=False),
    a_target=-0.42,
    should_stop=True,
    distance_to_stop_target_m=0.05,
    accel_limits=(-3.5, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=3.5,
  )
  # the deeper controller command is preserved; the floor never makes it shallower than -0.60
  assert out <= -0.60 + 1e-9


def test_stopping_planner_floor_disabled_by_flag(monkeypatch) -> None:
  # With the kill-switch off, the shallow coast-in command is preserved (baseline behavior).
  monkeypatch.setattr(longcontrol_module, "STOPPING_PLANNER_FLOOR_ENABLED", False)
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-0.12)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.12

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=1.5, a_ego=-0.10, standstill=False, cruise_standstill=False),
    a_target=-0.42,
    should_stop=True,
    distance_to_stop_target_m=0.05,
    accel_limits=(-3.5, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=3.5,
  )
  # flag off: the floor does not engage; command stays at the shallow coast-in value
  assert out == pytest.approx(-0.12, abs=1e-9)


def test_should_apply_stopping_planner_floor_santa_fe_only() -> None:
  assert should_apply_stopping_planner_floor(DummyCarParams())
  assert not should_apply_stopping_planner_floor(DummyCarParams(car_fingerprint="SOME_OTHER_CAR"))


# --- Terminal-glide V-GATED cap bypass at 0.30 (sub-0.30 redesign, FIX A) -------------------------
# Under the terminal-glide profile the leapfrog over-brake (glide cap + close-lead cap) is bypassed
# ONLY ABOVE STOPPING_PLANNER_FLOOR_V_EGO_MIN (0.30), where the tracker glides to 4.0 m and the seg24
# planner floor owns anti-collision. At v_ego <= 0.30 both caps stay ACTIVE exactly as legacy, so the
# sub-0.30 anti-collision authority is byte-identical to today (no new under-brake hole), and the firm
# hold counters creep at standstill. These pin the v=0.30 handoff (cap re-activates as the car slows
# past 0.30) and prove the over-brake stays retired above 0.30.


def test_should_apply_low_speed_close_lead_accel_cap_v_gated_under_terminal_glide(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED", True)
  cp = DummyCarParams()
  # flag ON, ABOVE 0.30 -> bypassed (cap OFF; the tracker glides, seg24 floor owns anti-collision)
  assert not longcontrol_module.should_apply_low_speed_close_lead_accel_cap(cp, STOPPING_PLANNER_FLOOR_V_EGO_MIN + 0.01)
  assert not longcontrol_module.should_apply_low_speed_close_lead_accel_cap(cp, 0.90)
  # flag ON, AT/BELOW 0.30 -> active exactly as legacy (byte-identical sub-0.30 authority)
  assert longcontrol_module.should_apply_low_speed_close_lead_accel_cap(cp, STOPPING_PLANNER_FLOOR_V_EGO_MIN)
  assert longcontrol_module.should_apply_low_speed_close_lead_accel_cap(cp, 0.05)


def test_should_apply_low_speed_close_lead_accel_cap_flag_off_active_at_all_speeds(monkeypatch) -> None:
  # With the terminal-glide kill switch off, the cap is active at every speed (legacy patchwork).
  monkeypatch.setattr(stopping_flags, "SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED", False)
  cp = DummyCarParams()
  for v in (0.05, STOPPING_PLANNER_FLOOR_V_EGO_MIN, 0.90):
    assert longcontrol_module.should_apply_low_speed_close_lead_accel_cap(cp, v)


def test_should_apply_low_speed_close_lead_accel_cap_santa_fe_only(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED", False)
  other = DummyCarParams(car_fingerprint="SOME_OTHER_CAR")
  for v in (0.05, 0.30, 0.90):
    assert not longcontrol_module.should_apply_low_speed_close_lead_accel_cap(other, v)


def test_longcontrol_close_lead_cap_reactivates_at_handoff_under_terminal_glide() -> None:
  # FIX A handoff: terminal-glide ON (default). At v=0.30 (<= STOPPING_PLANNER_FLOOR_V_EGO_MIN) the
  # close-lead cap is ACTIVE again as the car slows past 0.30 -- the legacy sub-0.30 anti-collision
  # authority is restored, so a closing error is braked deeper than the shallow controller command.
  assert stopping_flags.SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-0.42)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.42

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.30, a_ego=-0.08, standstill=False, cruise_standstill=False),
    a_target=-0.11,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=-0.01,
    lead_d_rel=1.50,
  )

  # cap re-activated at the 0.30 handoff even with the flag ON: deeper than the controller command
  assert out < -0.43
  assert out > low_speed_close_lead_accel_cap(0.30, -0.01, 1.50)


def test_longcontrol_close_lead_cap_bypassed_above_handoff_under_terminal_glide() -> None:
  # FIX A: ABOVE 0.30 with the flag ON the close-lead over-brake is RETIRED -- the controller command
  # passes through (no leapfrog-inducing deepen), the tracker/seg24 floor own the approach.
  assert stopping_flags.SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-0.42)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.42

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.40, a_ego=-0.08, standstill=False, cruise_standstill=False),
    a_target=-0.11,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=-0.01,
    lead_d_rel=1.50,
  )

  # above 0.30 the cap is bypassed: the controller command stands (the close-lead deepen never fires)
  assert out == pytest.approx(-0.42, abs=1e-12)


def _run_glide_cap_pid_frame(toggles, v_ego: float):
  cp = DummyCarParams()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.407
  return lc.update(
    active=True,
    CS=DummyCarState(v_ego=v_ego, a_ego=-0.49, standstill=False, cruise_standstill=False),
    a_target=-0.50,
    should_stop=False,
    distance_to_stop_target_m=4.40,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=-0.09,
    lead_d_rel=7.40,
  )


def test_longcontrol_glide_cap_v_gated_bypass_above_handoff_under_terminal_glide(monkeypatch) -> None:
  # FIX A on the glide cap: ABOVE 0.30 the v-gated bypass retires the binding glide over-brake. Proven
  # by toggling ONLY the kill switch at the SAME speed (0.83 > 0.30): with the flag OFF the legacy cap
  # DEEPENS the command (the leapfrog over-brake); with the flag ON (default) the bypass leaves the
  # command SHALLOWER -- i.e. the cap no longer rewrites it above the handoff.
  toggles = DummyFrogPilotToggles()
  monkeypatch.setattr(stopping_flags, "SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED", False)
  out_cap_active = _run_glide_cap_pid_frame(toggles, v_ego=0.83)
  monkeypatch.setattr(stopping_flags, "SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED", True)
  out_bypassed = _run_glide_cap_pid_frame(toggles, v_ego=0.83)
  # flag OFF deepens via the cap; flag ON (bypass) does not -> the bypassed command is shallower
  assert out_cap_active < low_speed_stopped_lead_glide_accel_cap(0.83, -0.09, 7.40, 4.40) + 1e-9 or out_cap_active < -0.407
  assert out_bypassed > out_cap_active


def test_longcontrol_glide_cap_reactivates_at_or_below_handoff_under_terminal_glide(monkeypatch) -> None:
  # FIX A handoff: AT/BELOW 0.30 the glide cap re-activates exactly as legacy even with the flag ON.
  # Proven by toggling ONLY the kill switch at the SAME sub-handoff speed (0.30 <= 0.30): both states
  # produce the identical command, i.e. the sub-0.30 glide authority is byte-identical to legacy.
  toggles = DummyFrogPilotToggles()
  monkeypatch.setattr(stopping_flags, "SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED", False)
  out_flag_off = _run_glide_cap_pid_frame(toggles, v_ego=0.30)
  monkeypatch.setattr(stopping_flags, "SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED", True)
  out_flag_on = _run_glide_cap_pid_frame(toggles, v_ego=0.30)
  assert out_flag_on == pytest.approx(out_flag_off, abs=1e-12)


# --- Close-the-gap forward creep behind a confirmed stopped lead (route 00001764 seg27) -----------
from openpilot.selfdrive.controls.lib.longcontrol import (
  stopping_close_gap_creep_should_arm,
  stopping_close_gap_creep_should_disarm,
  stopping_close_gap_creep_rest_target_m,
  stopping_close_gap_creep_accel_target,
  stopping_close_gap_creep_eff_floor_m,
  CREEP_ARM_STANDSTILL_TIME_S,
  CREEP_ACCEL_MAX,
  CREEP_DISARM_V_EGO_MAX,
  CREEP_REST_GAP_MAX_M,
  CREEP_REST_GAP_MIN_M,
)


def test_creep_arms_when_stopped_short_behind_stopped_lead():
  # measured incident: eff gap 5.4 (true 5.7), v~0.05, lead stopped -> must ARM
  rt = stopping_close_gap_creep_rest_target_m(0.3)
  assert stopping_close_gap_creep_should_arm(0.05, True, 0.05, 5.4, False, rt, 0.3)


def test_creep_disarms_at_target():
  rt = stopping_close_gap_creep_rest_target_m(0.3)
  assert stopping_close_gap_creep_should_disarm(0.20, True, 0.05, rt + 0.30, False, rt, 0.3)
  # and does NOT arm once already at/below target+margin
  assert not stopping_close_gap_creep_should_arm(0.05, True, 0.05, rt + 0.30, False, rt, 0.3)


def test_creep_never_for_moving_or_absent_lead_or_force_coast():
  rt = stopping_close_gap_creep_rest_target_m(0.3)
  assert not stopping_close_gap_creep_should_arm(0.05, True, 0.9, 5.4, False, rt, 0.3)   # moving lead
  assert not stopping_close_gap_creep_should_arm(0.05, False, 0.0, 5.4, False, rt, 0.3)  # no lead
  assert not stopping_close_gap_creep_should_arm(0.05, True, 0.05, 5.4, True, rt, 0.3)   # force-coast
  # all of those also force DISARM
  assert stopping_close_gap_creep_should_disarm(0.05, True, 0.9, 5.4, False, rt, 0.3)
  assert stopping_close_gap_creep_should_disarm(0.05, True, 0.05, 5.4, True, rt, 0.3)


def test_creep_overspeed_disarms():
  rt = stopping_close_gap_creep_rest_target_m(0.3)
  assert stopping_close_gap_creep_should_disarm(CREEP_DISARM_V_EGO_MAX + 0.01, True, 0.05, 5.0, False, rt, 0.3)
  assert stopping_close_gap_creep_should_disarm(0.35, True, 0.05, 5.0, False, rt, 0.3)


def test_close_gap_creep_retired_kill_switch_off():
  # RETIRED 2026-07-01 (escape-leapfrog review): post-stop motion is disliked by the user taxonomy
  # (a settle followed by a crawl IS the leapfrog feel) and 41 fresh settles show terminal glide
  # lands rests in-band without the creep. Pin the kill switch OFF so it cannot silently re-arm;
  # the mechanism tests below monkeypatch it ON while the code is retained for one release.
  import openpilot.selfdrive.controls.lib.longcontrol as lc_mod
  assert lc_mod.STOPPING_CLOSE_GAP_CREEP_ENABLED is False


def test_close_gap_creep_inert_when_retired():
  # With the kill switch off, an armed-in-all-other-ways standstill must NOT creep: the stable-
  # standstill timer stays untouched and the controller output passes through unmodified.
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-1.05)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.44
  lc.close_gap_creep_standstill_time_s = CREEP_ARM_STANDSTILL_TIME_S

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.05, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.0,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=4.80,
  )

  assert not lc.creeping
  assert out == pytest.approx(-1.05, abs=1e-12)


def test_close_gap_creep_waits_for_stable_standstill_before_last_writer_override(monkeypatch):
  import openpilot.selfdrive.controls.lib.longcontrol as lc_mod
  monkeypatch.setattr(lc_mod, "STOPPING_CLOSE_GAP_CREEP_ENABLED", True)
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-1.05)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.44

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.05, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.0,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=4.80,
  )

  assert not lc.creeping
  assert lc.close_gap_creep_standstill_time_s == pytest.approx(0.01)
  assert out == pytest.approx(-1.05, abs=1e-12)


def test_close_gap_creep_can_arm_after_stable_standstill_hold(monkeypatch):
  import openpilot.selfdrive.controls.lib.longcontrol as lc_mod
  monkeypatch.setattr(lc_mod, "STOPPING_CLOSE_GAP_CREEP_ENABLED", True)
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-1.05)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.44
  lc.close_gap_creep_standstill_time_s = CREEP_ARM_STANDSTILL_TIME_S

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.05, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.0,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    lead_status=True,
    lead_v=0.0,
    lead_d_rel=4.80,
  )

  assert lc.creeping
  assert out == pytest.approx(-0.438, abs=1e-12)


def test_creep_accel_is_gentle_positive_bounded():
  a = stopping_close_gap_creep_accel_target(0.05, 5.4)
  assert 0.0 <= a <= CREEP_ACCEL_MAX


def test_creep_true_rest_target_in_bounds_across_isd():
  # the eff target + ISD (true rest) must stay within [2.5, 5.0] for the whole ISD range incl. UI max
  for isd in (0.0, 0.3, 0.5, 1.0, 1.5, 2.0, 3.05):
    true_rest = stopping_close_gap_creep_rest_target_m(isd) + isd
    assert CREEP_REST_GAP_MIN_M - 1e-6 <= true_rest <= CREEP_REST_GAP_MAX_M + 1e-6, (isd, true_rest)
    # hard floor in eff-space keeps the TRUE floor >= 2.5
    assert stopping_close_gap_creep_eff_floor_m(isd) + isd >= CREEP_REST_GAP_MIN_M - 1e-6


def test_a_hold_firm_pinned_to_force_coast_standstill_hold():
  # Santa-Fe terminal-glide firm hold (correction 2) MUST stay equal to the already-proven
  # force-coast standstill creep-counter magnitude; pin it so a future edit to either cannot
  # silently desync the firm terminal-hold from the validated value.
  from openpilot.selfdrive.controls.lib.stopping_trajectory import A_HOLD_FIRM
  from openpilot.selfdrive.controls.lib.longcontrol import FORCE_COAST_STANDSTILL_HOLD_ACCEL
  assert A_HOLD_FIRM == FORCE_COAST_STANDSTILL_HOLD_ACCEL


# -- cycle 52 (2026-09-05): the force-coast no-target ramp -- deeper wins, and the tapered profile eases at <= 0.8 m/s^3 -------
def _force_coast_run(monkeypatch, a_target_fn, v_fn, n=200, taper=True, strength=1.4):
  from openpilot.selfdrive.controls.lib import stopping_flags as flags
  monkeypatch.setattr(flags, "FORCE_COAST_TERMINAL_TAPER", taper)
  toggles = DummyFrogPilotToggles()
  toggles.force_coast_strength = strength
  lc = LongControl(DummyCarParams())
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = a_target_fn(0)
  outs = []
  for i in range(n):
    out = lc.update(active=True, CS=DummyCarState(v_ego=v_fn(i), a_ego=-0.8, standstill=False, cruise_standstill=False),
                    a_target=a_target_fn(i), should_stop=False, distance_to_stop_target_m=-1.0, accel_limits=(-3.5, 2.0),
                    frogpilot_toggles=toggles, experimental_mode=True, lead_status=False, lead_v=0.0, lead_d_rel=0.0,
                    force_coast=True)
    outs.append(float(out))
  return lc, outs


def test_force_coast_profile_writes_the_wire_and_eases_through_the_knee_at_the_release_limit(monkeypatch):
  # a slow roll from 1.4 m/s to 0.25 m/s under force coast, the planner asking only -0.3: the wire is the profile
  # (-1.4 above 1 m/s at strength 1.4), then rises through the knee no faster than FORCE_COAST_RELEASE_J
  from openpilot.frogpilot.controls.lib.force_coast import FORCE_COAST_RELEASE_J
  def v_fn(i):
    return 1.6 if i < 160 else max(1.6 - 0.025 * (i - 160), 0.25)   # the 1.5 s ramp-in completes first, then 2.5 m/s^2
  lc, outs = _force_coast_run(monkeypatch, lambda i: -0.3, v_fn, n=260)
  # the tail rises 0.4 per m/s between 1.0 and 0.5 m/s; at 2.5 m/s^2 that asks 1.0 m/s^3: the limiter must bind
  rises = [outs[k + 1] - outs[k] for k in range(160, 259)]
  assert max(rises) <= FORCE_COAST_RELEASE_J * 0.01 + 1e-6
  assert max(rises) >= FORCE_COAST_RELEASE_J * 0.01 - 1e-4          # ... and really is the binding constraint somewhere
  assert outs[-1] > outs[160] + 0.3                     # the tail eased the command as the car slowed
  assert outs[-1] <= -0.5 + 0.05                        # and never above the tail's wheel-stop value


def test_force_coast_profile_flag_off_holds_todays_numbers_into_the_stop(monkeypatch):
  def v_fn(i):
    return max(1.4 - 0.006 * i, 0.2)
  lc, outs = _force_coast_run(monkeypatch, lambda i: -0.3, v_fn, n=220, taper=False)
  assert outs[-1] == pytest.approx(-0.98, abs=0.03)     # -0.7 x 1.4 held into the wheel stop (today)


def test_force_coast_rise_limit_does_not_ratchet_with_the_tracking_trim(monkeypatch):
  # round-2 review 20260905-200940: referencing last_output_accel (which carries the trim added AFTER the limiter) ratcheted
  # the command -1.68 -> -3.5 in six frames with a residual -0.40 trim. The limiter must reference its own previous command.
  from openpilot.selfdrive.controls.lib import stopping_flags as flags
  monkeypatch.setattr(flags, "FORCE_COAST_TERMINAL_TAPER", True)
  toggles = DummyFrogPilotToggles()
  toggles.force_coast_strength = 1.4
  lc = LongControl(DummyCarParams())
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -1.68
  lc._trim_i = -0.40
  outs = []
  for _ in range(150):
    outs.append(float(lc.update(active=True, CS=DummyCarState(v_ego=5.0, a_ego=-1.6, standstill=False, cruise_standstill=False),
                                a_target=-1.68, should_stop=False, distance_to_stop_target_m=-1.0, accel_limits=(-3.5, 2.0),
                                frogpilot_toggles=toggles, experimental_mode=True, lead_status=False, lead_v=0.0, lead_d_rel=0.0,
                                force_coast=True)))
  assert min(outs) >= -2.15                      # the trim adds at most its own residual once, never compounds
  assert outs[-1] >= -1.95                       # and the command recovers toward the demand as the trim decays


def test_force_coast_slow_down_with_no_lead_enters_the_stopping_service_below_one_metre_per_second(monkeypatch) -> None:
  # review 20260905-212027 [high] (creep): the service's terminal descent, monitor and hold own the ending
  from openpilot.selfdrive.controls.lib import stopping_flags as flags
  monkeypatch.setattr(flags, "SERVICE_MODE", "LIVE")
  toggles = DummyFrogPilotToggles()
  toggles.force_coast_strength = 1.4
  for v, lead, expect in ((0.8, False, True), (1.2, False, False), (0.8, True, False)):
    lc = LongControl(DummyCarParams())
    lc.long_control_state = LongCtrlState.pid
    for _ in range(30):
      lc.update(active=True, CS=DummyCarState(v_ego=v, a_ego=-0.5, standstill=False, cruise_standstill=False),
                a_target=-0.6, should_stop=False, distance_to_stop_target_m=-1.0, accel_limits=(-3.5, 2.0),
                frogpilot_toggles=toggles, experimental_mode=True, lead_status=lead, lead_v=3.0 if lead else 0.0, lead_d_rel=8.0 if lead else 0.0,
                lead_track_id=7 if lead else None, lead_model_prob=0.9 if lead else 0.0, force_coast=True)
    assert bool(lc._service_live_owning) == expect, (v, lead)


def test_longcontrol_force_coast_planner_sample_age_does_not_bypass_the_ramp(monkeypatch) -> None:
  # review 20260905-212824 [high]: the planner's demand is the profile at ITS (older, faster) speed sample -- a hair deeper than
  # the profile at the controller's speed; that is not a hazard and must ramp in
  from openpilot.selfdrive.controls.lib import stopping_flags as flags
  monkeypatch.setattr(flags, "FORCE_COAST_TERMINAL_TAPER", True)
  toggles = DummyFrogPilotToggles()
  toggles.force_coast_strength = 1.4
  lc = LongControl(DummyCarParams())
  lc.long_control_state = LongCtrlState.pid
  planner_demand = get_force_coast_target_from_toggles(2.20, toggles)          # the planner sampled 2.20 m/s
  out0 = float(lc.update(active=True, CS=DummyCarState(v_ego=2.19, a_ego=-1.0, standstill=False, cruise_standstill=False),
                         a_target=planner_demand / 1.11, should_stop=False, distance_to_stop_target_m=-1.0, accel_limits=(-3.5, 2.0),
                         frogpilot_toggles=toggles, experimental_mode=True, lead_status=False, lead_v=0.0, lead_d_rel=0.0, force_coast=True))
  assert out0 == pytest.approx(0.0, abs=1e-9)                                   # ramps from the wire, no step to -1.64


def test_longcontrol_force_coast_reactivation_while_braking_eases_at_the_limit(monkeypatch) -> None:
  # review 20260905-212824 [medium]: previous wire -2.0, demand lifted to -0.3, force coast re-enabled: the wire rises toward the
  # -1.68 floor at 0.8 m/s^3, not in one +0.32 step
  from openpilot.selfdrive.controls.lib import stopping_flags as flags
  monkeypatch.setattr(flags, "FORCE_COAST_TERMINAL_TAPER", True)
  toggles = DummyFrogPilotToggles()
  toggles.force_coast_strength = 1.4
  lc = LongControl(DummyCarParams())
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -2.0
  outs = [float(lc.update(active=True, CS=DummyCarState(v_ego=5.0, a_ego=-1.5, standstill=False, cruise_standstill=False),
                          a_target=-0.3, should_stop=False, distance_to_stop_target_m=-1.0, accel_limits=(-3.5, 2.0),
                          frogpilot_toggles=toggles, experimental_mode=True, lead_status=False, lead_v=0.0, lead_d_rel=0.0,
                          force_coast=True)) for _ in range(60)]
  assert outs[0] <= -2.0 + 0.8 * 0.01 + 1e-6
  assert max(outs[k + 1] - outs[k] for k in range(59)) <= 0.8 * 0.01 + 1e-6
  assert outs[-1] == pytest.approx(-1.68, abs=0.02)


def test_force_coast_service_entry_has_hysteresis_and_is_behind_the_flag(monkeypatch) -> None:
  from openpilot.selfdrive.controls.lib import stopping_flags as flags
  from openpilot.selfdrive.controls.lib.stopping_service import Phase
  monkeypatch.setattr(flags, "SERVICE_MODE", "LIVE")
  toggles = DummyFrogPilotToggles()
  toggles.force_coast_strength = 1.4
  def run(lc, v):
    lc.update(active=True, CS=DummyCarState(v_ego=v, a_ego=-0.5, standstill=False, cruise_standstill=False),
              a_target=-0.6, should_stop=False, distance_to_stop_target_m=-1.0, accel_limits=(-3.5, 2.0),
              frogpilot_toggles=toggles, experimental_mode=True, lead_status=False, lead_v=0.0, lead_d_rel=0.0, force_coast=True)
  monkeypatch.setattr(flags, "FORCE_COAST_TERMINAL_TAPER", True)
  lc = LongControl(DummyCarParams())
  lc.long_control_state = LongCtrlState.pid
  for _ in range(30):
    run(lc, 0.98)
  assert lc._service_live_owning
  phases = set()
  for i in range(60):                                   # 0.99 / 1.01 alternation: no APPROACH <-> RELEASE chatter
    run(lc, 0.99 if i % 2 else 1.01)
    phases.add(lc._service_shadow_svc.phase)
  assert lc._service_live_owning and Phase.RELEASE not in phases
  for _ in range(30):
    run(lc, 1.35)                                       # above the exit speed the latch clears
  assert not lc._force_coast_stop_latched
  monkeypatch.setattr(flags, "FORCE_COAST_TERMINAL_TAPER", False)
  lc = LongControl(DummyCarParams())
  lc.long_control_state = LongCtrlState.pid
  for _ in range(30):
    run(lc, 0.8)
  assert not lc._service_live_owning                    # flag off: today's terminal ownership

