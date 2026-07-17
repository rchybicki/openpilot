from types import SimpleNamespace

from cereal import car
from opendbc.car import ButtonType
from opendbc.car.hyundai.values import HyundaiFlags

import openpilot.selfdrive.car.card as card_module
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.car.live_update_handoff import DIAGNOSTIC, DIAGNOSTIC_REQUESTED, FAILED, MIN_COUNTER_ADVANCES, MIN_SCC14_FRAMES, \
                                                         POST_COMMIT_ACTIVE_GRACE_SECONDS, READY, REQUESTED, SCC11, SCC12, SCC14, \
                                                         SCC_LIVENESS_TIMEOUT_SECONDS, StockSccVerifier, VERIFYING, \
                                                         controls_fully_disengaged, is_supported_car, state_name, state_timestamp, timestamped_state
from openpilot.selfdrive.car.live_update_handoff import should_suppress_always_on_lateral


def test_state_encoding():
  state = timestamped_state("ready", 123.456789)
  assert state_name(state) == "ready"
  assert state_timestamp(state) == 123.456789
  assert state_timestamp("ready:not-a-number") is None


def test_supported_car_is_classic_can_hyundai_long_only():
  cp = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC)
  assert is_supported_car(cp)

  for flags in (HyundaiFlags.CANFD, HyundaiFlags.CAMERA_SCC, HyundaiFlags.CANFD_CAMERA_SCC):
    cp.flags = flags
    assert not is_supported_car(cp)

  cp.flags = HyundaiFlags.RADAR_SCC
  cp.openpilotLongitudinalControl = False
  assert not is_supported_car(cp)


def test_full_disengagement_requires_cruise_main_and_aol_off():
  cs = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False))
  cc = SimpleNamespace(enabled=False, latActive=False)
  fpcs = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert controls_fully_disengaged(cs, cc, fpcs, False)

  cs.cruiseState.available = True
  assert not controls_fully_disengaged(cs, cc, fpcs, False)
  cs.cruiseState.available = False
  fpcs.alwaysOnLateralEnabled = True
  assert not controls_fully_disengaged(cs, cc, fpcs, False)
  fpcs.alwaysOnLateralEnabled = False
  assert not controls_fully_disengaged(cs, cc, fpcs, False, selfdrive_active=True)
  assert not controls_fully_disengaged(cs, cc, fpcs, False, cruise_buttons_pressed=True)


def test_pending_handoff_suppresses_aol_once_scc_main_is_off():
  assert not should_suppress_always_on_lateral(REQUESTED, True)
  assert should_suppress_always_on_lateral(REQUESTED, False)
  assert should_suppress_always_on_lateral(VERIFYING, False)
  assert not should_suppress_always_on_lateral("", False)


def _packet(address, counter, src=0):
  data = bytearray(8)
  if address == SCC11:
    data[0] = counter << 4
  elif address == SCC12:
    data[7] = counter
  return (0, [(address, bytes(data), src)])


def test_stock_scc_verifier_requires_live_counters_and_scc14():
  verifier = StockSccVerifier()
  for counter in range(MIN_COUNTER_ADVANCES + 1):
    verifier.update([_packet(SCC11, counter % 16), _packet(SCC12, counter % 16)], counter * 0.01)
  assert not verifier.ready

  for _ in range(MIN_SCC14_FRAMES):
    verifier.update([_packet(SCC14, 0)], MIN_COUNTER_ADVANCES * 0.01)
  assert verifier.ready
  assert verifier.live(MIN_COUNTER_ADVANCES * 0.01)
  assert not verifier.live(MIN_COUNTER_ADVANCES * 0.01 + SCC_LIVENESS_TIMEOUT_SECONDS + 0.01)


def test_stock_scc_verifier_requires_passive_healthy_status():
  verifier = StockSccVerifier()
  for counter in range(MIN_COUNTER_ADVANCES + 1):
    verifier.update([_packet(SCC11, counter % 16), _packet(SCC12, counter % 16)], counter * 0.01)
  for _ in range(MIN_SCC14_FRAMES):
    verifier.update([_packet(SCC14, 0)], MIN_COUNTER_ADVANCES * 0.01)
  assert verifier.ready

  scc11_active = bytearray(_packet(SCC11, 0)[1][0][1])
  scc11_active[0] |= 0x1
  verifier.update([(0, [(SCC11, bytes(scc11_active), 0)])], 0.3)
  assert not verifier.ready

  scc12_faulted = bytearray(_packet(SCC12, 0)[1][0][1])
  scc12_faulted[1] = 1 << 3
  verifier.update([(0, [(SCC12, bytes(scc12_faulted), 0)])], 0.3)
  assert not verifier.ready


def test_stock_scc_verifier_ignores_send_echo_bus():
  verifier = StockSccVerifier()
  for counter in range(MIN_COUNTER_ADVANCES + 2):
    verifier.update([_packet(SCC11, counter % 16, src=128), _packet(SCC12, counter % 16, src=128), _packet(SCC14, 0, src=128)])
  assert not verifier.ready


class FakeParams:
  def __init__(self, state):
    self.state = state
    self.removed = []

  def get(self, key):
    return self.state

  def put(self, key, value):
    self.state = value

  def put_nonblocking(self, key, value):
    self.state = value

  def remove(self, key):
    self.removed.append(key)


class FakeSubMaster:
  def __init__(self):
    self.data = {
      "carControl": SimpleNamespace(enabled=False, latActive=False),
      "selfdriveState": SimpleNamespace(enabled=False, active=False),
      "pandaStates": [SimpleNamespace(safetyModel=car.CarParams.SafetyModel.hyundai, faults=[])],
    }

  def __getitem__(self, service):
    return self.data[service]

  def all_checks(self, services):
    return True

  def all_alive(self, services):
    return True


def test_card_keeps_sending_until_panda_confirms_elm(monkeypatch):
  now = [10.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(REQUESTED)
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = None
  card_instance.live_update_handoff_verify_started_at = None
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = {ButtonType.mainCruise}
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.last_actuators_output = None
  card_instance.initialized_prev = True
  card_instance.can_callbacks = (object(), object())
  card_instance.can_list = []
  deinit_calls = []
  init_calls = []
  card_instance.CI = SimpleNamespace(init=lambda *args: init_calls.append(args), deinit=lambda *args: deinit_calls.append(args) or True)

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False), gearShifter=car.CarState.GearShifter.park)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)

  now[0] += 2.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == REQUESTED
  assert card_instance.live_update_handoff_preconditions_since is None

  card_instance.live_update_handoff_pressed_buttons.clear()
  now[0] += 0.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  now[0] += 2.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == REQUESTED
  assert card_instance.live_update_handoff_preconditions_since is None

  CS.gearShifter = car.CarState.GearShifter.drive
  now[0] += 0.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  now[0] += 2.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == DIAGNOSTIC_REQUESTED
  assert not deinit_calls

  card_instance.params.state = DIAGNOSTIC
  now[0] += 0.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert not deinit_calls

  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  now[0] += 0.1
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert len(deinit_calls) == 1
  assert card_instance.params.removed == ["ControlsReady"]
  assert state_name(card_instance.params.state) == VERIFYING

  for counter in range(MIN_COUNTER_ADVANCES + 1):
    card_instance.can_list.extend([_packet(SCC11, counter % 16), _packet(SCC12, counter % 16)])
  for _ in range(MIN_SCC14_FRAMES):
    card_instance.can_list.append(_packet(SCC14, 0))
  now[0] += 0.1
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == READY

  card_instance.initialized_prev = False
  card_instance.controls_update(CS, card_instance.sm["carControl"], controls_quiesced=True)
  assert not init_calls


def test_committed_handoff_failure_stays_quiesced_and_restores_radar(monkeypatch):
  now = [20.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(DIAGNOSTIC_REQUESTED)
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = 19.0
  card_instance.live_update_handoff_verify_started_at = None
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.sm.data["carControl"].enabled = True
  card_instance.initialized_prev = True
  card_instance.can_callbacks = (object(), object())
  card_instance.can_list = []
  deinit_calls = []
  card_instance.CI = SimpleNamespace(deinit=lambda *args, **kwargs: deinit_calls.append((args, kwargs)) or True)

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=True, enabled=False), gearShifter=car.CarState.GearShifter.drive)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == DIAGNOSTIC_REQUESTED

  now[0] += POST_COMMIT_ACTIVE_GRACE_SECONDS + 0.01
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == FAILED

  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert len(deinit_calls) == 1
  assert deinit_calls[0][1] == {"retry": 2}
  assert card_instance.params.removed == ["ControlsReady"]


def test_transient_post_commit_activity_reverifies_instead_of_failing(monkeypatch):
  now = [25.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(timestamped_state(READY, now[0]))
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = 24.0
  card_instance.live_update_handoff_verify_started_at = 24.0
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  card_instance.sm.data["carControl"].enabled = True
  card_instance.can_list = []

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False), gearShifter=car.CarState.GearShifter.drive)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == VERIFYING

  card_instance.sm.data["carControl"].enabled = False
  now[0] += POST_COMMIT_ACTIVE_GRACE_SECONDS / 2
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == VERIFYING
  assert card_instance.live_update_handoff_controls_active_since is None


def test_request_waits_out_fingerprinting_elm_mode(monkeypatch):
  monkeypatch.setattr(card_module.time, "monotonic", lambda: 25.0)

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(REQUESTED)
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = None
  card_instance.live_update_handoff_verify_started_at = None
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  card_instance.initialized_prev = False
  card_instance.can_list = []

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False), gearShifter=car.CarState.GearShifter.drive)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert not card_instance.update_live_update_handoff(CS, FPCS, False)
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == REQUESTED
  assert card_instance.live_update_handoff_preconditions_since is None


def test_terminal_state_resets_request_dwell(monkeypatch):
  monkeypatch.setattr(card_module.time, "monotonic", lambda: 30.0)
  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(REQUESTED)
  card_instance.live_update_handoff_state = REQUESTED
  card_instance.live_update_handoff_last_read = 30.0
  card_instance.live_update_handoff_preconditions_since = 25.0

  card_instance._set_live_update_handoff_state(timestamped_state(FAILED, 30.0))
  assert card_instance.live_update_handoff_preconditions_since is None
