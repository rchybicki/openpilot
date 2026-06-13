# Fork CAN-bounds tests for the stopping-stack redesign (spec §4.3-4.5, §8, WP2).
#
# Covers:
#   - zero behavioral CAN delta with flags at spec defaults (frame-for-frame SCC11/SCC12/SCC14 byte
#     equality vs a verbatim copy of the pre-change pipeline, incl. engagement rising edges)
#   - non-finite accel neutralized to 0.0 BEFORE the clip (np.clip propagates NaN; CANPacker raises)
#   - unconditional accel/jerk clips at the packing boundary (CANPacker WRAPS, it does not clamp)
#   - StopReq gate + latch semantics for both flag states, incl. the always-active speed release (F1)
#   - telemetry == sent accel under the post-engagement cap, incl. the rising-edge frame (§4.3)
#   - synthetic-ramp jerk pin: 50 Hz cadence/divisor pairing of the dynamic SCC14 jerk (F6)
import opendbc.car.hyundai.tests.conftest  # noqa: F401  installs the openpilot.common.params stub (F18); required under --noconftest
import math
from collections import defaultdict
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.can import CANPacker
from opendbc.can.dbc import DBC
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import carcontroller as cc_mod
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.hyundaican import hyundai_checksum, create_acc_commands
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags

LongCtrlState = structs.CarControl.Actuators.LongControlState

SCC11_ADDR = 0x420
SCC12_ADDR = 0x421
SCC14_ADDR = 0x389
SCC_ADDRS = (SCC11_ADDR, SCC12_ADDR, SCC14_ADDR)

_DBC = DBC('hyundai_kia_generic')


def get_signal(msg_name, sig_name, dat):
  # all SCC signals of interest are little-endian unsigned
  s = _DBC.name_to_msg[msg_name].sigs[sig_name]
  ival = (int.from_bytes(dat, 'little') >> s.lsb) & ((1 << s.size) - 1)
  return ival * s.factor + s.offset


def make_cp():
  CP = CarInterface.get_non_essential_params(CAR.HYUNDAI_SANTA_FE_HEV_2022)
  CP.openpilotLongitudinalControl = True
  return CP


def make_controller(CP=None):
  CP = CP or make_cp()
  return CarController({Bus.pt: 'hyundai_kia_generic'}, CP), CP


def make_cc(accel=0.0, state=LongCtrlState.pid, long_active=True, enabled=True,
            set_speed=10.0, lead_visible=False, override=False):
  cc = structs.CarControl()
  cc.enabled = enabled
  cc.longActive = long_active
  cc.latActive = False
  cc.actuators.accel = float(accel)
  cc.actuators.longControlState = state
  cc.hudControl.setSpeed = set_speed
  cc.hudControl.leadDistanceBars = 2
  cc.hudControl.leadVisible = lead_visible
  cc.cruiseControl.override = override
  return cc.as_reader()


def make_cs(v_ego=0.0, a_ego=0.0, available=True):
  out = structs.CarState()
  out.vEgo = float(v_ego)
  out.aEgo = float(a_ego)
  out.cruiseState.available = available
  return SimpleNamespace(out=out, lkas11=defaultdict(int), clu11=defaultdict(int), is_metric=True)


def run_frame(ctrl, cmd):
  ccr = make_cc(accel=cmd.get('accel', 0.0), state=cmd.get('state', LongCtrlState.pid),
                long_active=cmd.get('long_active', True), enabled=cmd.get('enabled', True))
  cs = make_cs(v_ego=cmd.get('v_ego', 0.0), a_ego=cmd.get('a_ego', 0.0))
  new_act, sends = ctrl.update(ccr, cs, 0, SimpleNamespace())
  scc = {s[0]: s[1] for s in sends if s[0] in SCC_ADDRS}
  return new_act, scc


def step_scc(ctrl, **cmd):
  # one 50 Hz SCC interval: an even (send) frame + an odd frame with identical inputs
  assert ctrl.frame % 2 == 0
  new_act, scc = run_frame(ctrl, cmd)
  run_frame(ctrl, cmd)
  assert set(scc) == set(SCC_ADDRS)
  return new_act, scc


# ---------------------------------------------------------------------------
# verbatim copy of the PRE-CHANGE pipeline (base commit 3be25f5240) — the byte oracle
# ---------------------------------------------------------------------------

def _legacy_create_acc_commands(packer, enabled, accel, upper_jerk, idx, hud_control, set_speed, stopping, long_override, use_fca, cruise_available, CP):
  commands = []

  scc11_values = {
    "MainMode_ACC": 1 if cruise_available else 0,
    "TauGapSet": hud_control.leadDistanceBars,
    "VSetDis": set_speed if enabled else 0,
    "AliveCounterACC": idx % 0x10,
    "ObjValid": 1,
    "ACC_ObjStatus": 1,
    "ACC_ObjLatPos": 0,
    "ACC_ObjRelSpd": 0,
    "ACC_ObjDist": 1,
    }
  commands.append(packer.make_can_msg("SCC11", 0, scc11_values))

  scc12_values = {
    "ACCMode": 2 if enabled and long_override else 1 if enabled else 0,
    "StopReq": 1 if stopping else 0,
    "aReqRaw": accel,
    "aReqValue": accel,
    "CR_VSM_Alive": idx % 0xF,
  }

  if not use_fca:
    scc12_values["CF_VSM_ConfMode"] = 1
    scc12_values["AEB_Status"] = 1

  scc12_dat = packer.make_can_msg("SCC12", 0, scc12_values)[1]
  scc12_values["CR_VSM_ChkSum"] = 0x10 - sum(sum(divmod(i, 16)) for i in scc12_dat) % 0x10

  commands.append(packer.make_can_msg("SCC12", 0, scc12_values))

  scc14_values = {
    "ComfortBandUpper": 0.0,
    "ComfortBandLower": 0.0,
    "JerkUpperLimit": upper_jerk,
    "JerkLowerLimit": 5.0,
    "ACCMode": 2 if enabled and long_override else 1 if enabled else 4,
    "ObjGap": 2 if hud_control.leadVisible else 0,
  }
  commands.append(packer.make_can_msg("SCC14", 0, scc14_values))

  if use_fca and not (CP.flags & HyundaiFlags.CAMERA_SCC):
    fca11_values = {
      "CR_FCA_Alive": idx % 0xF,
      "PAINT1_Status": 1,
      "FCA_DrvSetStatus": 1,
      "FCA_Status": 1,
    }
    fca11_dat = packer.make_can_msg("FCA11", 0, fca11_values)[1]
    fca11_values["CR_FCA_ChkSum"] = hyundai_checksum(fca11_dat[:7])
    commands.append(packer.make_can_msg("FCA11", 0, fca11_values))

  return commands


class LegacyReference:
  """Pre-change carcontroller longitudinal pipeline (classic CAN), frame-for-frame."""

  def __init__(self, CP):
    self.CP = CP
    self.packer = CANPacker('hyundai_kia_generic')
    self.frame = 0
    self.engaged_frame = 0

  def update(self, CC, CS):
    actuators = CC.actuators
    accel = float(np.clip(actuators.accel, cc_mod.CarControllerParams.ACCEL_MIN, cc_mod.CarControllerParams.ACCEL_MAX))
    stopping = actuators.longControlState == LongCtrlState.stopping and CS.out.vEgo < 0.01
    set_speed_in_units = CC.hudControl.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    self.engaged_frame = self.frame if CC.longActive and self.engaged_frame == 0 else 0 if not CC.longActive else self.engaged_frame

    msgs = []
    if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
      v_ego_kph = CS.out.vEgo * CV.MS_TO_KPH
      engaged_active = v_ego_kph < 50.0 and (self.frame - self.engaged_frame) * DT_CTRL < 4.0
      if engaged_active and accel > CS.out.aEgo and CS.out.aEgo < 0.5:
        accel = min(accel, max(CS.out.aEgo * 1.3, 0.6))
      jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
      use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
      msgs = _legacy_create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                         CC.hudControl, set_speed_in_units, stopping,
                                         CC.cruiseControl.override, use_fca, CS.out.cruiseState.available, self.CP)
    self.frame += 1
    return msgs


def drive_sequence():
  """Scripted command sequence covering engagement edges (odd + even frame), the post-engagement cap
  window, accel ramps, stopping entry, standstill vEgo dither around the StopReq gate, and re-engage."""
  seq = []
  # disengaged cruise at 8 m/s
  for _ in range(21):
    seq.append(dict(accel=0.0, state=LongCtrlState.pid, v_ego=8.0, a_ego=0.0, long_active=False, enabled=False))
  # engage at frame 21 (ODD rising edge), gentle accel: cap window active (v < 50 km/h, aEgo < 0.5)
  for k in range(60):
    seq.append(dict(accel=0.8, state=LongCtrlState.pid, v_ego=8.0 + 0.005 * k, a_ego=min(0.45, 0.02 * k), long_active=True, enabled=True))
  # decel ramp toward a stop
  for k in range(200):
    accel = max(-1.2, 0.5 - 0.02 * k)
    v = max(0.0, 8.3 - 0.05 * k)
    seq.append(dict(accel=accel, state=LongCtrlState.pid, v_ego=v, a_ego=accel * 0.8, long_active=True, enabled=True))
  # stopping state, final approach + standstill dither around the 0.01 m/s StopReq gate
  for v in (0.45, 0.30, 0.18, 0.09, 0.04, 0.02, 0.012, 0.005, 0.02, 0.008, 0.005, 0.003, 0.011, 0.005):
    for _ in range(4):
      seq.append(dict(accel=-0.25, state=LongCtrlState.stopping, v_ego=v, a_ego=-0.1, long_active=True, enabled=True))
  # brief disengage at standstill
  for _ in range(11):
    seq.append(dict(accel=0.0, state=LongCtrlState.off, v_ego=0.0, a_ego=0.0, long_active=False, enabled=False))
  # re-engage at an EVEN rising edge (frame index parity differs from first engage), launch
  for k in range(80):
    seq.append(dict(accel=0.7, state=LongCtrlState.pid, v_ego=0.05 * k, a_ego=min(0.7, 0.05 * k), long_active=True, enabled=True))
  return seq


# STAGED FLIP (on_vehicle_protocols.md §1) — staged-flip procedure for the StopReq constants:
#
# This suite proves the CAN delta against the verbatim pre-change oracle (LegacyReference). When a
# StopReq stage flips a constant, the live config is no longer byte-identical to legacy — the StopReq
# bit (SCC12 only) diverges across the dither band by exactly the staged amount. The proof is staged:
#
#  - The DARK/LEGACY state stays covered by parametrization: monkeypatch the constants back to the
#    legacy values (latch off, gate 0.01) and assert frame-for-frame byte equality vs the oracle
#    (this is the revert-target proof — reverting the constants restores byte-identical legacy).
#  - The LIVE state pins the staged behavior: SCC11/SCC14 stay byte-identical to legacy (StopReq lives
#    only in SCC12), and SCC12 diverges from legacy ONLY in the StopReq bit, only inside the documented
#    speed band. No accel/jerk/HUD byte may move.
#  - test_default_constants_are_normative pins the LIVE staged values, with the legacy revert targets
#    documented inline. To revert: set the three constants back to the LEGACY_* values below.
#
# CURRENT STAGE: COMFORT EXPERIMENT (STOPREQ_LATCH=True, gate 0.01, release 0.10). The gate now equals
# the legacy set-threshold (0.01), so the StopReq bit ASSERTS at the same speed as legacy (vEgo < 0.01) —
# openpilot commands the terminal decel down to standstill instead of handing the final stop to the SCC at
# 0.04. The ONLY divergence from the legacy oracle is the LATCH: where legacy chatters StopReq back to 0 as
# Kalman-filtered vEgo dithers up between 0.01 and the 0.10 release, the latch holds StopReq=1. So the live
# SCC12 divergence is StopReq=1-where-legacy=0, confined to the dither band [0.01, 0.10) on a latched hold.

LEGACY_STOPREQ_LATCH = False
LEGACY_STOP_REQ_MAX_SPEED = 0.01
LEGACY_STOPREQ_RELEASE_SPEED = 0.10


class TestZeroCanDelta:
  def test_scc_bytes_identical_to_legacy_reference_in_dark_state(self, monkeypatch):
    # DARK/LEGACY revert target: with the constants monkeypatched back to legacy, the live pipeline is
    # frame-for-frame byte-identical to the pre-change oracle. This is the proof that reverting the
    # staged constants restores byte-identical legacy behavior (the rollback path).
    monkeypatch.setattr(cc_mod, 'STOPREQ_LATCH', LEGACY_STOPREQ_LATCH)
    monkeypatch.setattr(cc_mod, 'STOP_REQ_MAX_SPEED', LEGACY_STOP_REQ_MAX_SPEED)
    monkeypatch.setattr(cc_mod, 'STOPREQ_RELEASE_SPEED', LEGACY_STOPREQ_RELEASE_SPEED)
    ctrl, CP = make_controller()
    legacy = LegacyReference(CP)
    for i, cmd in enumerate(drive_sequence()):
      ccr = make_cc(accel=cmd['accel'], state=cmd['state'], long_active=cmd['long_active'], enabled=cmd['enabled'])
      cs = make_cs(v_ego=cmd['v_ego'], a_ego=cmd['a_ego'])
      _, sends = ctrl.update(ccr, cs, 0, SimpleNamespace())
      new_scc = [(s[0], bytes(s[1])) for s in sends if s[0] in SCC_ADDRS]
      legacy_scc = [(m[0], bytes(m[1])) for m in legacy.update(ccr, cs)]
      assert new_scc == legacy_scc, f"SCC byte mismatch at frame {i} (cmd={cmd})"

  def test_live_diverges_from_legacy_only_in_stopreq_bit(self):
    # LIVE comfort-experiment proof: with the shipped constants (latch ON, gate 0.01 == legacy set-
    # threshold), the ONLY byte that may diverge from the legacy oracle is SCC12's StopReq bit.
    # SCC11/SCC14 must remain byte-identical (StopReq lives only in SCC12), and the SCC12 divergence
    # must be the StopReq bit ALONE (every other field byte-equal). The gate now equals legacy's, so
    # the assert SPEED matches legacy — the divergence is purely the LATCH holding StopReq=1 through the
    # dither band [0.01, 0.10) where legacy chatters it back to 0.
    assert cc_mod.STOPREQ_LATCH is True and cc_mod.STOP_REQ_MAX_SPEED == 0.01  # guard: pins comfort experiment
    ctrl, CP = make_controller()
    legacy = LegacyReference(CP)
    saw_divergence = False
    for i, cmd in enumerate(drive_sequence()):
      ccr = make_cc(accel=cmd['accel'], state=cmd['state'], long_active=cmd['long_active'], enabled=cmd['enabled'])
      cs = make_cs(v_ego=cmd['v_ego'], a_ego=cmd['a_ego'])
      _, sends = ctrl.update(ccr, cs, 0, SimpleNamespace())
      new_scc = {s[0]: bytes(s[1]) for s in sends if s[0] in SCC_ADDRS}
      legacy_scc = {m[0]: bytes(m[1]) for m in legacy.update(ccr, cs)}
      assert set(new_scc) == set(legacy_scc)
      if not new_scc:  # odd 100 Hz frame: no SCC message emitted on either side (50 Hz send block)
        continue
      # SCC11 + SCC14 are not touched by StopReq — byte-identical at every frame
      assert new_scc[SCC11_ADDR] == legacy_scc[SCC11_ADDR], f"SCC11 moved at frame {i} (cmd={cmd})"
      assert new_scc[SCC14_ADDR] == legacy_scc[SCC14_ADDR], f"SCC14 moved at frame {i} (cmd={cmd})"
      # SCC12: the only permitted divergence is the StopReq bit
      new_sr = int(get_signal("SCC12", "StopReq", new_scc[SCC12_ADDR]))
      leg_sr = int(get_signal("SCC12", "StopReq", legacy_scc[SCC12_ADDR]))
      if new_scc[SCC12_ADDR] != legacy_scc[SCC12_ADDR]:
        saw_divergence = True
        assert new_sr != leg_sr, f"SCC12 moved but NOT on the StopReq bit at frame {i} (cmd={cmd})"
        # Every SCC12 PAYLOAD field except the StopReq bit must be byte-equal. CR_VSM_ChkSum is excluded
        # from the equality check because it is a function OF the StopReq bit — the checksum legitimately
        # recomputes when StopReq flips (a dependent field, not an independent command move).
        for sig_name in ("aReqRaw", "aReqValue", "ACCMode", "CR_VSM_Alive"):
          assert get_signal("SCC12", sig_name, new_scc[SCC12_ADDR]) == pytest.approx(
            get_signal("SCC12", sig_name, legacy_scc[SCC12_ADDR]), abs=0.011), \
            f"SCC12 field {sig_name} moved at frame {i} (cmd={cmd})"
        # divergence is only legitimate as a StopReq assert that legacy did not make (stage A asserts
        # earlier / latches through dither): stage-A StopReq=1 where legacy=0, never the reverse.
        assert new_sr == 1 and leg_sr == 0, f"unexpected StopReq divergence direction at frame {i} (cmd={cmd})"
    assert saw_divergence, "stage A must diverge from legacy somewhere in the drive sequence (dither band / latch)"

  def test_default_constants_are_normative(self):
    # spec §3 CAN-layer constants. StopReq constants pin the LIVE comfort-experiment stage; the other dark
    # flags stay off. Revert targets: STOPREQ_LATCH=False, STOP_REQ_MAX_SPEED=0.01 (== legacy).
    assert cc_mod.STOPREQ_LATCH is True             # COMFORT EXPERIMENT (was False == legacy)
    assert cc_mod.STOP_REQ_MAX_SPEED == 0.01        # COMFORT EXPERIMENT == legacy set-threshold (was 0.04 == STAGE A)
    assert cc_mod.STOPREQ_RELEASE_SPEED == 0.10     # unchanged from stage 0; always-active speed release (F1)
    assert cc_mod.DYNAMIC_SCC14_JERK is False
    assert cc_mod.REPORT_SENT_ACCEL is True
    assert cc_mod.SCC14_JERK_UPPER_PID == 3.0
    assert cc_mod.SCC14_JERK_UPPER_STOPPING == 1.0
    assert cc_mod.SCC14_JERK_LOWER == 5.0
    assert cc_mod.SCC14_JERK_MAX == 12.7
    assert cc_mod.TELEMETRY_VERSION == 2


class TestNonFiniteGuard:
  @pytest.mark.parametrize("bad", [float('nan'), float('inf'), float('-inf')])
  def test_nonfinite_accel_neutralized_before_clip(self, bad):
    ctrl, _ = make_controller()
    new_act, scc = step_scc(ctrl, accel=bad, state=LongCtrlState.pid, v_ego=5.0, a_ego=1.0)
    assert new_act.accel == 0.0
    for sig_name in ("aReqRaw", "aReqValue"):
      assert get_signal("SCC12", sig_name, scc[SCC12_ADDR]) == pytest.approx(0.0, abs=0.011)

  @pytest.mark.parametrize("bad", [float('nan'), float('inf')])
  def test_nonfinite_accel_with_dynamic_jerk(self, monkeypatch, bad):
    monkeypatch.setattr(cc_mod, 'DYNAMIC_SCC14_JERK', True)
    ctrl, _ = make_controller()
    _, scc = step_scc(ctrl, accel=bad, state=LongCtrlState.pid, v_ego=5.0, a_ego=1.0)
    for sig_name in ("JerkUpperLimit", "JerkLowerLimit"):
      val = get_signal("SCC14", sig_name, scc[SCC14_ADDR])
      assert math.isfinite(val) and 0.0 <= val <= 12.7

  def test_adversarial_accel_clipped(self):
    ctrl, _ = make_controller()
    # aEgo high so the engagement cap cannot mask the clip
    _, scc = step_scc(ctrl, accel=10.0, state=LongCtrlState.pid, v_ego=5.0, a_ego=1.0)
    assert get_signal("SCC12", "aReqValue", scc[SCC12_ADDR]) == pytest.approx(2.0, abs=0.011)
    ctrl, _ = make_controller()
    _, scc = step_scc(ctrl, accel=-10.0, state=LongCtrlState.pid, v_ego=5.0, a_ego=1.0)
    assert get_signal("SCC12", "aReqValue", scc[SCC12_ADDR]) == pytest.approx(-3.5, abs=0.011)


class TestJerkPackingClip:
  def test_jerk_fields_clipped_unconditionally(self):
    # direct packing-boundary test: CANPacker WRAPS out-of-range (13.0 -> 0.2), the clip must catch it
    packer = CANPacker('hyundai_kia_generic')
    hud = SimpleNamespace(leadDistanceBars=2, leadVisible=False)
    for upper, lower, exp_upper, exp_lower in [(50.0, 99.0, 12.7, 12.7),
                                               (13.0, 12.8, 12.7, 12.7),
                                               (-3.0, -1.0, 0.0, 0.0),
                                               (3.0, 5.0, 3.0, 5.0)]:
      msgs = create_acc_commands(packer, True, 0.0, upper, 0, hud, 10.0, False, False, 0, True,
                                 SimpleNamespace(flags=0), lower_jerk=lower)
      scc14 = next(m for m in msgs if m[0] == SCC14_ADDR)
      assert get_signal("SCC14", "JerkUpperLimit", scc14[1]) == pytest.approx(exp_upper, abs=0.051)
      assert get_signal("SCC14", "JerkLowerLimit", scc14[1]) == pytest.approx(exp_lower, abs=0.051)

  def test_dynamic_jerk_step_bounded_at_12_7(self, monkeypatch):
    # a full-range command step (-3.5 -> 2.0 in one SCC interval = 275 m/s^3) must pack at the ceiling
    monkeypatch.setattr(cc_mod, 'DYNAMIC_SCC14_JERK', True)
    ctrl, _ = make_controller()
    step_scc(ctrl, accel=-3.5, state=LongCtrlState.pid, v_ego=5.0, a_ego=1.0)
    _, scc = step_scc(ctrl, accel=2.0, state=LongCtrlState.pid, v_ego=5.0, a_ego=1.0)
    assert get_signal("SCC14", "JerkUpperLimit", scc[SCC14_ADDR]) == pytest.approx(12.7, abs=0.051)
    _, scc = step_scc(ctrl, accel=-3.5, state=LongCtrlState.pid, v_ego=5.0, a_ego=1.0)
    assert get_signal("SCC14", "JerkLowerLimit", scc[SCC14_ADDR]) == pytest.approx(12.7, abs=0.051)


class TestStopReq:
  def stopreq(self, scc):
    return int(get_signal("SCC12", "StopReq", scc[SCC12_ADDR]))

  def test_legacy_gate_dark_state(self, monkeypatch):
    # DARK/LEGACY (revert target): STOPREQ_LATCH=False, gate 0.01 — StopReq == (stopping AND vEgo < 0.01),
    # chatter included. Monkeypatched back to legacy now that stage A is the live default.
    monkeypatch.setattr(cc_mod, 'STOPREQ_LATCH', False)
    monkeypatch.setattr(cc_mod, 'STOP_REQ_MAX_SPEED', 0.01)
    ctrl, _ = make_controller()
    expected = []
    actual = []
    for v in (0.30, 0.005, 0.02, 0.005, 0.012, 0.009):
      _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=v, a_ego=0.0)
      actual.append(self.stopreq(scc))
      expected.append(1 if v < 0.01 else 0)
    assert actual == expected
    # state exit deasserts regardless of speed
    _, scc = step_scc(ctrl, accel=0.0, state=LongCtrlState.pid, v_ego=0.005, a_ego=0.0)
    assert self.stopreq(scc) == 0

  def test_live_gate_and_latch(self):
    # LIVE comfort experiment (shipped defaults, no monkeypatch): latch ON, gate 0.01, release 0.10.
    # StopReq asserts on stopping ∧ vEgo < 0.01 (== legacy set-threshold, openpilot commands the terminal
    # decel) and latches through the Kalman dither band (< release 0.10), clears on a genuine roll
    # (> release) or state exit.
    assert cc_mod.STOPREQ_LATCH is True and cc_mod.STOP_REQ_MAX_SPEED == 0.01  # guard: pins comfort experiment
    ctrl, _ = make_controller()
    # above the gate but stopping not yet at standstill: no assert yet
    _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=0.06, a_ego=-0.1)
    assert self.stopreq(scc) == 0
    # still above the 0.01 gate: no assert (openpilot still commanding the decel down)
    _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=0.02, a_ego=-0.05)
    assert self.stopreq(scc) == 0
    # drops below the 0.01 gate -> latch sets
    _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=0.005, a_ego=0.0)
    assert self.stopreq(scc) == 1
    # dither up to 0.09 (< release 0.10): latch holds (no 50 Hz chatter)
    for v in (0.05, 0.08, 0.09):
      _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=v, a_ego=0.0)
      assert self.stopreq(scc) == 1, f"latch must hold through dither at v={v}"
    # genuine roll past the release -> clears (F1: never hold StopReq on a rolling car)
    _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=0.15, a_ego=0.1)
    assert self.stopreq(scc) == 0
    # re-latch only on a genuine drop below the 0.01 gate, then state exit clears regardless
    _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=0.005, a_ego=0.0)
    assert self.stopreq(scc) == 1
    _, scc = step_scc(ctrl, accel=0.7, state=LongCtrlState.starting, v_ego=0.005, a_ego=0.0)
    assert self.stopreq(scc) == 0

  def test_latch_holds_through_dither(self, monkeypatch):
    monkeypatch.setattr(cc_mod, 'STOPREQ_LATCH', True)
    ctrl, _ = make_controller()
    _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=0.005, a_ego=0.0)
    assert self.stopreq(scc) == 1  # latched at standstill
    for v in (0.02, 0.05, 0.09):  # Kalman dither above the gate, below the release
      _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=v, a_ego=0.0)
      assert self.stopreq(scc) == 1, f"latch must hold through dither at v={v}"

  def test_latch_speed_release_on_creep_push(self, monkeypatch):
    # F1 (FATAL fix): the latch may NEVER hold StopReq on a rolling car. vEgo rising past
    # STOPREQ_RELEASE_SPEED clears it WHILE the state stays stopping (HEV creep-push case).
    monkeypatch.setattr(cc_mod, 'STOPREQ_LATCH', True)
    ctrl, _ = make_controller()
    _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=0.005, a_ego=0.0)
    assert self.stopreq(scc) == 1
    _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=0.15, a_ego=0.1)
    assert self.stopreq(scc) == 0, "speed release must clear the latch on a rolling car"
    # no re-latch until vEgo drops below the gate again
    _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=0.05, a_ego=0.0)
    assert self.stopreq(scc) == 0
    _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=0.005, a_ego=0.0)
    assert self.stopreq(scc) == 1

  def test_latch_clears_on_state_exit(self, monkeypatch):
    monkeypatch.setattr(cc_mod, 'STOPREQ_LATCH', True)
    ctrl, _ = make_controller()
    _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=0.005, a_ego=0.0)
    assert self.stopreq(scc) == 1
    _, scc = step_scc(ctrl, accel=0.7, state=LongCtrlState.starting, v_ego=0.005, a_ego=0.0)
    assert self.stopreq(scc) == 0
    # and it stays clear when stopping resumes above the gate
    _, scc = step_scc(ctrl, accel=-0.2, state=LongCtrlState.stopping, v_ego=0.05, a_ego=0.0)
    assert self.stopreq(scc) == 0


class TestTelemetryHoist:
  def test_reports_sent_accel_under_engagement_cap(self):
    # engagement cap: accel > aEgo, aEgo < 0.5, v < 50 km/h, within 4 s of rising edge
    ctrl, _ = make_controller()
    new_act, scc = step_scc(ctrl, accel=1.5, state=LongCtrlState.pid, v_ego=2.0, a_ego=0.0)
    assert new_act.accel == pytest.approx(0.6)  # min(1.5, max(0.0*1.3, 0.6))
    assert get_signal("SCC12", "aReqValue", scc[SCC12_ADDR]) == pytest.approx(0.6, abs=0.011)
    assert new_act.accel == pytest.approx(get_signal("SCC12", "aReqValue", scc[SCC12_ADDR]), abs=0.011)

  def test_rising_edge_frame_capped(self):
    # the rising-edge frame itself must already be inside the cap window (bookkeeping at top of update)
    ctrl, _ = make_controller()
    for _ in range(11):  # 11 inactive frames -> engagement lands on an ODD frame
      run_frame(ctrl, dict(accel=0.0, state=LongCtrlState.off, v_ego=2.0, a_ego=0.0, long_active=False, enabled=False))
    new_act, scc = run_frame(ctrl, dict(accel=1.5, state=LongCtrlState.pid, v_ego=2.0, a_ego=0.0))
    assert ctrl.engaged_frame == 11
    assert new_act.accel == pytest.approx(0.6)  # capped on the rising-edge frame itself
    new_act, scc = run_frame(ctrl, dict(accel=1.5, state=LongCtrlState.pid, v_ego=2.0, a_ego=0.0))
    assert get_signal("SCC12", "aReqValue", scc[SCC12_ADDR]) == pytest.approx(0.6, abs=0.011)
    assert new_act.accel == pytest.approx(0.6)

  def test_cap_expires_after_4s(self):
    ctrl, _ = make_controller()
    cmd = dict(accel=1.5, state=LongCtrlState.pid, v_ego=2.0, a_ego=0.0)
    # legacy bookkeeping quirk (preserved verbatim): engaged-from-boot settles engaged_frame at 1,
    # so the 4 s window runs through frame 400; the first uncapped SCC frame is 402
    for _ in range(201):
      step_scc(ctrl, **cmd)
    new_act, scc = step_scc(ctrl, **cmd)
    assert new_act.accel == pytest.approx(1.5)
    assert get_signal("SCC12", "aReqValue", scc[SCC12_ADDR]) == pytest.approx(1.5, abs=0.011)

  def test_kill_switch_restores_precap_telemetry(self, monkeypatch):
    monkeypatch.setattr(cc_mod, 'REPORT_SENT_ACCEL', False)
    ctrl, _ = make_controller()
    new_act, scc = step_scc(ctrl, accel=1.5, state=LongCtrlState.pid, v_ego=2.0, a_ego=0.0)
    assert new_act.accel == pytest.approx(1.5)  # legacy blind spot restored
    assert get_signal("SCC12", "aReqValue", scc[SCC12_ADDR]) == pytest.approx(0.6, abs=0.011)


class TestSyntheticRampJerk:
  """F6 pin: cmd_jerk must equal the true command slope — accel_last_scc updated only on sent frames,
  differencing interval 2*DT_CTRL. A 100 Hz-update/50 Hz-divisor mismatch halves the slope and fails."""

  def test_positive_ramp_pid_state(self, monkeypatch):
    monkeypatch.setattr(cc_mod, 'DYNAMIC_SCC14_JERK', True)
    slope = 4.0  # m/s^3 > pid floor 3.0 - margin, so the true slope shows through
    ctrl, _ = make_controller()
    accel = -2.0
    uppers, lowers = [], []
    for k in range(90):
      cmd = dict(accel=accel, state=LongCtrlState.pid, v_ego=5.0, a_ego=1.0)  # a_ego >= 0.5: cap can't fire
      _, scc = run_frame(ctrl, cmd)
      if k >= 4 and k % 2 == 0:
        uppers.append(get_signal("SCC14", "JerkUpperLimit", scc[SCC14_ADDR]))
        lowers.append(get_signal("SCC14", "JerkLowerLimit", scc[SCC14_ADDR]))
      accel += slope * DT_CTRL
    # cmd_jerk == true slope -> upper = clip(4.0 + 0.5, 3.0, 12.7) = 4.5 (exactly representable at 0.1)
    assert uppers and all(u == pytest.approx(4.5, abs=1e-6) for u in uppers)
    # releasing side sees no negative slope -> stays at the legacy lower floor
    assert all(lo == pytest.approx(5.0, abs=1e-6) for lo in lowers)

  def test_negative_ramp_stopping_state(self, monkeypatch):
    monkeypatch.setattr(cc_mod, 'DYNAMIC_SCC14_JERK', True)
    slope = -6.0  # |slope| > lower floor 5.0 + margin headroom check
    ctrl, _ = make_controller()
    accel = 1.0
    uppers, lowers = [], []
    for k in range(70):
      cmd = dict(accel=accel, state=LongCtrlState.stopping, v_ego=0.3, a_ego=0.6)
      _, scc = run_frame(ctrl, cmd)
      if k >= 4 and k % 2 == 0:
        uppers.append(get_signal("SCC14", "JerkUpperLimit", scc[SCC14_ADDR]))
        lowers.append(get_signal("SCC14", "JerkLowerLimit", scc[SCC14_ADDR]))
      accel += slope * DT_CTRL
    # lower = clip(6.0 + 0.5, 5.0, 12.7) = 6.5; upper floored at the stopping static 1.0
    assert lowers and all(lo == pytest.approx(6.5, abs=1e-6) for lo in lowers)
    assert all(u == pytest.approx(1.0, abs=1e-6) for u in uppers)

  def test_constant_command_floors_equal_legacy_statics(self, monkeypatch):
    # F6: dynamic floors == legacy static values — the dynamic path may never advertise LESS headroom
    monkeypatch.setattr(cc_mod, 'DYNAMIC_SCC14_JERK', True)
    ctrl, _ = make_controller()
    cmd = dict(accel=-0.5, state=LongCtrlState.pid, v_ego=5.0, a_ego=1.0)
    step_scc(ctrl, **cmd)
    _, scc = step_scc(ctrl, **cmd)
    assert get_signal("SCC14", "JerkUpperLimit", scc[SCC14_ADDR]) == pytest.approx(3.0, abs=1e-6)
    assert get_signal("SCC14", "JerkLowerLimit", scc[SCC14_ADDR]) == pytest.approx(5.0, abs=1e-6)
    ctrl, _ = make_controller()
    cmd = dict(accel=-0.2, state=LongCtrlState.stopping, v_ego=0.2, a_ego=0.6)
    step_scc(ctrl, **cmd)
    _, scc = step_scc(ctrl, **cmd)
    assert get_signal("SCC14", "JerkUpperLimit", scc[SCC14_ADDR]) == pytest.approx(1.0, abs=1e-6)
    assert get_signal("SCC14", "JerkLowerLimit", scc[SCC14_ADDR]) == pytest.approx(5.0, abs=1e-6)
