"""LongControl LIVE_TERMINAL wiring tests -- Stopping Service V3 stage 2
(docs/stopping/stopping_service_v3_plan.md §6 stage 2).

Proves the takeover/handback/exception semantics at the LongControl.update seam:
  - SHADOW and OFF stay byte-identical to the legacy wire (the one-flag revert path);
  - in LIVE_TERMINAL the service+context run in OBSERVATION over the full stage-1 band (warm
    a_coast/gap-filter/lead-latch before takeover) but the wire is written ONLY in the own band;
  - the service owns the stopping-state wire for v <= 0.85 (wire == service output), with a
    jerk-consistent takeover re-anchored on the live chain value (warm reseed / cold entry seed);
  - handback above the 0.95 hysteresis resumes the legacy wire while the service keeps observing
    (a re-takeover re-anchors the same way);
  - a service exception (owned OR observation frame) falls back to the legacy chain value for that
    frame, latches ownership OFF for the drive, and the wire stays finite/braking;
  - the legacy sub-0.30 over-brake cap family is bypassed ONLY on owned frames (SHADOW re-pins);
  - the force-coast -0.32 standstill hold still min()s (deepens) the service command;
  - a non-Santa-Fe CP never enters LIVE ownership.
"""

import math

import pytest

from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import (
  FORCE_COAST_STANDSTILL_HOLD_ACCEL,
  LongControl,
  SERVICE_LIVE_TERMINAL_V_OWN,
  SERVICE_LIVE_TERMINAL_V_RELEASE,
)
from openpilot.selfdrive.controls.lib.stopping_service import Phase, ServiceParams, StoppingService
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import (
  DummyCarParams,
  DummyCarState,
  DummyFrogPilotToggles,
)

P = ServiceParams()
DT = 0.01
EPS = 1e-9
LIMITS = (-3.0, 2.0)


# --- scripted-frame harness (LongControl does not integrate a plant: v/gap are scripted) -----------

def scripted_stop_frames(n: int = 700, v0: float = 1.5, decel: float = 0.5,
                         gap0: float = 6.0, gap_floor: float = 3.9):
  frames, v, gap = [], v0, gap0
  for _ in range(n):
    frames.append((v, -decel if v > 0.0 else 0.0, gap, v <= 0.0))
    v_new = max(v - decel * DT, 0.0)
    gap = max(gap - (v + v_new) / 2.0 * DT, gap_floor)
    v = v_new
  return frames


def close_stopped_lead_frames(n: int = 900, v0: float = 0.30, decel: float = 0.15,
                              gap0: float = 3.60):
  # the sub-0.30 legacy corridor: a close STOPPED lead while the ego glides the last 0.3 m/s out --
  # exactly where the legacy glide/close-lead cap family pins the wire deep (the felt grab)
  return scripted_stop_frames(n=n, v0=v0, decel=decel, gap0=gap0, gap_floor=2.0)


def run_frames(lc: LongControl, frames, *, a_target=-0.4, should_stop=True, dts=-1.0,
               force_coast=False, lead_v=0.0):
  toggles = DummyFrogPilotToggles()
  wires, ownings, phases, svc_cmds = [], [], [], []
  for (v, a, gap, standstill) in frames:
    cs = DummyCarState(v_ego=v, a_ego=a, standstill=standstill, cruise_standstill=False)
    out = lc.update(active=True, CS=cs, a_target=a_target, should_stop=should_stop,
                    distance_to_stop_target_m=dts, accel_limits=LIMITS,
                    frogpilot_toggles=toggles, lead_status=True, lead_v=lead_v,
                    lead_d_rel=gap, force_coast=force_coast)
    wires.append(float(out))
    ownings.append(bool(lc._service_live_owning))
    phases.append(lc._service_shadow_svc.phase)
    svc_cmds.append(float(lc._service_shadow_svc._last_cmd))
  return wires, ownings, phases, svc_cmds


# --- byte-identical wire in SHADOW and OFF (the one-flag revert path) ------------------------------

@pytest.mark.parametrize("frames_fn", [scripted_stop_frames, close_stopped_lead_frames])
def test_shadow_and_off_wires_byte_identical(monkeypatch, frames_fn) -> None:
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "OFF")
  wires_off, ownings_off, _, _ = run_frames(LongControl(DummyCarParams()), frames_fn())
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "SHADOW")
  wires_shadow, ownings_shadow, _, _ = run_frames(LongControl(DummyCarParams()), frames_fn())
  assert wires_off == wires_shadow
  assert not any(ownings_off) and not any(ownings_shadow)


# --- LIVE_TERMINAL: ownership, wire == service output, jerk-consistent takeover --------------------

def test_live_terminal_service_owns_wire_below_v_own(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE_TERMINAL")
  frames = scripted_stop_frames()
  wires, ownings, phases, svc_cmds = run_frames(LongControl(DummyCarParams()), frames)

  k_own = ownings.index(True)
  # ownership begins exactly when v first reaches the owned band, never above it
  assert frames[k_own][0] <= SERVICE_LIVE_TERMINAL_V_OWN + EPS
  assert frames[k_own - 1][0] > SERVICE_LIVE_TERMINAL_V_OWN
  # ... and holds through the stop (no spurious mid-stop handback on this nominal profile)
  assert all(ownings[k_own:])
  # the service IS the wire on every owned frame (no force_coast here, limits never bind)
  for k in range(k_own, len(wires)):
    assert wires[k] == pytest.approx(svc_cmds[k], abs=1e-12), f"wire != service at frame {k}"
    assert phases[k] != Phase.INACTIVE
  # terminal semantics: RAMP/HOLD at the -0.32 firm hold
  assert phases[-1] == Phase.HOLD
  assert wires[-1] == pytest.approx(P.A_HOLD_SECURE, abs=0.02)
  # everything stays finite and in braking territory during the owned stop
  assert all(math.isfinite(w) and w <= -0.03 + EPS for w in wires[k_own:])


def test_live_terminal_takeover_frame_is_jerk_consistent(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE_TERMINAL")
  seeds: list[tuple[bool, float]] = []
  orig = LongControl._run_stopping_service

  def spy(self, **kwargs):
    seeds.append((bool(kwargs["run"]), float(kwargs["wire_accel"])))
    return orig(self, **kwargs)

  monkeypatch.setattr(LongControl, "_run_stopping_service", spy)
  wires, ownings, phases, _ = run_frames(LongControl(DummyCarParams()), scripted_stop_frames())
  k_own = ownings.index(True)
  # full-band observation: the service is already WARM (ACTIVE, context conditioned) at takeover
  assert all(p != Phase.INACTIVE for p in phases[max(k_own - 50, 1):k_own])
  run_flag, seed_wire = seeds[k_own]
  assert run_flag
  # the warm service re-anchors its jerk limiter on the pre-takeover chain value (reseed_takeover):
  # the takeover-frame wire moves from that live value by no more than the service's own jerk
  # limits (deepen J_SAFE, release J_UP)
  step_from_seed = wires[k_own] - seed_wire
  assert step_from_seed >= -P.J_SAFE * DT - EPS, f"takeover deepen slam {step_from_seed:.4f}"
  assert step_from_seed <= P.J_UP * DT + EPS, f"takeover release slam {step_from_seed:.4f}"
  # end-to-end sanity: the wire step across the takeover boundary is bounded by one legacy jerk
  # step (the chain continues from last_output_accel) plus one service jerk step
  assert abs(wires[k_own] - wires[k_own - 1]) <= 2.0 * P.J_SAFE * DT + EPS


def test_live_terminal_wire_above_band_matches_legacy_prefix(monkeypatch) -> None:
  frames = scripted_stop_frames()
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "SHADOW")
  wires_shadow, _, _, _ = run_frames(LongControl(DummyCarParams()), frames)
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE_TERMINAL")
  wires_live, ownings, _, _ = run_frames(LongControl(DummyCarParams()), frames)
  k_own = ownings.index(True)
  # above the owned band the legacy chain is untouched: byte-identical prefix
  assert wires_live[:k_own] == wires_shadow[:k_own]
  # ... and the takeover genuinely changes the wire somewhere in the owned band
  assert wires_live[k_own:] != wires_shadow[k_own:]


# --- handback: hysteresis band + release above 0.95 resumes legacy ---------------------------------

def test_handback_above_hysteresis_resumes_legacy(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE_TERMINAL")
  frames = ([(1.20, -0.1, 8.0, False)] * 150     # above the band: legacy owns (service observes, warm)
            + [(0.60, -0.1, 8.0, False)] * 150   # owned band
            + [(0.90, -0.1, 8.0, False)] * 100   # 0.85 < v <= 0.95: hysteresis keeps ownership
            + [(1.05, -0.1, 8.0, False)] * 200   # above 0.95: handback (observation continues)
            + [(0.60, -0.1, 8.0, False)] * 100)  # back in the band: warm re-takeover
  wires, ownings, phases, _ = run_frames(LongControl(DummyCarParams()), frames, a_target=-0.2)

  assert not any(ownings[:150])                      # never owns above V_OWN
  assert all(ownings[151:300])                       # owns in the band (entry on frame 150)
  assert all(ownings[300:400]), "hysteresis: ownership must survive 0.85 < v <= 0.95"
  assert 0.90 > SERVICE_LIVE_TERMINAL_V_OWN and 0.90 <= SERVICE_LIVE_TERMINAL_V_RELEASE
  assert not any(ownings[400:600]), "must hand back above V_RELEASE"
  # handback keeps the service WARM in observation (full-band observation; the monitor ratchet and
  # context survive a mid-settle excursion past 0.95) -- but the wire is legacy on every frame
  assert all(p != Phase.INACTIVE for p in phases[401:600])
  # the legacy chain resumes from last_output_accel (which the service wrote): wire stays finite,
  # braking, and never steps harshly across the handback boundary
  assert all(math.isfinite(w) and w < 0.0 for w in wires[400:])
  assert abs(wires[400] - wires[399]) <= 0.35
  # dropping back into the band re-takes ownership with a jerk-consistent WARM re-anchor
  assert all(ownings[601:])
  assert abs(wires[600] - wires[599]) <= 0.35


# --- exception mid-stop: fallback to the chain value + drive-scoped ownership latch ----------------

def test_exception_mid_stop_falls_back_and_latches_ownership_off(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE_TERMINAL")
  calls = {"n": 0}
  orig = StoppingService.update

  def flaky(self, **kwargs):
    calls["n"] += 1
    if kwargs["v_ego"] < 0.50:  # raise mid-OWNED-stop (ownership begins at v <= 0.85)
      raise RuntimeError("injected mid-stop service defect")
    return orig(self, **kwargs)

  monkeypatch.setattr(StoppingService, "update", flaky)
  lc = LongControl(DummyCarParams())
  frames = scripted_stop_frames()
  wires, ownings, _, _ = run_frames(lc, frames)

  k_own = ownings.index(True)
  k_exc = next(k for k, (v, _, _, _) in enumerate(frames) if v < 0.50)
  assert k_own < k_exc
  assert all(ownings[k_own:k_exc])
  # the exception frame falls back to the legacy chain value computed that frame: finite, braking
  assert math.isfinite(wires[k_exc]) and wires[k_exc] < -0.05
  # Codex review (2026-07-02): the chain on a previously-owned frame ran with the cap family
  # bypassed, so the raw fallback could RELEASE in one frame -- the fault frame must never be
  # shallower than the previous (service-written) wire: output = min(chain, previous wire).
  assert wires[k_exc] <= wires[k_exc - 1] + 1e-9, \
    f"exception fallback released the wire: {wires[k_exc - 1]:.3f} -> {wires[k_exc]:.3f}"
  # ownership latches OFF for the rest of the drive; the service is never consulted again
  # (observation ran on every in-band frame up to and including the fault frame, then stops)
  assert lc._service_live_disabled
  assert not any(ownings[k_exc:])
  assert calls["n"] == k_exc + 1
  # the legacy chain keeps the wire to a finite braking stop (never a silent no-brake)
  assert all(math.isfinite(w) and w < 0.0 for w in wires[k_exc:])
  assert wires[-1] <= -0.10


def test_exception_on_observation_frame_keeps_legacy_wire(monkeypatch) -> None:
  # a defect on a NON-owned observation frame (v above the own band) must never touch the wire:
  # the latch trips before any takeover and the whole drive stays byte-identical to SHADOW/legacy
  frames = scripted_stop_frames()
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "SHADOW")
  wires_shadow, _, _, _ = run_frames(LongControl(DummyCarParams()), frames)

  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE_TERMINAL")
  orig = StoppingService.update

  def flaky(self, **kwargs):
    if kwargs["v_ego"] > 1.2:  # observation band only: well above V_OWN
      raise RuntimeError("injected observation-frame service defect")
    return orig(self, **kwargs)

  monkeypatch.setattr(StoppingService, "update", flaky)
  lc = LongControl(DummyCarParams())
  wires_live, ownings, phases, _ = run_frames(lc, frames)

  assert lc._service_live_disabled
  assert not any(ownings)
  assert all(p == Phase.INACTIVE for p in phases)
  assert wires_live == wires_shadow  # byte-identical legacy wire, frame by frame


# --- cap bypass: owned band follows the service, SHADOW re-pins the legacy glide grab --------------

def test_cap_bypass_owned_band_follows_service_not_legacy_pin(monkeypatch) -> None:
  # a_target = -0.05: the planner correctly glided out (seg 00001b6c shape), so neither the a_plan
  # lane nor the seg24 planner floor is in play -- the terminal wire is purely caps-vs-service.
  frames = close_stopped_lead_frames()
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "SHADOW")
  wires_shadow, _, _, _ = run_frames(LongControl(DummyCarParams()), frames, a_target=-0.05)
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE_TERMINAL")
  wires_live, ownings, phases, svc_cmds = run_frames(LongControl(DummyCarParams()), frames, a_target=-0.05)

  k_roll = max(k for k, (v, _, _, _) in enumerate(frames) if v > 0.01)
  # SHADOW (== legacy): the sub-0.30 cap family pins the wire deep through wheel-stop (the grab)
  assert wires_shadow[k_roll] <= -0.45, f"legacy pin expected, got {wires_shadow[k_roll]:.3f}"
  # LIVE_TERMINAL: the service owns the frame, the caps are bypassed, and the wire carries the
  # gentle EASE-shaped command through wheel-stop instead of the legacy pin
  assert all(ownings[1:]), "service must own the whole sub-0.30 corridor"
  assert phases[k_roll] in (Phase.PRE_STOP_EASE, Phase.RAMP_TO_HOLD)
  assert -0.36 - EPS <= wires_live[k_roll] <= -0.05 + EPS, f"wheel-stop wire {wires_live[k_roll]:.3f}"
  assert wires_live[k_roll] == pytest.approx(svc_cmds[k_roll], abs=1e-12)
  # ... and it still builds the firm -0.32 hold after the stop
  assert wires_live[-1] == pytest.approx(P.A_HOLD_SECURE, abs=0.02)


# --- force-coast standstill: the -0.32 hold still min()s (deepens) the service command -------------

def test_force_coast_standstill_hold_deepens_service_command(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE_TERMINAL")
  frames = [(0.0, 0.0, 4.0, True)] * 120
  wires, ownings, phases, svc_cmds = run_frames(LongControl(DummyCarParams()), frames,
                                                a_target=0.0, force_coast=True)
  assert all(ownings)
  # the service ramps to A_HOLD at J_HOLD (~0.5 s); while its command is still shallower than the
  # force-coast hold, the tail min() must deepen the wire to exactly -0.32 -- and never shallower
  shallow_frames = [k for k in range(len(frames)) if svc_cmds[k] > FORCE_COAST_STANDSTILL_HOLD_ACCEL + 1e-6]
  assert len(shallow_frames) >= 10, "fixture must catch the service mid-ramp"
  for k in shallow_frames:
    assert wires[k] == pytest.approx(FORCE_COAST_STANDSTILL_HOLD_ACCEL, abs=1e-12)
  assert all(w <= FORCE_COAST_STANDSTILL_HOLD_ACCEL + EPS for w in wires)
  # the service itself settles at the same firm hold
  assert phases[-1] == Phase.HOLD
  assert svc_cmds[-1] == pytest.approx(P.A_HOLD_SECURE, abs=0.02)


# --- Santa-Fe-only scope ---------------------------------------------------------------------------

def test_non_santa_fe_never_enters_live_ownership(monkeypatch) -> None:
  frames = scripted_stop_frames()
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "OFF")
  wires_off, _, _, _ = run_frames(LongControl(DummyCarParams(car_fingerprint=HYUNDAI_CAR.HYUNDAI_ELANTRA_2021)), frames)
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE_TERMINAL")
  lc = LongControl(DummyCarParams(car_fingerprint=HYUNDAI_CAR.HYUNDAI_ELANTRA_2021))
  wires_live, ownings, phases, _ = run_frames(lc, frames)
  assert not any(ownings)
  assert all(p == Phase.INACTIVE for p in phases)
  assert wires_live == wires_off  # byte-identical to the legacy wire, frame by frame
