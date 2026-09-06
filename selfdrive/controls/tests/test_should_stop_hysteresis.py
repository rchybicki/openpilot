"""Build-free tests for the planner shouldStop falling-edge hold (stopping redesign §4.1).

The full planner suite needs a scons build (msgq); this file is the locally-executable
acceptance gate for the extracted pure hold function (red-team F17).
"""

import random

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.drive_helpers import update_should_stop_falling_edge_hold

DT = 0.05  # planner runs at 20 Hz
V_EGO_STOPPING = 0.3
HOLD_S = 0.4


def run_sequence(frames, hold_s=HOLD_S, dt=DT, v_ego_stopping=V_EGO_STOPPING):
  """frames: iterable of (raw_should_stop, v_now, a_target). Returns list of held outputs."""
  timer = 0.0
  outputs = []
  for raw, v_now, a_target in frames:
    held, timer = update_should_stop_falling_edge_hold(raw, v_now, a_target, v_ego_stopping, timer, hold_s, dt)
    outputs.append(held)
  return outputs


def test_normative_flag_defaults() -> None:
  # FINAL_SPEC §3: flag defaults are normative
  assert stopping_flags.SHOULD_STOP_FALLING_EDGE_HOLD_S == 0.4
  assert stopping_flags.SHOULD_STOP_LOOKAHEAD_S == 0.0
  # Flipped 2026-06-10 (rollout plan stage 0): device ISD read 0.0, flip bit-identical.
  assert stopping_flags.PUBLISH_TRUE_LEAD_DISTANCE is True


def test_falling_edge_held_for_exactly_eight_frames_at_20hz() -> None:
  frames = [(True, 0.1, -0.3)] * 3 + [(False, 0.1, -0.3)] * 12
  outputs = run_sequence(frames)

  assert all(outputs[:3])
  # 0.4 s at 20 Hz == 8 held frames after the falling edge, then release
  assert outputs[3:11] == [True] * 8
  assert outputs[11:] == [False] * 4


def test_hold_releases_immediately_on_go_signal_a_target() -> None:
  frames = [(True, 0.1, -0.3), (False, 0.1, 0.25)]
  assert run_sequence(frames) == [True, False]
  # boundary: a_target == 0.2 still holds
  frames = [(True, 0.1, -0.3), (False, 0.1, 0.2)]
  assert run_sequence(frames) == [True, True]


def test_hold_releases_immediately_when_plan_speed_rises() -> None:
  frames = [(True, 0.1, -0.3), (False, V_EGO_STOPPING + 0.16, -0.3)]
  assert run_sequence(frames) == [True, False]


def test_hold_does_not_engage_when_raw_falls_at_speed() -> None:
  # raw falls while v is above the stopping band: no hold at all
  frames = [(True, 1.5, -0.3), (False, 1.5, -0.3)]
  assert run_sequence(frames) == [True, False]


def test_release_clears_the_timer_for_the_rest_of_the_window() -> None:
  # once released by a go signal, a later low-v low-a frame must not resurrect the hold
  frames = [(True, 0.1, -0.3), (False, 0.1, 0.25), (False, 0.1, -0.3)]
  assert run_sequence(frames) == [True, False, False]


def test_raw_true_always_passes_through_and_rearms() -> None:
  # additive on the deassert side: raw True passes through at any v / a_target
  frames = [(True, 5.0, 1.0), (True, 0.1, -0.3), (False, 0.1, -0.3), (True, 0.1, -0.3), (False, 0.1, -0.3)]
  outputs = run_sequence(frames)
  assert outputs == [True, True, True, True, True]


def test_hold_cannot_create_stops() -> None:
  # from a zero timer, no sequence of raw=False frames ever yields True
  rng = random.Random(42)
  frames = [(False, rng.uniform(0.0, 2.0), rng.uniform(-1.0, 1.0)) for _ in range(200)]
  assert not any(run_sequence(frames))


def test_kill_switch_zero_restores_raw_flag() -> None:
  rng = random.Random(1234)
  frames = [(rng.random() < 0.5, rng.uniform(0.0, 2.0), rng.uniform(-1.0, 1.0)) for _ in range(200)]
  outputs = run_sequence(frames, hold_s=0.0)
  assert outputs == [raw for raw, _, _ in frames]


def test_kill_switch_zero_keeps_timer_cleared() -> None:
  _, timer = update_should_stop_falling_edge_hold(True, 0.1, -0.3, V_EGO_STOPPING, 0.4, 0.0, DT)
  assert timer == 0.0


def test_hold_is_additive_only_on_deassert_side() -> None:
  # output may differ from raw only on frames where raw is False (never True -> False)
  rng = random.Random(7)
  timer = 0.0
  for _ in range(500):
    raw = rng.random() < 0.5
    v_now = rng.uniform(0.0, 1.0)
    a_target = rng.uniform(-1.0, 1.0)
    held, timer = update_should_stop_falling_edge_hold(raw, v_now, a_target, V_EGO_STOPPING, timer, HOLD_S, DT)
    if raw:
      assert held


def test_hold_duration_does_not_stretch_with_float_drift() -> None:
  # accumulated 0.05 subtractions leave ~1e-17 residue; the epsilon must swallow it
  frames = [(True, 0.1, -0.3)] + [(False, 0.1, -0.3)] * 10
  outputs = run_sequence(frames)
  assert sum(outputs[1:]) == 8
