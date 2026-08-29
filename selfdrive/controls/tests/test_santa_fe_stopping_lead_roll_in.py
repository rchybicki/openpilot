"""Focused pins for the santa_fe_stopping_lead_roll_in FLOOR (max()/RAISE), the MIRROR of
get_santa_fe_stopped_lead_smooth_approach_cap.

Real-function policy: every assertion below exercises the GENUINE functions imported from
selfdrive/controls/lib/longitudinal_planner.py -- nothing is reimplemented here.

Local-import note: longitudinal_planner.py pulls in native/Cython modules at import time
(msgq.ipc_pyx, common.params_pyx, common.transformations.transformations, the acados solver,
casadi) plus the IPC layer cereal.messaging. None of those are touched by the roll-in math, but
they must be present for the module body to load, and they require a scons build that is absent in
a plain checkout. The block below installs minimal sys.modules stubs for ONLY those native/IPC
modules, and ONLY when they are not already importable -- so on a built device/CI the real modules
are used unchanged and these stubs are a no-op. The roll-in functions themselves run on the real
numpy + stopping_flags + module constants, so the pins below test real behavior either way.
"""
import importlib
import sys
import types


def _ensure_stub(name, attrs=None, values=None):
  """Install a stand-in for `name` only if it cannot already be imported (built system wins)."""
  if name in sys.modules:
    return sys.modules[name]
  try:
    return importlib.import_module(name)
  except Exception:
    pass
  mod = types.ModuleType(name)
  for attr in attrs or ():
    setattr(mod, attr, type(attr, (object,), {}))
  for key, val in (values or {}).items():
    setattr(mod, key, val)
  sys.modules[name] = mod
  return mod


class _Master:
  def __init__(self, *a, **k):
    pass

  def __getitem__(self, k):
    return None

  def update(self, *a, **k):
    return None

  def send(self, *a, **k):
    return None

  @property
  def updated(self):
    return {}

  @property
  def alive(self):
    return {}


class _Params:
  def __init__(self, *a, **k):
    pass

  def get(self, *a, **k):
    return None

  def get_bool(self, *a, **k):
    return False

  def put(self, *a, **k):
    return None


def _native_ipc_is_built():
  """True only when the real msgq Cython IPC extension imports (i.e. a scons build exists).
  When False we are in a plain checkout and must neutralise the IPC layer entirely, because
  frogpilot_variables instantiates messaging.SubMaster([...]) at IMPORT time (a live connect)."""
  if "msgq.ipc_pyx" in sys.modules and not isinstance(sys.modules["msgq.ipc_pyx"], types.ModuleType):
    return True
  try:
    real = importlib.import_module("msgq.ipc_pyx")
    return hasattr(real, "Context") and not getattr(real, "_roll_in_test_stub", False)
  except Exception:
    return False


def _install_native_stubs():
  built = _native_ipc_is_built()

  # msgq Cython IPC extension (+ its pure-python package wrapper) -- only ipc_pyx is native.
  ipc = _ensure_stub("msgq.ipc_pyx", attrs=[
    "Context", "Poller", "SubSocket", "PubSocket", "SocketEventHandle", "toggle_fake_events",
    "set_fake_prefix", "get_fake_prefix", "delete_fake_prefix", "wait_for_one_event",
    "MultiplePublishersError", "IpcError",
  ])
  ipc._roll_in_test_stub = True
  _ensure_stub("msgq", values={
    "fake_event_handle": lambda *a, **k: None,
    "drain_sock_raw": lambda *a, **k: None,
  })
  # common.params_pyx Cython extension (Params is only instantiated inside methods, never at load).
  for params_name in ("common.params_pyx", "openpilot.common.params_pyx"):
    _ensure_stub(params_name, values={
      "Params": _Params,
      "ParamKeyFlag": type("ParamKeyFlag", (object,), {}),
      "ParamKeyType": type("ParamKeyType", (object,), {}),
      "UnknownKeyName": type("UnknownKeyName", (Exception,), {}),
    })
  # common.transformations.transformations compiled C++ ext (orientation.py uses its *_single
  # callables only at runtime, never at module load).
  tf_syms = {n: (lambda *a, **k: None) for n in [
    "ecef_euler_from_ned_single", "euler2quat_single", "euler2rot_single",
    "ned_euler_from_ecef_single", "quat2euler_single", "quat2rot_single",
    "rot2euler_single", "rot2quat_single", "ecef2geodetic_single", "geodetic2ecef_single",
  ]}
  tf_syms["LocalCoord"] = type("LocalCoord", (object,), {})
  for tf_name in ("common.transformations.transformations", "openpilot.common.transformations.transformations"):
    _ensure_stub(tf_name, values=tf_syms)
  # casadi + acados generated solver (only used to BUILD the MPC; never called by our functions).
  _ensure_stub("casadi", values={"SX": type("SX", (object,), {}), "vertcat": lambda *a, **k: None})
  _ensure_stub("openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code")
  _ensure_stub(
    "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx",
    attrs=["AcadosOcpSolverCython"],
  )
  # cereal.messaging IPC layer (PubMaster/SubMaster are pure I/O, never used by the functions under
  # test). On a build-less checkout we MUST install an inert stand-in even though the real
  # cereal.messaging module would import: frogpilot_variables calls messaging.SubMaster([...]) as a
  # module-level default arg, which performs a live socket connect that the unbuilt IPC layer cannot
  # do. Re-export the REAL cereal schemas (cereal/__init__.py loads them via pycapnp, no build).
  if not built:
    real_cereal = importlib.import_module("cereal")  # real schemas (pycapnp)
    cm = types.ModuleType("cereal.messaging")
    cm.log = real_cereal.log
    cm.car = real_cereal.car
    cm.custom = real_cereal.custom
    cm.SubMaster = _Master
    cm.PubMaster = _Master
    cm.new_message = lambda *a, **k: None
    cm.log_from_bytes = lambda *a, **k: None
    sys.modules["cereal.messaging"] = cm
    real_cereal.messaging = cm


_install_native_stubs()

# Import the GENUINE functions under test from the real module.
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  LEAD_STOP_DISTANCE_TARGET,
  apply_santa_fe_stopping_lead_roll_in,
  get_santa_fe_stopped_lead_hold_gap_required_decel,
  get_santa_fe_stopping_lead_roll_in,
  santa_fe_stopping_lead_roll_in_latch_triggered,
  update_santa_fe_stopping_lead_roll_in_oncoming_frames,
)
# SEG24 safety-crux dependencies: the EXACT functions the planner gate and longcontrol consume.
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import get_stopped_lead_control_target
from openpilot.selfdrive.controls.lib.stop_target_arbiter import should_enter_stop_target_mode, should_hold_stop_target_mode

from types import SimpleNamespace


def make_lead(status=True, d_rel=0.0, v_ego=0.0, lead_v=0.0, a_lead_k=0.0, radar_track_id=-1):
  """radarState.leadOne surrogate. vRel derived from lead_v - v_ego so the functions' vLead/vRel
  resolution matches the real message."""
  return SimpleNamespace(status=status, dRel=d_rel, vRel=lead_v - v_ego, vLead=lead_v, aLeadK=a_lead_k,
                         radarTrackId=radar_track_id)


def test_flags_enabled():
  # The pins below assume the roll-in is live and the cap/floor share the creep-extension geometry.
  assert stopping_flags.SANTA_FE_STOPPING_LEAD_ROLL_IN is True
  assert stopping_flags.SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_EXTENSION is True


def test_imported_functions_are_the_real_source():
  # Guard against accidentally testing a stub: the function must live in the real planner file.
  assert get_santa_fe_stopping_lead_roll_in.__code__.co_filename.endswith("longitudinal_planner.py")


# (1) Measured frame: v_ego=1.30, confirmed lead dRel=9.70 slowing to ~0, MPC output_a_target=-0.50.
# The floor must RAISE the over-brake to a gentle stop-at-hold-gap decel so the car rolls in.
def test_measured_frame_raises_overbrake_to_gentle_roll_in():
  v_ego = 1.30
  lead = make_lead(d_rel=9.70, v_ego=v_ego, lead_v=0.0)

  floor = get_santa_fe_stopping_lead_roll_in(v_ego, lead)
  assert floor is not None
  # rolls in, not a near-stop: gentle decel in roughly [-0.30, -0.05].
  assert -0.30 <= floor <= -0.05

  raised = apply_santa_fe_stopping_lead_roll_in(-0.50, v_ego, lead)
  # The over-brake (-0.50) is raised UP to the gentle floor (a shallower/less-negative command).
  assert raised == floor
  assert raised > -0.50


# (2) Carry-past guard: get_santa_fe_stopping_lead_roll_in returns None when required_decel >
# max_decel (before the clip), so max() can never raise the command shallower than needed to stop at
# the hold gap. This case is constructed so EVERY other gate passes (TTC>=4.0, closing in-band,
# remaining>margin, lead_v<=cap, v in band) and ONLY required_decel > max_decel fires -> None.
def test_carry_past_guard_returns_none_when_required_decel_exceeds_max():
  v_ego = 1.0
  lead = make_lead(d_rel=4.5, v_ego=v_ego, lead_v=0.0)  # closing=1.0, ttc=4.5, remaining=0.5

  # Sanity: this really is a required_decel > max_decel situation in the shared geometry.
  remaining_to_hold_gap = 4.5 - LEAD_STOP_DISTANCE_TARGET  # ISD comp is 0.0 here
  buffer_m = 0.28  # creep-approach buffer at v=1.0 (>= table[0]); required_decel only grows if smaller
  required_decel = get_santa_fe_stopped_lead_hold_gap_required_decel(v_ego, remaining_to_hold_gap, buffer_m)
  assert required_decel > 0.55  # well above the v=1.0 creep-approach max_decel ceiling

  assert get_santa_fe_stopping_lead_roll_in(v_ego, lead) is None
  # apply() therefore leaves a strong MPC brake fully intact (no shallow raise that carries past).
  assert apply_santa_fe_stopping_lead_roll_in(-1.2, v_ego, lead) == -1.2


# (3) The floor returns None outside its band: v<0.30, v>=2.50, lead_v>0.55, TTC<4.0,
# remaining<=margin. Each case is constructed so the named gate is the one that fires.
def test_floor_off_below_v_min():
  assert get_santa_fe_stopping_lead_roll_in(0.20, make_lead(d_rel=9.70, v_ego=0.20, lead_v=0.0)) is None


def test_floor_off_at_or_above_v_max():
  assert get_santa_fe_stopping_lead_roll_in(2.60, make_lead(d_rel=9.70, v_ego=2.60, lead_v=0.0)) is None
  # exactly at the upper bound is excluded (>= V_EGO_MAX)
  assert get_santa_fe_stopping_lead_roll_in(2.50, make_lead(d_rel=9.70, v_ego=2.50, lead_v=0.0)) is None


def test_floor_off_when_lead_moving_above_creep_edge():
  # lead_v=0.80 > 0.55 creep edge: not a confirmed stopped/creeping lead.
  assert get_santa_fe_stopping_lead_roll_in(1.30, make_lead(d_rel=9.70, v_ego=1.30, lead_v=0.80)) is None


def test_floor_off_when_ttc_below_min():
  # v=2.0, dRel=7.0, lead_v=0 -> closing=2.0 (in band), remaining=3.0 (>margin), but ttc=3.5 < 4.0.
  assert get_santa_fe_stopping_lead_roll_in(2.0, make_lead(d_rel=7.0, v_ego=2.0, lead_v=0.0)) is None


def test_floor_off_when_remaining_within_gate_off_margin():
  # dRel=4.30 -> remaining = 4.30 - 4.0 = 0.30 <= 0.40 margin: hand the finish to the cap/glide.
  assert get_santa_fe_stopping_lead_roll_in(1.30, make_lead(d_rel=4.30, v_ego=1.30, lead_v=0.0)) is None


def test_floor_off_without_lead():
  assert get_santa_fe_stopping_lead_roll_in(1.30, make_lead(status=False, d_rel=9.70, v_ego=1.30)) is None


# (4) apply_santa_fe_stopping_lead_roll_in only RAISES (never lowers) output_a_target.
def test_apply_only_raises_never_lowers():
  v_ego = 1.30
  lead = make_lead(d_rel=9.70, v_ego=v_ego, lead_v=0.0)
  floor = get_santa_fe_stopping_lead_roll_in(v_ego, lead)
  assert floor is not None

  # When the floor is ACTIVE it is a max-raise: a command deeper than the floor is RAISED up to the
  # floor (that is the whole point -- "do not brake harder than the gentle hold-gap decel"), and the
  # raise is always toward zero, never further negative.
  raised = apply_santa_fe_stopping_lead_roll_in(-1.5, v_ego, lead)
  assert raised == floor
  assert raised > -1.5  # raised, never lowered
  # A command already shallower than the floor is left untouched (apply is a pure max-raise).
  assert apply_santa_fe_stopping_lead_roll_in(-0.02, v_ego, lead) == -0.02
  # At/above floor passes through; an over-brake is raised exactly to the floor.
  assert apply_santa_fe_stopping_lead_roll_in(floor, v_ego, lead) == floor
  assert apply_santa_fe_stopping_lead_roll_in(-0.50, v_ego, lead) == floor

  # The core only-raise invariant across a sweep: the result is NEVER below the input.
  for output in (-2.0, -1.0, -0.50, -0.20, -0.157, -0.10, -0.01, 0.05):
    assert apply_santa_fe_stopping_lead_roll_in(output, v_ego, lead) >= output

  # And where the floor is OFF (carry-past guard fires), apply is the identity -- it cannot lower a
  # strong MPC brake, so the only-raise property holds even when no roll-in target exists.
  off_lead = make_lead(d_rel=4.5, v_ego=1.0, lead_v=0.0)
  assert get_santa_fe_stopping_lead_roll_in(1.0, off_lead) is None
  assert apply_santa_fe_stopping_lead_roll_in(-1.5, 1.0, off_lead) == -1.5


# Latch (FIX-C): a lead hard-stop trips the latch trigger (handing brake to the MPC), a mild
# lead-decel does not.
def test_latch_triggers_on_lead_hard_decel_only():
  v_ego = 1.30
  assert santa_fe_stopping_lead_roll_in_latch_triggered(v_ego, make_lead(d_rel=9.70, v_ego=v_ego, a_lead_k=-0.60)) is True
  assert santa_fe_stopping_lead_roll_in_latch_triggered(v_ego, make_lead(d_rel=9.70, v_ego=v_ego, a_lead_k=-0.10)) is False
  # outside the speed band the latch never arms
  assert santa_fe_stopping_lead_roll_in_latch_triggered(0.20, make_lead(d_rel=9.70, v_ego=0.20, a_lead_k=-0.60)) is False


# Shared geometry: cap and floor consume the SAME get_santa_fe_stopped_lead_hold_gap_required_decel,
# so at the measured frame the floor magnitude equals that kinematic required_decel exactly.
def test_floor_equals_shared_required_decel_at_measured_frame():
  v_ego = 1.30
  lead = make_lead(d_rel=9.70, v_ego=v_ego, lead_v=0.0)
  floor = get_santa_fe_stopping_lead_roll_in(v_ego, lead)
  assert floor is not None

  remaining_to_hold_gap = 9.70 - LEAD_STOP_DISTANCE_TARGET  # ISD comp = 0.0
  buffer_m = 0.3085714285714286  # creep-approach buffer interp at v=1.30 (matches the floor's call)
  required_decel = get_santa_fe_stopped_lead_hold_gap_required_decel(v_ego, remaining_to_hold_gap, buffer_m)
  assert abs((-floor) - required_decel) < 1e-9


# ---------------------------------------------------------------------------------------------------
# SEG24 safety crux. The floor must be OFF in every frame longcontrol's anti-collision net is live.
# longcontrol enters/persists stopping (longcontrol.py:485-487) when, on the PRE-floor a_target:
#   should_stop OR should_enter_stop_target_mode(...) OR (in-stopping) should_hold_stop_target_mode(...)
# and the arbiter additionally drives the stopped-lead stop via get_stopped_lead_control_target(...).
# The planner gate (longitudinal_planner.py:1059-1064) mirrors these EXACT predicates plus the
# synthetic stopped-lead term, so the floor cannot raise a_target while the net is live.
# ---------------------------------------------------------------------------------------------------


def _floor_active(v_ego, d_rel, lead_v=0.0):
  return get_santa_fe_stopping_lead_roll_in(v_ego, make_lead(d_rel=d_rel, v_ego=v_ego, lead_v=lead_v)) is not None


def test_floor_and_synthetic_stopped_lead_bands_are_disjoint():
  # The dangerous window is the synthetic stopped-lead closure (arbiter min-merges it AND drives the
  # stopped-lead stop, so longcontrol enters stopping on a target the raw distance does not see). Sweep
  # the whole roll-in speed/gap space and assert no frame has BOTH the floor active and the synthetic
  # target active -- the floor owns only the far approach beyond the arbiter's close-closure band.
  import numpy as np
  overlaps = []
  for v_ego in np.arange(0.30, 2.50, 0.05):
    for d_rel in np.arange(1.5, 14.0, 0.05):
      floor_on = _floor_active(float(v_ego), float(d_rel))
      synthetic_on = get_stopped_lead_control_target(float(v_ego), 0.0, float(d_rel)) is not None
      if floor_on and synthetic_on:
        overlaps.append((round(float(v_ego), 2), round(float(d_rel), 2)))
  assert overlaps == [], f"floor active while synthetic stopped-lead stop active (seg24 net live): {overlaps[:8]}"


def test_synthetic_target_active_implies_gate_term_true():
  # Where the synthetic stopped-lead target IS active (a real creeping-then-stopping close lead), the
  # planner gate's synthetic_stopped_lead_stop_active term is True -> floor gated OFF. Pin a concrete
  # close frame inside the speed band.
  v_ego, lead_v, d_rel = 1.30, 0.0, 3.0
  assert get_stopped_lead_control_target(v_ego, lead_v, d_rel) is not None  # gate term -> True
  assert not _floor_active(v_ego, d_rel)  # and the floor is independently OFF here


def test_above_synthetic_upper_v_bound_should_enter_or_hold_covers():
  # The synthetic gate's upper v-bound is 1.90 m/s. Just above it the synthetic term stops covering,
  # so should_enter / should_hold (which the gate also mirrors) must carry the seg24 coverage. With a
  # planner stop-target distance present and a braking a_target, both predicates fire above 1.90.
  for v_ego in (1.95, 2.10, 2.40):
    assert get_stopped_lead_control_target(v_ego, 0.0, 3.0) is None   # synthetic no longer covers
    assert bool(should_enter_stop_target_mode(v_ego, -0.5, 1.0))      # ...but should_enter does
    assert bool(should_hold_stop_target_mode(v_ego, -0.5, 1.0))       # ...and should_hold does


def test_gate_predicates_are_the_real_arbiter_functions():
  # Pin that the predicates the planner gate calls are the genuine arbiter/helpers (not stubs).
  assert should_enter_stop_target_mode.__code__.co_filename.endswith("stop_target_arbiter.py")
  assert should_hold_stop_target_mode.__code__.co_filename.endswith("stop_target_arbiter.py")
  assert get_stopped_lead_control_target.__code__.co_filename.endswith("stop_target_helpers.py")


def test_floor_off_for_sustained_oncoming_lead():
  # 00002041 seg3: a 33.5 m crossing-car phantom (vLead -5.5) at v_ego 1.87 passed every gate
  # through the max(vLead, 0.0) clamp (fake closing 1.87 vs real 7.4, fake ttc 17.9 s vs 4.5 s)
  # and released a -0.9 model stop to -0.05 for 1.5 s. Sustained negative raw vLead (3 frames,
  # 0.15 s) rejects the floor BEFORE the clamp -- at every magnitude, since the corpus shows
  # one-frame track-jump spikes to -0.75..-13.7 inside genuinely-stopped-lead windows (~1/1000
  # frames), so an instant tier at any depth would punch the deep MPC command through.
  assert get_santa_fe_stopping_lead_roll_in(1.87, make_lead(d_rel=33.5, v_ego=1.87, lead_v=-5.5), oncoming_frames=3) is None
  assert get_santa_fe_stopping_lead_roll_in(1.30, make_lead(d_rel=9.70, v_ego=1.30, lead_v=-0.30), oncoming_frames=3) is None


def test_active_floor_survives_one_frame_track_jump_spike():
  # Review round-2 sequence: an ACTIVE floor mid-approach, then one-frame vLead spikes (borderline
  # -0.30 and deep track-jump -0.9/-13.7) -- the floor must hold on every spike frame (no one-frame
  # deep-command punch-through); it drops only once the negative reading SUSTAINS 3 frames.
  v_ego = 1.30
  seq = [-0.10, -0.05, -0.30, -0.08, -0.90, -0.02, -13.7, -0.04, -5.5, -5.5, -5.5]
  n, tid = 0, None
  drops = []
  for v_lead in seq:
    lead = make_lead(d_rel=9.70, v_ego=v_ego, lead_v=v_lead, radar_track_id=7)
    n, tid = update_santa_fe_stopping_lead_roll_in_oncoming_frames(n, tid, lead)
    drops.append(get_santa_fe_stopping_lead_roll_in(v_ego, lead, oncoming_frames=n) is None)
  assert drops == [False, False, False, False, False, False, False, False, False, False, True]


def test_oncoming_persistence_is_track_local():
  # R3: two negative frames on track A, then leadOne jumps to a freshly-selected genuinely stopped
  # track B whose FIRST frame carries a one-frame Doppler glitch -- the counts must not sum to a
  # rejection; the floor stays active on the glitch frame.
  v_ego = 1.30
  n, tid = 0, None
  for _ in range(2):
    n, tid = update_santa_fe_stopping_lead_roll_in_oncoming_frames(n, tid, make_lead(d_rel=9.7, v_ego=v_ego, lead_v=-0.90, radar_track_id=7))
  assert n == 2
  glitch_b = make_lead(d_rel=9.7, v_ego=v_ego, lead_v=-0.90, radar_track_id=9)
  n, tid = update_santa_fe_stopping_lead_roll_in_oncoming_frames(n, tid, glitch_b)
  assert n == 1 and tid == 9
  assert get_santa_fe_stopping_lead_roll_in(v_ego, glitch_b, oncoming_frames=n) is not None


def test_floor_tolerates_stopped_lead_measurement_noise():
  # a genuinely stopped lead reads small negative vLead from radar noise; the floor stays available
  v_ego = 1.30
  floor = get_santa_fe_stopping_lead_roll_in(v_ego, make_lead(d_rel=9.70, v_ego=v_ego, lead_v=-0.20))
  assert floor is not None and floor < 0.0


def test_oncoming_frames_counter_resets_on_recovery_and_lead_loss():
  # alternating around the threshold never accumulates persistence; recovery or lead loss resets
  n, tid = 0, None
  n, tid = update_santa_fe_stopping_lead_roll_in_oncoming_frames(n, tid, make_lead(d_rel=9.7, lead_v=-0.30, radar_track_id=4))
  assert n == 1
  n, tid = update_santa_fe_stopping_lead_roll_in_oncoming_frames(n, tid, make_lead(d_rel=9.7, lead_v=-0.30, radar_track_id=4))
  assert n == 2
  n, tid = update_santa_fe_stopping_lead_roll_in_oncoming_frames(n, tid, make_lead(d_rel=9.7, lead_v=-0.20, radar_track_id=4))
  assert n == 0
  n, tid = update_santa_fe_stopping_lead_roll_in_oncoming_frames(2, 4, make_lead(status=False))
  assert n == 0 and tid is None


def test_apply_passes_persistence_through():
  # end-to-end through apply: the sustained phantom raises nothing (deep command passes through)
  v_ego = 1.30
  lead = make_lead(d_rel=9.70, v_ego=v_ego, lead_v=-0.30)
  assert apply_santa_fe_stopping_lead_roll_in(-1.50, v_ego, lead, oncoming_frames=3) == -1.50
  raised = apply_santa_fe_stopping_lead_roll_in(-1.50, v_ego, lead, oncoming_frames=1)
  assert raised > -1.50
