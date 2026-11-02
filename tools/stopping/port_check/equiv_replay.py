#!/usr/bin/env python3
"""Port equivalence replay for one source tree: radard, LongitudinalPlanner and LongControl on the same recorded inputs.

The rlogs are decoded with the schema that recorded them (the subtract tip's cereal, loaded by path into a private parser), then
every message a stage reads as capnp is copied field by field, by name, into the process tree's own schema (fields the tree does
not have are recorded in 'dropped'). Stages, each on recorded inputs only, so a difference is attributable to that stage:
  radard   per logged radarState: the tree's RadarD on the logged modelV2 / carState / liveTracks / frogpilotPlan it read.
  planner  per logged longitudinalPlan: the tree's planner on the latest inputs <= modelMonoTime + 3 ms (modelV2 <= modelMonoTime).
  loc      per logged carControl (variant C inputs: latest <= the matching selfdriveState): the tree's LongControl called as
           controlsd calls it, on the logged plan ('loc') and on this tree's own replayed plan of the same tick ('chain').
  With EQUIV_STORE_JSON also 'fp' (FrogPilotPlanner), 'controls' (controlsd) and 'card' (Car.state_update: set speed, buttons, always
  on lateral).
EQUIV_WITH_GAS=1 sets LONGITUDINAL_ACTIVE_WITH_GAS (route 00002310 logged alternativeExperience 0).
Toggles: the tree's toggles_default.json + the logged frogpilotToggles; with EQUIV_TOGGLES_JSON, that derive_toggles.py output instead.
Run with cwd = the tree and PYTHONPATH=<tree>:<tree>/frogpilot/third_party; argv: out.pkl tip_tree segment_dir...
"""
import bisect
import contextlib
import functools
import json
import os
import pickle
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

PSERV = ('carControl', 'carState', 'controlsState', 'liveParameters', 'radarState', 'modelV2', 'selfdriveState', 'frogpilotCarState',
         'frogpilotPlan')
SERV = PSERV + ('longitudinalPlan', 'liveTracks', 'frogpilotRadarState')
LEAD_F = ('status', 'radar', 'radarTrackId', 'fcw', 'dRel', 'yRel', 'vRel', 'vLead', 'vLeadK', 'aLeadK', 'aLeadTau', 'modelProb')
PLAN_F = ('at', 'ss', 'dts', 'dtsm', 'fcw', 'traj', 'trajv')
LOC_F = ('accel', 'lcs', 'phase', 'owning', 'svc_active')
BOUND_NS = 3_000_000


def tip_schema(tip):
  import capnp
  parser = capnp.SchemaParser()
  cereal = Path(tip) / 'cereal'
  return parser.load(str(cereal / 'log.capnp'))


def fill(builder, d, path, dropped):
  """Set builder's fields from to_dict() output d by field name (recursively); record what the target schema cannot take."""
  fields = builder.schema.fields
  for k, v in d.items():
    if k not in fields:
      dropped.add(path + k)
      continue
    try:
      if isinstance(v, dict):
        sub = getattr(builder, k) if fields[k].proto.which() == 'group' else builder.init(k)
        fill(sub, v, f'{path}{k}.', dropped)
      elif isinstance(v, list) and v and isinstance(v[0], dict):
        lst = builder.init(k, len(v))
        for i, x in enumerate(v):
          fill(lst[i], x, f'{path}{k}[].', dropped)
      else:
        setattr(builder, k, v)
    except Exception as e:  # an enum name or type the target schema does not have: recorded, not fatal
      dropped.add(f'{path}{k} ({type(e).__name__})')


class Converter:
  """Tip-schema events -> owned readers of the tree's schema, memoized per (service, logMonoTime)."""
  def __init__(self):
    from cereal import log
    self.log = log
    self.memo = {}
    self.dropped = {}

  def __call__(self, ev):
    if ev is None:
      return None
    w = ev.which()
    key = (w, ev.logMonoTime)
    if key not in self.memo:
      new = self.log.Event.new_message()
      new.logMonoTime, new.valid = ev.logMonoTime, ev.valid
      dropped = self.dropped.setdefault(w, set())
      if self.log.Event.schema.fields[w].proto.slot.type.which() == 'list':   # onroadEvents
        items = getattr(ev, w)
        lst = new.init(w, len(items))
        for i, item in enumerate(items):
          fill(lst[i], item.to_dict(), '[].', dropped)
      else:
        fill(new.init(w), getattr(ev, w).to_dict(), '', dropped)
      self.memo[key] = new.as_reader()
    return self.memo[key]


def load(tip_log, seg_dirs):
  import zstandard
  evs = []
  for d in seg_dirs:
    raw = zstandard.ZstdDecompressor().decompress((Path(d) / 'rlog.zst').read_bytes(), max_output_size=int(1.5e9))
    evs += list(tip_log.Event.read_multiple_bytes(raw))
  init = next(e for e in evs if e.which() == 'initData')
  cpm = next(e.carParams for e in evs if e.which() == 'carParams')
  by = {s: ([], []) for s in SERV}
  for e in evs:
    w = e.which()
    by.setdefault(w, ([], []))
    by[w][0].append(e.logMonoTime)
    by[w][1].append(e)
  for s in by:
    o = np.argsort(by[s][0], kind='stable')
    by[s] = ([by[s][0][i] for i in o], [by[s][1][i] for i in o])
  return init, cpm, by


def latest(by, s, tns):
  i = bisect.bisect_right(by[s][0], tns) - 1
  return by[s][1][i] if i >= 0 else None


def exact(by, s, tns):
  i = bisect.bisect_left(by[s][0], tns)
  return by[s][1][i] if i < len(by[s][0]) and by[s][0][i] == tns else None


def tree_toggles(tree, by):
  if os.environ.get('EQUIV_TOGGLES_JSON'):   # the tree's own FrogPilotVariables toggles on the device store (derive_toggles.py)
    return SimpleNamespace(**json.loads(Path(os.environ['EQUIV_TOGGLES_JSON']).read_text())['toggles'])
  fp = next((e.frogpilotPlan.frogpilotToggles for e in by['frogpilotPlan'][1] if e.frogpilotPlan.frogpilotToggles), None)
  tog = json.loads((Path(tree) / 'tools/stopping/sim/toggles_default.json').read_text())
  tog.update(json.loads(fp) if fp else {})
  return SimpleNamespace(**tog)


def tree_cp(cpm, alternative_experience=0):
  from cereal import car
  cp = car.CarParams.new_message()
  dropped = set()
  fill(cp, cpm.to_dict(), '', dropped)
  cp.alternativeExperience |= alternative_experience
  return cp.as_reader(), dropped


class SM:
  def __init__(self, msgs, prev):
    self.msgs = msgs
    self.logMonoTime = {s: m.logMonoTime for s, m in msgs.items()}
    self.updated = {s: prev is None or prev.get(s) != m.logMonoTime for s, m in msgs.items()}
    self.valid = {s: m.valid for s, m in msgs.items()}
    self.alive = dict.fromkeys(msgs, True)

  def __getitem__(self, s):
    return getattr(self.msgs[s], s)

  def all_checks(self, service_list=None):
    return all(self.valid[s] for s in (service_list or self.msgs))


class PM:
  def send(self, s, msg):
    self.last = msg


def radard_pass(by, conv, toggles, fingerprint):
  from cereal import messaging
  from openpilot.selfdrive.controls import radard
  from openpilot.tools.stopping.sim import radar_replay
  radard.get_frogpilot_toggles = lambda *a, **k: toggles
  delay = radar_replay.radar_delay(fingerprint)
  rd = radard.RadarD(delay, **radar_replay.radard_kw(radard, fingerprint))
  empty_lt = messaging.new_message('liveTracks').as_reader()
  seen = dict.fromkeys(('modelV2', 'carState', 'liveTracks', 'frogpilotPlan'), False)
  rows, skipped = [], 0
  for R in by['radarState'][1]:
    rs = R.radarState
    md, cs = exact(by, 'modelV2', rs.mdMonoTime), exact(by, 'carState', rs.carStateMonoTime)
    fp = latest(by, 'frogpilotPlan', R.logMonoTime)
    if md is None or cs is None or fp is None:
      skipped += 1
      continue
    j, _, _ = radar_replay.live_tracks_index(by, R, cs.logMonoTime)
    lt = conv(by['liveTracks'][1][j]) if j >= 0 else empty_lt
    seen.update(modelV2=True, carState=True, liveTracks=seen['liveTracks'] or j >= 0, frogpilotPlan=True)
    msgs = dict(modelV2=conv(md), carState=conv(cs), liveTracks=lt, frogpilotPlan=conv(fp))
    sm = radar_replay._RSM(msgs, dict(seen), dict(carState=cs.logMonoTime), R.valid)
    rd.update(sm, sm['liveTracks'])
    out = rd.radar_state
    rows.append([R.logMonoTime] + [float(getattr(lead, f)) for lead in (out.leadOne, out.leadTwo) for f in LEAD_F]
                + [float(getattr(lead, f)) for lead in (rs.leadOne, rs.leadTwo) for f in LEAD_F])
  cols = ['ns'] + [f'{w}_{f}' for w in ('l1', 'l2') for f in LEAD_F] + [f'log_{w}_{f}' for w in ('l1', 'l2') for f in LEAD_F]
  arr = np.array(rows, dtype=np.float64).reshape(-1, len(cols))
  R = {c: arr[:, i] for i, c in enumerate(cols)}
  R['ns'] = arr[:, 0].astype(np.int64)
  R['skipped'], R['delay'] = skipped, delay
  return R


def planner_pass(by, conv, toggles, cp):
  import openpilot.selfdrive.controls.lib.longitudinal_planner as LP
  planner = LP.LongitudinalPlanner(cp)
  rows, prev, skipped = [], None, 0
  for lp in by['longitudinalPlan'][1]:
    md = lp.longitudinalPlan.modelMonoTime
    raw = {s: latest(by, s, md + BOUND_NS) for s in PSERV}
    raw['modelV2'] = latest(by, 'modelV2', md)
    if any(x is None for x in raw.values()):
      skipped += 1
      continue
    msgs = {s: conv(e) for s, e in raw.items()}
    sm = SM(msgs, prev)
    prev = {s: m.logMonoTime for s, m in msgs.items()}
    planner.update(sm, toggles)
    pm = PM()
    planner.publish(sm, pm, toggles)
    o, rec = pm.last.longitudinalPlan, lp.longitudinalPlan
    rows.append((lp.logMonoTime, rec.aTarget, rec.shouldStop, o.aTarget, o.shouldStop, o.distanceToStopTarget, o.distanceToStopTargetModel,
                 o.fcw, o.aTargetTrajectory, o.aTargetTrajectoryValid))
  cols = ('ns', 'log_at', 'log_ss') + PLAN_F
  arr = np.array(rows, dtype=np.float64).reshape(-1, len(cols))
  P = {c: arr[:, i] for i, c in enumerate(cols)}
  P['ns'] = arr[:, 0].astype(np.int64)
  P['skipped'] = skipped
  return P


def loc_frames(by, P):
  """Per logged carControl: the controlsd inputs of its tick (latest <= the selfdriveState it read) and the plan tick it read."""
  frames, sds = [], by['selfdriveState'][0]
  for e in by['carControl'][1]:
    i = bisect.bisect_right(sds, e.logMonoTime) - 1
    if i < 0:
      continue
    s_ns = sds[i]
    snap = {s: latest(by, s, s_ns) for s in ('carState', 'radarState', 'longitudinalPlan', 'modelV2', 'frogpilotCarState', 'frogpilotPlan',
                                             'selfdriveState')}
    if any(x is None for x in snap.values()):
      continue
    k = int(np.searchsorted(P['ns'], snap['longitudinalPlan'].logMonoTime, side='right')) - 1
    if k < 0 or P['ns'][k] != snap['longitudinalPlan'].logMonoTime:
      k = -1
    frames.append((e, snap, k))
  return frames


def loc_pass(frames, conv, toggles, cp, P=None):
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.safety import ALTERNATIVE_EXPERIENCE
  from openpilot.common.constants import CV
  from openpilot.selfdrive.controls.lib.drive_helpers import longitudinal_accel_with_gas
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
  lc = LongControl(cp)
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  svc, box = lc._service_shadow_svc, {}
  inner = svc.update

  def cap(**kw):
    box['r'] = inner(**kw)
    return box['r']
  svc.update = cap
  with_gas = bool(cp.alternativeExperience & ALTERNATIVE_EXPERIENCE.LONGITUDINAL_ACTIVE_WITH_GAS) or os.environ.get('EQUIV_WITH_GAS') == '1'
  rows = []
  for e, snap, k in frames:
    if P is not None and k < 0:
      continue
    cc = e.carControl
    CS = conv(snap['carState']).carState
    rs = snap['radarState'].radarState
    lp = snap['longitudinalPlan'].longitudinalPlan
    t = e.logMonoTime * 1e-9
    if P is None:
      at, ss, dts, dtsm, fcw = lp.aTarget, lp.shouldStop, lp.distanceToStopTarget, lp.distanceToStopTargetModel, lp.fcw
      traj = lp.aTargetTrajectory if lp.aTargetTrajectoryValid else None
    else:
      at, ss, dts, dtsm, fcw = P['at'][k], bool(P['ss'][k]), P['dts'][k], P['dtsm'][k], bool(P['fcw'][k])
      traj = P['traj'][k] if P['trajv'][k] else None
    if not cc.longActive:
      lc.reset()
    box.clear()
    limits = CarInterface.get_pid_accel_limits(cp, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    a = float(min(lc.update(cc.longActive, CS, float(at), bool(ss), float(dts), limits, toggles,
                            experimental_mode=snap['selfdriveState'].selfdriveState.experimentalMode,
                            lead_status=rs.leadOne.status, lead_v=rs.leadOne.vLead, lead_d_rel=rs.leadOne.dRel, lead_a=rs.leadOne.aLeadK,
                            lead_track_id=rs.leadOne.radarTrackId, lead_model_prob=rs.leadOne.modelProb,
                            lead2_status=rs.leadTwo.status, lead2_v=rs.leadTwo.vLead, lead2_d_rel=rs.leadTwo.dRel,
                            fcw=bool(fcw), model_stop_d=float(dtsm), model_should_stop=snap['modelV2'].modelV2.action.shouldStop,
                            force_coast=snap['frogpilotCarState'].frogpilotCarState.forceCoast,
                            increased_stopped_distance=snap['frogpilotPlan'].frogpilotPlan.increasedStoppedDistance,
                            a_target_trajectory=(None if traj is None else float(traj)),
                            freeze_integrator=with_gas and CS.gasPressed, plan_valid=snap['longitudinalPlan'].valid, request_time=t),
                  toggles.max_desired_acceleration))
    a = longitudinal_accel_with_gas(a, with_gas, CS.gasPressed)
    authorized = bool(e.valid and cc.enabled and cc.longActive and not cc.cruiseControl.override and not CS.gasPressed
                      and not CS.brakePressed and CS.canValid and not CS.canTimeout)
    lc.observe_accel_request(a, t, authorized=authorized)
    r = box.get('r')
    rows.append((e.logMonoTime, a, int(lc.long_control_state), int(svc.phase), int(bool(getattr(lc, '_service_live_owning', False))),
                 int(bool(r.active)) if r is not None else -1, float(cc.actuators.accel), CS.vEgo, int(cc.longActive), int(CS.gasPressed),
                 int(CS.brakePressed), float(at), int(bool(ss))))
  cols = ('ns',) + LOC_F + ('rec_accel', 'v', 'act', 'gas', 'brake', 'in_at', 'in_ss')
  arr = np.array(rows, dtype=np.float64).reshape(-1, len(cols))
  L = {c: arr[:, i] for i, c in enumerate(cols)}
  L['ns'] = arr[:, 0].astype(np.int64)
  return L


FP_OUT = ('tFollow', 'minAcceleration', 'maxAcceleration', 'accelerationJerk', 'dangerJerk', 'speedJerk', 'dangerFactor', 'increasedStoppedDistance',
          'vCruise', 'experimentalMode', 'laneWidthLeft', 'laneWidthRight', 'redLight', 'forcingStop', 'desiredFollowDistance',
          'cscControllingSpeed', 'cscSpeed', 'slcSpeedLimit', 'slcSpeedLimitOffset', 'slcMapSpeedLimit', 'slcNextSpeedLimit',
          'slcOverriddenSpeed', 'unconfirmedSlcSpeedLimit', 'speedLimitChanged', 'roadCurvature', 'lateralCheck')
CTRL_OUT = ('accel', 'torque', 'curvature', 'steeringAngleDeg', 'longControlState', 'latActive', 'longActive', 'desiredCurvature',
            'forceDecel', 'lac_output', 'lac_error', 'lac_f', 'lac_active', 'lac_saturated')
CARD_CS = ('vCruise', 'vCruiseCluster')
CARD_FPCS = ('alwaysOnLateralEnabled', 'distanceLongPressed', 'distanceVeryLongPressed', 'forceCoast', 'pauseLateral', 'pauseLongitudinal',
             'trafficModeEnabled')
CARD_OUT = CARD_CS + CARD_FPCS + ('experimentalModePresses', 'resumePrevButton', 'handoffButtonsHeld')


class _View(dict):
  def __init__(self, fn):
    super().__init__()
    self.fn = fn

  def __missing__(self, key):
    value = self[key] = self.fn(key)
    return value


class _Stub:
  """A collaborator the replayed process gets but the stage never exercises (theme manager, API, error log)."""
  def __getattr__(self, name):
    return _Stub()

  def __call__(self, *args, **kwargs):
    return None

  def __bool__(self):
    return False


@functools.cache
def default_message(s):
  """A valid empty message of a service the log does not have (logMonoTime 0, so it is never 'updated')."""
  from cereal import log
  ev = log.Event.new_message()
  ev.valid = True
  if log.Event.schema.fields[s].proto.slot.type.which() == 'list':
    ev.init(s, 0)
  else:
    ev.init(s)
  return ev.as_reader()


class LazySM:
  """A process's SubMaster at one tick: per service the latest logged message at or before tick_ns, copied into the tree's schema
  (a default message where the log has none of that service); updated = a new logMonoTime since the previous tick (prev)."""
  def __init__(self, by, conv, tick_ns, prev, override=None):
    self.by, self.conv, self.tick_ns, self.override = by, conv, tick_ns, override or {}
    self.msgs = _View(self._get)
    self.logMonoTime = _View(lambda s: self.msgs[s].logMonoTime)
    self.updated = _View(lambda s: self.msgs[s].logMonoTime not in (0, prev.get(s)))   # a default message is never updated
    self.valid = _View(lambda s: self.msgs[s].valid)
    self.alive = _View(lambda s: True)
    self.freq_ok = _View(lambda s: True)
    self.recv_time = _View(lambda s: self.msgs[s].logMonoTime / 1e9)
    self.recv_frame = _View(lambda s: 0)
    self.alive_timeout = _View(lambda s: 1e9)
    self.frame = 0

  def _get(self, s):
    if s in self.override:
      return self.override[s]
    e = latest(self.by, s, self.tick_ns) if s in self.by else None
    return self.conv(e) if e is not None else default_message(s)

  def __getitem__(self, s):
    return getattr(self.msgs[s], s)

  def update(self, timeout=None):
    pass

  def extend(self, services):
    return self

  def all_checks(self, service_list=None):
    return all(self.valid[s] for s in (service_list or list(self.msgs)))

  def all_valid(self, service_list=None):
    return self.all_checks(service_list)

  def all_alive(self, service_list=None):
    return True

  def seen(self):
    return {s: m.logMonoTime for s, m in self.msgs.items()}


def fp_planner_pass(by, conv, toggles, init):
  """The tree's FrogPilotPlanner (frogpilot_process) per logged frogpilotPlan tick on the logged inputs at that tick. The new base sends
  button presses as counters: each tick whose logged frogpilotCarState has accelPressed / decelPressed adds one press. The old base's
  memory Params live in <PARAMS_ROOT>/memory."""
  import datetime
  import inspect
  from cereal import log
  from openpilot.frogpilot.controls import frogpilot_planner as FPM
  real_params = FPM.Params
  mem = Path(os.environ['PARAMS_ROOT']) / 'memory'
  FPM.Params = lambda *a, memory=False, **k: real_params(str(mem), return_defaults=True) if memory else real_params(*a, **k)
  args = dict(error_log=Path(os.environ['PARAMS_ROOT']) / 'error.txt', ThemeManager=_Stub(), frogpilot_api=_Stub())
  planner = FPM.FrogPilotPlanner(*[args[n] for n in list(inspect.signature(FPM.FrogPilotPlanner.__init__).parameters)[1:]])
  counters = 'accelPressCount' in log.Event.schema.fields['frogpilotCarState'].schema.fields
  theme_arg = 0 if 'theme_update_count' in inspect.signature(planner.publish).parameters else False
  wall0, mono0 = init.initData.wallTimeNanos, init.logMonoTime
  presses = dict(accel=0, decel=0)
  rows, prev = [], {}
  for fp in by['frogpilotPlan'][1]:
    t = fp.logMonoTime
    override = {}
    fcs = latest(by, 'frogpilotCarState', t)
    if fcs is None:
      continue
    if counters:
      presses['accel'] += bool(fcs.frogpilotCarState.accelPressed)
      presses['decel'] += bool(fcs.frogpilotCarState.decelPressed)
      b = conv(fcs).as_builder()
      b.frogpilotCarState.accelPressCount, b.frogpilotCarState.decelPressCount = presses['accel'], presses['decel']
      override['frogpilotCarState'] = b.as_reader()
    sm = LazySM(by, conv, t, prev, override)
    now = datetime.datetime.fromtimestamp((wall0 + t - mono0) / 1e9)
    planner.update(now, True, sm, toggles)
    pm = PM()
    planner.publish(theme_arg, sm, pm, toggles)
    prev = sm.seen()
    o = pm.last.frogpilotPlan
    cem = planner.frogpilot_cem
    ce = cem.ce_status if hasattr(cem, 'ce_status') else (planner.params_memory.get("CEStatus") or 0)
    rec = fp.frogpilotPlan
    rows.append([t] + [float(getattr(o, f)) for f in FP_OUT] + [float(ce)] + [float(getattr(rec, f)) for f in FP_OUT])
  cols = ['ns'] + list(FP_OUT) + ['ceStatus'] + [f'log_{f}' for f in FP_OUT]
  arr = np.array(rows, dtype=np.float64).reshape(-1, len(cols))
  F = {c: arr[:, i] for i, c in enumerate(cols)}
  F['ns'] = arr[:, 0].astype(np.int64)
  return F


def controls_pass(by, conv, toggles):
  """The tree's own controlsd (Controls.update / state_control / publish) per logged carControl, on the logged inputs at the
  selfdriveState it read, with CarParams / FrogPilotCarParams from <PARAMS_ROOT> and the given toggles. Message stamps and the request
  time are the logged carControl time, so LongControl sees the timeline the car saw."""
  import cereal.messaging as messaging
  clock = [0]
  sent = {}

  class _PM:
    def __init__(self, *args, **kwargs):
      pass

    def send(self, s, msg):
      sent[s] = msg.as_reader() if hasattr(msg, 'as_reader') else msg

  real_new_message = messaging.new_message

  def new_message(*args, **kwargs):
    msg = real_new_message(*args, **kwargs)
    msg.logMonoTime = clock[0]
    return msg

  messaging.PubMaster, messaging.SubMaster, messaging.new_message = _PM, lambda *a, **k: LazySM(by, conv, 0, {}), new_message
  from opendbc.car import interfaces as car_interfaces
  if hasattr(car_interfaces, 'Params'):   # the old base's CarInterfaceBase opens memory Params (/dev/shm, absent on macOS)
    real_params = car_interfaces.Params
    mem = str(Path(os.environ['PARAMS_ROOT']) / 'memory')
    car_interfaces.Params = lambda *a, memory=False, **k: real_params(mem, return_defaults=True) if memory else real_params(*a, **k)
  from openpilot.frogpilot.common import frogpilot_variables
  from openpilot.selfdrive.controls import controlsd
  for mod in (controlsd, frogpilot_variables):
    if hasattr(mod, 'get_frogpilot_toggles'):
      mod.get_frogpilot_toggles = lambda *a, **k: toggles
  c = controlsd.Controls()
  sds = by['selfdriveState'][0]
  rows, prev = [], {}
  for e in by['carControl'][1]:
    i = bisect.bisect_right(sds, e.logMonoTime) - 1
    if i < 0:
      continue
    clock[0] = e.logMonoTime
    c.sm = LazySM(by, conv, sds[i], prev)
    c._accel_request_time = lambda: clock[0] / 1e9
    c.update()
    CC, lac_log = c.state_control()
    c.publish(CC, lac_log)
    prev = c.sm.seen()
    cc, cs = sent['carControl'].carControl, sent['controlsState'].controlsState
    lac = cs.lateralControlState.torqueState if cs.lateralControlState.which() == 'torqueState' else None
    a = cc.actuators
    rows.append([e.logMonoTime, a.accel, a.torque, a.curvature, a.steeringAngleDeg, a.longControlState.raw, cc.latActive, cc.longActive,
                 cs.desiredCurvature, cs.forceDecel] + ([lac.output, lac.error, lac.f, lac.active, lac.saturated] if lac is not None else [np.nan] * 5)
                + [e.carControl.actuators.accel, e.carControl.actuators.torque])
  cols = ['ns'] + list(CTRL_OUT) + ['rec_accel', 'rec_torque']
  arr = np.array(rows, dtype=np.float64).reshape(-1, len(cols))
  C = {c: arr[:, i] for i, c in enumerate(cols)}
  C['ns'] = arr[:, 0].astype(np.int64)
  return C


def card_pass(by, conv, toggles, cp):
  """The tree's own Car.state_update per logged carState: set speed (update / initialize_v_cruise), the OPGM resume flag,
  FrogPilotCard.update (always on lateral allowed and gate, distance long / very long, force coast, pause, Traffic Mode, experimental
  mode presses) and the live-update handoff's held buttons and AOL suppression. CAN is replaced by the logged carState and the
  car-interface fields of the frogpilotCarState card published in the same step; the SubMaster is the logged inputs before the
  carState. CC_prev / CS_prev follow card's step. A group starts from the car's state: the set speed of its first logged carState and
  the carControl nearest its start, so a group that starts engaged has no engagement on its first tick. Needs EQUIV_STORE_JSON
  (FrogPilotCarParams, IsMetric, ExperimentalMode)."""
  import inspect
  import cereal.messaging as messaging
  from cereal import car, custom
  from openpilot.common.params import Params
  from openpilot.frogpilot.controls import frogpilot_card as FPC
  from openpilot.selfdrive.car import card as card_module
  from openpilot.selfdrive.car.cruise import VCruiseHelper
  messaging.sub_sock = lambda *a, **k: None   # the new base's FrogPilotCard reads UI distance presses from a socket: none in the replay
  messaging.drain_sock = lambda *a, **k: []
  messaging.drain_sock_raw = lambda *a, **k: []
  card_module.can_capnp_to_list = lambda can_strs: []
  if hasattr(FPC, 'Params'):   # the old base's FrogPilotCard opens memory Params (/dev/shm, absent on macOS)
    real_params = FPC.Params
    mem = str(Path(os.environ['PARAMS_ROOT']) / 'memory')
    FPC.Params = lambda *a, memory=False, **k: real_params(mem, return_defaults=True) if memory else real_params(*a, **k)
  presses = [0]
  real_handle = FPC.FrogPilotCard.handle_experimental_mode

  def handle_experimental_mode(self, *args):
    presses[0] += 1
    return real_handle(self, *args)
  FPC.FrogPilotCard.handle_experimental_mode = handle_experimental_mode

  params = Params()
  with custom.FrogPilotCarParams.from_bytes(params.get('FrogPilotCarParams')) as m:
    fpcp = m.as_builder().as_reader()
  card_args = dict(CP=cp, FPCP=fpcp)
  frogpilot_card = FPC.FrogPilotCard(*[card_args[n] for n in list(inspect.signature(FPC.FrogPilotCard.__init__).parameters)[1:]])
  tick = {}
  c = card_module.Car.__new__(card_module.Car)
  c.CP = cp
  c.CI = SimpleNamespace(update=lambda can_list, frogpilot_toggles: (tick['CS'], tick['FPCS']))
  c.RI = SimpleNamespace(update=lambda can_list: None)
  c.can_sock = None
  c.can_rcv_cum_timeout_counter = 0
  c.CC_prev = car.CarControl.new_message()
  c.CS_prev = car.CarState.new_message()
  c.v_cruise_helper = VCruiseHelper(cp)
  c.experimental_mode = params.get_bool('ExperimentalMode') and cp.openpilotLongitudinalControl
  c.is_metric = params.get_bool('IsMetric')
  c.resume_prev_button = False
  c.frogpilot_card = frogpilot_card
  c.frogpilot_toggles = toggles
  c.live_update_handoff_pressed_buttons = set()
  c.live_update_handoff_state = ''
  first = by['carState'][1][0]
  cc0 = latest(by, 'carControl', first.logMonoTime) or (by['carControl'][1][0] if by['carControl'][1] else None)
  if cc0 is not None:
    c.CC_prev = conv(cc0).carControl
  vh = c.v_cruise_helper
  vh.v_cruise_kph = vh.v_cruise_kph_last = int(first.carState.vCruise)
  vh.v_cruise_cluster_kph = int(first.carState.vCruiseCluster)
  rows, prev = [], {}
  for e in by['carState'][1]:
    t = e.logMonoTime
    fcs = latest(by, 'frogpilotCarState', t + BOUND_NS)
    if fcs is None:
      continue
    tick['CS'] = conv(e).as_builder().carState
    tick['FPCS'] = conv(fcs).as_builder().frogpilotCarState
    c.sm = LazySM(by, conv, t, prev)
    CS, _, FPCS = c.state_update()
    prev = c.sm.seen()
    c.CC_prev = c.sm['carControl']
    c.CS_prev = CS.as_reader()
    rec, frec = e.carState, fcs.frogpilotCarState
    rows.append([t] + [float(getattr(CS, f)) for f in CARD_CS] + [float(getattr(FPCS, f)) for f in CARD_FPCS] +
                [float(presses[0]), float(c.resume_prev_button), float(len(c.live_update_handoff_pressed_buttons))] +
                [float(getattr(rec, f)) for f in CARD_CS] + [float(getattr(frec, f)) for f in CARD_FPCS])
  cols = ['ns'] + list(CARD_OUT) + [f'log_{f}' for f in CARD_CS + CARD_FPCS]
  arr = np.array(rows, dtype=np.float64).reshape(-1, len(cols))
  K = {k: arr[:, i] for i, k in enumerate(cols)}
  K['ns'] = arr[:, 0].astype(np.int64)
  return K


def main(out, tip, seg_dirs):
  store_json = os.environ.get('EQUIV_STORE_JSON')
  if store_json:   # the FrogPilot planner and controlsd stages read Params: a private store per process
    os.environ['PARAMS_ROOT'] = str(Path(out).resolve().with_suffix('.params'))
  from openpilot.common.swaglog import cloudlog, ipchandler
  with contextlib.suppress(ValueError):
    cloudlog.removeHandler(ipchandler)
  cloudlog.disabled = True
  tree = Path.cwd()
  ts = tip_schema(tip)
  init, cpm, by = load(ts, seg_dirs)
  conv = Converter()
  toggles = tree_toggles(tree, by)
  alternative_experience = 0
  if store_json:
    import derive_toggles
    alternative_experience = derive_toggles.seed_store(store_json, tip, seg_dirs[0])[-1]
  cp, cp_dropped = tree_cp(cpm, alternative_experience)
  res = dict(tree=str(tree), segs=[str(s) for s in seg_dirs], origin=init.logMonoTime)
  res['radard'] = radard_pass(by, conv, toggles, cpm.carFingerprint)
  res['planner'] = planner_pass(by, conv, toggles, cp)
  frames = loc_frames(by, res['planner'])
  res['loc'] = loc_pass(frames, conv, toggles, cp)
  res['chain'] = loc_pass(frames, conv, toggles, cp, res['planner'])
  if store_json:
    res['fp'] = fp_planner_pass(by, conv, toggles, init)
    res['controls'] = controls_pass(by, conv, toggles)
    res['card'] = card_pass(by, conv, toggles, cp)
  res['dropped'] = {s: sorted(v) for s, v in conv.dropped.items() if v}
  res['cp_dropped'] = sorted(cp_dropped)
  sample = {s: by[s][1][len(by[s][1]) // 2] for s in PSERV + ('liveTracks',) if by[s][1]}
  res['roundtrip_equal'] = {s: repr(getattr(conv(e), s).to_dict()) == repr(getattr(e, s).to_dict()) for s, e in sample.items()}  # repr: NaN == NaN
  with open(out, 'wb') as f:
    pickle.dump(res, f)


if __name__ == "__main__":
  main(sys.argv[1], sys.argv[2], sys.argv[3:])
