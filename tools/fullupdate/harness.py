#!/usr/bin/env python3
"""Stubbed host harness for fullupdate.sh. It never runs the live script: every copy it runs has /data and /proc/uptime moved into the
harness directory, and git, sudo, flock, setsid and sleep are stubs on PATH. sudo records each `reboot` with the stand-in car's state at
that moment and returns success.

macOS has no /proc: the supervisor's pid check (`grep "__reboot_when_parked" /proc/$p/cmdline`) reads `ps -o command=` instead.
A `scons` stub fails any build the script would start, so a failed runtime-module check shows up as a failed run.
msgq on macOS is ZMQ, which refuses a second publisher of a service; on the device msgq lets the newest publisher take over and the old
one's next send raises MultiplePublishersError. A sitecustomize shim (shim/) gives the scripts' Python that takeover.

Two setups:
  first-deploy: the old tree's script and Python run first (what the device has loaded); `git reset` switches the tree to the new source
                with the old tree's compiled Python modules (the device keeps its old build until the reboot), and the car stand-in speaks
                the old tree's schema (the old card stays in memory until the reboot).
  new:          the new tree's script, Python and schema throughout.

The car stand-in publishes pandaStates, carControl, frogpilotCarState and selfdriveState, sets IsOnroad / IsEngaged, and plays card's part
of the live handoff: requested -> (controls off for 0.5 s) -> verifying -> ready:<monotonic>, refreshed every 0.5 s (or failed).

Verdict per reboot: offroad (ignition, onroad and engaged all off), verified (fresh ready, controls off, Panda in ELM327), fail_open
(no pandaStates and not onroad: the script's documented off-road behaviour), or RAW (anything else, never allowed).

usage: harness.py --old-tree <dir> --new-tree <dir> --out <dir> [--scenario <name> ...]
"""
import argparse
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

PY_EXTENSIONS = ("common/params_pyx.so", "msgq_repo/msgq/ipc_pyx.so")
UPDATE = []
SAFE_REBOOT = ["__safe_reboot"]

ON_ENGAGED = {"ignition": True, "onroad": True, "engaged": True, "aol": True, "pandas": True, "handoff": "ok"}
OFF = {"ignition": False, "onroad": False, "engaged": False, "aol": False, "pandas": True, "handoff": "ok"}
DISENGAGE_AFTER_REQUEST = {"when": {"state": "requested", "delay": 1.0}, "set": {"engaged": False, "aol": False}}
PARK = {"ignition": False, "onroad": False, "engaged": False, "aol": False}

SCENARIOS = {
  "offroad_update": dict(args=UPDATE, uptime=30, car=OFF, expect="offroad"),
  "onroad_update_verified": dict(args=UPDATE, uptime=30, settle=True, car=ON_ENGAGED, events=[DISENGAGE_AFTER_REQUEST], expect="verified"),
  "onroad_update_handoff_failed_then_parked": dict(args=UPDATE, car={**ON_ENGAGED, "handoff": "fail"},
                                                   events=[DISENGAGE_AFTER_REQUEST, {"when": {"state": "failed", "delay": 3.0}, "set": PARK}],
                                                   expect="offroad"),
  "onroad_update_pandas_stale_then_parked": dict(args=UPDATE, car={**ON_ENGAGED, "engaged": False, "aol": False, "pandas": False},
                                                 events=[{"when": {"after": 8.0}, "set": {**PARK, "pandas": True}}], expect="offroad"),
  "onroad_update_cancelled": dict(args=UPDATE, car=ON_ENGAGED, cancel_after_supervisor=1.5, expect="none"),
  "safe_reboot_onroad_verified": dict(args=SAFE_REBOOT, car=ON_ENGAGED, events=[DISENGAGE_AFTER_REQUEST], expect="verified", setups=("new",)),
  "safe_reboot_offroad": dict(args=SAFE_REBOOT, car=OFF, expect="offroad", setups=("new",)),
  "safe_reboot_twice_while_pending": dict(args=SAFE_REBOOT, car=ON_ENGAGED, second_run_after_supervisor=1.0,
                                          events=[{"when": {"state": "requested", "delay": 4.0}, "set": {"engaged": False, "aol": False}}],
                                          expect="verified", setups=("new",)),
  "pandas_missing_offroad_fail_open": dict(args=UPDATE, car={**OFF, "ignition": True, "pandas": False}, expect="fail_open", setups=("new",)),
}

STUBS = {
  "git": r'''#!/usr/bin/env bash
log() { "$HARNESS_PYTHON" "$HARNESS_DIR/bin/_event.py" git "$@"; }
case "$1 $2" in
  "rev-parse --show-toplevel") echo "$HARNESS_DIR/tree" ;;
  "symbolic-ref --quiet") echo "$HARNESS_BRANCH" ;;
  "rev-parse --abbrev-ref") echo "origin/$HARNESS_BRANCH" ;;
  "fetch "*) log "$@" ;;
  "reset --hard") log "$@"; ln -sfn "$(cat "$HARNESS_DIR/next_tree")" "$HARNESS_DIR/tree" ;;
  "submodule update") log "$@" ;;
  *) log "$@"; echo "harness git stub: unexpected: $*" >&2; exit 1 ;;
esac
''',
  "sudo": r'''#!/usr/bin/env bash
if [ "$*" = "reboot" ]; then
  exec "$HARNESS_PYTHON" "$HARNESS_DIR/bin/_event.py" reboot
fi
"$HARNESS_PYTHON" "$HARNESS_DIR/bin/_event.py" sudo_refused "$@"
exit 1
''',
  "setsid": '#!/usr/bin/env bash\nexec "$@"\n',
  "scons": '#!/usr/bin/env bash\n"$HARNESS_PYTHON" "$HARNESS_DIR/bin/_event.py" scons "$@"\nexit 1\n',
  "flock": '#!/usr/bin/env bash\nexec "$HARNESS_PYTHON" "$HARNESS_DIR/bin/_flock.py" "$@"\n',
  "sleep": '#!/usr/bin/env bash\n"$HARNESS_PYTHON" "$HARNESS_DIR/bin/_event.py" sleep "$@"\n/bin/sleep 0.2\n',
  "_event.py": r'''import json, os, sys, time
from pathlib import Path
h = Path(os.environ["HARNESS_DIR"])
event = {"event": sys.argv[1], "args": sys.argv[2:], "t": time.monotonic()}
if sys.argv[1] == "reboot":
  event["car"] = json.loads((h / "car_state.json").read_text())
with open(h / "events.jsonl", "a") as f:
  f.write(json.dumps(event) + "\n")
''',
  "_flock.py": r'''import fcntl, sys
args = sys.argv[1:]
fd = int([a for a in args if a != "-n"][0])
try:
  fcntl.flock(fd, fcntl.LOCK_EX | (fcntl.LOCK_NB if "-n" in args else 0))
except BlockingIOError:
  sys.exit(1)
''',
}


PROC_CMDLINE = 'grep -qa "__reboot_when_parked" "/proc/$p/cmdline" 2>/dev/null'


SHIM = r'''import os
import time
from pathlib import Path

try:
  import msgq
  from msgq.ipc_pyx import MultiplePublishersError
except ImportError:
  msgq = None


class _OwnedPub:
  def __init__(self, sock, owner):
    self._sock, self._owner = sock, owner

  def send(self, data):
    if self._owner.read_text() != str(os.getpid()):
      raise MultiplePublishersError(self._owner.name)
    return self._sock.send(data)

  def __getattr__(self, name):
    return getattr(self._sock, name)


if msgq is not None:
  _pub_sock = msgq.pub_sock

  def pub_sock(endpoint, segment_size=0):
    owner = Path(os.environ["HARNESS_DIR"]) / "pub_owners" / endpoint
    owner.parent.mkdir(exist_ok=True)
    owner.write_text(str(os.getpid()))
    deadline = time.monotonic() + 2.0
    while True:
      try:
        return _OwnedPub(_pub_sock(endpoint, segment_size), owner)
      except MultiplePublishersError:
        if time.monotonic() > deadline:
          raise
        time.sleep(0.05)

  msgq.pub_sock = pub_sock
'''


def remap(script: str, h: Path) -> str:
  assert script.count(PROC_CMDLINE) == 1, "fullupdate.sh changed its supervisor pid check: update PROC_CMDLINE"
  script = script.replace(PROC_CMDLINE, 'ps -o command= -p "$p" 2>/dev/null | grep -q "__reboot_when_parked"')
  return script.replace("/data", f"{h}/data").replace("/proc/uptime", f"{h}/proc/uptime")


def farm(dst: Path, tree: Path, script: str, extensions_from: Path | None = None) -> None:
  """`tree` as symlinks, with fullupdate.sh replaced by `script`. Relative links (openpilot/*, msgq, opendbc) resolve inside the farm, so
  extensions_from can swap the compiled Python modules (PY_EXTENSIONS) for another tree's."""
  overrides = {rel: extensions_from / rel for rel in PY_EXTENSIONS} if extensions_from else {}
  rebuilt = {"openpilot"} | {str(Path(rel).parents[i]) for rel in overrides for i in range(len(Path(rel).parts) - 1)}

  def build(src: Path, d: Path, prefix: str) -> None:
    d.mkdir(parents=True)
    for entry in os.scandir(src):
      rel = prefix + entry.name
      target = d / entry.name
      if rel == "fullupdate.sh":
        continue
      if rel in overrides:
        target.symlink_to(overrides[rel])
      elif rel in rebuilt and entry.is_dir(follow_symlinks=False):
        build(Path(entry.path), target, rel + "/")
      elif entry.is_symlink() and not os.path.isabs(os.readlink(entry.path)):
        target.symlink_to(os.readlink(entry.path))
      else:
        target.symlink_to(entry.path)

  build(tree, dst, "")
  (dst / "fullupdate.sh").write_text(script)
  (dst / "fullupdate.sh").chmod(0o755)


def tree_env(tree: Path, h: Path) -> dict:
  env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "OPENPILOT_PREFIX")}
  env.update(PARAMS_ROOT=str(h / "params"), HARNESS_DIR=str(h), HARNESS_PYTHON=sys.executable, PYTHON=sys.executable,
             PYTHONPATH=f"{tree}:{tree}/frogpilot/third_party")
  return env


def read_events(h: Path) -> list[dict]:
  events = []
  for line in (h / "events.jsonl").read_text().splitlines() if (h / "events.jsonl").exists() else []:
    try:
      events.append(json.loads(line))
    except json.JSONDecodeError:
      pass  # a line still being written
  return events


def run_scenario(name: str, setup: str, sc: dict, old_tree: Path, new_tree: Path, out: Path) -> dict:
  h = out / f"{setup}-{name}"
  shutil.rmtree(h, ignore_errors=True)
  for d in ("data/openpilot", "proc", "params", "bin"):
    (h / d).mkdir(parents=True)
  (h / "data/openpilot/prebuilt").touch()
  (h / "proc/uptime").write_text(f"{sc.get('uptime', 1000)}.00 1.00\n")
  for stub, text in STUBS.items():
    (h / "bin" / stub).write_text(text)
    (h / "bin" / stub).chmod(0o755)
  (h / "shim").mkdir()
  (h / "shim/sitecustomize.py").write_text(SHIM)

  old_script, new_script = (old_tree / "fullupdate.sh").read_text(), (new_tree / "fullupdate.sh").read_text()
  if setup == "first-deploy":
    farm(h / "farm_old", old_tree, remap(old_script, h))
    farm(h / "farm_next", new_tree, remap(new_script, h), extensions_from=old_tree)
    start_tree, next_tree, car_tree = h / "farm_old", h / "farm_next", old_tree
  else:
    farm(h / "farm_new", new_tree, remap(new_script, h))
    start_tree = next_tree = h / "farm_new"
    car_tree = new_tree
  (h / "tree").symlink_to(start_tree)
  (h / "next_tree").write_text(str(next_tree))
  (h / "scenario.json").write_text(json.dumps({"car": sc["car"], "events": sc.get("events", []), "timeout": sc.get("timeout", 40)}))

  car_env = tree_env(car_tree, h)
  car = subprocess.Popen([sys.executable, __file__, "fake-car", str(h)], env=car_env, cwd=h, stdout=open(h / "car.log", "w"), stderr=subprocess.STDOUT)
  deadline = time.monotonic() + 10
  while not (h / "car_state.json").exists():
    assert car.poll() is None and time.monotonic() < deadline, f"car stand-in did not start, see {h / 'car.log'}"
    time.sleep(0.1)
  time.sleep(1.0)

  env = tree_env(car_tree, h)
  env.update(HARNESS_BRANCH="!my-fp-1031", PATH=f"{h / 'bin'}:{os.environ['PATH']}", PYTHONPATH=str(h / "shim"))
  run_log = open(h / "run.log", "w")
  script = subprocess.Popen(["bash", str(h / "tree/fullupdate.sh"), *sc["args"]], env=env, cwd=h, stdout=run_log, stderr=subprocess.STDOUT,
                            start_new_session=True)
  pidfile, supervisor_seen_at, extra_runs = h / "data/fullupdate_reboot.pid", None, []
  deadline = time.monotonic() + sc.get("timeout", 40)
  while time.monotonic() < deadline:
    time.sleep(0.2)
    events = read_events(h)
    if any(e["event"] == "reboot" for e in events):
      time.sleep(0.5)
      break
    if supervisor_seen_at is None and pidfile.exists():
      supervisor_seen_at = time.monotonic()
    if supervisor_seen_at is not None:
      if "cancel_after_supervisor" in sc and time.monotonic() - supervisor_seen_at >= sc["cancel_after_supervisor"]:
        (h / "data/fullupdate_reboot.cancel").touch()
        sc = {**sc, "cancel_after_supervisor": float("inf")}
      if "second_run_after_supervisor" in sc and not extra_runs and time.monotonic() - supervisor_seen_at >= sc["second_run_after_supervisor"]:
        extra_runs.append(subprocess.run(["bash", str(h / "tree/fullupdate.sh"), *sc["args"]], env=env, cwd=h, stdout=run_log,
                                         stderr=subprocess.STDOUT, timeout=30, start_new_session=True).returncode)
    if script.poll() is not None and supervisor_seen_at is not None and not pidfile.exists():
      break
  (h / "stop").touch()
  try:
    os.killpg(script.pid, signal.SIGTERM)
  except (ProcessLookupError, PermissionError):
    pass
  subprocess.run(["pkill", "-TERM", "-f", f"{h}/"], check=False)
  car.wait(timeout=10)
  time.sleep(0.5)
  subprocess.run(["pkill", "-KILL", "-f", f"{h}/"], check=False)
  return judge(name, setup, sc, h, read_events(h), extra_runs)


def classify(car: dict) -> str:
  state, _, stamp = car["handoff"].partition(":")
  if not car["ignition"] and not car["onroad"] and not car["engaged"]:
    return "offroad"
  if state == "ready" and car["t"] - float(stamp) <= 2.0 and not car["engaged"] and not car["aol"] and car["elm"] and car["pandas"]:
    return "verified"
  if not car["onroad"] and not car["engaged"] and not car["pandas"]:
    return "fail_open"
  return "RAW"


def judge(name: str, setup: str, sc: dict, h: Path, events: list[dict], extra_runs: list[int]) -> dict:
  reboots = [classify(e["car"]) for e in events if e["event"] == "reboot"]
  git = [e["args"][0] for e in events if e["event"] == "git"]
  settle = [float(e["args"][0]) for e in events if e["event"] == "sleep" and float(e["args"][0]) >= 100]
  fetch_t = next((e["t"] for e in events if e["event"] == "git" and e["args"][0] == "fetch"), None)
  settle_t = next((e["t"] for e in events if e["event"] == "sleep" and float(e["args"][0]) >= 100), None)
  supervisor_log = (h / "data/fullupdate_reboot.log").read_text() if (h / "data/fullupdate_reboot.log").exists() else ""
  checks = {
    "no RAW reboot": "RAW" not in reboots,
    f"reboots == {[sc['expect']] if sc['expect'] != 'none' else []}": reboots == ([sc["expect"]] if sc["expect"] != "none" else []),
    "git: fetch, reset, submodule" if sc["args"] == UPDATE else "git: none": git == (["fetch", "reset", "submodule"] if sc["args"] == UPDATE else []),
    "prebuilt removed only for an update": (h / "data/openpilot/prebuilt").exists() == (sc["args"] != UPDATE),
    "no sudo other than reboot": not any(e["event"] == "sudo_refused" for e in events),
    "no build started": not any(e["event"] == "scons" for e in events),
  }
  if sc["args"] == UPDATE:
    checks["boot settle wait (150 s) before fetch" if sc.get("settle") else "no boot settle wait"] = \
      (settle == [150.0] and settle_t < fetch_t) if sc.get("settle") else not settle
  if "second_run_after_supervisor" in sc:
    checks["second run exits 0, one supervisor ran"] = extra_runs == [0] and supervisor_log.count("reboot supervisor started") == 1
  if sc["expect"] == "none":
    checks["supervisor exited on cancel"] = "reboot cancelled" in supervisor_log
  car_log = [e for e in events if e["event"] in ("handoff", "car", "banner")]
  return {"scenario": name, "setup": setup, "pass": all(checks.values()), "checks": checks, "reboots": reboots, "git": git,
          "timeline": [f"{e['t'] - events[0]['t']:6.2f} {e['event']} {' '.join(map(str, e['args']))}" for e in events] if events else [],
          "car_log": len(car_log), "dir": str(h)}


def fake_car(h: Path) -> None:
  import cereal.messaging as messaging
  from cereal import car, log
  from openpilot.common.params import Params

  print(f"cereal from {Path(messaging.__file__).parents[1]}", flush=True)
  cfg = json.loads((h / "scenario.json").read_text())
  state, pending = dict(cfg["car"]), list(cfg["events"])
  params = Params()
  pm = messaging.PubMaster(["pandaStates", "carControl", "frogpilotCarState", "selfdriveState"])
  sm = messaging.SubMaster(["alertDebug"])
  events = open(h / "events.jsonl", "a", buffering=1)

  def event(kind, *args):
    events.write(json.dumps({"event": kind, "args": list(args), "t": time.monotonic()}) + "\n")

  start, seen, elm, ok_since, ready_at, last_ready, banner = time.monotonic(), {}, False, None, None, 0.0, None
  onroad_prev = engaged_prev = None
  while not (h / "stop").exists() and time.monotonic() - start < cfg["timeout"] + 10:
    now = time.monotonic()
    raw = params.get("LiveUpdateHandoffState") or ""
    raw = raw.decode() if isinstance(raw, bytes) else raw
    name = raw.partition(":")[0]
    if name and name not in seen:
      seen[name] = now
      event("handoff", raw)
    for ev in list(pending):
      when = ev["when"]
      t0 = start if "after" in when else seen.get(when["state"])
      if t0 is not None and now - t0 >= when.get("after", when.get("delay", 0.0)):
        state.update(ev["set"])
        pending.remove(ev)
        event("car", json.dumps(ev["set"], sort_keys=True))

    controls_off = state["onroad"] and not state["engaged"] and not state["aol"] and state["pandas"]
    if name == "requested" and controls_off:
      ok_since = ok_since or now
      if now - ok_since >= 0.5:
        if state["handoff"] == "fail":
          params.put("LiveUpdateHandoffState", "failed")
        else:
          elm, ready_at = True, now + 0.5
          params.put("LiveUpdateHandoffState", f"verifying:{now:.6f}")
    else:
      ok_since = None
    if controls_off and ((name == "verifying" and now >= ready_at) or (name == "ready" and now - last_ready >= 0.5)):
      params.put("LiveUpdateHandoffState", f"ready:{now:.6f}")
      last_ready = now
    if not state["onroad"]:
      elm = False

    if state["onroad"] != onroad_prev or state["engaged"] != engaged_prev:
      params.put_bool("IsOnroad", state["onroad"])
      params.put_bool("IsEngaged", state["engaged"])
      onroad_prev, engaged_prev = state["onroad"], state["engaged"]

    if state["pandas"]:
      msg = messaging.new_message("pandaStates", 1)
      msg.valid = True
      ps = msg.pandaStates[0]
      ps.pandaType = log.PandaState.PandaType.tres
      ps.ignitionLine = state["ignition"]
      ps.safetyModel = car.CarParams.SafetyModel.elm327 if elm else car.CarParams.SafetyModel.hyundai
      pm.send("pandaStates", msg)
    msg = messaging.new_message("carControl")
    msg.valid = True
    msg.carControl.enabled = msg.carControl.longActive = state["engaged"]
    msg.carControl.latActive = state["engaged"] or state["aol"]
    pm.send("carControl", msg)
    msg = messaging.new_message("frogpilotCarState")
    msg.valid = True
    msg.frogpilotCarState.alwaysOnLateralEnabled = state["aol"]
    pm.send("frogpilotCarState", msg)
    msg = messaging.new_message("selfdriveState")
    msg.valid = True
    msg.selfdriveState.enabled = msg.selfdriveState.active = state["engaged"]
    pm.send("selfdriveState", msg)

    tmp = h / "car_state.json.tmp"
    tmp.write_text(json.dumps({**state, "elm": elm, "handoff": params.get("LiveUpdateHandoffState") or "", "t": time.monotonic()}))
    tmp.replace(h / "car_state.json")

    sm.update(0)
    if sm.updated["alertDebug"]:
      text = f"{sm['alertDebug'].alertText1} / {sm['alertDebug'].alertText2}"
      if text != banner:
        banner = text
        event("banner", text)
    time.sleep(0.05)


def decode_check(old_tree: Path, new_tree: Path, out: Path) -> str:
  """What the new tree's cereal reads from an old-schema frogpilotCarState with alwaysOnLateralEnabled set (the first-deploy supervisor
  decodes the old card's messages)."""
  blob = out / "old_frogpilotCarState.bin"
  encode = "\n".join(["import sys", "import cereal.messaging as m", "msg = m.new_message('frogpilotCarState')",
                      "msg.frogpilotCarState.alwaysOnLateralEnabled = True", "open(sys.argv[1], 'wb').write(msg.to_bytes())"])
  decode = "\n".join(["import sys", "from cereal import log", "with log.Event.from_bytes(open(sys.argv[1], 'rb').read()) as ev:",
                      "  print(ev.which(), ev.frogpilotCarState.alwaysOnLateralEnabled)"])
  # cwd=out: `python -c` puts the cwd first on sys.path, ahead of the tree's PYTHONPATH
  subprocess.run([sys.executable, "-c", encode, str(blob)], env=tree_env(old_tree, out), cwd=out, check=True)
  return subprocess.run([sys.executable, "-c", decode, str(blob)], env=tree_env(new_tree, out), cwd=out, check=True, capture_output=True,
                        text=True).stdout.strip()


def main() -> int:
  if len(sys.argv) == 3 and sys.argv[1] == "fake-car":
    fake_car(Path(sys.argv[2]))
    return 0
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--old-tree", type=Path, required=True)
  parser.add_argument("--new-tree", type=Path, required=True)
  parser.add_argument("--out", type=Path, required=True)
  parser.add_argument("--scenario", action="append", choices=sorted(SCENARIOS))
  args = parser.parse_args()
  old_tree, new_tree, out = args.old_tree.resolve(), args.new_tree.resolve(), args.out.resolve()
  out.mkdir(parents=True, exist_ok=True)

  results = []
  for name in args.scenario or SCENARIOS:
    for setup in SCENARIOS[name].get("setups", ("first-deploy", "new")):
      result = run_scenario(name, setup, SCENARIOS[name], old_tree, new_tree, out)
      results.append(result)
      print(f"{'PASS' if result['pass'] else 'FAIL'} {setup:12s} {name:44s} reboots={result['reboots']} git={result['git']}", flush=True)
      for check, ok in result["checks"].items():
        if not ok:
          print(f"     failed: {check}  ({result['dir']})", flush=True)

  summary = {
    "scripts": {tree.name: hashlib.sha256((tree / "fullupdate.sh").read_bytes()).hexdigest()[:16] for tree in (old_tree, new_tree)},
    "decode_old_frogpilotCarState_with_new_cereal": decode_check(old_tree, new_tree, out),
    "results": results,
  }
  (out / "results.json").write_text(json.dumps(summary, indent=1))
  print(f"scripts sha256: {summary['scripts']}")
  print(f"old-schema frogpilotCarState(alwaysOnLateralEnabled=True) read by the new cereal: {summary['decode_old_frogpilotCarState_with_new_cereal']}")
  return 0 if all(r["pass"] for r in results) else 1


if __name__ == "__main__":
  sys.exit(main())
