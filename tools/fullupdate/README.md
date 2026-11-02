# fullupdate.sh host harness

`harness.py` runs `fullupdate.sh` on the Mac with stubs. It never runs the
live script: each copy has `/data` and `/proc/uptime` moved into the run
directory. git, sudo, flock, setsid, sleep and scons are stubs on PATH, and
`sudo reboot` only records the car stand-in's state. Activate the openpilot
venv first; both trees must be built.

```sh
python tools/fullupdate/harness.py --old-tree <subtract tip> --new-tree <port tree> --out <dir>   # exit 0: every scenario passes
```

- `first-deploy`: the old tree's script and Python run until `git reset`.
  The reset switches to the new source with the old tree's compiled Python
  modules. The car stand-in publishes in the old tree's schema, as the old
  card does until the reboot.
- `new`: the new tree throughout.

Each reboot is classified:
- `offroad`;
- `verified`: fresh `ready`, controls off, Panda in ELM327;
- `fail_open`: no pandaStates and not on-road, the script's documented
  off-road case;
- `RAW`: never allowed.

Each scenario names its expected outcome. `results.json` keeps the event
timelines, and every run directory keeps `run.log`, the supervisor log and
`events.jsonl`. The summary also prints what the new cereal reads from an
old-schema `frogpilotCarState`.
