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

# Phase 8 procedure rehearsal

`phase8_rehearse.py` runs the host blocks of
`docs/frogpilot_refresh_20261031/PHASE8_PROCEDURE.md`, from any git revision,
in zsh against a sandboxed device. The sandbox has a fake root (both params
stores, `/data/openpilot` as a git repo with an origin, `/VERSION`, `/proc`,
cgroups and the overlay staging) and a process table. ssh, systemctl, sudo,
pgrep, ps, tmux, flock, mountpoint, umount, reboot and sleep are shims. The
ssh shim runs the remote command in the sandbox with every device path
rewritten under the sandbox root. It never opens a connection, and the
harness refuses to run when `ssh` does not resolve to it. Each block runs in its
own process group, which is killed when the block ends or times out.

```sh
python tools/fullupdate/phase8_rehearse.py                    # revised text: every item SAFE, exit 0
python tools/fullupdate/phase8_rehearse.py --rev f3a2cadec7   # reviewed text: items 1-7 UNSAFE
python tools/fullupdate/phase8_rehearse.py --item 0 --verbose # the whole procedure in one sandbox
```

Item 0 runs every device script once, in procedure order: switch, reboot,
build, verification, rollback, R2 and resume. Items 1 to 7 are the Act-on
items of the 2026-11-02 Astra review (revision 1, `f3a2cadec7`). Items 8 to
11 are those of its re-review (revision 2, `ede4f0ad8b`): cgroup
quiescence, a rollback that tolerates stale progress params, A3 coverage
and PID changes, and hard ssh bounds with a stalled connection. Each item
runs the failure the reviewer reproduced, plus the normal case. The tool
also checks every block with `zsh -n` or `bash -n`, or compiles it
(Python). `test_phase8_rehearse.py` asserts that each reviewed revision is
UNSAFE on its items and that the current text is SAFE on all of them.
`P8_REHEARSAL_TIME_SCALE` shortens the procedure's wait loops, and the
`timeout` shim scales its limits by it. It is set only inside the sandbox. Needs zsh, bash 5 and GNU coreutils
(`/opt/homebrew/opt/coreutils/libexec/gnubin` on the Mac); the test skips
without them.
