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
pgrep, ps, tmux, flock, mountpoint, umount, rm, reboot, sleep, mktemp, df, ss,
tar and timeout are shims. The
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
runs the failure the reviewer reproduced, plus the normal case. Items 12
and 13 are those of round 3 (revision 3, `7b11e2a78e`): an unreadable child
`cgroup.procs` on v1, v2 and hybrid layouts in the switch, R2 and R2-key,
and wait timing that an inherited environment variable must not change.
Item 14 is the device stop of 2026-11-14 (the process check counted the
script itself). Items 15, 16 and 18 to 21 came with revision 7 (`39a689761c`
fails them): the candidate's mapd against the base's msgq reader slots (box
2.1a), the tile staging (section 2a, since revision 9 into
`/data/media/0/osm-v240`), the speed-limit settings gate (box 3.9), the
running mapd and both tile directories after the boot (5a.10), A3's span and
segment event rules, and the 3.1 owners record. Items 22, 25 and 26 are
Act-on items of the review of revision 7 (`0a2301158f` fails them): the
switch and R1 cut before every command that changes the device (a DEBUG trap
kills the script there, as a power cut would) and inside each git write (its
lock file left), then the unit started, so today's tiles never change, the
checked-out mapd never meets the other format, and R1 recovers or, after a
git write killed half way, ends in the documented stale-lock STOP (the lock
kept, nothing deleted); `MapdSettings`
keys read as Go's decoder reads them; and no write through a symbolic link.
Items 27 and 28 are those of the review of revision 8 (`b9077abd7a` fails
them, and items 16, 19 and 22 too): no delete through a bind mount, and a
`settings_version` the new mapd can load. The review of revision 9
(`8d89e18e32` fails items 19, 27 and 28) extended them: staging runs no `rm`
on the NVMe and never replaces a set in place (27, with the reviewer's
reverse bind mount), 5a.10 and R3 compare today's tiles with the 2a.5
listing (19, with a changed tile of the same name), and box 3.9 also stops
the optional `mapd_*.json` files and a `MAPD_` variable in mapd's
environment (28). Item 31 is from the review of revision 10 (`f6daae923d`
fails it): box 2a.5 only reads, accepts no listing of today's tiles after a
read, hash or transfer error (the reviewer's three tiles, one unreadable),
refuses and names an entry that is not a regular file or a directory, and
compares the listing with the old-format sets on the host. Items 17, 23 and
24 tested the tile renames of revisions 7 and 8. Item 29 tested the host
copy of today's tiles with its restore R4, and item 30 the git lock cleanup
(revisions 9 and 10). Each was deleted with its step. The harness judges a tile set by
the bytes of the tiles in the directory of the checked-out mapd
(`Sandbox.pair()`: the old mapd reads `/data/media/0/osm/offline`, the new one
`/data/media/0/osm-v240/offline`), and `Sandbox.old_tiles_kept()` compares
today's tiles with the start. The sandbox carries stand-in tiles (today's set;
on the host a main set and a Slovenia overlay, and the three old-format sets
of the map checkout), stand-in mapd blobs and
`msgq.h` in both trees, `/dev/shm/msgq_mapdOut`, a `/proc/<pid>/exe` link for
mapd, a `df` shim whose `/data/media` answer the config can set, and an `rm`
shim that treats the config's bind mounts as the kernel does (`rm -r` of the
mount point deletes the source's contents, then fails on the busy point;
`rm -r` of the source empties what the mount shows). The device `tar` shim
acts as GNU tar does on the device: it skips an entry it cannot read,
archives the rest and exits 2 (the macOS tar loses other entries after such
an error). Each modeled process
has `/proc/<pid>/cwd` and `fd/<n>` links, and a device script run through
the ssh shim has a `/proc` entry of its own (its command line, and
`/data/openpilot` as its working directory) that goes away when it ends.
The tool
also checks every block with `zsh -n` or `bash -n`, or compiles it
(Python). `test_phase8_rehearse.py` asserts that each reviewed revision is
UNSAFE on its items and that the current text is SAFE on all of them.
The harness speeds up its own extracted copy of the wait loops by scaling
their literal limits (`accelerate`). The procedure has no timing setting.
`P8_REHEARSAL_TIME_SCALE` scales only the sandbox `timeout` shim. Needs zsh, bash 5 and GNU coreutils
(`/opt/homebrew/opt/coreutils/libexec/gnubin` on the Mac); the test skips
without them.
