# AGENTS.md

This file provides guidance to coding agents when working with code in this repository.

## Build Commands
- Full build: `scons -j$(nproc)`
- Build specific component: `scons -j8 selfdrive/ui/` or `cd selfdrive/ui/ && scons -u -j8`
- Install dependencies: `tools/ubuntu_setup.sh` or `tools/mac_setup.sh`

## Local macOS Development
- For a repeatable host build/test environment without the sudo-only Panda ARM toolchain, run `OPENPILOT_SKIP_PANDA_TOOLCHAIN=1 tools/mac_setup.sh`.
- For Panda firmware builds, use the full `tools/op.sh setup` workflow instead; installing `gcc-arm-embedded` may prompt for sudo.
- Always activate the managed Python 3.11 environment before building or testing: `source .venv/bin/activate`. Do not use the system Python.
- Verify the environment with `tools/op.sh check`.
- Build the UI and core pytest extensions with `scons -j8 selfdrive/ui/ test-dependencies`. The `test-dependencies` target builds Params, msgq, and transformations bindings required by the global pytest fixtures.
- Run targeted tests from the repository root, for example `pytest common/tests/test_params.py`.
- On macOS, use `multiprocessing.get_context("spawn")` for analysis workers,
  including `ProcessPoolExecutor(mp_context=...)`. Do not use `fork` or disable
  Objective-C fork checks: NumPy and setproctitle can abort forked workers and
  hang the pool. Worker functions must be importable; guard pool creation with
  `if __name__ == "__main__":`.
- macOS uses ZMQ, which cannot use `OPENPILOT_PREFIX`; fake socket events also
  require Linux. Keep test isolation and mark these integration tests unsupported
  on macOS instead of removing the prefix. Rebuild msgq after binding changes.
- For repeatedly reused Cap'n Proto replay messages, cache a validated owned
  reader (`event.as_builder().as_reader()`) once before replay. Repeated reads and
  copies of a serialized reader consume its traversal budget and can abort
  pycapnp. Keep input traversal limits; do not set them to unlimited.
- SCons objects, libraries, the UI binary, Qt `moc_*.cc` files, and compiled translations are ignored. After building/testing, confirm `git status --short` contains only intentional source changes; do not use destructive Git cleanup commands to remove build output.

## Test Commands
- Run all tests: `pytest .`
- Run specific test: `pytest path/to/test_file.py::test_function_name`
- Run tests for specific component: `cd system/loggerd && pytest .`

## Device Update Workflow
- Preferred SSH profile for local device updates: `comma`.
- If `ssh comma` times out or fails, immediately retry the same command with `ssh commawifi`.
- Deploy the currently pushed branch on-device with:
  - `ssh -tt comma 'cd /data/openpilot && ./fullupdate.sh'`
  - fallback: `ssh -tt commawifi 'cd /data/openpilot && ./fullupdate.sh'`
- Note: `fullupdate.sh` can close SSH during restart/relaunch; this is expected.
- The driver can start the same script from the device Settings -> Software -> Full Update (output appended to `/data/fullupdate.log`).
- Automatic updates: with `AutomaticUpdates` on (it is on on the device), `frogpilot_process` checks the device branch every 5 minutes and runs `fullupdate.sh` when the remote has a newer commit. A push to the device branch reaches the car within about 5 minutes, also while Radek drives.
  - Never push a docs-only change while Radek drives: it stages an update for nothing, and the car restarts the next time cruise is turned off.
  - After the branch switch to `!my-fp-1031`, every push to `!my-fp-1031` reaches the car.
- On-road runs wait until device uptime is at least 180 s before fetching (the fetch/reset load during the boot storm delayed pandad's safety-mode switch by 5 s on 2026-09-05, the car saw no SCC12 and latched the ACC fault). Parked runs do not wait.
- Deploy updates right away even when the vehicle is on-road; stopping or going off-road is not required. The script stages the update and detaches a background supervisor that survives the SSH session closing.
  - On-road, the UI shows a passive `Update Ready` notice. Do not click it; clicks intentionally do nothing.
  - To apply the update on-road, press and release the cruise-main button once to fully turn SCC/cruise off, then keep cruise off. The supervisor waits through a release dwell and reboots only after openpilot is disengaged and verified stock SCC takeover is fresh and fault-free.
  - The same flow works in Park with the ignition on (same cruise-off + verification gates; only Reverse/Neutral block the handoff). If cruise was never on while parked, the gates are already satisfied and the restart proceeds after verification without a button press.
  - Every restart request while the ignition is on or the car is started (settings Reboot, Soft Reboot, the offroad-alert reboot buttons, The Pond's endpoints) is routed by the manager to `fullupdate.sh __safe_reboot` before `manager_cleanup` (no fetch/stage, verified stock-SCC takeover, then reboot). A raw `DoReboot` with ignition on faults the cluster because the spoofed SCC stream dies without a handoff.
  - If the vehicle is already off-road, the update still reboots immediately through the existing parked path.
  - If live SCC takeover cannot be verified, the supervisor fails safe and leaves the update staged for an off-road reboot instead of forcing an on-road restart.
  - Cancel a pending reboot: `touch /data/fullupdate_reboot.cancel` (graceful; update stays staged, applies on next reboot/deploy).
  - Watch a pending reboot: `tail -f /data/fullupdate_reboot.log`.
  - A change to `fullupdate.sh` itself takes effect on the SECOND deploy after it lands (the deploy that pulls it still runs the previously-loaded script).
- The device fetches `origin` over SSH with a read-only deploy key at `/data/ssh/id_ed25519`, registered on the repo as "comma device read-only persistent (2026-09-05)". The device's system SSH config already selects this path; no SSH-command override is needed. `/data/ssh` must be mode 700 and the private key mode 600.
- Never store device keys under `~/.ssh`: `/home` uses a temporary overlay backed by `/rwtmp` (tmpfs), so those keys disappear at reboot. The original 2026-09-02 key was lost this way; its obsolete GitHub registration was removed after the persistent replacement passed a normal fetch.
- Verify GitHub access with `ssh comma 'cd /data/openpilot && git ls-remote origin "refs/heads/!my-fp-1031"'` (the device branch; `!my-fp-new` before the switch; same commawifi fallback). Anonymous HTTPS previously failed with 401 on `git-upload-pack`; a missing SSH key instead reports `Permission denied (publickey)`.
- If SSH auth fails, diagnose key presence/selection first. Emergency fallback: a git bundle of `<device_head>..<device branch>` copied to `/data/op_update.bundle`, with `origin` pointed at it for one `fullupdate.sh` run, then restored to `git@github.com:rchybicki/openpilot.git`.
- After deploy, verify device commit hash:
  - `ssh -o BatchMode=yes -o ConnectTimeout=8 comma 'cd /data/openpilot && git rev-parse --short HEAD'`
  - fallback: `ssh -o BatchMode=yes -o ConnectTimeout=8 commawifi 'cd /data/openpilot && git rev-parse --short HEAD'`
- If device is temporarily unavailable after update, retry SSH verification until it comes back.
- During on-road operation, inspect existing logs copied to the host. Do not launch diagnostic `SubMaster`/`sub_sock` readers or import driving modules on the device for health checks. msgq has 15 reader slots; new subscribers can evict active readers. Repeated live checks coincided with a communication fault on 2026-09-05; a later complete log minute had no fault after those readers stopped. Causality is not proved. Treat live subscriptions as intrusive, not as passive file reads.

## Model Update Commit Naming
- Name model update commits after the model name.
- If importing from `main`, use the upstream commit title as the commit message (example: `CD210 model`).
- If importing from a non-main branch, use the source branch name as the commit message.

## Model Update Import Workflow
- Do not assume upstream model-update commits can be cherry-picked cleanly into this fork.
- If the upstream branch stores model artifacts in Git LFS, prefer `git worktree add` on the upstream ref and copy the materialized model files from the worktree instead of cherry-picking the commit.
- This fork's runtime may diverge from upstream `selfdrive/modeld`; when importing off-policy or split-model updates, check whether the active integration point is `frogpilot/tinygrad_modeld` or another FrogPilot-specific path before patching `selfdrive/modeld`.
- For FrogPilot tinygrad model imports, update the ONNX assets in `frogpilot/tinygrad_modeld/models/` and keep `frogpilot/tinygrad_modeld/tinygrad_modeld.py`, `frogpilot/tinygrad_modeld/parse_model_outputs.py`, and `frogpilot/common/frogpilot_variables.py` in sync with any new compiled/metadata files that will be required.
- In the current setup, off-policy support is conditional: only the default FrogPilot tinygrad model should require and run `driving_off_policy`; non-default tinygrad models should stay on the existing vision + policy path.
- If using the local helper at `skills/frogpilot-model-import/scripts/import_model.py`, keep it aligned with the current upstream asset set so future imports stay repeatable.
- In this repo, the FrogPilot ONNX assets are currently tracked as normal git blobs, not repo-managed Git LFS objects, so copied model binaries should be committed directly unless the repo's attributes change later.
- Before pushing a commit that changes `selfdrive/modeld/` (models, runtime, `SConscript`), `tinygrad_repo/`, `common/file_chunker.py`, or `SConstruct`, run `tools/modeld/build_check.py --device comma <commit>` (fallback `commawifi`) with the car parked, and push only after it prints `PASS`. The check covers the modeld build path on the car: compile, chunk, load, and numerics.
  - On the car it copies the car's checkout (`git archive` of its HEAD) and build outputs (its untracked files) to `/data/model_build_check.<run>`, applies the commit's diff, and runs `scons --minimal` (which compiles the QCOM model bundle and splits it into chunks), then `selfdrive/modeld/tests/test_model_load.py`, then 8 stateful policy steps compared with onnxruntime. A step fails when the desired lateral or longitudinal acceleration differs by more than 0.05 m/s^2 (Cheddar's QCOM bundle: 0.020 and 0.006) or any output by more than 1 % of the output range (Cheddar: 0.34 %; a dropped recurrent state: 3.9 % to 9 %).
  - It is not a full replay of the boot build (`system/manager/build.py`): the panda version string differs, build.py's Params artifact cleanup is skipped, and a failed build is retried at `-j1` only (build.py: `-j2`, then `-j1`).
  - Why: the car compiles a different bundle (QCOM, FLOAT16, IMAGE; Cheddar: 96.1 MB on the car, 76.2 MB on the Mac) and runs the larch64 SConscript branches. On 2026-11-18 a Cheddar commit passed every Mac check and failed the car's boot build mid-drive ("openpilot failed to build"). A hand-run `compile_modeld.py` on the car also missed it, because it skipped the SCons chunk step.
  - Car safety: every heavy command (copy, build, test, delete) runs in its own process group at idle CPU and I/O priority and is stopped (TERM, then KILL) when `IsOnroad` turns 1 or after 40 minutes (the check prints `FAIL ... exit 3`), or on Ctrl-C. The copy is deleted only while the car is parked; a run left by a car that powered off is deleted by the first check that starts more than 60 minutes after it. If the payload transfer fails before the job starts, the receiver deletes its few files at once, without the parked check. It refuses to start (`exit 5`) unless `/data` keeps 5 GiB and 10 % free after the copy (about 1 GB) and 2 GiB for the build.
  - Writes on the car: only `/data/model_build_check.<run>`. `/data/openpilot` is only read. SCons runs with `--cache-disable` (even a read-only cache refreshes the timestamps in `/data/scons_cache`); tinygrad's cache (`HOME`), temporary files (`TMPDIR`), bytecode (`PYTHONPYCACHEPREFIX`), and test Params (`PARAMS_ROOT`, prefix `build_check`) stay in the run directory, also for the model compilers SCons runs (a `python3` first in `PATH` sets them, because SCons passes its commands only `PATH` and `PYTHONPATH`); the test runs with `--noconftest`, so the repository fixture writes nothing to `/dev/shm`.
  - Harmless writes left outside the run directory: SConstruct's `CacheDir()` creates and deletes `/data/scons_cache/config.lock` to read the cache's config (no cache entry is read or written); the C compilers SCons runs get no `TMPDIR`, so their temporary files go to `/tmp` (tmpfs) and the compilers delete them, as in the boot build.
  - It takes about 10 minutes when the build passes and 16 when it fails. The job runs detached on the car, so a dropped connection only delays the result; follow it with the `tail -f` command the check prints. It needs `uv` on the Mac (onnxruntime) and the car's HEAD commit in the local checkout.
  - Without `--device` it builds a CPU bundle on the Mac and runs the same tests: a fast first check, not the gate.
  - Known gap: `release/prepare_release.py` packages `selfdrive/modeld/models/*.pkl`, not the chunked bundle files (`*.pkl.chunk*`, `*.chunkmanifest`).

## Lint Commands
- Run all linters: `pre-commit run --all`
- Run ruff linter: `ruff check .`
- Run mypy type checker: `mypy .`

## Code Style Guidelines
- IMPORTANT: Never run automatic any automatic code formatting, only adjust the parts of the cod eyou are working on
- Python: Follow the ruff linting rules defined in pyproject.toml
- Line length: 160 characters maximum
- Python version: 3.11
- Use pytest for testing, not unittest
- Error handling: Prefer explicit error handling over generic try/except blocks
- PRs should have a clear purpose and be focused (keep them under 500 lines)
- Do not make arbitrary style changes to existing code
- Params safety: Before using a new Params key (`Params.get*` / `put*`), add it to the params key registry in `common/params_keys.h`; unregistered keys crash at runtime with `UnknownKeyName`.
- Params policy: Use Params only for real user-facing settings (UI toggles/controls). For temporary experiments, prefer explicit in-code constants/branches.

## Repository Organization
- `selfdrive/`: Core driving code
- `common/`: Shared utilities
- `system/`: System services
- `tools/`: Development tools
- `panda/`: Hardware interface
- `cereal/`: Messaging interfaces
