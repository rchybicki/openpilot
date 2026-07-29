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
- Deploy updates right away even when the vehicle is on-road; stopping or going off-road is not required. The script stages the update and detaches a background supervisor that survives the SSH session closing.
  - On-road, the UI shows a passive `Update Ready` notice. Do not click it; clicks intentionally do nothing.
  - To apply the update on-road, press and release the cruise-main button once to fully turn SCC/cruise off, then keep cruise off. The supervisor waits through a release dwell and reboots only after openpilot is disengaged and verified stock SCC takeover is fresh and fault-free.
  - The same flow works in Park with the ignition on (same cruise-off + verification gates; only Reverse/Neutral block the handoff). If cruise was never on while parked, the gates are already satisfied and the restart proceeds after verification without a button press.
  - If the vehicle is already off-road, the update still reboots immediately through the existing parked path.
  - If live SCC takeover cannot be verified, the supervisor fails safe and leaves the update staged for an off-road reboot instead of forcing an on-road restart.
  - Cancel a pending reboot: `touch /data/fullupdate_reboot.cancel` (graceful; update stays staged, applies on next reboot/deploy).
  - Watch a pending reboot: `tail -f /data/fullupdate_reboot.log`.
  - A change to `fullupdate.sh` itself takes effect on the SECOND deploy after it lands (the deploy that pulls it still runs the previously-loaded script).
- After deploy, verify device commit hash:
  - `ssh -o BatchMode=yes -o ConnectTimeout=8 comma 'cd /data/openpilot && git rev-parse --short HEAD'`
  - fallback: `ssh -o BatchMode=yes -o ConnectTimeout=8 commawifi 'cd /data/openpilot && git rev-parse --short HEAD'`
- If device is temporarily unavailable after update, retry SSH verification until it comes back.

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
- Params safety: Before using a new Params key (`Params.get*` / `put*`), add it to the params key registry in `common/params.cc`; unregistered keys crash at runtime with `UnknownKeyName`.
- Params policy: Use Params only for real user-facing settings (UI toggles/controls). For temporary experiments, prefer explicit in-code constants/branches.

## Repository Organization
- `selfdrive/`: Core driving code
- `common/`: Shared utilities
- `system/`: System services
- `tools/`: Development tools
- `panda/`: Hardware interface
- `cereal/`: Messaging interfaces
