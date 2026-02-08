# AGENTS.md

This file provides guidance to coding agents when working with code in this repository.

## Build Commands
- Full build: `scons -j$(nproc)`
- Build specific component: `scons -j8 selfdrive/ui/` or `cd selfdrive/ui/ && scons -u -j8`
- Install dependencies: `tools/ubuntu_setup.sh` or `tools/mac_setup.sh`

## Test Commands
- Run all tests: `pytest .`
- Run specific test: `pytest path/to/test_file.py::test_function_name`
- Run tests for specific component: `cd system/loggerd && pytest .`

## Device Update Workflow
- Preferred SSH profile for local device updates: `commawifi` (use `comma` only if explicitly needed and reachable).
- Deploy the currently pushed branch on-device with:
  - `ssh -tt commawifi 'cd /data/openpilot && ./fullupdate.sh'`
- Note: `fullupdate.sh` can close SSH during restart/relaunch; this is expected.
- After deploy, verify device commit hash:
  - `ssh -o BatchMode=yes -o ConnectTimeout=8 commawifi 'cd /data/openpilot && git rev-parse --short HEAD'`
- If device is temporarily unavailable after update, retry SSH verification until it comes back.

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

## Repository Organization
- `selfdrive/`: Core driving code
- `common/`: Shared utilities
- `system/`: System services
- `tools/`: Development tools
- `panda/`: Hardware interface
- `cereal/`: Messaging interfaces
