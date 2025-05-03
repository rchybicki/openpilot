# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands
- Full build: `scons -j$(nproc)`
- Build specific component: `scons -j8 selfdrive/ui/` or `cd selfdrive/ui/ && scons -u -j8`
- Install dependencies: `tools/ubuntu_setup.sh` or `tools/mac_setup.sh`

## Test Commands
- Run all tests: `pytest .`
- Run specific test: `pytest path/to/test_file.py::test_function_name`
- Run tests for specific component: `cd system/loggerd && pytest .`

## Lint Commands
- Run all linters: `pre-commit run --all`
- Run ruff linter: `ruff check .`
- Run mypy type checker: `mypy .`

## Code Style Guidelines
- Python: Follow the ruff linting rules defined in pyproject.toml
- Line length: 160 characters maximum
- Python version: 3.11
- Use pytest for testing, not unittest
- Imports: Use openpilot module prefix (e.g., `from openpilot.selfdrive` not just `from selfdrive`)
- Error handling: Prefer explicit error handling over generic try/except blocks
- PRs should have a clear purpose and be focused (keep them under 500 lines)
- Do not make arbitrary style changes to existing code

## Repository Organization
- `selfdrive/`: Core driving code
- `common/`: Shared utilities
- `system/`: System services
- `tools/`: Development tools
- `panda/`: Hardware interface
- `cereal/`: Messaging interfaces