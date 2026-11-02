#!/usr/bin/env bash
set -euo pipefail

# Increase the pip timeout to handle TimeoutError
export PIP_DEFAULT_TIMEOUT=200

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
ROOT="$DIR"/../
cd "$ROOT"

if ! command -v "uv" > /dev/null 2>&1; then
  echo "installing uv..."
  curl -LsSf --retry 5 --retry-delay 5 --retry-all-errors https://astral.sh/uv/install.sh | sh
  UV_BIN="$HOME/.local/bin"
  PATH="$UV_BIN:$PATH"
fi

echo "updating uv..."
# ok to fail, can also fail due to installing with brew
uv self update || true

echo "installing python packages..."
uv sync --frozen --all-extras
source .venv/bin/activate

if [[ "$(uname)" == 'Darwin' ]]; then
  touch "$ROOT"/.env
  grep -qxF "# msgq doesn't work on mac" "$ROOT"/.env || echo "# msgq doesn't work on mac" >> "$ROOT"/.env
  grep -qxF "export ZMQ=1" "$ROOT"/.env || echo "export ZMQ=1" >> "$ROOT"/.env
  # Keep macOS fork checks enabled. Workers must use spawn, not bypass the check.
  python3 - "$ROOT/.env" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
lines = path.read_text().splitlines(keepends=True)
path.write_text(''.join(line for line in lines if line.strip() != 'export OBJC_DISABLE_INITIALIZE_FORK_SAFETY=YES'))
PY
fi
