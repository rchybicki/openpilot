#!/usr/bin/env python3
"""Read and tune stop-related FrogPilot/openpilot params on a remote comma device."""

from __future__ import annotations

import argparse
import base64
import json
import shlex
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

DEFAULT_SETTINGS_DIR = Path.home() / ".comma" / "stopping_behavior" / "settings"
DEFAULT_PARAM_DIRS = ["/data/params/d", "/persist/params/d"]
DEFAULT_HOST = "comma"
FALLBACK_HOST = "commawifi"


@dataclass(frozen=True)
class ParamSpec:
  key: str
  kind: str  # bool | float
  minimum: float | None = None
  maximum: float | None = None
  read_only: bool = False
  description: str = ""


PARAM_SPECS: dict[str, ParamSpec] = {
  "AdvancedLongitudinalTune": ParamSpec("AdvancedLongitudinalTune", "bool", description="Enable advanced longitudinal tuning"),
  "LongitudinalTune": ParamSpec("LongitudinalTune", "bool", description="Enable longitudinal tuning controls"),
  "HumanAcceleration": ParamSpec("HumanAcceleration", "bool", description="Use human-like acceleration profile"),
  "ForceStops": ParamSpec("ForceStops", "bool", description="Enable force-stop behavior on detected lights/signs"),
  "LongitudinalActuatorDelay": ParamSpec("LongitudinalActuatorDelay", "float", 0.0, 1.0, description="Longitudinal actuator delay"),
  "MaxDesiredAcceleration": ParamSpec("MaxDesiredAcceleration", "float", 0.1, 4.0, description="Max commanded acceleration"),
}


def utc_now_iso() -> str:
  return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def parse_bool(text: str) -> bool:
  normalized = text.strip().lower()
  if normalized in {"1", "true", "yes", "on"}:
    return True
  if normalized in {"0", "false", "no", "off"}:
    return False
  raise ValueError(f"Invalid boolean value: {text}")


def parse_key_value(pair: str) -> tuple[str, str]:
  if "=" not in pair:
    raise ValueError(f"Expected KEY=VALUE format, got: {pair}")
  key, value = pair.split("=", 1)
  key = key.strip()
  value = value.strip()
  if not key:
    raise ValueError(f"Missing key in pair: {pair}")
  return key, value


def validate_assignment(key: str, raw_value: str) -> Any:
  spec = PARAM_SPECS.get(key)
  if spec is None:
    raise ValueError(f"Unknown key: {key}")
  if spec.read_only:
    raise ValueError(f"Key is read-only: {key}")

  if spec.kind == "bool":
    return parse_bool(raw_value)

  if spec.kind == "float":
    value = float(raw_value)
    if spec.minimum is not None and value < spec.minimum:
      raise ValueError(f"{key} below minimum {spec.minimum}: {value}")
    if spec.maximum is not None and value > spec.maximum:
      raise ValueError(f"{key} above maximum {spec.maximum}: {value}")
    return value

  raise ValueError(f"Unsupported key type for {key}: {spec.kind}")


def build_remote_shell(payload_b64: str, param_dirs: list[str]) -> str:
  dirs = " ".join(shlex.quote(path) for path in param_dirs)
  return f"""
set -e
PAYLOAD_B64={shlex.quote(payload_b64)} python3 - <<'PY'
import base64
import json
import os
from datetime import datetime, timezone

payload = json.loads(base64.b64decode(os.environ["PAYLOAD_B64"]).decode("utf-8"))
candidate_dirs = payload.get("param_dirs", [])
params_dir = next((item for item in candidate_dirs if os.path.isdir(item)), None)


def read_value(key: str, kind: str):
  path = os.path.join(params_dir, key)
  out = {{"exists": False, "path": path, "raw": None, "value": None}}
  if not params_dir:
    out["error"] = "No params directory found"
    return out

  if not os.path.isfile(path):
    return out

  try:
    raw = open(path, "rb").read().decode("utf-8", "replace")
    clean = raw.strip()
    out["exists"] = True
    out["raw"] = raw
    if kind == "bool":
      out["value"] = clean.lower() in {{"1", "true", "yes", "on"}}
    elif kind == "float":
      out["value"] = float(clean) if clean else 0.0
    else:
      out["value"] = raw
  except Exception as exc:
    out["error"] = str(exc)
  return out


def write_value(key: str, kind: str, value):
  if not params_dir:
    raise RuntimeError("No params directory found")

  path = os.path.join(params_dir, key)
  tmp_path = f"{{path}}.tmp_codex"

  if kind == "bool":
    encoded = "1" if bool(value) else "0"
  elif kind == "float":
    encoded = f"{{float(value):.6f}}"
  else:
    encoded = str(value)

  with open(tmp_path, "wb") as handle:
    handle.write(encoded.encode("utf-8"))
  os.replace(tmp_path, path)


result = {{
  "timestamp_utc": datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
  "params_dir": params_dir,
  "action": payload.get("action", "snapshot"),
  "values": {{}},
  "applied": {{}},
  "errors": [],
}}

keys = payload.get("keys", {{}})
action = payload.get("action", "snapshot")

if action == "set":
  assignments = payload.get("assignments", {{}})
  for key, value in assignments.items():
    kind = keys.get(key, {{}}).get("kind", "float")
    try:
      write_value(key, kind, value)
      result["applied"][key] = value
    except Exception as exc:
      result["errors"].append(f"{{key}}: {{exc}}")

for key, spec in keys.items():
  kind = spec.get("kind", "float")
  result["values"][key] = read_value(key, kind)

print(json.dumps(result, sort_keys=True))
PY
""".strip()


def run_remote_payload(host: str, payload: dict[str, Any], connect_timeout: int, param_dirs: list[str]) -> dict[str, Any]:
  payload = dict(payload)
  payload["param_dirs"] = param_dirs
  payload_b64 = base64.b64encode(json.dumps(payload).encode("utf-8")).decode("utf-8")
  shell_script = build_remote_shell(payload_b64, param_dirs)
  cmd = [
    "ssh",
    "-o",
    "BatchMode=yes",
    "-o",
    f"ConnectTimeout={connect_timeout}",
    host,
    "sh",
    "-lc",
    shell_script,
  ]

  result = subprocess.run(cmd, capture_output=True, text=True)
  if result.returncode != 0:
    message = result.stderr.strip() or result.stdout.strip() or "unknown ssh error"
    raise RuntimeError(message)

  stdout = result.stdout.strip()
  if not stdout:
    raise RuntimeError("remote command returned empty output")

  def parse_json_output(text: str) -> dict[str, Any]:
    try:
      parsed = json.loads(text)
      if isinstance(parsed, dict):
        return parsed
      raise RuntimeError("remote JSON payload was not an object")
    except json.JSONDecodeError:
      pass

    lines = [line.strip() for line in text.splitlines() if line.strip()]
    for start in range(len(lines) - 1, -1, -1):
      candidate = "\n".join(lines[start:])
      try:
        parsed = json.loads(candidate)
      except json.JSONDecodeError:
        continue
      if isinstance(parsed, dict):
        return parsed
      raise RuntimeError("remote JSON payload was not an object")

    snippet = text if len(text) <= 500 else text[:500] + "..."
    raise RuntimeError(f"failed to parse remote JSON output; output={snippet}")

  try:
    return parse_json_output(stdout)
  except RuntimeError:
    raise


def serialize_specs(keys: list[str]) -> dict[str, Any]:
  output: dict[str, Any] = {}
  for key in keys:
    spec = PARAM_SPECS[key]
    output[key] = {
      "kind": spec.kind,
      "minimum": spec.minimum,
      "maximum": spec.maximum,
      "read_only": spec.read_only,
    }
  return output


def build_snapshot_path(output: str | None, settings_dir: Path, host: str) -> Path:
  if output:
    return Path(output).expanduser()

  settings_dir.mkdir(parents=True, exist_ok=True)
  stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
  return settings_dir / f"stop_settings_{host}_{stamp}.json"


def save_json(path: Path, payload: dict[str, Any]) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def print_snapshot_table(snapshot: dict[str, Any]) -> None:
  values = snapshot.get("values", {}) if isinstance(snapshot.get("values", {}), dict) else {}
  host = snapshot.get("host", "unknown")
  ssh_host = snapshot.get("ssh_host")
  host_label = f"{host}"
  if ssh_host and ssh_host != host:
    host_label += f" (ssh={ssh_host})"
  print(f"[settings] host={host_label} params_dir={snapshot.get('params_dir', '?')}")
  for key in sorted(values):
    item = values.get(key, {})
    value = item.get("value")
    raw = item.get("raw")
    suffix = ""
    if item.get("error"):
      suffix = f" (error: {item['error']})"
    elif not item.get("exists", False):
      suffix = " (unset)"
    print(f"[settings] {key}={value} raw={raw}{suffix}")


def list_keys() -> int:
  print("Stop-related tunable keys")
  for key in sorted(PARAM_SPECS):
    spec = PARAM_SPECS[key]
    bounds = ""
    if spec.kind == "float":
      low = "-inf" if spec.minimum is None else str(spec.minimum)
      high = "inf" if spec.maximum is None else str(spec.maximum)
      bounds = f", range=[{low}, {high}]"
    ro = ", read-only" if spec.read_only else ""
    print(f"- {key}: type={spec.kind}{bounds}{ro} -- {spec.description}")
  return 0


def do_snapshot(args: argparse.Namespace) -> int:
  keys = sorted(PARAM_SPECS)
  payload = {
    "action": "snapshot",
    "keys": serialize_specs(keys),
  }

  ssh_host = args.host
  try:
    snapshot = run_remote_payload(ssh_host, payload, args.connect_timeout, args.params_dir)
  except RuntimeError as exc:
    if args.host == DEFAULT_HOST:
      snapshot = run_remote_payload(FALLBACK_HOST, payload, args.connect_timeout, args.params_dir)
      ssh_host = FALLBACK_HOST
      print(f"[settings] {DEFAULT_HOST} unavailable, falling back to {FALLBACK_HOST}", file=sys.stderr)
    else:
      raise
  snapshot["host"] = args.host
  snapshot["ssh_host"] = ssh_host
  snapshot["captured_utc"] = utc_now_iso()

  output_path = build_snapshot_path(args.output, Path(args.settings_dir).expanduser(), args.host)
  save_json(output_path, snapshot)
  print_snapshot_table(snapshot)
  print(f"[settings] snapshot: {output_path}")
  return 0


def do_set(args: argparse.Namespace) -> int:
  if not args.set:
    raise ValueError("No assignments provided. Use --set KEY=VALUE")

  assignments: dict[str, Any] = {}
  for pair in args.set:
    key, raw_value = parse_key_value(pair)
    assignments[key] = validate_assignment(key, raw_value)

  keys_to_read = sorted(set(assignments.keys()) | set(args.include_key))
  payload = {
    "action": "set" if not args.dry_run else "snapshot",
    "keys": serialize_specs(keys_to_read),
    "assignments": assignments,
  }

  ssh_host = args.host
  try:
    response = run_remote_payload(ssh_host, payload, args.connect_timeout, args.params_dir)
  except RuntimeError as exc:
    if args.host == DEFAULT_HOST:
      response = run_remote_payload(FALLBACK_HOST, payload, args.connect_timeout, args.params_dir)
      ssh_host = FALLBACK_HOST
      print(f"[settings] {DEFAULT_HOST} unavailable, falling back to {FALLBACK_HOST}", file=sys.stderr)
    else:
      raise
  response["host"] = args.host
  response["ssh_host"] = ssh_host
  response["captured_utc"] = utc_now_iso()
  response["requested_assignments"] = assignments
  response["dry_run"] = bool(args.dry_run)

  output_path = build_snapshot_path(args.output, Path(args.settings_dir).expanduser(), args.host)
  save_json(output_path, response)
  print_snapshot_table(response)
  if args.dry_run:
    print("[settings] dry-run only, no values were written")
  else:
    applied = response.get("applied", {})
    print(f"[settings] applied keys: {', '.join(sorted(applied)) if applied else 'none'}")
  print(f"[settings] result: {output_path}")

  if response.get("errors"):
    print(f"[settings] remote errors: {response['errors']}", file=sys.stderr)
    return 1
  return 0


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(description="Read and tune stop-related settings on a remote comma device")
  sub = parser.add_subparsers(dest="command", required=True)

  list_parser = sub.add_parser("list", help="List supported stop-related keys")
  list_parser.set_defaults(handler=lambda _args: list_keys())

  for name, help_text in (("snapshot", "Read current stop-related settings from device"), ("set", "Write one or more settings")):
    sub_parser = sub.add_parser(name, help=help_text)
    sub_parser.add_argument(
      "--host",
      default=DEFAULT_HOST,
      help=f"SSH host alias (defaults to {DEFAULT_HOST}; falls back to {FALLBACK_HOST} when {DEFAULT_HOST} is unreachable)",
    )
    sub_parser.add_argument("--params-dir", action="append", default=list(DEFAULT_PARAM_DIRS),
                            help="Remote params directory candidate (repeatable)")
    sub_parser.add_argument("--connect-timeout", type=int, default=8, help="SSH connect timeout in seconds")
    sub_parser.add_argument("--settings-dir", default=str(DEFAULT_SETTINGS_DIR),
                            help=f"Directory for snapshots/results. Default: {DEFAULT_SETTINGS_DIR}")
    sub_parser.add_argument("--output", default=None, help="Explicit output JSON path")

  set_parser = sub.choices["set"]
  set_parser.add_argument("--set", action="append", default=[], help="Assignment in KEY=VALUE format (repeatable)")
  set_parser.add_argument("--include-key", action="append", default=[], help="Extra key to include in readback")
  set_parser.add_argument("--dry-run", action="store_true", help="Validate and read, but do not write")

  sub.choices["snapshot"].set_defaults(handler=do_snapshot)
  set_parser.set_defaults(handler=do_set)

  return parser


def main() -> int:
  parser = build_parser()
  args = parser.parse_args()

  try:
    return args.handler(args)
  except Exception as exc:
    print(f"[settings] error: {exc}", file=sys.stderr)
    return 2


if __name__ == "__main__":
  sys.exit(main())
