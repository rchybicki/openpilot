#!/usr/bin/env python3
import argparse
import os
import sys

from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.frogpilot.common.frogpilot_utilities import run_cmd

METADATA_SCRIPT = Path(BASEDIR) / "frogpilot/tinygrad_modeld/get_model_metadata.py"
TINYGRAD_REPO = Path(BASEDIR) / "tinygrad_repo"

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("onnx", help="Path to the ONNX model to compile")
  args = parser.parse_args()

  onnx_path = Path(args.onnx).resolve()
  out_path = onnx_path.with_name(f"{onnx_path.stem}_tinygrad.pkl")

  env = os.environ.copy()
  env["PYTHONPATH"] = f"{env.get('PYTHONPATH','')}:{TINYGRAD_REPO}"

  run_cmd([sys.executable, str(TINYGRAD_REPO / "examples/openpilot/compile3.py"), str(onnx_path), str(out_path)], "Model compiled successfully!", "Failed to compile the model...", env=env)
  run_cmd([sys.executable, str(METADATA_SCRIPT), str(onnx_path)], f"Successfully extracted metadata from {onnx_path.name}!", f"Failed to extract metadata from {onnx_path.name}...")

if __name__ == "__main__":
  sys.exit(main())
