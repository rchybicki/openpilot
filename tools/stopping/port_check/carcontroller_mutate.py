#!/usr/bin/env python3
"""carcontroller_replay.py with one Hyundai carcontroller module constant overridden, to prove the byte comparison sees a change.
argv: out.pkl NAME=VALUE segment_dir/rlog... (VALUE is a Python literal)."""
import ast
import pickle
import sys

import carcontroller_replay
from opendbc.car.hyundai import carcontroller

if __name__ == "__main__":
  name, value = sys.argv[2].split('=', 1)
  setattr(carcontroller, name, ast.literal_eval(value))
  with open(sys.argv[1], 'wb') as f:
    pickle.dump(carcontroller_replay.replay(sys.argv[3:]), f)
