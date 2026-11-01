"""Standard stopping simulator runner (README.md). Large data lives in SIM_HOME, never in the repo."""
import os
from pathlib import Path

SIM_HOME = Path(os.environ.get('STOP_SIM_HOME', Path.home() / '.route_sync/work/sim'))
