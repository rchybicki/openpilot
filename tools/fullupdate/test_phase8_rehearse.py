import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import phase8_rehearse as P

REVISION_1 = 'f3a2cadec7'   # REWORK, Astra review 2026-11-02: items 1-7
REVISION_2 = 'ede4f0ad8b'   # REWORK, Astra re-review 2026-11-02: items 8-11
REVISION_3 = '7b11e2a78e'   # REWORK, Astra round 3 2026-11-02: items 12-13
REVISION_4 = '50a498eaa2'   # approved 2026-11-02; stopped at box 4.4 on the device 2026-11-14: item 14
REVISION_5 = 'eb52f87ff7'   # REWORK, Astra round 5 2026-11-14 (the filter by command line hid an identical second run): item 14
REVISION_6 = '39a689761c'   # approved 2026-11-14; the first boot crash-looped (15-slot mapd), no tiles for the new mapd: items 15-21
REVISION_7 = '0a2301158f'   # REWORK, Astra round 7 2026-11-14 (tile moves, settings keys, symlinks): items 22, 25, 26
REVISION_8 = 'b9077abd7a'   # REWORK, Astra round 8 2026-11-14 (a mount, the tile renames, the rollback set, settings_version): 16, 19, 22, 27-29

pytestmark = pytest.mark.skipif(bool(P.prerequisites()), reason=f'missing: {", ".join(P.prerequisites())}')


@pytest.fixture(scope='module')
def base():
  scratch = Path.home() / '.route_sync/work/fp1031/port-scratch'
  with tempfile.TemporaryDirectory(prefix='p8_test_', dir=scratch if scratch.is_dir() else None) as d:
    yield Path(d)


def test_every_block_parses():
  counts, errors = P.syntax(P.doc_text(None))
  assert errors == []
  assert counts['zsh'] > 40 and counts['bash'] > 20 and counts['python'] == 3


@pytest.mark.parametrize('item', sorted(P.ITEMS))
def test_revised_procedure_is_safe(base, item):
  res = P.run_item(item, P.doc_text(None), base)
  assert res.verdict == 'SAFE', '\n'.join(res.evidence)


@pytest.mark.parametrize('rev,item', [(REVISION_1, n) for n in range(1, 8)] + [(REVISION_2, n) for n in range(8, 12)] +
                         [(REVISION_3, n) for n in (12, 13)] + [(REVISION_4, 14), (REVISION_5, 14)] +
                         [(REVISION_6, n) for n in (15, 16, 18, 19, 20, 21)] + [(REVISION_7, n) for n in (22, 25, 26)] +
                         [(REVISION_8, n) for n in (16, 19, 22, 27, 28, 29)])
def test_each_reviewed_revision_reproduces_its_act_on_items(base, rev, item):
  res = P.run_item(item, P.doc_text(rev), base)
  assert res.verdict == 'UNSAFE', '\n'.join(res.evidence)


def test_ssh_never_leaves_the_sandbox(base):
  sb = P.Sandbox(base)
  r = sb.host('whence -p ssh')
  assert r.stdout.strip() == str(sb.root / 'hostshims/ssh')
  sb.update_config(ssh_down=True)
  assert sb.host('ssh $DEV true').returncode == 255


def test_a_timed_out_block_leaves_no_process(base):
  sb = P.Sandbox(base, config={'ssh_stall': True})
  with pytest.raises(subprocess.TimeoutExpired):
    sb.host('ssh $DEV true | tee /dev/null', timeout=2)
  left = subprocess.run(['ps', '-axo', 'args'], capture_output=True, text=True).stdout
  assert str(sb.root) not in left
