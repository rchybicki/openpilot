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
                         [(REVISION_3, n) for n in (12, 13)])
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
