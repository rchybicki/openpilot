import sys
import tempfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import phase8_rehearse as P

REVIEWED = 'f3a2cadec7'   # revision 1, REWORK from the 2026-11-02 Astra review

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


@pytest.mark.parametrize('item', [n for n in sorted(P.ITEMS) if n])
def test_reviewed_procedure_reproduces_each_act_on_item(base, item):
  res = P.run_item(item, P.doc_text(REVIEWED), base)
  assert res.verdict == 'UNSAFE', '\n'.join(res.evidence)


def test_ssh_never_leaves_the_sandbox(base):
  sb = P.Sandbox(base)
  r = sb.host('whence -p ssh')
  assert r.stdout.strip() == str(sb.root / 'hostshims/ssh')
  sb.update_config(ssh_down=True)
  assert sb.host('ssh $DEV true').returncode == 255
