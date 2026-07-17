import os

from openpilot.system.manager.process import get_process_diagnostics


def test_get_process_diagnostics(tmp_path):
  pid = 123
  tid = 456
  task_root = tmp_path / "proc" / str(pid) / "task" / str(tid)
  task_root.mkdir(parents=True)

  files = {
    "comm": "ui\n",
    "status": "Name:\tui\nState:\tD (disk sleep)\nPid:\t456\nCpus_allowed_list:\t0-7\nvoluntary_ctxt_switches:\t12\n",
    "wchan": "do_wait\n",
    "syscall": "202 0 0 0 0 0 0\n",
    "schedstat": "100 200 3\n",
    "stack": "[<0>] do_wait+0x1/0x2\n",
  }
  for name, content in files.items():
    (task_root / name).write_text(content)

  phase_prefix = str(tmp_path / "wd_phase_")
  (tmp_path / f"wd_phase_{pid}").write_bytes(b"paint_camera\0ignored")

  diagnostics = get_process_diagnostics(pid, proc_root=str(tmp_path / "proc"), phase_fn_prefix=phase_prefix)

  assert diagnostics["phase"] == "paint_camera"
  assert diagnostics["thread_count"] == 1
  assert diagnostics["threads_captured"] == 1
  assert diagnostics["truncated"] is False
  assert diagnostics["threads"][0] == {
    "tid": tid,
    "comm": "ui",
    "status": "State:\tD (disk sleep) | Pid:\t456 | Cpus_allowed_list:\t0-7 | voluntary_ctxt_switches:\t12",
    "wchan": "do_wait",
    "syscall": "202 0 0 0 0 0 0",
    "schedstat": "100 200 3",
    "stack": "[<0>] do_wait+0x1/0x2",
  }


def test_get_process_diagnostics_handles_missing_process(tmp_path):
  diagnostics = get_process_diagnostics(os.getpid(), proc_root=str(tmp_path / "missing"), phase_fn_prefix=str(tmp_path / "missing_phase_"))

  assert diagnostics["thread_count"] == 0
  assert diagnostics["threads"] == []
  assert diagnostics["task_error"].startswith("<FileNotFoundError:")
