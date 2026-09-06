#!/usr/bin/env python3
import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

UPSTREAM_POLICY = "selfdrive/modeld/models/driving_policy.onnx"
UPSTREAM_VISION = "selfdrive/modeld/models/driving_vision.onnx"

LFS_POINTER_PREFIX = "version https://git-lfs.github.com/spec/v1"


def fail(message: str, code: int = 1) -> None:
    print(f"[ERROR] {message}")
    sys.exit(code)


def run(cmd: list[str], cwd: Path | None = None, check: bool = True) -> subprocess.CompletedProcess:
    result = subprocess.run(cmd, cwd=cwd, text=True, capture_output=True)
    if check and result.returncode != 0:
        stderr = result.stderr.strip()
        stdout = result.stdout.strip()
        details = stderr or stdout or "(no output)"
        fail(f"Command failed: {' '.join(cmd)}\n{details}")
    return result


def run_stdout(cmd: list[str], cwd: Path | None = None) -> str:
    return run(cmd, cwd=cwd).stdout.strip()


def git_root() -> Path:
    root = run_stdout(["git", "rev-parse", "--show-toplevel"])
    if not root:
        fail("Unable to locate git repository root.")
    return Path(root)


def list_remotes() -> set[str]:
    remotes = run_stdout(["git", "remote"]).splitlines()
    return {remote.strip() for remote in remotes if remote.strip()}


def git_ls_files(pattern: str) -> list[str]:
    result = run(["git", "ls-files", "-z", pattern])
    entries = result.stdout.split("\0")
    return [entry for entry in entries if entry]


def resolve_target_path(repo_root: Path, override: str | None, pattern: str, label: str) -> Path:
    if override:
        candidate = (repo_root / override).resolve()
        if not candidate.exists():
            fail(f"{label} override path does not exist: {candidate}")
        return candidate

    matches = git_ls_files(pattern)
    if not matches:
        fail(f"Could not find {label} using pattern {pattern}. Use --target-{label} to override.")
    if len(matches) > 1:
        formatted = "\n".join(f"- {match}" for match in matches)
        fail(f"Multiple {label} targets found. Use --target-{label} to pick one.\n{formatted}")
    return (repo_root / matches[0]).resolve()


def is_commit_like(value: str) -> bool:
    return re.fullmatch(r"[0-9a-fA-F]{7,40}", value) is not None


def ensure_commit_exists(commit: str, remote: str) -> None:
    result = run(["git", "cat-file", "-e", f"{commit}^{{commit}}"], check=False)
    if result.returncode == 0:
        return
    run(["git", "fetch", remote, commit])
    result = run(["git", "cat-file", "-e", f"{commit}^{{commit}}"], check=False)
    if result.returncode != 0:
        fail(f"Commit {commit} not found after fetching from {remote}.")


def resolve_branch_ref(branch: str, default_remote: str, remotes: set[str]) -> tuple[str, str]:
    if "/" in branch:
        remote_candidate, branch_name = branch.split("/", 1)
        if remote_candidate in remotes:
            return remote_candidate, branch_name
    return default_remote, branch


def latest_model_commit(ref: str) -> str:
    commit = run_stdout([
        "git",
        "log",
        "-n",
        "1",
        "--format=%H",
        ref,
        "--",
        UPSTREAM_POLICY,
        UPSTREAM_VISION,
    ])
    if not commit:
        fail(f"No model update found on {ref} for {UPSTREAM_POLICY} / {UPSTREAM_VISION}.")
    return commit


def ensure_paths_in_commit(commit: str) -> None:
    for path in (UPSTREAM_POLICY, UPSTREAM_VISION):
        result = run(["git", "cat-file", "-e", f"{commit}:{path}"], check=False)
        if result.returncode != 0:
            fail(f"Commit {commit} does not contain {path}.")


def is_lfs_pointer(path: Path) -> bool:
    try:
        with path.open("rb") as handle:
            first_line = handle.readline().decode("utf-8", errors="replace").strip()
        return first_line.startswith(LFS_POINTER_PREFIX)
    except FileNotFoundError:
        fail(f"Expected file missing: {path}")
    return True


def pull_lfs(worktree: Path, remote: str) -> None:
    include_paths = f"{UPSTREAM_POLICY},{UPSTREAM_VISION}"
    run(["git", "lfs", "pull", remote, "-I", include_paths], cwd=worktree)


def copy_model(src: Path, dst: Path, label: str) -> None:
    if is_lfs_pointer(src):
        fail(f"{label} is still an LFS pointer after pull: {src}")
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    print(f"Copied {label}: {src} -> {dst}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Import upstream driving models into FrogPilot paths.",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--commit", help="Upstream commit hash with model update")
    group.add_argument("--branch", help="Upstream branch name to scan for latest model update")
    parser.add_argument("ref", nargs="?", help="Commit hash or branch name")
    parser.add_argument("--remote", default="openpilot", help="Git remote for upstream repo (default: openpilot)")
    parser.add_argument("--target-policy", help="Override FrogPilot policy ONNX path (repo-relative)")
    parser.add_argument("--target-vision", help="Override FrogPilot vision ONNX path (repo-relative)")
    parser.add_argument("--keep-worktree", action="store_true", help="Keep temp worktree for debugging")
    parser.add_argument("--dry-run", action="store_true", help="Print planned actions without copying")

    args = parser.parse_args()

    ref_input = args.commit or args.branch or args.ref
    if not ref_input:
        fail("Provide a commit hash or branch name via --commit, --branch, or positional ref.")

    repo_root = git_root()
    remotes = list_remotes()
    if args.remote not in remotes:
        fail(f"Remote '{args.remote}' not found. Available: {', '.join(sorted(remotes))}")

    policy_target = resolve_target_path(
        repo_root,
        args.target_policy,
        ":(glob)frogpilot/**/driving_policy.onnx",
        "policy",
    )
    vision_target = resolve_target_path(
        repo_root,
        args.target_vision,
        ":(glob)frogpilot/**/driving_vision.onnx",
        "vision",
    )

    if args.commit or (not args.branch and is_commit_like(ref_input)):
        commit = ref_input
        ensure_commit_exists(commit, args.remote)
    else:
        remote_name, branch_name = resolve_branch_ref(ref_input, args.remote, remotes)
        run(["git", "fetch", remote_name, branch_name])
        commit = latest_model_commit(f"{remote_name}/{branch_name}")

    ensure_paths_in_commit(commit)

    print(f"Using upstream commit: {commit}")
    print(f"Target policy path: {policy_target}")
    print(f"Target vision path: {vision_target}")

    if args.dry_run:
        print("Dry run enabled. Skipping worktree creation and copy.")
        return

    worktree_dir = Path(tempfile.mkdtemp(prefix="frogpilot-model-import-"))
    worktree_added = False

    try:
        run(["git", "worktree", "add", "--detach", str(worktree_dir), commit])
        worktree_added = True

        pull_lfs(worktree_dir, args.remote)

        policy_src = worktree_dir / UPSTREAM_POLICY
        vision_src = worktree_dir / UPSTREAM_VISION

        copy_model(policy_src, policy_target, "driving_policy.onnx")
        copy_model(vision_src, vision_target, "driving_vision.onnx")

        print("Model import completed.")
    finally:
        if worktree_added and not args.keep_worktree:
            run(["git", "worktree", "remove", "--force", str(worktree_dir)], check=False)
            if worktree_dir.exists():
                shutil.rmtree(worktree_dir, ignore_errors=True)


if __name__ == "__main__":
    main()
