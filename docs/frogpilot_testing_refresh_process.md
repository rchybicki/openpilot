# FrogPilot Testing Refresh Process

Use this process when rebasing this fork onto the current `FrogPilot-Testing`
branch while preserving local changes as a small, reviewable commit stack.

As of 2026-05-31, this fork's active branch was based on the FrogPilot Testing
family, not the older FrogPilot release branch. Re-check this every time because
FrogPilot force-pushes its public branches.

## When to Use This

- Use this for a full FrogPilot Testing refresh.
- Use `skills/git-commit-cleanup` first when the current branch has many local
  commits that need to be grouped or squashed.
- Use `skills/frogpilot-model-import` instead of cherry-picking when the only
  change is a model asset import.

## Ground Rules

- Do not start with an unexplained dirty worktree. Stash, commit, or explicitly
  exclude local dirt before creating the backup.
- Always quote branch names that contain `!` in shell commands.
- Treat `frogpilot/FrogPilot-Testing` as replaceable history. Fetch with a forced
  refspec before comparing or branching from it.
- Do not mechanically accept "ours" for conflicts. Keep the new upstream shape
  and reapply the local behavior narrowly.
- When adding or keeping Params keys, update the registry used by the target
  branch. Current Testing uses `common/params_keys.h`; older branches may use
  `common/params.cc`.
- Do not deploy to the device until the refreshed branch has been reviewed,
  tested, pushed, and the user explicitly asks for deployment.

## Phase 1: Establish the Current State

```sh
git status --short
git remote -v
git fetch origin
git fetch frogpilot '+refs/heads/*:refs/remotes/frogpilot/*' --prune
git rev-parse --short HEAD
git log -1 --oneline
git merge-base HEAD frogpilot/FrogPilot-Testing
git show -s --format='%h %ci %s' frogpilot/FrogPilot-Testing
```

Record the branch name, current commit, merge base, and the fetched Testing head
in your notes before changing history.

## Phase 2: Create a Backup

Use a timestamped backup branch from the exact current head.

```sh
backup="backup/!my-fp-new_$(date +%Y-%m-%d_%H-%M)_pre-fp-testing-refresh"
git branch "$backup" HEAD
git push origin "$backup"
```

If the tree is dirty, decide what to do before the backup. The backup branch only
protects committed history, not unstaged local files.

## Phase 3: Clean Up Local Commits

Create a cleanup branch from the current branch so the original branch remains
untouched after the backup.

```sh
cleanup="codex/fp-testing-cleanup-$(date +%Y%m%d)"
git switch -c "$cleanup"
base="$(git merge-base HEAD frogpilot/FrogPilot-Testing)"
git log --reverse --oneline "$base"..HEAD
```

Then use `skills/git-commit-cleanup` with that base:

```sh
git rebase -i --rebase-merges "$base"
```

Group commits by behavior, not by the order they happened locally. A useful
ordering for this fork is:

1. Base compatibility and build fixes.
2. Params/schema/config changes.
3. Model/runtime integration changes.
4. Controls and driving behavior changes.
5. FrogPilot UI/settings changes.
6. Navigation, mapd, and speed-limit changes.
7. Device update workflow, scripts, tests, and docs.

Drop superseded fixups, revert/reapply noise, obsolete model commits, and commits
that only compensated for old upstream code that no longer exists.

Verify the cleanup before refreshing:

```sh
git range-diff "$backup"...HEAD
git log --oneline --reverse "$base"..HEAD
git diff --check "$base"..HEAD
```

Run focused tests for touched areas when practical.

## Phase 4: Branch From Current FrogPilot Testing

Create the refreshed branch from the freshly fetched Testing head.

```sh
refresh="codex/fp-testing-refresh-$(date +%Y%m%d)"
git switch -c "$refresh" frogpilot/FrogPilot-Testing
```

Confirm the target branch version and head before applying local changes.

```sh
git show -s --format='%h %ci %s' HEAD
sed -n '1,40p' common/version.h
```

## Phase 5: Cherry-Pick the Cleaned Batches

From the cleanup branch, list the cleaned local commits:

```sh
git log --reverse --format='%H %s' "$base".."$cleanup"
```

Cherry-pick one logical batch at a time. If the cleanup branch already contains
one commit per batch, cherry-pick those commits directly:

```sh
git cherry-pick <commit>
```

If a batch still has multiple commits, apply them without committing and create a
new consolidated commit:

```sh
git cherry-pick --no-commit <first>^..<last>
git status --short
git commit
```

After each batch:

```sh
git status --short
git diff --check HEAD~1..HEAD
```

Resolve conflicts in favor of the current Testing architecture, then reapply only
the local behavior that still matters. For model assets, do not blindly
cherry-pick old binaries. Use `skills/frogpilot-model-import` if the target
Testing branch needs a model import.

## Phase 6: Verify the Refreshed Branch

Compare the cleaned stack to the refreshed stack:

```sh
git range-diff "$base".."$cleanup" frogpilot/FrogPilot-Testing..HEAD
git log --oneline --reverse frogpilot/FrogPilot-Testing..HEAD
git diff --stat frogpilot/FrogPilot-Testing..HEAD
git diff --check frogpilot/FrogPilot-Testing..HEAD
```

Run validation scaled to the touched areas. Examples:

```sh
python -m compileall <changed-python-package>
pytest <changed-test-or-package>
scons -j8 selfdrive/ui/
```

Before pushing, check that only intended files are dirty:

```sh
git status --short
```

Then push the refreshed branch:

```sh
git push -u origin HEAD
```

## Device Deployment

Deploy only after an explicit user request. Follow the repository `AGENTS.md`
device update workflow and verify the device commit after `fullupdate.sh`.
