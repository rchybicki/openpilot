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

`git merge-base` is unreliable here because FrogPilot rebases its branches; find the
true base as described in `skills/git-commit-cleanup` (last upstream-authored commit).
Then use that skill: interactive rebase for short stacks, the blame-based feature rebuild
(`scripts/regroup_by_feature.py`) for long interleaved ones.

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

## Lessons From the 2026-10-31 Refresh

The 2026-10-31 refresh replaced phases 3 to 5 with a regroup, a removal step and
a batched port. Its records are in `docs/frogpilot_refresh_20261031/` (plan,
decisions, inventories, closure).

- **True base and target.** The Testing tip is "Compile FrogPilot": compiled
  output with the sources stripped. The source target is
  `frogpilot/FrogPilot-Testing^`; check that its tree equals another FrogPilot
  source ref (`MAKE-PRS-HERE` was identical). Our base is the old Testing
  source commit below our first own commit, not `git merge-base`. Pin both on
  `origin` before changing history.
- **Regroup v2.** `skills/git-commit-cleanup` attributes every line by blame.
  `SPLITS` and `LINE_RULES` split coarse commits that blame cannot split.
  Features to remove go last as `drop_*` units, so their removal is a plain
  revert. Its `rewrites` keeps a line that a drop unit wrote over a kept line.
  The rebuild must print `TREE_IDENTICAL`. Intermediate commits do not
  compile: blame splits shared lines.
- **Port from the final state, in batches.** For each file:
  - Unchanged upstream (OLD equals NEW): take ours.
  - Ours only: carry it whole.
  - Otherwise: a 3-way merge (`git merge-file`, base OLD, ours = the removal
    tip) onto the new base.

  Resolve toward the new upstream shape and reapply our behaviour. Merge one
  source commit's delta at a time when a batch must leave other features'
  lines out. One commit per batch; stop on a red batch.
- **The schema-decode trap.** The new base renumbered the FrogPilot structs in
  `custom.capnp`; `log.capnp` and `car.capnp` kept their ordinals. A log
  recorded by the old base decodes without an error and with wrong FrogPilot
  values. `tools/stopping/sim/recorded.py` decodes the corpus with the
  recording schema. A log written after the device move uses the new schema,
  so any tool that mixes old and new logs must choose the schema per log.
  Messages cross the same boundary live on the first deploy: the old card
  still runs when the new updater supervisor reads its messages.
- **`Params::clearAll` deletes stored keys the registry lacks.** It runs at
  every manager start. Before removing a registry row, check whether a stored
  value must survive. Keep every PERSISTENT row the device uses; the
  `migrate_params` probe on a copy of the device store shows the losses.
- **Stubbed updater harness.** `tools/fullupdate/harness.py` runs
  `fullupdate.sh` with stubbed git, sudo, flock, setsid and sleep, against a
  car stand-in. It covers the first deploy (the old script and old compiled
  modules after the reset) and later deploys. Every reboot must be off-road
  or a verified handoff. `fullupdate.sh` has no dry-run mode: never run the
  live script on a host.
- **Equivalence replay.** `tools/stopping/port_check` replays a logged route
  through radard, the planner, LongControl, the FrogPilot planner and
  controlsd in both trees. The comparison is exact; a mutation proves each
  stage can see a change. Run it after every batch that touches driving, then
  the full sim verdict.
- **Restarts with the car on.** Every restart request (`DoReboot`,
  `DoSoftReboot`, The Pond, UI buttons) must reach `fullupdate.sh
  __safe_reboot` before `manager_cleanup`. New upstream reboot paths appear
  in each refresh; search for them.

## Device Deployment

Deploy only after an explicit user request. Follow the repository `AGENTS.md`
device update workflow and verify the device commit after `fullupdate.sh`.
