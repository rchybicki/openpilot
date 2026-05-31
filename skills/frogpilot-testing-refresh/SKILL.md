# FrogPilot Testing Refresh Skill

Use this skill when refreshing this fork onto the current upstream
`frogpilot/FrogPilot-Testing` branch. This is the full branch-refresh workflow:
backup current history, clean up local commits, branch from current Testing, and
cherry-pick the cleaned local batches.

Do not use this skill for a single model import. Use `skills/frogpilot-model-import`
for that narrower workflow.

## Required Context

- Work from `/Users/radoslawchybicki/Repos/openpilot-rch` unless the user gives a
  different checkout.
- Respect `AGENTS.md`.
- This repo often has branch names containing `!`; quote those branch names in
  shell commands.
- FrogPilot force-pushes public branches, so fetch with a forced refspec before
  making claims about the current Testing head.

## Workflow

1. Read `docs/frogpilot_testing_refresh_process.md`.
2. Check the worktree with `git status --short`. Do not hide pre-existing user
   dirt in the refresh commits.
3. Fetch `origin` and forced FrogPilot refs:

   ```sh
   git fetch origin
   git fetch frogpilot '+refs/heads/*:refs/remotes/frogpilot/*' --prune
   ```

4. Record the current branch, current head, Testing head, and merge base.
5. Create and push a timestamped backup branch from the exact current head.
6. Use `skills/git-commit-cleanup` to clean the local commit stack relative to
   the old Testing merge base.
7. Create a new refresh branch from `frogpilot/FrogPilot-Testing`.
8. Cherry-pick cleaned local commits in logical batches, resolving conflicts to
   preserve the current upstream architecture and reapply local behavior narrowly.
9. Verify with `git range-diff`, `git diff --check`, focused tests/builds, and a
   final `git status --short`.
10. Do not deploy to the comma device unless the user explicitly asks.

## Reporting

Report:

- The backup branch name.
- The old base and new Testing head.
- The cleanup branch and refreshed branch names.
- The final local commit list on top of `frogpilot/FrogPilot-Testing`.
- Tests/builds run and any skipped validation.
