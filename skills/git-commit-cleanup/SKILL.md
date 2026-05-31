---
name: git-commit-cleanup
description: Clean up the !my-fp branch commit history by creating a backup branch, reviewing recent commits, and running an interactive rebase that preserves merges, always regroups similar commits by feature, squashes tuning or fixup commits into their parent features, and keeps only the newest model commit. Use when preparing a clean history before pushing or sharing changes.
---

# Git Commit Cleanup

## Overview

Use this skill to turn a long !my-fp commit chain into a small set of clear, intentional commits without losing work. Always create a backup branch in backup/ before rebasing, and always regroup similar commits into feature commits during the rebase instead of leaving a chronological tuning chain behind.

## Workflow

### 1) Preflight and safety

- Ensure the working tree is clean.
  - `git status --porcelain`
- Fetch remotes so log and base checks are current.
  - `git fetch --all --prune`
- Confirm the branch and handle `!` in zsh by quoting or escaping.
  - `git switch '!my-fp'`
  - Alternative: `git switch \!my-fp`

### 2) Create a backup branch (manual naming)

- Review the existing naming pattern first.
  - `git branch --list 'backup/*'`
- Use the current-year format: `backup/!my-fp_MM-DD_HH-MM`.
- If you ever need to create a historical backup for a prior year, place it under `backup/YYYY/!my-fp_MM-DD_HH-MM` to match existing layout.
- Create the backup branch with quotes to avoid `!` expansion:
  - Example: `git branch 'backup/!my-fp_02-04_12-30' '!my-fp'`

### 3) Review and categorize commits

- Start by reviewing the commit list.
  - `git log --oneline --decorate --graph --date-order --reverse --max-count=200`
  - If you already know the base, use `<base>..HEAD` to focus on just your branch.
- Tag commits into buckets:
  - **Model commits**: messages like `WMI Model vX`, `MS2b model`, `DS default model`, `st-model`, `neurips model`, `cgwm model`.
  - **Features**: new functionality that should remain as separate commits.
  - **Tuning/fixups/UI**: small tweaks or adjustments to an existing feature or model.
  - **Reverts/Reapply**: commits that only undo or reapply earlier changes.
  - **Merge commits**: keep and preserve these.

### 4) Apply the model replacement rule

- Keep only the single newest model commit overall.
- Drop older model commits that are fully superseded by the newest model.
- Drop reverts/reapply commits that only relate to the dropped model commits.
- If an older model commit contains unrelated changes, split or retain those unrelated changes before dropping the model part.

### 5) Plan mandatory grouping for tuning and fixups

- For each tuning/fixup/UI commit, decide which feature commit it belongs to.
- Always regroup adjacent or nearby commits by feature before finalizing the rebase todo, even if that means reordering commits away from strict chronology.
- Keep new features as separate commits; only merge the tiny follow-ups into their original feature commit.

### 6) Run the interactive rebase (preserve merges)

- Determine the base for the rebase:
  - If the user specifies an upstream base branch, use it.
  - Otherwise, list likely candidates (for example: `FrogPilot`, `openpilot-master`, `master`) and ask which one is the true base.
  - If you want a pure history cleanup without pulling upstream changes, use the merge-base commit:
    - `BASE_SHA=$(git merge-base '!my-fp' <base-branch>)`
    - `git rebase -i --rebase-merges "$BASE_SHA"`
  - If you want to rebase onto the upstream tip, use the branch name directly:
    - `git rebase -i --rebase-merges <base-branch>`
- In the todo list:
  - Reorder commits to group by feature. This is required, not optional.
  - Mark tuning/fixup commits as `fixup` or `squash` under their parent feature.
  - Mark old model commits and their now-irrelevant reverts as `drop`.
  - If the grouped result still leaves several nearly-identical commits for the same feature, keep squashing until each feature reads as one intentional commit or a very small feature-sized cluster.
  - Keep merge commits intact.

### 7) Verify and push

- Compare against the backup branch and ensure only intended changes were dropped.
  - `git range-diff <backup-branch>...HEAD`
- Confirm only the newest model commit remains.
  - `git log --oneline --decorate --graph --date-order --max-count=200`
  - If you already used a base, use `<base>..HEAD` to focus on just your branch.
- If tests are needed, run them after the rebase.
- Push with lease once satisfied.
  - `git push --force-with-lease origin '!my-fp'`
