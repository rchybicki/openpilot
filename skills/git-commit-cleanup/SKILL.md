---
name: git-commit-cleanup
description: Clean up the active fork branch (currently !my-fp-new) commit history by creating a pushed backup branch, finding the true upstream base, and collapsing the local stack into one commit per feature (target 10-15). Short stacks use an interactive rebase that regroups by feature and keeps only the newest model commit; long interleaved stacks are rebuilt with scripts/regroup_by_feature.py, which attributes every changed line to its original commit via git blame so shared files are split by feature, and is verified by tree identity. Use when preparing a clean history before pushing, sharing, or refreshing onto upstream.
---

# Git Commit Cleanup

## Overview

Turn a long local commit chain into a small set of clear feature commits (target 10-15)
without losing work. Always create and push a backup branch first. Group by **feature**, not by
file or by time: a file shared by two features (longcontrol.py, card.py, params_keys.h,
AGENTS.md) must be split so each feature commit carries only its own lines. The owner rejected
a file-level partition on 2026-09-06; do not offer it again.

Two mechanisms, chosen in step 4:

- **Interactive rebase** for short stacks (roughly under 60 commits).
- **Feature attribution rebuild** (`scripts/regroup_by_feature.py`) for long interleaved
  stacks. Each original commit is mapped to a feature; each added line in the final diff is
  attributed via `git blame`, each deleted line via `git blame --reverse`; the branch is then
  rebuilt from the base with one commit per feature. It cannot conflict, keeps only the newest
  model automatically, and ends with a tree-hash check. Used on 2026-09-06 to collapse 481 -> 15
  in about 6 minutes.

## Workflow

### 1) Preflight and safety

- Working tree must be clean: `git status --porcelain`.
- Fetch with the forced refspec (FrogPilot force-pushes):
  - `git fetch origin`
  - `git fetch frogpilot '+refs/heads/*:refs/remotes/frogpilot/*' --prune`
- Branch names contain `!`; always quote them in zsh: `git switch '!my-fp-new'`.
- Confirm local == origin before rewriting: `git rev-parse HEAD 'refs/remotes/origin/!my-fp-new'`.

### 2) Create and push a backup branch

- Pattern: `backup/!my-fp-new_MM-DD_HH-MM_<reason>` (historical: `backup/YYYY/...`).
- `git branch 'backup/!my-fp-new_09-06_11-08_pre-cleanup' HEAD`
- `git push origin 'backup/!my-fp-new_09-06_11-08_pre-cleanup'`

### 3) Find the true base

`git merge-base HEAD frogpilot/FrogPilot-Testing` is usually WRONG: FrogPilot rebases its
branches, so the merge base falls back to an ancient commit and the range includes ~160
upstream commits that only look local. Find the last upstream-authored commit instead:

```sh
MB=$(git merge-base HEAD frogpilot/FrogPilot-Testing)
git log --reverse --format='%h|%ad|%an|%s' --date=short "$MB..HEAD" \
  | awk -F'|' '{n++; if ($3!=prev) {print n": "$0; prev=$3}}'
```

`OLD` = the commit just before the author switches to Radek for good. Check:
`git log --format='%an' "$OLD..HEAD" | sort | uniq -c` lists only Radek.
Never use a `Compile FrogPilot` commit as a base (sources stripped); use its parent.

### 4) Review and categorize

- `git log --reverse --format='%h|%ad|%s' --date=short "$OLD..HEAD" > /tmp/stack.txt`
- Buckets: model commits (deep_rl3, rdf-driving, tsfdo, "X model", bump tinygrad), features,
  tuning/fixups, reverts, docs/worklogs, merges (keep).
- Model rule: keep only the newest model overall.
- Short stack, few interleaved areas -> step 5. Long stack (100+) or heavily interleaved areas
  (stopping + update + model + UI in one chain) -> step 6. Do not reorder hundreds of commits
  by hand.

### 5) Interactive rebase (short stacks)

- `git rebase -i --rebase-merges "$OLD"`
- Reorder to group by feature (required), `fixup`/`squash` tuning into its feature, `drop`
  superseded model commits and their reverts, keep merges. Squash until each feature reads as
  one intentional commit.

### 6) Feature attribution rebuild (long stacks)

Files: `scripts/classify.py` (feature list, subject regex rules, path rules) and
`scripts/regroup_by_feature.py` (the builder).

1. Edit `FEATURES` in `classify.py`: ordered `(key, commit subject)` list, 10-15 entries.
   Order roughly: dev tooling; base compat; tinygrad; model; car/hyundai; the big driving
   features; system; update; ui. Edit `BODIES` in `regroup_by_feature.py` to match.
2. Edit `RULES`: subject regexes -> feature key, first match wins. Iterate until
   `python3 scripts/classify.py "$OLD" "$ORIG"` prints `unmatched: 0`. Reverts map to the
   same feature as what they revert (net zero). A commit that mixes docs and code goes to the
   code feature; `PATH_RULES` still route its doc files to the docs feature.
3. `PATH_RULES` override attribution per path regardless of commit: vendored trees
   (`tinygrad_repo/`), `.onnx` models (blaming them as text takes minutes), single-feature
   directories (`tools/stopping/`, `docs/stopping/`). Multi-feature docs such as AGENTS.md are
   left to blame so each feature carries its own lines.
4. Build in a worktree and verify:

```sh
OLD=<base>; ORIG=$(git rev-parse HEAD); CLEAN=codex/fp-feature-cleanup-$(date +%Y%m%d)
git worktree add /tmp/fpr-wt -b "$CLEAN" "$OLD"
python3 skills/git-commit-cleanup/scripts/regroup_by_feature.py "$OLD" "$ORIG" /tmp/fpr-wt
# prints one line per feature commit, then TREE_IDENTICAL and an empty status
git worktree remove --force /tmp/fpr-wt
git switch '!my-fp-new' && git reset --hard "$CLEAN"
```

5. Spot-check shared files: `git log --reverse --format='%h %s' "$OLD..HEAD" -- selfdrive/controls/lib/longcontrol.py`
   should list one commit per feature that touched it, each with only its own hunks. If a
   feature commit carries lines it should not, fix the subject or path rule and rerun.

Known limits: a line added by feature A and later edited by feature B is attributed to B (A's
original text never appears). Intermediate commits are therefore not guaranteed to build; the
final tree is exact. Symlinks (CLAUDE.md) and mode-only changes are handled per feature.

### 7) Verify and push

- `TREE_IDENTICAL` from the script (or `git range-diff <backup>...HEAD` after a rebase).
- `git log --oneline --reverse "$OLD..HEAD"`: 10-15 commits, one model commit.
- `git status --porcelain` empty.
- `git push --force-with-lease='refs/heads/!my-fp-new:<old-tip-sha>' origin '!my-fp-new'`
- The device resets to `!my-fp-new` on `fullupdate.sh`; a rewritten branch with an identical
  tree deploys safely. Nothing deploys by itself (AGENTS.md).

### 8) Upstream check (what the cleanup is usually for)

- The Testing tip may be `Compile FrogPilot`; compare against its source parent
  (`frogpilot/FrogPilot-Testing^`) and list real news with
  `git log --cherry-pick --right-only --oneline "$OLD...<source-ref>"`.
- Conflict forecast: `comm -12 <(git diff --name-only $OLD <source-ref> | sort) <(git diff --name-only $OLD HEAD | sort)`.
- Check `FrogPilot`, `FrogPilot-Staging`, `FrogPilot-Vetting` too: since 2026-07-04 they are on
  the openpilot 0.9.7 lineage with no shared history with Testing's source; moving there is a
  re-port of every feature commit, not a refresh.
