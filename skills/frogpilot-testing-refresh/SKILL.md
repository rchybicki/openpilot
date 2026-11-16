# FrogPilot Testing Refresh Skill

Use this skill when refreshing this fork onto the current upstream
`frogpilot/FrogPilot-Testing` branch. This is the full branch-refresh workflow:
backup and pin, inventory, decisions, port in batches, device move, and the
parity audit after the first drive.

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

`docs/frogpilot_testing_refresh_process.md` is the process. Read it whole
before any step. Its worked example is `docs/frogpilot_refresh_20261031/`.

1. Phases 0 to 2 are read-only: backup and pin on `origin`, sync the car's
   logs, inventory our features, map the upstream delta with the check table
   of phase 2.
2. Choose the size with the owner. The short path (cleaned stack,
   cherry-pick) is only for a refresh with no breaking change. Otherwise use
   all phases: decision table, regroup, removal, batched port, verification,
   reviewed device procedure, first-day parity audit.
3. Never switch the device branch, roll back, or push to the device branch
   before the first reviewed drive without the owner's explicit go.

## Reporting

Report:

- The backup branch name.
- The old base and new Testing head.
- The cleanup branch and refreshed branch names.
- The final local commit list on top of `frogpilot/FrogPilot-Testing`.
- Tests/builds run and any skipped validation.
