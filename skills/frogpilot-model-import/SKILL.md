---
name: frogpilot-model-import
description: Import upstream openpilot driving model ONNX files into FrogPilot model paths. Use when asked to bring a new model from an upstream commit or branch, handle Git LFS downloads, and copy driving_policy.onnx/driving_vision.onnx into FrogPilot without cherry-picking.
---

# Frogpilot Model Import

## Overview

Pull the upstream ONNX driving models from a commit or branch and copy them into the FrogPilot model paths. This workflow uses a temporary worktree so Git LFS can download the real binaries.

## Workflow

1. Identify the upstream reference.
Use a commit hash or a branch name. If the branch name could be mistaken for a hash, pass `--branch` explicitly.

2. Run the import script.
The script resolves the commit, creates a temp worktree, runs `git lfs pull`, and copies the two ONNX files into FrogPilot paths.

3. Verify the updated files.
Confirm `frogpilot/**/driving_policy.onnx` and `frogpilot/**/driving_vision.onnx` are updated (file sizes should change, and the files should not be LFS pointers).

## Script

Use `scripts/import_model.py`.

```bash
python3 skills/frogpilot-model-import/scripts/import_model.py <commit-or-branch>
```

Common examples:

```bash
python3 skills/frogpilot-model-import/scripts/import_model.py 5b6436a90cf6902b8aaa71c2b6f3d7164d8ae391
python3 skills/frogpilot-model-import/scripts/import_model.py --branch master
python3 skills/frogpilot-model-import/scripts/import_model.py --branch openpilot/master
python3 skills/frogpilot-model-import/scripts/import_model.py --remote origin --commit <hash>
```

Options:
- `--commit` or `--branch` to force interpretation.
- `--remote` to select the upstream remote (default: `openpilot`).
- `--target-policy` and `--target-vision` to override FrogPilot target paths if multiple matches exist.
- `--dry-run` to print the plan without copying.
- `--keep-worktree` to keep the temporary checkout for debugging.

## Notes

- Do not cherry-pick. The workflow copies files only.
- The script fails fast if LFS downloads are missing or if the ONNX files are still pointers.
- Target paths are discovered from tracked FrogPilot files matching `frogpilot/**/driving_policy.onnx` and `frogpilot/**/driving_vision.onnx`.
