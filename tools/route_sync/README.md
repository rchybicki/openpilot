# Route Refresh Tooling

This folder owns shared route discovery and download for route-driven improvement workflows.

## Source Of Truth

- Shared process contract: `docs/route_refresh_process.md`
- Canonical CLI: `tools/route_sync/refresh_routes.py`

## What It Does

- lists route log files on device over SSH,
- skips live segment directories while `*.lock` files are present,
- compares remote files against shared local refresh state,
- downloads only new or changed files,
- writes a JSON refresh report,
- falls back from `comma` to `commawifi` when needed.
- treats `comma` and `commawifi` as the same local cache/state identity so alias swaps do not cause duplicate downloads.
- normalizes the local cache under `/data/media/0/realdata`.

## What It Does Not Do

- snapshot process-specific settings,
- append any worklog,
- decide whether a route is a valid tuning target.

## Default Local Paths

- Route-sync root: `~/.route_sync`
- Download root: `~/.route_sync`
- State file: `~/.route_sync/state.json`
- Report dir: `~/.route_sync/reports`

## Typical Commands

```bash
python tools/route_sync/refresh_routes.py --host comma --max-downloads 80 --newest-first
```

```bash
python tools/route_sync/refresh_routes.py --host comma --include-rlog --newest-first
```

```bash
python tools/route_sync/refresh_routes.py --host comma --dry-run
```

## Useful Options

- `--host comma`
- `--dry-run`
- `--max-downloads N`
- `--newest-first`
- `--spread-routes`
- `--state-file ~/.route_sync/state.json`
- `--download-root ~/.route_sync`
- `--report-dir ~/.route_sync/reports`
- `--remote-root /data/media/0/realdata`
- `--file-name qlog.zst`
- `--include-rlog`
- `--verbose`

## Root Policy

- Remote refresh scans `/data/media/0/realdata` by default.
- Legacy roots can still be requested explicitly with `--remote-root` if an old route needs one-off recovery.
- Active segments with lock files are skipped until finalized, avoiding corrupt partial qlog/rlog downloads.
- Local cache is normalized under `~/.route_sync/data/media/0/realdata/` so there is one place to inspect locally.
- If a file changes on-device (size or mtime), refresh re-downloads it into that canonical local path.

## Alias Behavior

- `comma` remains the preferred SSH alias and `commawifi` remains the network fallback.
- Local cache/state now treats them as the same device identity.
- Old files previously stored under the older `~/.comma/route_sync/...` layout are migrated into the shared canonical cache path on first reuse.
- This avoids re-downloading the same route files when a run switches from `comma` to `commawifi` or back.
