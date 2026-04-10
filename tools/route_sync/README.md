# Route Refresh Tooling

This folder owns shared route discovery and download for route-driven improvement workflows.

## Source Of Truth

- Shared process contract: `docs/route_refresh_process.md`
- Canonical CLI: `tools/route_sync/refresh_routes.py`

## What It Does

- lists route log files on device over SSH,
- compares remote files against shared local refresh state,
- downloads only new or changed files,
- writes a JSON refresh report,
- falls back from `commawifi` to `comma` when needed.
- treats `commawifi` and `comma` as the same local cache/state identity so alias swaps do not cause duplicate downloads.
- treats `/data/media/0/realdata` as the canonical remote and local cache root for normal workflow.

## What It Does Not Do

- snapshot process-specific settings,
- append any worklog,
- decide whether a route is a valid tuning target.

## Default Local Paths

- Download root: `~/.comma/route_sync/downloads`
- State file: `~/.comma/route_sync/state.json`
- Report dir: `~/.comma/route_sync/reports`

## Typical Commands

```bash
python tools/route_sync/refresh_routes.py --host commawifi --max-downloads 80 --newest-first
```

```bash
python tools/route_sync/refresh_routes.py --host commawifi --include-rlog --newest-first
```

```bash
python tools/route_sync/refresh_routes.py --host commawifi --dry-run
```

## Useful Options

- `--host commawifi`
- `--dry-run`
- `--max-downloads N`
- `--newest-first`
- `--spread-routes`
- `--state-file ~/.comma/route_sync/state.json`
- `--download-root ~/.comma/route_sync/downloads`
- `--report-dir ~/.comma/route_sync/reports`
- `--remote-root /data/media/0/realdata`
- `--file-name qlog.zst`
- `--include-rlog`
- `--verbose`

## Root Policy

- Default refresh now uses only `/data/media/0/realdata`.
- `/data/media/0/realdata_HD` and `/data/media/0/realdata_konik` are legacy/exception roots, not normal workflow.
- Local cache is normalized under `~/.comma/route_sync/downloads/<host>/data/media/0/realdata/` even if an old root is inspected explicitly.

## Alias Behavior

- `commawifi` remains the preferred SSH alias and `comma` remains the network fallback.
- Local cache/state now treats them as the same device identity.
- Old files previously stored under `downloads/comma/...` are lazily migrated into the shared canonical cache path on first reuse.
- This avoids re-downloading the same route files when a run switches from `commawifi` to `comma` or back.
