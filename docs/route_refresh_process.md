# Shared Route Refresh Process

- Updated: 2026-09-12
- Scope: shared route discovery, download, and refresh reporting for improvement workflows
- Canonical CLI: `python tools/route_sync/refresh_routes.py`
- Shared tooling reference: `tools/route_sync/README.md`

## Purpose

Route refresh is a shared prerequisite for any improvement process that consumes recent device logs. Stopping, tuning, and future
route-driven workflows all read from the same local route cache and the same refresh report shape.

This process owns:

- discovering route log files on device,
- downloading only new or changed files,
- maintaining shared local refresh state,
- retaining ordinary canonical qlog/rlog data for at most 30 days / 30 GiB, oldest routes first,
- reserving a separate 10 GiB budget for temporarily protected routes,
- writing a structured JSON refresh report.

This process does not own:

- process-specific settings snapshots,
- analysis outputs, models, or holdout definitions,
- worklog append behavior or keep/reject decisions.

## Shared Contract

- Shared route-sync root: `~/.route_sync`
- Shared download root: `~/.route_sync`
- Shared state file: `~/.route_sync/state.json`
- Shared report dir: `~/.route_sync/reports`
- Preferred SSH host: `comma`
- Fallback SSH host: `commawifi`
- Shared local cache identity for those aliases: `comma`
- Operational rule: start with `comma`, but if direct `ssh` or `scp` to `comma` fails, switch to `commawifi` for that session instead of waiting on the alias.
- Canonical remote root:
  - `/data/media/0/realdata`
- Legacy remote roots available with explicit `--remote-root`:
  - `/data/media/0/realdata_HD`
  - `/data/media/0/realdata_konik`
- Default file names:
  - `qlog`
  - `qlog.bz2`
  - `qlog.zst`
- Optional extra file names:
  - `rlog`
  - `rlog.bz2`
  - `rlog.zst`

Every consumer may assume the refresh JSON includes:

- `timestamp_utc`
- `host`
- `ssh_host`
- `cache_host`
- `remote_roots`
- `file_names`
- `counts`
- `new_routes`
- `new_segments`
- `downloaded_files`
- `retention` (planned/applied pruning, protected routes, separate ordinary/protected usage and limits, and projected total bytes)

Consumers must not assume:

- that a refresh appends to any process worklog,
- that a refresh implies a route is suitable for tuning,
- that all new routes are clean or relevant to the current process.

## When To Refresh

- Refresh after a drive before any route-based analysis.
- Refresh again when a process needs the newest routes and the local cache may be stale.
- Use `--max-downloads` and `--newest-first` when you want the newest routes first on a slow link.
- Use `--spread-routes` when you want breadth across routes instead of draining one route completely.
- Switching between `comma` and `commawifi` should no longer cause duplicate downloads, because they now share the same local cache/state identity.
- `refresh_routes.py` already falls back from `comma` to `commawifi`; for manual SSH/SCP inspection, apply the same fallback yourself.
- The refresher scans `/data/media/0/realdata` by default. Explicit legacy roots normalize into the canonical local cache.
- Retention applies automatically after successful downloads. `--prune-only --dry-run` previews local cleanup without SSH;
  `--prune-only` applies it. See `tools/route_sync/README.md` for protection rules and scope.

## Typical Commands

Refresh newest qlogs:

```bash
python tools/route_sync/refresh_routes.py --host comma --max-downloads 80 --newest-first
```

Refresh qlogs and rlogs:

```bash
python tools/route_sync/refresh_routes.py --host comma --include-rlog --newest-first
```

Discover only:

```bash
python tools/route_sync/refresh_routes.py --host comma --dry-run
```

Inspect a legacy root explicitly:

```bash
python tools/route_sync/refresh_routes.py \
  --host comma \
  --remote-root /data/media/0/realdata_konik \
  --max-downloads 40 \
  --newest-first
```

## Consumer Rules

- Stopping uses shared route refresh before stopping analysis and gates.
- Future tuning or improvement cycles should call the shared refresher instead of introducing their own download ownership.
- A process-specific cycle runner may wrap route refresh, but route refresh remains shared infrastructure, not process-owned logic.
- Local cache should now be treated as canonical under `~/.route_sync/data/media/0/realdata/`; older `~/.comma/route_sync/...` paths are compatibility leftovers, not the first place to look.
