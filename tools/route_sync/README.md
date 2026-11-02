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
- limits ordinary qlogs/rlogs to 30 days / 30 GiB and uses a separate 10 GiB protected-data budget,
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
- `--keep-days 30`
- `--max-cache-gib 30`
- `--max-protected-gib 10`
- `--prune-only` (no device connection)
- `--verbose`

## Retention

Every successful refresh applies the limits to qlog/rlog files and their partial downloads under the canonical local cache, including files missing from state.
The budgets are independent: 30 GiB for ordinary routes and 10 GiB for protected routes, a combined target of 40 GiB.
Protected bytes never reduce the ordinary-route allowance. `--max-cache-gib` sets the ordinary budget; `--max-protected-gib` sets protection's budget.
All encodings and cached segments of a route are counted together. A route's newest source mtime determines its age; local mtime
is used when source metadata is absent. Downloads preserve source mtimes. Limits apply regardless of which log types this run requests.
Other files, videos, reports, and analysis/corpus outputs are outside this policy. To keep a regression fixture indefinitely, copy it
outside the route cache before it expires; old analysis reports can otherwise refer to logs that have been removed.

Preview and apply cleanup without the device:

```bash
python tools/route_sync/refresh_routes.py --prune-only --dry-run
python tools/route_sync/refresh_routes.py --prune-only
```

The JSON `retention` field lists excluded routes, local files/bytes to prune, protected routes/bytes, expired protection, and the projected size.
Dry-run writes a report and uses a cache lock, but does not move/delete logs or write state. Legacy files are migrated on normal runs;
the pruning preview covers the current canonical cache. A local `*.lock` or log partial file protects its route only while that route
has local activity within the last 24 hours. All segments and files are checked, using both mtime and ctime so a new copy with a preserved
source timestamp renews protection. After 24 idle hours, leftover locks/partials are removed and completed logs follow the normal age/size limits.
Active partial bytes and all completed logs in their protected route count against the separate 10 GiB budget.
Downloads that add protected bytes beyond that budget are deferred without consuming `--max-downloads` slots; ordinary downloads and retention continue.
Same-size or smaller replacements remain allowed when the protected cache is already over budget.
The report counts these files in `counts.skipped_due_to_protected_limit` and shows each bucket's usage, limit, and excess separately.
Protection is checked again immediately before deletion. Expired routes return to the ordinary 30 GiB / 30-day policy.
Pre-existing data, other writers, or an interrupted transfer that protects an already-large route can exceed the protected target.
That condition is reported with a nonzero exit status; active files remain safe and the ordinary 30 GiB allowance stays intact until protection expires.
If the newest ordinary route alone exceeds its 30 GiB budget, the run stops without downloads or pruning.
Local locks/partials still allow finalized remote logs to sync. Symlinks block downloads and are never followed or deleted;
they do not protect ordinary sibling log files from retention.

Download candidates are filtered before transfer; space is reserved only for this run's attempted downloads after `--max-downloads`.
A compact eviction cutoff, saved before deletion, prevents size-evicted
routes from being downloaded again, including on a later qlog-only refresh. Increasing the limit does not undo that cutoff.
Pruning runs after downloads succeed (or with `--prune-only`); a failed transfer leaves the old cache in place. Temporary usage can
therefore exceed the limit during a refresh. Concurrent refresh/prune commands fail fast on the shared `.refresh.lock`.
Other cache writers and analysis readers must finish before cleanup; they do not use this lock.

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
