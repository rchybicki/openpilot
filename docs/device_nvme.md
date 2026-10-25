# NVMe missing at boot

The comma three keeps drive logs and Tailscale on an NVMe that is mounted at `/data/media` (unit `data-media.mount`).
Sometimes the NVMe controller wedges and stays broken through warm reboots (incident 2026-10-25).

## Failure modes

- Not mounted: `/data/media` is the bare directory on `/data`. loggerd cannot create the route directory under it.
- Mounted but not responding: something is mounted at `/data/media`, but I/O on it fails or blocks. On 2026-10-25 the bare
  directory was never written (debugfs: crtime == mtime 2025-07-02, no entries), so this is the likely mode of that incident.

## Symptom

- Before the guard: the car does not engage and shows "Process Not Running: loggerd". loggerd aborts every 2 s because it cannot
  create the route directory.
- Tailscale is offline although LTE has full bars, because Tailscale lives on the NVMe.

## What the manager does at boot

`ensure_media_storage()` in `system/manager/helpers.py` runs first in `manager_init`, on `tici` only.

1. Not mounted: up to 30 s, it starts `data-media.mount` whenever `/dev/nvme0n1` exists.
2. Still not mounted and `/dev/nvme0n1` is absent: it removes PCI function `0001:01:00.0`, rescans the PCI bus, waits up to
   15 s more, and starts the mount again. A present or mounted device is never removed.
3. Mounted: it creates and removes a temporary directory in `/data/media`, as loggerd does, in a thread with a 5 s timeout.
   When that works, nothing else happens. A normal boot costs only this probe.
4. Still not mounted, or the probe fails or times out: it sets `LOG_ROOT=/data/realdata_fallback` for every process. loggerd,
   bootlog, the deleter, the uploader, and athenad use that directory, and the car engages. FrogPilot does not create its
   `/data/media` directories. A dead mount is never unmounted or reset.

openpilot starts up to about 75 s later than usual when nothing is mounted, and about 5 s later when a dead mount blocks.
Every step writes a `nvme_guard` event to swaglog (`/data/log/swaglog.*`). The `probe` event has the `OSError` with its errno, or
`timeout`. The `fallback` event has `error`.

## Alerts

- Offroad (red): "NVMe storage is missing or not responding. Drives are logged to internal storage and are not synced. ..."
- Startup alert on the road: "NVMe missing: logs on internal disk" / "Cut device power when parked".

## Fallback logs

- Routes and bootlogs go to `/data/realdata_fallback` on the internal 30 GB `/data`. The deleter keeps 5 GB and 10 % free
  there and deletes the oldest segments first.
- route_sync reads only `/data/media/0/realdata`, so fallback routes are not synced.
- After a normal boot the deleter no longer manages `/data/realdata_fallback`, and the routes there keep `/data` near its 5 GB
  floor until they are moved. When parked and the NVMe is back, merge them into the NVMe:

  ```sh
  mountpoint -q /data/media && rsync -a --remove-source-files /data/realdata_fallback/ /data/media/0/realdata/ && find /data/realdata_fallback -mindepth 1 -type d -empty -delete
  ```

## Fix

Cut the device's power when parked. A warm reboot (menu or `reboot`) does not reset a wedged controller.
