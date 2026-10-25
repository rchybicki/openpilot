# NVMe missing at boot

The comma three keeps drive logs and Tailscale on an NVMe that is mounted at `/data/media` (unit `data-media.mount`).
Sometimes the NVMe controller wedges and stays missing through warm reboots (incident 2026-10-25).

## Symptom

- Before the guard: the car does not engage and shows "Process Not Running: loggerd". loggerd aborts every 2 s because it cannot
  create the route directory under the bare mountpoint.
- Tailscale is offline although LTE has full bars, because Tailscale lives on the NVMe.

## What the manager does at boot

`ensure_media_storage()` in `system/manager/helpers.py` runs first in `manager_init`, on `tici` only.

1. `/data/media` is mounted: nothing happens, no delay.
2. Up to 30 s: when `/dev/nvme0n1` exists, it starts `data-media.mount`.
3. It removes PCI function `0001:01:00.0`, rescans the PCI bus, waits up to 15 s more, and starts the mount again.
4. Still not mounted: it sets `LOG_ROOT=/data/realdata_fallback` for every process. loggerd, bootlog, the deleter, the uploader,
   and athenad use that directory, and the car engages. openpilot starts up to about a minute later than usual.

Every step writes a `nvme_guard` event to swaglog (`/data/log/swaglog.*`). The fallback event has `error`.

## Alerts

- Offroad (red): "NVMe storage is not mounted. Drives are logged to internal storage and are not synced. ..."
- Startup alert on the road: "NVMe not mounted: logs on internal disk" / "Cut device power when parked".

## Fallback logs

- Routes and bootlogs go to `/data/realdata_fallback` on the internal 30 GB `/data`. The deleter keeps 5 GB and 10 % free
  there and deletes the oldest segments first.
- route_sync reads only `/data/media/0/realdata`, so fallback routes are not synced.
- After a normal boot nothing manages `/data/realdata_fallback`. When parked and the NVMe is back, move the routes:

  ```sh
  mountpoint -q /data/media && mv /data/realdata_fallback/* /data/media/0/realdata/
  ```

## Fix

Cut the device's power when parked. A warm reboot (menu or `reboot`) does not reset a wedged controller.
