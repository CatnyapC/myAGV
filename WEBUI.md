# Local WebUI

Demo map editor and simulated navigation on the development Mac. No ROS/serial
connection or hardware motion. All displayed poses/routes are explicitly simulated.
The four panels follow `WEBUI_PLAN.md`; task status is in `WEBUI_TASKS.md`.

## Start

Requires Python 3.11+, uv, Node 22.12+ or 24+, npm and PM2. The current Node 26
development environment was checked. Robot ROS/Python compatibility is deferred.

```bash
uv sync --extra web
npm --prefix web ci
npm --prefix web run build
pm2 start ecosystem.config.cjs --only myagv-api,myagv-web
```

Development UI: <http://127.0.0.1:5173>. The API and built static UI are also served
at <http://127.0.0.1:8791>. The API binds to loopback; the development frontend
binds to all interfaces. On a trusted LAN or phone hotspot, open
`http://<Mac-LAN-IP>:5173` from another device. The development frontend proxies
`/api` and WebSocket traffic to the backend; frontend and backend sources stay
separate. For a built UI only, start just `myagv-api` with PM2.

```bash
pm2 restart myagv-api myagv-web
pm2 logs myagv-api --lines 20 --nostream
pm2 logs myagv-web --lines 20 --nostream
pm2 stop myagv-api myagv-web
```

Only these two process names belong to this project. PM2's existing process list
is left intact. No login/startup registration or global `pm2 save` is performed.

## Manual acceptance

1. Open the UI yourself. Editing and simulation are available immediately while
   connected and stopped. This is a single-user UI. Closing the last UI WebSocket
   stops simulation; reconnecting never resumes it.
2. The Demo map loads automatically. Menu > Settings > **Auto-align walls**
   aligns dominant walls. Disable it to enter a manual display angle. Changes
   auto-save after 400 ms; **Saved** requires backend acknowledgement. Retry is
   offered on failure. Display rotation never changes map-frame zone coordinates.
3. MAP > More > **No-go zone**: click two opposite corners, then **Save**. Escape
   or Cancel discards the preview. Coordinates can also be entered in view meters
   through the map's expandable coordinate row. Zones must fit the original map
   bounds and be at least one map cell wide/high.
4. Select a saved zone on the map or in OPERATE, then **Delete**. Rotate the map,
   reload the page and restart `myagv-api`; saved zones should stay in place.
5. Shared divider handles work immediately while connected and stopped; hover
   highlights them. No Edit layout is required for resizing. Menu >
   **Edit layout** unlocks only panel repositioning by title.
   Drag a divider to resize adjacent panels together, or focus it and use arrow keys
   (Shift moves two grid units). Settings W/H inputs adjust the same boundaries.
   Resize the window: panels fill
   the available height; short/narrow windows scroll at minimum content sizes.
6. The top-toolbar circular-arrow button **Restore default layout** exits maximization
   and restores the four default panels. It also appears in Menu and Settings.
   Reload to confirm automatic layout persistence. Verify desktop/mobile layout
   and visible STOP yourself.

7. The cyan chassis outline shows simulated position and heading. OPERATE shows
   map-frame X/Y/yaw. MAP > More > **Fit robot** centers it without resetting zoom.
8. Click free map space to preview a cyan goal, or enter X/Y/yaw in OPERATE.
   Click **Simulate** explicitly; the active goal turns orange.
   The cyan line is the actual remaining A* route;
   the chassis moves/turns with a breathing glow. Arrival adopts the requested yaw
   and clears the line/glow. Reduced-motion settings disable breathing.
9. While stopped, draw a no-go rectangle across the direct route. MAP > More >
   **Global costmap** shows red blocked/inflated cells. Navigate again: expect a
   detour or No path. Goals in walls, unknown cells, zones or clearance are rejected.
   A zone covering the chassis clearance envelope is rejected. Zone edits and
   layout changes are disabled during navigation. Delete the zone and retry.
10. Try **Cancel**, STOP and closing the last UI tab during simulation.
    They clear navigation without moving on reconnect. MAP > More > **Reset demo
    pose** restores a safe start; service restart re-applies zones before simulation.

Camera panes and Fetch remain unavailable. Simulation uses a nominal 0.32 × 0.28 m
chassis and 0.05 m clearance, with conservative cell padding. This is not measured
robot geometry. ROS global/local costmaps, real localization/cameras and hardware
movement remain unconnected. Demo A* does not replace ROS1 move_base.

## Data and test map

- `web_runtime/settings.json`: shared display preferences and revision.
- `web_runtime/no_go_zones.json`: original map-frame rectangle corners and revision.
- Browser localStorage `myagv:local-demo:layout:v1`: validated panel geometry only.

Simulated pose/goals/routes are runtime-only. `/api/navigation-map` returns the
derived source-plus-zones OccupancyGrid; `/api/global-costmap` returns its inflated
simulation grid and applied zone revision. Both preserve source origin/frame.
`/api/global-costmap.png` is an optional transparent overlay in the same display
coordinates. Small `telemetry` WebSocket messages update pose/path without
reloading the raster or resetting map pan/zoom.

Backend JSON uses atomic writes, one writer lock and revision checks.
Runtime data and dependencies are ignored by Git. Invalid saved data fails startup
instead of silently replacing it. Credentials are not stored by this milestone.

`web_backend/map_data.py` generates a 240 × 200 OccupancyGrid, 0.05 m/cell, with
free/occupied/unknown cells, rotated walls, a divider and furniture. Its origin
has nonzero translation and 17° yaw; wall direction is independently rotated 21°.
The backend detects the dominant orthogonal direction and generates a separate
display PNG/origin. The original cells are unchanged. For weak wall evidence it
uses the original grid view; manual display angle handles ambiguous rooms.

The Demo has no remote-control authentication/TLS. Use network access only on a
trusted LAN or phone hotspot; do not expose it to the public internet.

## Necessary checks

```bash
.venv/bin/python -m unittest test_web_backend test_web_simulation
npm --prefix web run build
npm --prefix web test
```

These check map/chassis geometry, conservative zone rasterization, inflation,
detour/no-path/deletion, simulated arrival, API guards/STOP/cancel/disconnect,
direct edits/persistence/revisions and frontend types/build.
HTTP, PNG, static asset and WebSocket smoke checks passed for both
PM2 services. Browser visual/interaction checks are deliberately left to the user.
