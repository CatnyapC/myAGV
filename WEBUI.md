# Local WebUI

Demo-only map editor on the development Mac. No ROS/serial connection or motion.
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
at <http://127.0.0.1:8791>. Both bind to loopback. The development frontend proxies
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

1. Open the UI yourself. Select **Take** to acquire editing ownership. The lease
   renews while connected; closing/disconnecting expires it within 30 seconds.
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
5. Menu > **Edit layout** unlocks panel dragging by title and shared divider handles.
   Drag a divider to resize adjacent panels together, or focus it and use arrow keys
   (Shift moves two grid units). Settings W/H inputs adjust the same boundaries.
   Browser layout editing does not require Take. Resize the window: panels fill
   the available height; short/narrow windows scroll at minimum content sizes.
6. The top-toolbar circular-arrow button **Restore default layout** exits maximization
   and restores the four default panels. It also appears in Menu and Settings.
   Reload to confirm automatic layout persistence. Verify desktop/mobile layout
   and visible STOP yourself.

The camera panes are unavailable; Go/Fetch/Cancel are disabled. STOP reports
that hardware is unavailable. No-go zones are persisted editor annotations;
ROS navigation costmap enforcement is not active in this local milestone.

## Data and test map

- `web_runtime/settings.json`: shared display preferences and revision.
- `web_runtime/no_go_zones.json`: original map-frame rectangle corners and revision.
- Browser localStorage `myagv:local-demo:layout:v1`: validated panel geometry only.

Backend JSON uses atomic writes and one writer lock; observers cannot modify it.
Runtime data and dependencies are ignored by Git. Invalid saved data fails startup
instead of silently replacing it. Credentials are not stored by this milestone.

`web_backend/map_data.py` generates a 240 × 200 OccupancyGrid, 0.05 m/cell, with
free/occupied/unknown cells, rotated walls, a divider and furniture. Its origin
has nonzero translation and 17° yaw; wall direction is independently rotated 21°.
The backend detects the dominant orthogonal direction and generates a separate
display PNG/origin. The original cells are unchanged. For weak wall evidence it
uses the original grid view; manual display angle handles ambiguous rooms.

The current loopback Demo boundary is not remote-control authentication/TLS.
Do not expose this service on a network before the robot integration task.

## Necessary checks

```bash
.venv/bin/python -m unittest test_web_backend
npm --prefix web run build
npm --prefix web test
```

These check map geometry, API edits/persistence/revisions/ownership and frontend
types/build. HTTP, PNG, static asset and WebSocket smoke checks passed for both
PM2 services. Browser visual/interaction checks are deliberately left to the user.
