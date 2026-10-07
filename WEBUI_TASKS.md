# Local WebUI Milestone

Scope: development Mac, no ROS or hardware control. Read the cited plan section
before each task. Manage running services with PM2. Do not open a browser for QA.

- [x] Backend and PM2 setup — plan sections 4 and 6. One Python API, same-origin
  static serving, development proxy, Demo status, persisted JSON and edit lease.
- [x] Test map and auto-alignment — section 3, Leaflet map rendering. Generate a
  small rotated occupancy map; show an aligned derived raster without changing
  map coordinates. Manual angle fallback; resizing preserves the map.
- [x] Four-panel UI and no-go editing — sections 2/3 and Minimal no-go zones.
  Compact dark dashboard; independent GridStack sizing. Two-corner rectangle
  preview, Save/Cancel, selection/Delete, keyboard coordinates, stale revision checks.
- [x] Automatic settings save — sections 3/4. Backend display settings; browser
  panel layout. Restore after reload/restart; show failures without false Saved state.
- [x] Necessary checks only — section 6. TypeScript/build, coordinate and API
  persistence/validation checks; PM2 process/HTTP smoke check. Commit and push.
- [x] Reference workspace layout — section 3. Fill the current window height,
  paired divider handles with pointer/keyboard resizing, minimum content sizes,
  top-toolbar default restore and automatic browser persistence. Build and geometry check.

Implementation checks passed. UI checkboxes below remain for the user; no browser
was opened. PM2 processes: `myagv-api` (8791), `myagv-web` (5173).

## User manual acceptance

- [ ] Load the Demo map; toggle auto-align and adjust manual display angle.
- [ ] Add/delete a rectangle before and after rotation; reload to confirm persistence.
- [ ] Unlock Edit layout; drag the shared vertical/horizontal handles and use arrow keys.
- [ ] Resize the window; panels fill available height or scroll at minimum sizes.
- [ ] Move/maximize panels, then use the top-toolbar default restore; reload to confirm.
- [ ] Check Settings restore after restart.
- [ ] Check compact layout on desktop/mobile; STOP stays visible.

## Deferred robot integration

ROS map subscription, real localization/path/cameras, global/local costmap zone
enforcement, motion watchdog, navigation/fetch, survey and grasp alignment are not
implemented by this milestone. Controls must not claim hardware readiness.
