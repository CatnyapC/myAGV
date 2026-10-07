# Local WebUI Milestone

Scope: development Mac, no ROS or hardware control. Read the cited plan section
before each task. Manage running services with PM2. Do not open a browser for QA.

- [ ] Backend and PM2 setup — plan sections 4 and 6. One Python API, same-origin
  static serving, development proxy, Demo status, persisted JSON and edit lease.
- [ ] Test map and auto-alignment — section 3, Leaflet map rendering. Generate a
  small rotated occupancy map; show an aligned derived raster without changing
  map coordinates. Manual angle fallback; resizing preserves the map.
- [ ] Four-panel UI and no-go editing — sections 2/3 and Minimal no-go zones.
  Compact dark dashboard; independent GridStack sizing. Two-corner rectangle
  preview, Save/Cancel, selection/Delete, keyboard coordinates, stale revision checks.
- [ ] Automatic settings save — sections 3/4. Backend display settings; browser
  panel layout. Restore after reload/restart; show failures without false Saved state.
- [ ] Necessary checks only — section 6. TypeScript/build, coordinate and API
  persistence/validation checks; PM2 process/HTTP smoke check. Commit and push.

## User manual acceptance

- [ ] Load the Demo map; toggle auto-align and adjust manual display angle.
- [ ] Add/delete a rectangle before and after rotation; reload to confirm persistence.
- [ ] Resize, move, maximize and reset panels; check Settings restore after restart.
- [ ] Check compact layout on desktop/mobile; STOP stays visible.

## Deferred robot integration

ROS map subscription, real localization/path/cameras, global/local costmap zone
enforcement, motion watchdog, navigation/fetch, survey and grasp alignment are not
implemented by this milestone. Controls must not claim hardware readiness.
