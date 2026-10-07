# Minimal Remote Robot UI and Service Plan

Updated: 2026-10-07. Status: local WebUI/backend milestone in progress. This
milestone covers a synthetic map, display alignment, persisted no-go editing and
settings on the development Mac. ROS navigation enforcement, cameras, photo index
and calibrated grasp alignment remain separate robot integration tasks.

## 1. Selected implementation

Use **a 2D navigation map plus a photo-based item index**. Navigate to a known
approach pose, stop, then use the arm-mounted downward camera for local measurement
and small alignment steps. Start with taught grasp height and operator confirmation.

| Responsibility | Minimum implementation |
| --- | --- |
| Mapping and localization | Existing ROS1 GMapping map, AMCL localization and move_base obstacle handling |
| Map preview | Leaflet planar image map with robot, path, goal and item markers |
| Item memory | Photos, capture poses and a small JSON index linked to taught stations |
| Scene interpretation | Optional VLM skill labels photos; operator can label them manually |
| Command interpretation | Optional LLM skill resolves text to an existing item ID |
| Route planning | Existing ROS1 move_base planners, called through navigation.py |
| Photo survey | Ordered viewing poses through existing navigation plus distance/heading keyframes |
| No-go zones | Leaflet rectangles persisted by the backend and applied to both navigation costmaps |
| Final alignment | Calibrated top-down image geometry, bounded motion and fresh feedback |
| Grasp and return | Existing taught arm poses and this run's startup return poses |

No VLMaps/3D reconstruction, feature voxel map, vector database, local model server,
VLA or mandatory fast multimodal model in the minimum release. Add a model to
local alignment only if measured hardware results show geometry is insufficient.
Skills are bounded backend workflows, not independent services or motion authorities.
Keep ROS1; do not introduce ROS2/Nav2 or PhotoAtWaypoint. The existing
`Navigation.go_to()` handles one goal, not a complete patrol. Add only the survey
queue and photo sampling around it; retain upstream planning and obstacle handling.

Reuse `teleop_control.py`, `navigation.py` and the movement sequence in
`fetch_demo.py`. Preserve BASE / ARM / PICKUP modes, homing requirements, measured
station recording, J1=0 pointing left, transport posture and cancellation behavior.
The current fetch function replays a taught grasp without a camera gate; web fetch
integration must expose a stopped review/alignment stage before closing the gripper.
Do not label the existing CLI sequence as visually aligned.

## 2. Minimum WebUI: four panels

```text
+--------------------------------------------------------------------------+
| [Menu] MYAGV CONTROL   Connection   Observer/Operator   [Take] [STOP]      |
+-------------------------------+---------------------+--------------------+
| MAP                           | FRONT CAMERA        | OPERATE            |
| Robot / goal / path           | Live view           | Target + item list |
| Item observation markers      |                     | Go / Fetch / Cancel|
|                               +---------------------+ Manual / Align     |
|                               | ARM CAMERA          | Relevant controls  |
|                               | Target / grip point | One task phase     |
+-------------------------------+---------------------+--------------------+
```

| Panel | Required content | Minimum content size |
| --- | --- | --- |
| MAP | Occupancy map, robot heading, selected item's observation/approach markers, proposed goal, active path and no-go rectangles; pan/zoom/fit | 360 × 280px |
| FRONT CAMERA | Navigation/context view, maximize, unavailable/stale state | 280 × 180px |
| ARM CAMERA | Downward live view; calibrated target/gripper overlay when valid; pixel-only overlay otherwise | 280 × 180px |
| OPERATE | Compact item rows with thumbnail/name, target field, task actions, mode-specific controls and current task phase/blocking reason | 280 × 360px |

Keep STOP and ownership outside the grid, visible in dialogs, maximized views and
mobile layouts. No separate STATIONS, TASK, EVENTS, chat, service-status or analytics
panels. Station teaching belongs in OPERATE; diagnostics belong in Details.

OPERATE shows only controls relevant to the current mode:

- **Target:** select an item; optionally resolve a natural-language request.
  Show the resolved item and photo before Execute. Unknown/ambiguous requests
  require selection; never guess a motion target. A chat transcript is unnecessary.
- **Navigate/Fetch:** Go, Fetch, Cancel and one backend phase/result. Item selection
  previews only. Fetch requires a valid taught grasp and compatible map/calibration.
- **Manual:** BASE / ARM / PICKUP, hold-to-run jog, speed, Home and grip/release.
  Show measured pose only when teaching or adjusting needs it.
- **Teach:** Capture photo, item name, Record pose and explicit Overwrite.
  Capture and pose recording require valid timestamps/feedback; station recording
  waits for standstill and stable measured arm joints.
- **Survey:** add ordered viewing poses through MAP's More menu; preview the route
  order, then Start survey in OPERATE. Reuse Cancel and the current task phase.
  Keep capture distance/heading thresholds in hidden Settings; no new panel.
- **Align:** target selection in the image, valid-plane status, measured residual
  and height source; small-step controls and Confirm grasp. Enable automatic Step
  only after physical calibration validation. Unknown height blocks auto grasp.

Healthy-state explanations, introductory text, repeated badges, full coordinates,
IDs, timestamps, calibration metadata and logs stay out of the default view. Show
short stale/blocking reasons where they affect the next action. Never display a
fabricated measurement, live state or successful grasp.

## 3. Visual and layout requirements

Retain the established references: [shadcn Mail](https://v3.shadcn.com/examples/mail)
for the workspace, [Tabler Admin](https://tabler.io/admin-template) for density,
[Radix Colors](https://www.radix-ui.com/colors) for palette and
[GridStack](https://gridstackjs.com/) for independent panel geometry.

- Dark mode; 8px rounded rectangles, 1px borders, 8–12px gaps, compact lists and
  separators. No nested cards, large shadows, neon or gradient backgrounds.
- Bold uppercase compact English panel titles; yellow subsection labels.
  Body text 13–14px; numerical feedback uses tabular numerals.
- Single-line action bars; secondary actions move into More on narrow screens.
  Hidden side navigation opens Workspace or Settings; no separate item-management page.
- Radix Gray Dark: steps 1/2 backgrounds, 3/4 controls, 6/7 borders, 11/12 text.
  Orange 9/10 means execute/write; Cyan 3/11 selection/recall and Cyan 8 focus.
  Yellow 11 labels; Red 9/11 STOP/errors. Verify foreground contrast.
- Accessible icon names, visible focus, text/icon state cues; desktop hit areas
  at least 32px, touch motion controls at least 44px.

Use React + TypeScript + Vite, selected shadcn controls, Radix colors, GridStack
and **Leaflet 1.9.4**; no full admin starter, additional layout engine or global
state framework.
Native fetch/WebSocket and component state cover the minimum UI.

Start with a 24-column grid: MAP 12, cameras 6, OPERATE 6; map/operation 12 rows,
each camera 6 rows, 40px row units. Both cameras remain independent widgets.
Resize width/height, drag by title handle, maximize/restore and reset layout.
Layout editing is locked during operation; unlock only while stopped. Provide
keyboard width/height inputs. Camera/map gestures never drag a panel.

Save validated geometry and visibility in browser localStorage, scoped to robot
identity and layout version; never store robot state, credentials or panel HTML.
Unknown panel IDs/invalid dimensions restore defaults. ResizeObserver updates map
and video bounds without resetting camera sessions, selection or control state.
Use two columns when space permits and one column/view tabs on phones; preserve
camera aspect ratio, vertical scrolling and persistent STOP/ownership.

### Leaflet map rendering

Use [Leaflet](https://leafletjs.com/reference.html) with `L.CRS.Simple`, integrated
directly into one React component; no React wrapper is required. Create the map
once, update its layers in place and remove it on unmount. Bundle assets locally
for offline use. The map needs no geographic tile provider, ROS2D.js, NAV2D.js,
roslibjs, rosbridge or 3D renderer.

- **Base layer:** backend-generated PNG from the active `/map` OccupancyGrid,
  displayed with `L.imageOverlay`. Keep the ROS `.pgm + .yaml` files for navigation;
  the browser consumes PNG plus normalized JSON metadata. Distinguish free,
  occupied and unknown cells; disable image smoothing where possible.
- **Overlays:** `L.polyline` for the actual planner path, default SVG rendering;
  lightweight markers for robot/heading, proposed/active goal, taught stations
  and photo observations. Use Cyan for selection/path and Orange for the active
  execution goal. Observation markers remain distinct from measured item positions.
- **Interaction:** pan/zoom, Fit map/robot, select a marker to load its item/photo,
  click to preview a goal, then set heading through an input or directional drag.
  Provide keyboard equivalents. Execute goes through the backend command/lease
  checks; clicking or dragging never sends a navigation goal automatically.
- **Resize:** call `map.invalidateSize({pan: false})` from ResizeObserver after
  GridStack resizing, maximize/restore or revealing a hidden panel. Preserve the
  view and selection; do not remount the map on every telemetry update.

Use one reversible coordinate transform for all overlays and pointer input.
API poses/path points stay in the ROS map frame, in meters and explicit angle units.
For display, subtract the grid origin and apply its inverse yaw rotation to obtain
grid-local meters `(u,v)`; pass Leaflet `[v,u]`. Image bounds are
`[[0,0],[height * resolution,width * resolution]]`. Convert selected positions
back through the origin rotation/translation before sending a goal. Heading
overlays use world yaw minus origin yaw. Handle grid-to-image Y reversal exactly
once in PNG generation and account for cell centers when plotting discrete cells.
Do not assume zero origin yaw or use screen pixels as robot coordinates.

The backend subscribes to `/map`, reuses `map -> base_footprint` TF for robot pose,
and subscribes to the planner's `nav_msgs/Path`. The selected upstream launch uses
`global_planner/GlobalPlanner`, normally `/move_base/GlobalPlanner/plan`; confirm
the installed topic/remappings on the robot. Transform non-map paths into the map
frame before publishing. Show unavailable/stale localization explicitly and clear
obsolete paths when a task ends or the map changes; never fabricate a straight
line as a planned route.

Cache the raster until map content/geometry changes; send small pose/path updates
over the shared API WebSocket. During SLAM, replace PNG and metadata as one revision,
including changed bounds/origin. Keep map identity separate from update revision;
a new active map invalidates pending goals and incompatible item/station links.
For rectangular no-go editing, auto-align the display to the dominant orthogonal
wall direction estimated from occupied cells. Do not rotate the navigation map or
change its frame. The backend supplies a derived PNG with separate display
origin/yaw/bounds, used by the same reversible overlay transform. Provide an
auto-align toggle and a manual angle fallback in Settings; weak wall evidence uses
the unaligned view. Saved zone corners stay in the original map frame.
Reference: [official GMapping configuration](https://github.com/elephantrobotics/myagv_ros/blob/myagv_ros_2023Pi/myagv_navigation/launch/gmapping.launch)
and [navigation configuration](https://github.com/elephantrobotics/myagv_ros/blob/myagv_ros_2023Pi/myagv_navigation/launch/navigation_active.launch).

### Minimal no-go zones

Leaflet core supplies [rectangle overlays](https://github.com/Leaflet/Leaflet/blob/v1.9.4/src/layer/vector/Rectangle.js),
not navigation restrictions or an interactive drawing/editor toolbar. Reuse core
`L.rectangle` with a small two-corner selection flow; no drawing dependency in the
minimum release. If drag-to-draw and resize handles become necessary,
[Leaflet.draw](https://leaflet.github.io/Leaflet.draw/docs/leaflet-draw-latest.html)
provides rectangle drawing/editing/deletion; enable only those tools.

- **UI:** one No-go action in MAP's More menu. While stopped with an operator lease,
  select two opposite corners, preview a translucent red rectangle, then Save or
  Cancel. Select a saved zone to Delete; changing it means delete/redraw. Escape
  cancels selection; provide coordinate inputs for keyboard use. Zone selection
  suppresses goal selection and never starts motion. No new panel or polygon editor.
- **Data:** one `no_go_zones.json`, scoped to map identity with its own revision.
  Store each zone's ID and four ordered corners in ROS map-frame meters; convert
  all corners through the existing transform, not just screen bounds. Rectangles
  are aligned to the display grid when drawn; use core `L.polygon` to redisplay
  their stored geometry if the grid orientation changes. Backend validates finite,
  nondegenerate rectangles within map bounds, lease, map and expected zone revision.
  Reject edits covering the current robot footprint; edit only while idle/stopped.
  Persist atomically; a map identity change requires explicit zone review/rebinding.
- **Enforcement:** the same backend publishes a latched derived OccupancyGrid on
  `/navigation_map`: copy the original `/map`, conservatively rasterize every cell
  intersecting a zone as occupied (`100`), and rebuild from the original on deletion.
  Leave the SLAM/AMCL map and saved PGM untouched. Configure existing ROS1
  `costmap_2d/StaticLayer` in **both global and rolling local costmaps** to consume
  this map (`map_topic`, `first_map_only: false`); use static layer, obstacle layer
  with maximum combination, then inflation. Retain sensing/footprint clearance
  and verify sensor clearing cannot erase virtual obstacles. No new ROS service,
  custom costmap plugin or planner is required. Verify the installed configuration
  and rolling-window behavior against the [StaticLayer implementation](https://github.com/ros-planning/navigation/blob/noetic-devel/costmap_2d/plugins/static_layer.cpp).
- **Execution:** reject goals/staging/return poses whose chassis footprint intersects
  a zone. Zone save remains Pending until fresh global/local costmaps reflect the
  update; block navigation and pickup approach until application is verified.
  The existing direct `cmd_vel` pickup alignment must check the updated local grid,
  not bypass zones after move_base finishes. Reload/reapply zones before allowing
  navigation after restart; missing/mismatched enforcement stops autonomous travel.
  Manual teleop is outside this navigation-only restriction and must not be presented
  as protected by it. No temporary bypass, scheduling or live edits during a task.

## 4. Minimum robot-side services and data

Run **one Python backend service**, in the robot's existing ROS-compatible runtime.
Serve the independently built static frontend from it under the same origin.
The frontend executes in the remote browser; control, files and model keys remain
on the robot. Reuse ROS navigation/drivers and existing camera publishers when
available; otherwise use one capture worker for both cameras, not a new camera API
service per device. Confirm the actual ROS/Python compatibility before integration.

Manage the backend and local frontend development server with PM2. Production
serves the separately built frontend from the one backend process. This local
milestone binds to loopback and is explicitly Demo-only: no ROS/serial imports,
motion commands, simulated live pose or camera imagery. No-go persistence here
is editing only; navigation enforcement stays unavailable until robot integration.
Store shared map-display settings atomically on the backend; keep panel geometry
in validated browser localStorage as described above. Local loopback access is not
the authentication/TLS deployment for remote robot control.

```text
Browser: four panels + input
    | same-origin HTTPS / WSS / camera streams
One robot backend: auth + operator lease + watchdog + command/task execution
                   + photo index + alignment geometry + optional skill/API calls
    | existing control helpers, ROS navigation and camera sources
myAGV + P340 + LiDAR + front/downward cameras
```

These are functions in one backend, not separately deployed microservices. Keep
blocking robot/model work off the request/watchdog loop. A single worker owns
hardware; existing terminal teleop/fetch cannot control it concurrently. Reuse
control helpers instead of spawning a process per keypress or parsing CLI stdout.

Minimum persistent data:

- Existing GMapping 2D map (`.pgm + .yaml`) and map identity/revision; web PNG is
  a derived preview, not a replacement navigation map.
- One map-scoped `no_go_zones.json`; derived `/navigation_map` is rebuilt from
  the active original map plus saved zones, never used as localization evidence.
- Existing `stations.json`, unchanged: measured base pose and arm joint angles.
  Its validator rejects extra fields; do not insert photo/semantic fields there.
- One `photo_index.json` plus an image directory, outside tracked source/runtime
  secrets. Each observation stores an observation ID, image reference, capture
  time, map ID/revision, measured capture base pose and camera identity. Survey
  frames may be unlabeled; add item ID/name/appearance after labeling/confirmation.
  Arm-camera captures also store synchronized measured arm pose when needed.
  Optional station link supplies the validated approach/grasp; repeated
  observations share an item ID only after confirmation.
- One local calibration file: camera intrinsics/distortion, camera-to-arm/gripper
  relationship, validated plane/height and workspace/error/step limits.

Capture pose is where the robot saw something, **not the object's metric position**.
Display it as an observation marker. The minimum reliable approach uses a linked,
operator-taught station. An unlinked observation supports inspection/navigation to
a validated viewing location only; it does not enable Fetch. Future object-position
estimates must carry their source/uncertainty, not overwrite capture pose.

Write JSON updates atomically through the single backend writer. Reject mismatched
maps and stale station links. A VLM returns candidate labels/regions; an LLM returns
an allowed item ID or ambiguity. Validate both, use timeouts, and never execute
model-generated code, joint targets or routes. Manual labels/selection work offline.

### ROS1 survey and keyframes

Use a few operator-selected viewing poses on the existing 2D map. Do not navigate
to every occupancy cell or treat photo spacing as map resolution. The minimum
survey does not automatically generate a coverage route or guarantee visibility
of every object; supplement missed shelves/tables with selected viewing poses.

- **Motion:** the backend owns one ordered queue and calls `Navigation.go_to()`
  sequentially. Validate each pose/map/no-go revision, hold the operator lease,
  and keep the arm in transport posture. Preserve existing arrival/standstill
  checks, so the robot stops at each viewing pose; photos can also be sampled
  during travel between poses. Do not silently loosen arrival checks for speed.
  Cancel, STOP, lease loss, localization failure, navigation failure or map
  identity change cancels the active goal and clears pending motion. No automatic
  restart, simultaneous fetch or second navigation client.
- **Sampling:** use the existing front-camera source. Save the first valid frame,
  then a frame when translation from the last accepted frame reaches the distance
  threshold **or** wrapped heading change reaches the angle threshold. Start with
  configurable **0.5 m / 30 degrees**; permit **0.1 m** for selected dense surveys.
  These are initial sampling settings, not localization/object accuracy claims.
  Skip stale/blurred frames; bounded capture/storage work must not delay STOP,
  navigation or the watchdog. Stop accepting frames on invalid localization,
  timestamp/TF failure or storage exhaustion; report the blocking reason.
- **Pose binding:** reuse the already-used ROS1 `tf2_ros.Buffer` and listener.
  Query `map -> base_footprint` at the image's acquisition timestamp, rather than
  using `Navigation.get_pose()` unchanged: it currently queries the latest pose.
  Validate camera timestamps against ROS time; never substitute save time or a
  latest-pose fallback. Store a camera pose only when its calibrated TF chain is
  available. TF supplies coordinate transforms, not object distance or height.
- **Index:** save images and synchronized metadata into the same photo index.
  Manual/VLM labeling runs outside the motion loop; only confirmed item/station
  links enable Fetch. Reuse camera transport and image conversion/saving utilities;
  no separate survey service, follow_waypoints dependency or RTAB-Map stack.

### Minimal API contract (proposed, not implemented)

| Channel | Purpose |
| --- | --- |
| `GET /api/state` | Capabilities, limits, measured state, ownership, task phase, stream health, calibration readiness and applied no-go revision/readiness |
| `GET /api/map` | Map identity/revision, frame, width/height, resolution in m/cell, origin x/y/yaw with explicit units, and authenticated PNG URL tied to that revision |
| `GET /api/items`, `/api/stations` | Compact photo index and taught records |
| `GET /api/no-go-zones` | Active map's saved zone geometry and revision |
| `POST /api/control/claim`, `/release` | One expiring operator lease; observers cannot move the robot |
| `POST /api/commands` | Typed commands: navigate/fetch, survey start with ordered poses and validated capture thresholds, cancel, home/gripper, teach, photo capture/label, optional resolve, alignment measure/step/confirm, zone add/delete with expected revision |
| `POST /api/stop` | Priority stop/cancel with confirmed or unconfirmed result |
| `WSS /api/events` | State changes, map/zone revision and application notifications, map-frame robot pose/planner path with timestamps, and ordered, expiring leased manual input |
| Authenticated camera/image URLs | Two live streams and index photos; separate from control WebSocket |

Commands carry an ID, lease and bounded typed arguments. Distinguish accepted,
running and completed/failed; deduplicate requests and reject stale/out-of-order
manual inputs. Map goals include frame/map identity, units and heading. Start with
existing camera transport or MJPEG on the trusted LAN/VPN; measure frame age before
considering WebRTC. Never send base64 video through the control channel.

Use authenticated same-origin access over HTTPS or a trusted VPN; validate WebSocket
Origin/CSRF boundaries. The server watchdog stops motion on expired input even if
the browser sends no cleanup. Initial input timeout must be no looser than existing
0.6-second teleop behavior. Blur, hidden tab, pointer cancellation, mode/lease loss
and disconnect stop manual input; reconnect never resumes it.

Stale required pose/video blocks affected actions. STOP cancels navigation and
requests chassis/arm stop independently of blocking work; retain best-effort arm
stop semantics and display Stop unconfirmed until acknowledged. Never open the
gripper automatically on STOP. Software STOP does not replace the physical stop.

### Optional LLM credentials

Provision locally, outside Git, never in browser builds/storage, arguments, logs
or API responses. Use existing Ubuntu tools; verify the installed systemd version.

- systemd >=250: `systemd-creds` encrypted file, service
  `LoadCredentialEncrypted=llm_api_key:/etc/credstore.encrypted/myagv-llm.cred`;
  backend reads `llm_api_key` inside `$CREDENTIALS_DIRECTORY`.
- Older systemd: GPG symmetric encryption; separate passphrase, explicit startup
  unlock directly into process memory. Restart requires unlocking again.

Restrict file/directory permissions; chmod alone is not encryption. Host-key mode
allows unattended startup but does not protect a disk containing both ciphertext
and host key; do not assume a TPM. Settings show configured/locked/error only.
Missing key/network disables skill actions while manual control/index use remains.
See the [systemd-creds manual](https://manpages.ubuntu.com/manpages/noble/man1/systemd-creds.1.html)
and [GPG manual](https://manpages.ubuntu.com/manpages/focal/man1/gpg.1.html).

## 5. Local alignment and fetch sequence

1. Resolve/select an indexed item and preview its linked station/photo. Capture
   this run's measured startup base and arm poses before moving; never home implicitly.
2. Fold to transport posture; reuse existing navigation/pickup approach. Stop and
   verify standstill and fresh localization before deploying the arm.
3. Acquire a fresh downward image at a measured, settled arm pose. Confirm that
   the item is still present and select its intended grasp feature.
4. Use calibrated geometry at a **known target plane** for planar error. An
   uncalibrated crosshair supplies pixels only. Camera/gripper offsets and image
   orientation must be explicit; a camera center is not automatically the grip point.
5. Apply one bounded adjustment, wait for measured settling, then capture again.
   Limit step size, reachable workspace, total attempts and residual error.
   For longitudinal errors requiring chassis movement, retract to transport first,
   reposition the base, stop and redeploy; never silently creep with the arm extended.
6. Use taught grasp height for the initial fixed-surface setup. A single top-down
   RGB image does not measure absolute Z. Variable height needs calibrated stopped
   multi-view geometry or a depth/range sensor; otherwise require manual teaching.
   Separate cup rim height, object surface plane and intended grip height.
7. After alignment/height readiness, operator confirms grasp. Close, fold and
   inspect/check possession. Until a validated sensor/vision check or explicit
   operator confirmation, report Grasp unverified and hold before automatic return.
8. Return to captured startup base pose, restore startup arm pose and release.
   Show the actual completion/cancellation result; no Pause/Resume in the minimum API.

A failed target match, unknown Z, stale frame, excessive uncertainty/residual or
retry exhaustion stops alignment and offers manual correction. Do not infer
millimeter accuracy from model confidence. Test geometric error across the real
workspace before enabling automatic steps. Keep the front camera for driving and
context; it need not observe the cup's live height. The two cameras are not assumed
to be calibrated stereo.

## 6. Delivery and acceptance

1. Four-panel frontend mock, theme, responsive layout and persisted resize/drag;
   persistent Demo indication. No fake hardware state.
2. One backend: read-only map/state, both cameras, lease/watchdog/STOP, manual
   modes and measured teaching. Attach photos to stations; manually label/select.
3. Reuse navigation/fetch stages with a stopped manual alignment/grasp gate and
   startup return behavior. Add persisted no-go rectangles and global/local costmap
   enforcement before enabling web navigation. Validate on hardware before claiming
   autonomous pickup.
4. Calibrate fixed-plane geometry; implement measure/step/recheck with taught Z.
   Add optional photo-label/target-resolution skills after the deterministic flow works.
5. Add the ROS1 survey queue and front-camera keyframes to the same backend/index.
   Reuse single-goal navigation; verify capture-time TF and cancellation on hardware.

Variable-height estimation and a fast model remain separate, measured additions.

Acceptance for implementation:

- [ ] Four panels only; core information visible, diagnostics on demand; established
  dark palette, 8px corners, compact titles/actions and hidden navigation.
- [ ] Each panel resizes in both dimensions; drag handles, keyboard sizing,
  maximize/reset and valid layout persistence work. Check desktop/tablet/phone;
  STOP stays reachable and map/camera gestures do not move panels.
- [ ] Leaflet displays raster, real path and robot/goal/item overlays with matching
  coordinates. Check nonzero origin/rotation, Y orientation, cell centers and
  click-to-goal round trips. Resize preserves view; SLAM bounds/revision updates
  remain aligned. Clicking previews only; a new map identity invalidates pending goals.
- [ ] No-go add/delete persists and reapplies after restart. Invalid/stale edits
  and robot-overlapping zones are rejected. A zone across the direct route produces
  a detour or no path; blocked goals/staging/return and direct pickup alignment are
  rejected. Local rolling costmaps, sensor clearing and costmap reset preserve zones;
  deletion removes only virtual obstacles. Pending enforcement blocks navigation.
- [ ] Item photo/map/station links survive restart; map mismatch/unknown item
  cannot trigger fetch. No observation pose displayed as a measured object pose.
- [ ] Survey reuses ROS1 navigation with one active goal; ordered viewing poses
  respect no-go zones. Cancel/STOP/lease loss/failure clears the queue without restart.
- [ ] Keyframes follow configured distance/heading thresholds, including angle
  wraparound. Delayed images use acquisition-time TF; stale/missing transforms
  produce no false pose binding. Storage/model work cannot block motion stopping.
- [ ] Manual fallback works without a model key; skills cannot bypass validation.
- [ ] Blur/disconnect/lease expiry and missing browser cleanup stop input;
  duplicates and competing controllers cannot issue extra movement.
- [ ] Stale data and unknown Z block relevant actions; alignment uses calibrated
  measured steps and bounded retries, never an invented height or success badge.
- [ ] Teaching saves measured poses; transport posture gates chassis movement;
  fetch returns to this run's startup poses. Unverified grasp is explicit.

Physical commissioning must supply camera endpoints/orientation/timestamps, actual
ROS/Python runtime, transport posture and calibration results. This plan provisions
no credentials, installs no dependencies and performs no hardware operation.

## 7. Design provenance

The user's earlier reference is section 3, near line 87, of
`/Users/lincoln/Git/AstrBot/data/plugins/astrbot_plugin_thread_router/docs/plan/unified_memory_live_view_plan.md`.
Retain its compact borders/lists/palette and the supplied October 5 rules:
bold uppercase titles, yellow labels, single-line actions and hidden navigation.
Panel dragging is enabled only through stopped-state layout editing.
