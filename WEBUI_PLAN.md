# Minimal Remote Robot UI and Service Plan

Updated: 2026-10-07. Status: implementation plan; this revision changes documentation
only. The repository has robot control/navigation code, but no web UI, web API,
photo index or calibrated visual alignment implementation yet.

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
| Route planning | Existing deterministic navigation code |
| Final alignment | Calibrated top-down image geometry, bounded motion and fresh feedback |
| Grasp and return | Existing taught arm poses and this run's startup return poses |

No VLMaps/3D reconstruction, feature voxel map, vector database, local model server,
VLA or mandatory fast multimodal model in the minimum release. Add a model to
local alignment only if measured hardware results show geometry is insufficient.
Skills are bounded backend workflows, not independent services or motion authorities.

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
| MAP | Occupancy map, robot heading, selected item's observation/approach markers, proposed goal and active path; pan/zoom/fit | 360 × 280px |
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
Reference: [official GMapping configuration](https://github.com/elephantrobotics/myagv_ros/blob/myagv_ros_2023Pi/myagv_navigation/launch/gmapping.launch)
and [navigation configuration](https://github.com/elephantrobotics/myagv_ros/blob/myagv_ros_2023Pi/myagv_navigation/launch/navigation_active.launch).

## 4. Minimum robot-side services and data

Run **one Python backend service**, in the robot's existing ROS-compatible runtime.
Serve the independently built static frontend from it under the same origin.
The frontend executes in the remote browser; control, files and model keys remain
on the robot. Reuse ROS navigation/drivers and existing camera publishers when
available; otherwise use one capture worker for both cameras, not a new camera API
service per device. Confirm the actual ROS/Python compatibility before integration.

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
- Existing `stations.json`, unchanged: measured base pose and arm joint angles.
  Its validator rejects extra fields; do not insert photo/semantic fields there.
- One `photo_index.json` plus an image directory, outside tracked source/runtime
  secrets. Each observation stores item ID/name/appearance, image reference,
  capture time, map ID/revision, measured capture base pose and camera identity.
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

### Minimal API contract (proposed, not implemented)

| Channel | Purpose |
| --- | --- |
| `GET /api/state` | Capabilities, limits, measured state, ownership, task phase, stream health and calibration readiness |
| `GET /api/map` | Map identity/revision, frame, width/height, resolution in m/cell, origin x/y/yaw with explicit units, and authenticated PNG URL tied to that revision |
| `GET /api/items`, `/api/stations` | Compact photo index and taught records |
| `POST /api/control/claim`, `/release` | One expiring operator lease; observers cannot move the robot |
| `POST /api/commands` | Typed commands: navigate/fetch, home/gripper, teach, photo capture/label, optional resolve, alignment measure/step/confirm |
| `POST /api/stop` | Priority stop/cancel with confirmed or unconfirmed result |
| `WSS /api/events` | State changes, map revision notifications, map-frame robot pose/planner path with timestamps, and ordered, expiring leased manual input |
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
   startup return behavior. Validate on hardware before claiming autonomous pickup.
4. Calibrate fixed-plane geometry; implement measure/step/recheck with taught Z.
   Add optional photo-label/target-resolution skills after the deterministic flow works.

Later, distance-triggered front photos may be captured every 10cm of measured
travel with timestamps/map poses and image/pose synchronization checks. Stop capture
on invalid localization, bound storage and process labels outside the motion loop.
This adds observations to the same index; it does not require a 3D mapping service.
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
- [ ] Item photo/map/station links survive restart; map mismatch/unknown item
  cannot trigger fetch. No observation pose displayed as a measured object pose.
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
