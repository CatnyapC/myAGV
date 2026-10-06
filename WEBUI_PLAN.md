# Remote Robot Web UI — Frontend Plan

Date: 2026-10-06. Status: design proposal; no web frontend or backend implemented by this change.

## 1. Scope and existing behavior

Build a remote operator workstation for one myAGV with a side-mounted P340 arm,
a front camera and an arm-mounted downward-facing camera. Keep frontend and
backend separate. This document specifies the frontend, its interactions and
the backend contract it needs; it does not implement a robot service.

Existing behavior to preserve:

- `teleop_control.py`: BASE, ARM and PICKUP modes; arm homing, bounded jogging,
  gripper commands and stopped-pose station recording.
- `navigation.py`: map localization, named destinations, pickup approach,
  navigation cancellation and return trips.
- `fetch_demo.py`: capture this run's startup base/arm poses, travel, replay the
  taught grasp, return and restore the startup arm pose. No separate dropoff.
- `stations.json`: item names, map poses and measured arm joint angles.
- J1=0 reaches to the robot's left. PICKUP teaching uses base forward/backward,
  arm reach and height. Preserve that convention in labels and diagrams.

These are software capabilities, not proof of successful hardware acceptance.
See [teleop](TELEOP.md), [navigation](NAVIGATION.md) and
[pickup acceptance](PICKUP_TASKS.md). Camera streaming and camera-based alignment
are new integration work. A completed gripper command does not prove possession.

First release: live observation, manual control, station teaching and existing
navigation/fetch operations. Later: calibrated visual alignment, multi-view Z
estimation, distance-triggered photographs and semantic goal selection.

## 2. Reference selection

Official pages reviewed on 2026-10-06. Use the selected references as a coherent
workstation design; do not combine two complete admin themes.

| Reference | Useful existing material | Decision |
| --- | --- | --- |
| [shadcn Mail](https://v3.shadcn.com/examples/mail) | Multi-column workspace, dense lists, compact icon actions and dark styling | Primary visual reference. Adapt the shell and selected controls; replace mail content with robot panels. The v3 page is a visual reference, not a dependency-version instruction. |
| [Tabler Admin](https://tabler.io/admin-template) | Bootstrap-based admin template, dark layouts, forms and dense status displays; MIT core | Secondary reference for information density. Do not install Bootstrap or purchase Pro for this design. |
| [Radix Colors](https://www.radix-ui.com/colors) | Ready-made dark neutral and accent scales | Use Gray Dark with Orange and Cyan; Yellow for subsection labels. |
| [GridStack](https://gridstackjs.com/) | Movable, independently resizable dashboard widgets | Use for the outer panel layout. Persist layout through its serialization support. |

Recommended frontend: React + TypeScript + Vite, selected shadcn/ui components,
Radix color CSS and GridStack. The repository currently has no frontend stack to
reuse. [shadcn's Vite setup](https://ui.shadcn.com/docs/installation/vite) supports
this choice without requiring Next.js. Use one icon family, Lucide, throughout.
Pin compatible versions during implementation and preserve upstream licenses.

GridStack owns outer widget geometry; React owns stable panel content. Validate
their integration against the [official React example](https://gridstackjs.com/demo/react.html)
before adding robot data. Do not also install a split-pane engine. Use native
fetch, WebSocket and React state initially; no global state framework or dashboard
starter is needed for this scope.

## 3. Visual rules

Carry forward the supplied October 2 reference rules and October 5 additions:

- Dark mode is the default and the first-release theme.
- Rectangular panels with **8px corner radius**, **1px borders**, 8–12px gaps
  and 12–16px workspace padding. No nested cards, large shadows, neon or
  gradient backgrounds. Use lists, rows and separators inside panels.
- **Bold uppercase English page and panel titles**, compact rather than large:
  `MYAGV CONTROL`, `MAP`, `FRONT CAMERA`, `ARM CAMERA`, `CONTROL`, `STATIONS`, `TASK`.
  Use 14–16px, weight 700, with restrained letter spacing. CJK labels remain natural.
- **Yellow subsection labels**, 12–13px semibold. Body text 13–14px; units,
  coordinates and timestamps use tabular numerals. Do not color entire paragraphs.
- **Single-line action bars**. On narrow widths, move secondary actions into
  an overflow menu instead of wrapping. Keep STOP and control ownership visible.
- **Hidden side navigation** by default, opened from the top-left menu button.
  Desktop may pin it; smaller screens use a dismissible drawer. Navigation lists
  Workspace, Stations and Settings. It is separate from the resizable panel grid.
- Compact 16–18px icons with tooltips and accessible names. Desktop icon buttons
  have at least 32px hit areas; touch movement controls have at least 44px hit areas.
- Cyan keyboard focus remains visible. State always has a text/icon label as
  well as color. Verify text contrast and control boundaries in the actual theme.

### Core information only

Show only information needed to understand the robot's current state or choose
the next action. This rule governs default visibility throughout the plan;
available data does not automatically deserve permanent space on the page.

- Keep connection/control ownership, STOP, active mode/target, relevant controls,
  map, camera views and the current task phase visible. Show blocking errors where
  the affected action lives, with a short reason.
- Use short labels and compact values. Avoid introductory paragraphs, repeated
  panel descriptions, decorative slogans, redundant status badges and permanent
  instructions explaining obvious controls.
- Put full coordinates, map/stream IDs, timestamps, calibration metadata, task
  history and diagnostic logs in Details, tooltips or an optional EVENTS panel
  hidden by default. Expose measured values when teaching or aligning needs them.
- Show freshness problems and unconfirmed outcomes when relevant; do not repeat
  healthy-state explanations across panels. Essential feedback must remain visible
  and must not depend on a tooltip.
- Before adding text, ask whether it changes the operator's next decision. If
  not, remove it from the default view or move it into on-demand details.

### Theme tokens

Use semantic aliases over Radix dark scales, following the official
[scale guidance](https://www.radix-ui.com/colors/docs/palette-composition/understanding-the-scale).
The semantic assignments below are this project's design choices.

| Alias | Radix token | Usage |
| --- | --- | --- |
| `app-bg` / `panel-bg` | Gray 1 / Gray 2 | Workspace and flat panel surfaces |
| `control-bg` / `control-hover` | Gray 3 / Gray 4 | Inputs and neutral controls |
| `separator` / `control-border` | Gray 6 / Gray 7 | Fine dividers and inputs |
| `text-muted` / `text` | Gray 11 / Gray 12 | Secondary and primary text |
| `execute` / `execute-hover` | Orange 9 / Orange 10 | Start navigation/fetch, record/save, home, grip/release |
| `execute-text` | Orange 11 | Execution status text on dark surfaces |
| `selected-bg` / `selected-text` | Cyan 3 / Cyan 11 | Selection, loading a saved target, active view |
| `focus` | Cyan 8 | Focus ring; verify contrast against each adjacent surface |
| `subheading` | Yellow 11 | Small subsection headings only |
| `danger` / `danger-text` | Red 9 / Red 11 | STOP, failures and destructive actions |

Orange means executing/writing; Cyan means recalling/loading, selecting or focusing.
Here, recall means loading a saved station for inspection, not starting robot motion.
Yellow is a supporting typographic accent, not a third primary action color.
Use a tested contrasting foreground on solid buttons; do not assume white works
on every accent. Neutral status text plus a small green indicator may show health.

## 4. Workspace and panel layout

Desktop starting layout at roughly 1440px width, with navigation hidden:

```text
+--------------------------------------------------------------------------------+
| [Menu] MYAGV CONTROL   Robot / connection   Observer / Operator   [Take] [STOP]   |
+--------------------------------------------------------------------------------+
| [Manual / Navigate / Fetch]   Target [v]   [Execute]   [Layout]   [More]           |
+--------------------------------------+---------------------+-------------------+
| MAP                                  | FRONT CAMERA        | CONTROL           |
| Pose / path / goal                    | Live frame + age    | BASE / ARM /      |
|                                      |                     | PICKUP            |
|                                      +---------------------+                   |
|                                      | ARM CAMERA          | Jog / grip / home |
|                                      | Downward view + age | Measured feedback |
+-----------------------+--------------+---------------------+-------------------+
| STATIONS              | TASK                              | EVENTS             |
| Names / taught poses  | Phase / return pose / cancel      | Time / state/error |
+-----------------------+-----------------------------------+--------------------+
```

Every named content panel is an independent widget, including both cameras.
The top bar and action bar stay outside the grid. STOP remains visible when a
panel is maximized, navigation is open or a dialog is displayed.

Default geometry: 24 columns, 40px row unit, 8px gaps. Main region: map 12 columns,
camera column 6, controls 6; map/controls 12 rows, each camera 6 rows. Lower region:
stations 6 columns, task 12, events 6, each initially 6 rows. These are editable
starting sizes. Allow vertical page scrolling rather than compressing controls.

| Panel | Minimum usable content size | Required behavior |
| --- | --- | --- |
| MAP | 360 × 280px | Pan/zoom, fit robot/map, current heading, selected station, preview goal and path. Show map ID and localization freshness. |
| FRONT CAMERA | 280 × 180px | Navigation view; timestamp/age, unavailable/stale state, maximize. Preserve aspect ratio. |
| ARM CAMERA | 280 × 180px | Downward pickup view; separate stream identity and orientation label. Optional visual crosshair only. |
| CONTROL | 280 × 360px | Explicit mode, hold-to-move, speed units, homing status, gripper actions and measured pose. Scroll body if needed. |
| STATIONS | 280 × 200px | Select/inspect saved item, record new pose, explicit overwrite, navigate/fetch actions. |
| TASK | 320 × 200px | Actual backend phase, elapsed time, captured return pose, cancellation and terminal result. |
| EVENTS | 280 × 160px | Bounded recent event list, severity filter and clear local display. No shell terminal. |

### Resize, drag and persistence

- Each panel can resize horizontally, vertically and from a corner in grid units.
  GridStack handles collision and reflow; panels never overlap.
- Drag only by a dedicated title-bar handle. Map panning, text selection, camera
  inspection and control presses must not drag the panel. Layout editing starts
  locked; unlock through Layout while stopped, then relock for operation.
- Provide maximize/restore, show/hide, reset layout and width/height inputs for
  keyboard users. During active manual control, keep CONTROL visible.
- Recompute map canvas and video bounds on resize with ResizeObserver. Preserve
  camera sessions, map viewport, selection and form edits across layout changes.
- Save validated panel IDs and geometry, visibility, schema version and breakpoint
  to localStorage, scoped to this UI and robot identity. Use GridStack's
  [save/load pattern](https://gridstackjs.com/demo/serialization.html); serialize
  layout only, never panel HTML, credentials or robot state.
- Restore known IDs only, clamp sizes, discard invalid/old layouts and fall back
  to defaults if storage is blocked. Separate desktop and narrow-screen layouts.
- At 1024–1279px, use two columns. Below 1024px, use single-column panels or view
  tabs with persistent STOP and ownership state. Collapse earlier whenever panel
  minimum widths cannot fit. Secondary toolbar actions move into More.

## 5. Operator workflows

### Connect and observe

Start as Observer. Display connection, localization, controller ownership and
camera state independently; a live video stream does not prove control is connected.
Show explicit Loading, Empty, Offline, Stale, Busy and Error states. Missing battery
or sensor data reads Unavailable, never a fabricated zero or green status.

### Manual control and pickup teaching

Take control explicitly. Use the existing BASE / ARM / PICKUP conventions with
visible mode labels and axis units. BASE covers forward/back, turn and strafe.
ARM covers existing unrestricted axes. PICKUP exposes forward/back base motion,
slow heading adjustment, arm extension/retraction and Z height; J1 must be near zero.

Movement is hold-to-run with pointer capture and an explicit keyboard-control
toggle. Release, pointer cancellation, blur, hidden tab, mode change, lease loss
or disconnect ends manual input. Typing into a field never drives the robot.
Use the documented speed limits as initial values; actual allowed ranges come
from the backend. Never remove homing, coordinate or feedback checks for UI convenience.

Recording stops motion, waits for standstill, displays measured base/arm poses,
then asks for the item name. An overwrite requires an explicit overwrite action.
Do not save the last requested pose as measured feedback. Save completion appears
only after backend acknowledgement; a late response must not overwrite newer edits.

### Navigate and fetch

Selecting a station or clicking the map only previews a goal. An orange Execute
action sends it. A map goal includes heading; arbitrary map navigation must not
pretend to supply the taught arm pose needed for fetching.

Before fetch, show the chosen station, transport-pose readiness and return-to-start
behavior. Preserve the existing startup base and arm capture; never home implicitly.
Display backend stages: preparing, travel to staging, final alignment, grasp,
fold, return, restore arm, release, complete or failed/cancelled.

Navigation success, gripper command completion and grasp confirmation are separate
states. Display "Grasp unverified" until a real sensor or validated vision result
supports a stronger statement. Provide Cancel, but no Pause/Resume until backend
semantics exist. Cancelled means cancellation was confirmed, not merely requested.

### Cameras and future alignment

Show front and downward cameras simultaneously on desktop. Label source, image
orientation, frame age and stream health. Do not crop away the pickup area; any
rotation/mirroring must be explicit and must not silently invert control axes.

First release supports observation and manual alignment. A crosshair is an image
reference, not a calibrated gripper target. Do not display a millimeter Z estimate
or an automatic Align action before calibration and backend support exist.

A later alignment workflow can capture several stopped views with measured camera
poses, estimate object geometry and preview a proposed grasp pose. It needs camera
intrinsics, camera-to-arm calibration, frame/pose timestamps and an uncertainty
check. Cup rim height and intended grip height remain separate values. Front and
arm cameras are not assumed to be a calibrated stereo pair.

## 6. Frontend/backend boundary

Proposed deployment: independently built static frontend under `web/`; robot-side
service in the ROS-compatible Python environment. Separate source/build and API
contracts; a reverse proxy may expose both under one HTTPS origin. This avoids
browser access to serial ports, ROS topics or executable shell commands.

```text
Browser: panels + presentation + operator input
       | HTTPS snapshots/requests, WSS state + manual input, separate video
Robot service: authentication + ownership + validation + task/controller adapter
       | reuse current navigation, teaching and arm behavior
ROS1 / chassis / P340 / cameras
```

The service must own a single robot controller. Existing terminal teleop and fetch
cannot run alongside it. Reuse tested control helpers behind an adapter; do not
spawn a process per keypress or scrape CLI output as the long-term state contract.
Blocking arm operations must not block the service watchdog or STOP handling.

### Minimum proposed contract (not existing endpoints)

| Channel | Frontend needs |
| --- | --- |
| `GET /api/state` | Robot/session identity, capabilities, limits, state revision, ownership, modes, measured poses, health, active task and stream metadata |
| `GET /api/stations` | Existing item records, revision and associated active map identity |
| `POST /api/control/claim`, `/release` | Explicit operator lease with expiry and owner display; no silent takeover |
| `POST /api/commands` | Typed bounded commands with command ID, lease ID and target; accepted/rejected reply, later execution result |
| `POST /api/stop` | Priority stop/cancel request from an authorized operator, followed by confirmed or unconfirmed result |
| `WSS /api/events` | State/phase updates and command results; leased manual input with sequence numbers and short expiry |
| Camera stream descriptors | Independent front/arm stream identity, transport URL, frame timestamps and availability |

Command types cover mode selection, home, gripper, station capture/save, navigate,
fetch and cancel. Map snapshot/overlay data must include map ID, frame, resolution,
origin pose and revision so the frontend converts pixels to metric goals correctly.
Pose fields use named units: base meters/radians, arm millimeters/degrees. Avoid
anonymous XYZ arrays crossing coordinate systems.

Requests carry unique IDs. Backend deduplicates discrete execution commands and
rejects expired/out-of-order motion messages; frontend never replays movement
after reconnect. On reconnect fetch a fresh snapshot, discard obsolete revisions
and show any surviving task's real state. First-release policy: loss of the active
control lease requests stop/cancel for manual and automatic motion, with no automatic
resume. A normal panel/view change does not create a new robot session.

Start video integration with existing stream outputs if available. Otherwise try
separate low-resolution MJPEG streams for the LAN prototype, measure frame age and
Pi load with both cameras, and choose WebRTC if measured latency/bandwidth requires
it. Video frames do not travel as base64 in the control WebSocket. If trustworthy
capture timing is unavailable, label latency Unknown and block camera-dependent
remote motion until freshness can be established.

### Remote-control requirements reflected in the UI

- Authenticated access; operator mutations validated server-side. Prefer one
  same-origin gateway with secure sessions and Origin/CSRF checks as appropriate.
  Initial access is through a trusted LAN/VPN, not unauthenticated public ports.
- One active operator lease, multiple observers. A second controller sees Busy.
  Server-side expiry/watchdog stops input even if browser cleanup never executes.
- Tune watchdog and frame-age thresholds during hardware tests; an initial manual
  input budget must be no looser than the existing 0.6-second teleop timeout.
- Loss of required telemetry or video disables affected actions and requests stop
  for camera-dependent manual motion. Preserve explicit last-known values with
  stale labels; never portray cached frames or poses as live.
- STOP is a software request, not a certified hardware emergency stop. It requests
  chassis stop, navigation cancellation and best-effort arm stop without opening
  the gripper automatically. Until acknowledged, show "Stop unconfirmed".

## 7. Delivery sequence and acceptance

1. **Frontend mock:** implement theme, shell, seven independent panels and typed
   fixtures for normal/offline/stale/busy/error states. Display a persistent Demo
   label. Finish layout interactions without connecting to hardware.
2. **Read-only integration:** authenticate; show map, measured state, stations,
   task events and both cameras. Measure stream age and resource use.
3. **Manual operation and teaching:** integrate ownership, watchdog, STOP, modes,
   arm/gripper controls and measured station capture. Validate empty-gripper motion.
4. **Navigation and fetch:** integrate goal preview, execution and cancellation;
   preserve startup return behavior and the unverified-grasp distinction.
5. **Later vision work:** implement calibrated capture and alignment only after
   its measurement accuracy has been demonstrated on the physical robot.

Acceptance checklist for implementation:

- [ ] Default views contain only core state, actions and relevant feedback;
  explanations, repeated labels and diagnostics appear only on demand.
- [ ] Match 8px radii, fine borders, uppercase bold titles, yellow subsection labels,
  single-line actions and hidden navigation; no nested-card styling.
- [ ] Resize each panel in both dimensions; drag by handle; maximize/restore;
  refresh preserves valid layout; invalid storage restores defaults.
- [ ] Check 1440×900, 1280×800, 768×1024 and 390×844. No overlapping controls or
  inaccessible STOP; narrow camera panels preserve aspect ratio.
- [ ] Keyboard resize, focus indicators and named icon controls work. Camera/map
  gestures never move panels. Form typing never triggers robot motion.
- [ ] Camera sources and stale states remain independent; unavailable telemetry
  cannot appear healthy. No fake alignment accuracy or grasp success.
- [ ] Losing focus, pointer, network or ownership during jog stops input; backend
  watchdog test covers a browser that sends no cleanup message.
- [ ] Duplicate/late commands and competing operators cannot issue extra movement.
  Reconnect never resumes motion. Stop/cancel results remain honest on failure.
- [ ] Teaching stores measured poses; fetch returns to this run's startup poses.
  Hardware validation is recorded separately from frontend and mocked checks.

Before hardware integration, resolve actual camera endpoints/orientation, available
stream timestamps, ROS deployment details and measured transport pose. These do
not block the frontend mock. No dependency installation or hardware operation is
part of this planning change.

## 8. Local design provenance

The supplied earlier design is in
`/Users/lincoln/Git/AstrBot/data/plugins/astrbot_plugin_thread_router/docs/plan/unified_memory_live_view_plan.md`,
section 3 (starting near line 87). It informs visual density, palette and panel
behavior only; its memory-specific workflow does not belong in this robot UI.
The October 5 additions are taken directly from the user's instructions. Unlike
that earlier page's initial resize-only policy, this plan includes the requested
panel dragging behind the layout lock.
