# Robot WebUI

The WebUI controls ROS1 Noetic and the P340 through one ROS/serial process.
The web API uses Python 3.11+; `ros_web_bridge.py` uses Ubuntu Python 3.8.
Atomic snapshots carry telemetry. A local Unix socket carries commands and STOP.
Starting PM2 does not home the arm or send a navigation goal.

## Install and start

Close `myAGV_UI`, teleop and other ROS controllers first. Do not run a second
chassis driver or serial client. The vendor chassis driver retains its last
velocity indefinitely. Install the source watchdog and rebuild before enabling
WebUI driving:

```bash
cd ~/myAGV
git pull --ff-only
source /opt/ros/noetic/setup.bash
source ~/myagv_ros/devel/setup.bash
python3 install_driver_watchdog.py ~/myagv_ros/src/myagv_odometry/src/myAGVSub.cpp
cd ~/myagv_ros
catkin_make --pkg myagv_odometry -j2
cd ~/myAGV
uv sync --locked --extra web
npm --prefix web ci
npm --prefix web run build
pm2 stop myagv-bridge myagv-localization myagv-ros
pm2 startOrRestart ecosystem.robot.config.cjs
MYAGV_HARDWARE_DIR="$HOME/myAGV/web_runtime/ros" \
MYAGV_WEB_ORIGIN="http://172.28.13.26:5173" \
pm2 restart myagv-api --update-env
```

The driver patch uses a wall-clock loop and makes commands expire after 0.5 seconds. It refuses
unknown source layouts and saves `.before-web-watchdog` beside the source.
The ROS launch initially starts map_server, AMCL and move_base. Both costmaps consume
`/navigation_map`. The move_base output is remapped to `/myagv/input_vel`; only
the bridge publishes `/cmd_vel`. Fresh LiDAR/odometry, exclusive publishers and
the driver watchdog are required for motion.

## Initial setup in the UI

The initial map is a saved map: AMCL updates the robot pose, not the occupancy
cells. In the MAP toolbar, **Reset map** opens a confirmation to archive the
current map, no-go zones and taught stations and start headless Gmapping. The
robot must be idle and measured stopped; no arm connection or homing is needed.
The saved room YAML/PGM, bridge map snapshots and pending captures are archived
under `web_runtime/map_backups/<session>/`, with original paths in `manifest.json`.
Photos remain in the library; old map positions and station associations become
invalid. A startup failure restores the archived files.

After reset, the label becomes **Static SLAM snapshot**. SLAM continues collecting
scans through the small-obstacle filter. The map-top slider sets the minimum
isolated obstacle size (default 15 cm; 0 disables it). It filters scan clusters
before SLAM and both costmaps, and occupied map components before no-go masks.
Long walls and explicitly drawn no-go zones remain. Change it while idle;
the threshold persists across restarts and map resets. Raw map cells are retained
so lowering the threshold restores components in the accepted snapshot.
SLAM continues collecting
scans, but the displayed map and navigation map stay fixed. **Pause updates**
waits for the base to stop, then publishes the latest complete snapshot once.
It also works while idle, without starting a survey. Time-limit pauses publish
the snapshot too. Robot position remains live. Map identity stays fixed within
one SLAM session; existing no-go zones keep their world coordinates. SLAM
confirms its pose from fresh sensors automatically. **Set pose from goal** is
disabled. The accepted snapshot survives API and bridge restarts. Mapping mode survives service
restarts through `web_runtime/mapping.json`. Restarting the SLAM launch begins
a fresh map session; stale no-go zones and taught stations are archived and
must be taught again. Restarting only the API or bridge keeps the SLAM session.

1. Reload the page. Check front/arm camera roles and live ROS pose.
2. Enter the actual robot pose in Goal, use **Set pose from goal**, then verify
   the map position and heading. Use **Confirm localization** only after checking.
   RViz scan overlay remains useful for precise AMCL validation.
3. **Home arm** explicitly moves the arm. Clear its workspace first. **Already
   homed** is only for an arm homed since its last power-on. A process restart
   clears this confirmation.
4. In ARM/PICKUP mode, hold the manual buttons to position the arm. Motion uses
   bounded 1 mm targets. PICKUP uses logical X forward/back, Y right/left and Z
   height with the arm facing forward at J1=90 degrees.
5. Fold the arm and measure a circular clearance radius covering the
   base, folded arm and carried object. Check the measurement box and **Record
   transport pose**. This updates both ROS planner footprints. No transport pose
   is guessed. Fetch and survey remain disabled until this is done.
   Transport keeps the measured joint angles, including J1 rotation. Front
   pickup station poses require J1 near 90 degrees.
6. Teach a named pickup station at its real base/arm pose (J1=90).
   Existing names require overwrite confirmation; changed station links must be
   reconfirmed. Capture a photo, edit its item, and explicitly associate that station.

Manual BASE driving is supervised and bypasses no-go planning. It uses at most
0.1 m/s and 0.3 rad/s; PICKUP base motion uses 0.03 m/s and 0.05 rad/s. Releasing
buttons stops the task. Fetch folds the arm before travel. Map updates use the
front camera and the same base-only readiness conditions as Go. They do not
require arm homing or move the arm. Select a goal and press **Start updates**,
or add several goals under **Manual update points**. Disabled-start reasons
appear below the buttons. Starting updates also validates each selected goal;
it does not require the robot's current footprint to be observed as free.

## Navigation, no-go zones and Fetch

**Go** sends the selected map-frame goal to move_base. A goal requires current
localization, live sensors, planner footprint clearance and both costmaps applying
the current no-go revision. Go does not require an online or homed arm or a
recorded transport pose, and does not move the arm. Unknown cells are treated as
free in the derived navigation map; the displayed source map keeps them unknown.
Occupied footprint clearance and goals outside map bounds are rejected. Go,
Fetch and map surveys share this policy. Live obstacles and no-go zones remain blocked.
No-go rectangles are rasterized into a derived map. Both global and rolling
local StaticLayers consume it; published full costmaps are checked for virtual
obstacles. The original map is preserved. Recovery rotations are disabled.

**Fetch** requires an explicitly selected item linked to an unchanged taught
station on the current map. It captures the startup base/arm poses, folds,
approaches the station, opens the gripper and reaches the taught arm pose.
With the configured LLM key, it aligns using fresh front and arm camera images,
then pauses for live-camera alignment review; bounded X/Y/Z adjustments are
available. After confirmation it closes, folds, and pauses again for the user
to confirm possession. Only then does it return and restore the startup pose.
There is no force-based grasp detection. Without an LLM key, alignment stays manual.
A five-minute review timeout cancels the task.

### Dual-camera Fetch calibration

The arm base mounts facing chassis left; J1=90 faces forward. The arm camera
mounts 90 degrees clockwise, with the target near the bottom of its image.
Its image X controls logical arm X (chassis forward/back). Front camera image X
controls logical arm Y (chassis right/left). A target right of the alignment point
commands positive motion on either logical axis. The SDK receives native +Y for
logical X+, and native -X for logical Y+; images retain their mounted orientation.

Import and photo/item editing open as a child page inside OPERATE. The upper-left
back arrow returns to its main controls. Save item & capture both cameras records
the current, operator-positioned correct grasp without moving the robot. It needs
an idle, localized robot and homed arm. New items are saved before acquisition.
The paired goal photos remain in the photo index with a compressed JPEG copy for
LLM input. Deleting or reassigning either goal photo invalidates that item's goal.

Each round sends current front/arm views, the item's correct-grasp GOAL views and
the previous two image pairs with their executed X/Y commands. All LLM images are
JPEGs (longest side at most 480 pixels, quality 60, low detail). The configured
DeepSeek Flash model uses observed image response to adapt the next correction;
there is no fixed pixel-to-mm gain or assumed center alignment point. Thinking
is disabled and output is capped at 64 tokens: `{"x_mm":0.5,"y_mm":-1}`.
Both zeros mean the current grasp matches both GOAL views. Missing/ambiguous
targets or goals use `null` and stop Fetch. Only these fields are accepted.
The controller checks and bounds arm targets; the model cannot close the gripper,
drive the base or change Z. A configured-LLM Fetch needs a saved goal for the item.
It recaptures after each correction and stops on malformed output, timeout,
expired control lease, changed map/zones, missing sensors or non-convergence.

The GRASP TEST section below TASK in OPERATE edits the shared calibration prompt,
per-axis step limit and combined X/Y travel budget. Save settings applies them to
normal Fetch; both test buttons save edits before starting. Settings persist in
`web_runtime/fetch_settings.json`. Preview one round captures current views and
requests a suggestion without moving. Align here runs the same bounded alignment
from the current arm pose, with no navigation or gripper command. Both need a
localized, homed arm at J1=90 and a selected item with saved GOAL views. Testing
does not require a station association.

LLM ROUNDS displays each round's GOAL, chronological history and current input
photos, actual sent prompt/item/limits, raw returned JSON, limited suggestion,
elapsed time and errors. Suggestions do not confirm successful movement; TASK
shows controller completion or faults. Photos are returned only when the log
revision changes. Logs hold the latest task's nine rounds in API memory and clear
on API restart. They contain no provider key.

The optional `fetch_calibration` object in `web_runtime/robot_config.json` remains
a fallback for commands outside the WebUI. Old pixel target/gain fields are ignored.
Old J1=0 stations must be
physically re-taught at J1=90 and their photo associations reconfirmed.

```json
"fetch_calibration": {
  "max_step_mm": 2,
  "max_total_mm": 20
}
```

Defaults permit at most eight corrections and 20 mm combined travel. A ninth
observation verifies the final correction without moving again. Each
correction is divided into absolute waypoints no larger than 1 mm, with measured
feedback. Grasp and possession still require the existing operator confirmations.

**Map updates** repeatedly refresh observations in the current reachable area.
Use **Preview update points**, then **Start updating**. **Pause updating** retains
the current view and all map/photo records. The default window is 10 minutes (1–60 minutes);
expiry pauses updating. Starting again after expiry opens a new window. Operator
pauses count toward the same wall-clock window. STOP/disconnect cancel motion;
reconnection never starts updating automatically.

Automatic points use the robot's connected free global-costmap region, measured
clearance and no-go enforcement. Unknown/disconnected/blocked space is excluded.
Default viewing spacing is 1 m (0.5–3 m), with four stopped headings at each site
and at most 200 views spread across the region. Square clearance is conservative.
Points are viewing samples, not guaranteed visual coverage. A new map revision
refreshes the automatic points on the next round. Up to 20 manually
selected views can instead be repeated. Preview markers show headings; move_base
still computes the actual travel paths. Every goal is revalidated before travel.

Each revisit replaces its latest unlabelled photo. Confirmed old photos stay as
historical evidence; new images never inherit a moving object's old identity.
Historical associations cannot authorize Fetch until confirmed in a current
observation. This refreshes scene evidence, not measured object coordinates or
automatic identity tracking. Capture/import failure stops updating and preserves
the source frame for recovery; a 20-frame handoff backlog pauses updates.

Live captures bind the received camera frame to TF at that frame's host receipt
time, map identity and camera identity. The robot must be stopped. Arm captures
also require stable arm feedback. These USB cameras expose no calibrated exposure
clock; metadata labels the timestamp `host_frame_receipt`. Captures are not
precision visual-servo measurements. During active tasks the camera panels show
live frames, even if a stored photo was selected previously.

## STOP and reconnect behavior

Starting `teleop_control.py` requests a local hardware handoff before opening
the P340 or publishing chassis commands. WebUI releases the arm serial port and
pauses its chassis output while continuing camera/map monitoring. Active WebUI
tasks must be stopped first. The terminal owns the hardware until TELEOP exits;
browser blur or disconnect does not stop terminal control. An explicit WebUI
STOP terminates TELEOP and leaves motion locked pending measured stop recovery.
After TELEOP exits, verify stopped hardware in WebUI and reconfirm homing and
localization. TELEOP also opens the arm with exclusive serial access.

The server sends control challenges every 150 ms. Automatic tasks (Go, Fetch,
map updates and other commands) accept replies within 5 seconds and have a
60-second control lease, tolerating network stalls. Manual control accepts
replies within 300 ms and keeps a 450 ms lease, enforced by the ROS controller.
Explicit STOP and controlling-socket disconnect still stop immediately. Stale/duplicate
heartbeats cannot extend a lease. Another tab cannot renew or take over an
active task. Tab blur or hiding clears held manual input. Disconnect, API shutdown,
stale sensors and STOP cancel the task. STOP uses an independent socket request and invalidates queued
commands. Reconnecting never resumes motion; explicitly issue a new command.

STOP cancels move_base and sends zero base velocity plus P340 M15. UI reports
**stopping** until fresh odometry and stable arm feedback confirm a stop.
Unavailable feedback reports **Stop unconfirmed** and locks motion. After physically
checking that both mechanisms stopped, use **Verify stopped and clear fault**;
fresh base/arm feedback is still required and homing confirmation is reset. It never substitutes an
acknowledgement for feedback.

The manufacturer's [P340 protocol](https://docs.elephantrobotics.com/docs/ultraArm-en/3-HowToUseultraArm/2-SoftwareControl/6-SerialPort/6-SerialPort.html)
documents M15 as jog stop, not a guaranteed emergency stop for homing or queued
absolute moves. Automatic arm travel therefore sends bounded 2-degree waypoints;
cancellation prevents subsequent waypoints. The current waypoint or gripper
command may finish. Homing cancellation is reported unconfirmed and requires
re-homing. Web STOP is not a substitute for physical power/emergency control.
Physical stopping distance and firmware behavior need supervised acceptance.

## Devices and map

Defaults: front Raspberry Pi `camera0` at `/dev/video2`, arm USB camera at
`/dev/v4l/by-id/usb-USB_corp._usb_camera_usb_camera-video-index0`,
P340 `/dev/ttyUSB0` at 115200. The USB path follows the capture device across
reconnections; its `video-index1` node carries metadata, not images.
Verify these roles. Override camera devices when restarting the bridge:

```bash
MYAGV_FRONT_DEVICE=/dev/video2 \
MYAGV_ARM_DEVICE=/dev/v4l/by-id/usb-USB_corp._usb_camera_usb_camera-video-index0 \
pm2 startOrRestart ecosystem.robot.config.cjs --only myagv-bridge --update-env
```

Set `MYAGV_P340_PORT` in the bridge environment for a stable `/dev/serial/by-id`
path. Configure `MYAGV_MAP_FILE` for localization; default `~/maps/room.yaml`.
A map identity change invalidates localization and photo/station associations.
If saved hardware zones belong to another map, the bridge refuses them. Review
and archive `web_runtime/hardware_no_go_zones.json` before changing rooms.
Transport calibration is in `web_runtime/robot_config.json`; taught stations
remain in `stations.json`. Keep these files with the robot's configuration.

## LLM key

### Estimated item positions

Each current front-camera observation is automatically analyzed against the named
item catalog. The LLM receives the scene, occupancy-map image and acquisition-time
base pose. Camera-relative bearing/range estimates are transformed into map X/Y;
they never create confirmed item or pickup-station associations. The latest view
per item appears as an orange location pin with an uncertainty circle. Hover or
focus the pin for the source image, coordinates, confidence and capture time.

Click a pin to preview the nearest reachable approach, facing the item. Then use
the existing **Go** button. Approach selection excludes unknown space, no-go zones,
occupied ROS costmap cells and disconnected regions, and leaves space for the
chassis and estimate uncertainty. Low-confidence estimates cannot provide a goal.
**Resolve** can also select this approach. Fetch and grasping are unchanged.

**Map actions > Estimate item positions** retries existing front observations.
Missing credentials, malformed results and STOP/map changes retain the photos
without applying a location result. Simulator fixtures are analyzed only on
explicit request. Background estimation does not hold the grasp-calibration lock.

Calibration defaults can be overridden in `web_runtime/item_location_config.json`
before restarting `myagv-api`:

```json
{"camera_yaw_deg": 0, "camera_forward_m": 0, "camera_left_m": 0,
 "stand_off_m": 0.45, "minimum_uncertainty_m": 0.35}
```

Camera yaw is positive left from chassis forward; offsets are chassis forward/left
in meters. A single RGB image supplies approximate depth, not a measured object
position. This version uses the newest observation, not multi-view triangulation.

Run in Bash on the robot:

```bash
read -rsp 'OpenRouter key: ' OPENROUTER_API_KEY
printf '\n'
export OPENROUTER_API_KEY
pm2 restart myagv-api --update-env
unset OPENROUTER_API_KEY
```

The key enables text item selection. Resolve does not move the robot; Go/Fetch
remain explicit actions. `configured` means loaded, not provider-verified.
PM2 retains the environment in memory; `pm2 save` writes it to disk. Reboot
startup has not been enabled.

## Verification

```bash
.venv/bin/python -m unittest discover -p 'test_*.py'
npm --prefix web test
npm --prefix web run build
pm2 logs myagv-bridge --lines 30 --nostream
pm2 logs myagv-localization --lines 30 --nostream
```

Automated tests use mocked hardware and never move the robot. The operator must
validate camera roles, actual localization, homing, transport envelope, low-speed
manual motion/release, browser disconnect, STOP, no-go avoidance, pickup alignment
and both grasp confirmations on the supervised robot. No browser or physical
motion test is performed by deployment.

## Deployment check (2026-10-08)

The patched chassis driver compiled on the robot. PM2 services started; P340
reported four joint angles, LiDAR/odometry were fresh, and both ROS costmaps
reported the active no-go revision. The frontend built and hardware-free tests
passed. No browser or movement test was run. The arm camera had dropped off USB
port `1-1.4` with descriptor errors `-71`. Stopping the control bridge, resetting
the USB hub, and restarting the bridge restored `/dev/video1` and the P340 serial
connection. Both camera streams were checked through the API. A hub reset also
resets arm USB, so stop control first and reconfirm homing afterward. Localization,
homing and transport-envelope confirmation remain operator actions.
