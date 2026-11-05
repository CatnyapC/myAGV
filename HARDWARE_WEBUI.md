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
python3 install_navigation_turn.py ~/myagv_ros/src/navigation/base_local_planner/src/trajectory_planner.cpp
cd ~/myagv_ros
catkin_make --pkg myagv_odometry -j2
catkin_make --pkg base_local_planner -j1
catkin_make --pkg global_planner -j1
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

The driver patch uses a wall-clock loop and makes commands expire after 0.5 seconds.
The subscriber keeps only the latest command. At the serial writer, all callers
(navigation, Approach and keyboard teleop) share a motion gate. Continuous forward
or backward travel passes speed and steering corrections directly, including turn
sign changes and crossing zero. Changing translation axes/direction, entering or
leaving pure rotation, and reversing pure rotation send zero first and require
0.5 seconds of continuous fresh stationary feedback. An already confirmed idle
stop is reused for startup without a second delay. Cancellation never queues a
later restart. Stop confirmation times out after 5 seconds and latches off.

One 150 ms deadline covers an entire feedback frame, including header search and
partial reads. Invalid or timed-out feedback immediately commands zero and latches
off instead of blocking command processing indefinitely. Command values have not
been calibrated against MCU speed feedback, and translation on a slippery floor
can include angular drift. No angular-speed threshold or command/feedback ratio
latches motion during travel or a brief coast after zero. STOP and lease expiry
still send zero; a new motion mode or direction must wait for measured stop. If
stop confirmation fails for 5 seconds, the driver latches off. Non-finite feedback
or invalid commands also latch off. These checks do not enforce a calibrated
physical speed limit during requested motion.
The existing `angular_speed_fault` parameter reports
all these motion faults; the WebUI stops and disables motion. Logs include requested
and sent velocities, encoded velocity bytes, feedback and the fault reason.

The patch preserves the explicit wheel-overcurrent recovery gate. Installation
validates both source changes before writing and preserves earlier backups; this
upgrade saves `.before-web-motion-v2` beside changed sources and installs two helper
headers. Rebuild before restarting. A later valid command or feedback sample cannot
clear a safety latch. Diagnose the cause before restarting the driver. These guards
do not calibrate legacy MCU speed units or prove a firmware cause.
MCU v1.0 uses legacy motion packets and 27-byte telemetry. Firmware v1.2 and later
require the matching vendor driver with motion opcode `0x010B`, 43-byte telemetry,
and automatic reporting enabled. Update those together. The manufacturer's
[firmware catalog](https://download-elephantrobotics.oss-cn-shenzhen.aliyuncs.com/software/mystudio/firmware_json_files/myAGV2023Pi.json)
lists v1.3 (an alpha image) as fixing inconsistent wheel speeds after
forward/rotation/forward movement; flashing remains a separate hardware action.
The ROS launch initially starts map_server, AMCL and move_base. Both costmaps consume
`/navigation_map`. The move_base output is remapped to `/myagv/input_vel`; only
the bridge publishes `/cmd_vel`. Fresh LiDAR/odometry, exclusive publishers and
the driver watchdog are required for motion.

The navigation patch ranks legal in-place turns by the shortest angle toward a
visible path point, using the planner's existing `headingDiff` calculation.
`heading_lookahead: 0.0` scores turns at the chassis center. The previous 32.5 cm
virtual point could enter an inflated obstacle and reject collision-free rotation.
The upstream grid-distance score can tie at 0.1 rad/s and prefer the clockwise
sample even when counterclockwise is shorter. Collision checks, speed limits,
oscillation checks and final goal-heading alignment stay in the planner.
The patch also scores the endpoint heading of forward arcs: grid distance alone
can tie at low speeds and choose a turn away from the path. It reuses the existing
0.3 heading weight without increasing simulation steps, and rejects a trajectory
when no path point is visible. Collision checks still cover every simulated point.
The global planner merges safely visible path nodes into straight segments while
checking every crossed inflated costmap cell, including diagonal corner contacts.
It retains dense samples for ROS distance fields. Local heading tracks the current
segment's endpoint and advances to the next segment within 15 cm of a corner.
Forward tracking starts within 15 degrees and continues until heading error exceeds
30 degrees. Legal forward trajectories take priority over in-place turns; a blocked
forward trajectory still permits collision-checked rotation and replanning.
Velocity sampling stays within configured limits even when measured feedback is
outside those limits. The driver still requires a measured stop for drive-mode changes.
The upstream escape fallback's collision override is removed: a blocked reverse
trajectory stays invalid instead of being relabeled as a valid motion command.
Arrival braking sends zero velocity and retains the planner's measured-odometry
stop gate before final rotation. Feedback-minus-acceleration braking can keep
driving on this chassis when its low-speed response exceeds the command.
Web Go and Fetch use 15 cm / 30 degree arrival tolerances in both the ROS planner
and the caller's completion check. The position latch retains arrival during final rotation.
The installer refuses unknown source layouts and keeps `.before-shortest-turn-v7`,
preserving earlier backups when upgrading.
Web navigation caps turns at 0.1 rad/s and forward travel at 0.05 m/s;
automatic reverse escape is disabled. When no legal trajectory exists, the local
planner stops and requests replanning; it cannot extend Fetch's measured 10 cm retreat.
Global planning runs on demand:
periodic replanning resets TrajectoryPlannerROS's accepted-position
latch and can send the robot back toward the point during final heading alignment.
Local obstacle checks stay active and a failed local trajectory requests a new plan.
The displayed path stays visible for the current navigation and clears at the next goal.
Rebuild and restart navigation to load the patch. Run the offline regression
on the ROS machine; it publishes no motion commands:

```bash
g++ -std=c++14 -fno-access-control tests/test_navigation_turn.cpp -o /tmp/myagv-navigation-turn $(pkg-config --cflags --libs base_local_planner)
/tmp/myagv-navigation-turn
```

The access-control flag is only for testing the installed private braking helper.

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
scans, but the displayed map and navigation map stay fixed. **Refresh LiDAR map**
waits for the base to stop, then publishes and saves the latest complete snapshot.
It works while idle or while item position updates are paused, without arm motion
or photo capture. The stopped refresh tolerates sensor callback delays during
map processing; motion still requires fresh sensors, and STOP, control lease,
driver watchdog and exclusive chassis ownership remain enforced.
Robot position remains live. Map identity stays fixed within
one SLAM session; existing no-go zones keep their world coordinates. SLAM
confirms its pose from fresh sensors automatically. **Set pose from goal** is
disabled. The accepted snapshot survives API and bridge restarts. An ongoing
SLAM session is identified by `web_runtime/mapping.json`. Each accepted map is synchronously
saved to the single authoritative `web_runtime/saved_map.json` checkpoint before
publication. **Refresh LiDAR map** completes only after the checkpoint and bridge
snapshots are written. The UI shows the saved revision or a storage error;
storage failures stop automatic movement. Atomic checkpoint replacement keeps
the previous file intact when writing the new file fails before replacement.

PM2 starts localization through `map_persistence.py`. On localization/service or
machine restart, it exports the saved raw occupancy to `saved_map.pgm` and
`saved_map.yaml`, then starts AMCL on that map. It preserves map identity,
revision, world coordinates, no-go zones, taught stations and photo associations.
Set and confirm the robot pose before navigating again. The original
`MYAGV_MAP_FILE` (default `~/maps/room.yaml`) is used only before a checkpoint
exists. Existing valid bridge snapshots are adopted automatically. A corrupt
checkpoint fails startup instead of silently loading a different map.

There is one active map and no map selector. **Reset map** archives the checkpoint
and generated ROS files along with the existing reset data, then starts a fresh
SLAM session. Failed reset restores the archived map. Restarting only the API or
bridge keeps an ongoing SLAM session; restarting localization uses the accepted
map for navigation rather than resuming Gmapping internals. Unaccepted SLAM
scans are not durable. Back up `web_runtime/` and `stations.json` together for
recovery from disk loss.

After pulling this change, update PM2's localization command while the robot is
stopped: `pm2 startOrRestart ecosystem.robot.config.cjs --only myagv-localization --update-env`.
Restart the bridge and API too. A plain restart using PM2's old process definition
does not install the new startup command.

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
buttons stops the task. Fetch folds the arm before travel. Item position updates
currently support only manual positioning: drive the base to the desired viewing
position, stop, then click **Manual Item Update** in MAP. This captures the front
camera without moving the base or arm. With a configured vision LLM and named
reference items, the LLM estimates and saves item map positions from the photo
and measured capture pose. Repeat manually at another position when needed.
Saved observation points are not required or added; the old 20-point limit does
not apply. MAP no longer exposes automatic update start/pause, observation-point
management or Refine Approach. LiDAR map refresh remains a separate action.

## Navigation, no-go zones and Fetch

**Go** sends the selected map-frame goal to move_base. A goal requires current
localization, live sensors, planner footprint clearance and both costmaps applying
the current no-go revision. Go does not require an online or homed arm or a
recorded transport pose, and does not move the arm. Unknown cells are treated as
free in the derived navigation map; the displayed source map keeps them unknown.
Occupied footprint clearance and goals outside map bounds are rejected. Go and
map surveys share this policy. Fetch approach selection and camera-guided
adjustments exclude unknown space.
Live obstacles and no-go zones remain blocked.
No-go rectangles are rasterized into a derived map. Both global and rolling
local StaticLayers consume it; published full costmaps are checked for virtual
obstacles. The original map is preserved. Recovery rotations are disabled.

**Fetch** requires a selected item with a current-map location estimate (confidence
at least 0.7), an uploaded reference, a measured ARM CALIBRATION GOAL pose, and a
saved grasp height below that GOAL. A taught station is not required. Both cameras,
configured LLM, confirmed localization and measured transport clearance are required.
It records the base pose when Fetch starts, folds, navigates to a reachable approach
facing the item, and runs front-camera refinement. The final approach uses fresh
stopped images and safe stand-off checks; navigation speed stays unchanged.
It then opens, restores the saved GOAL arm pose, runs ALIGN, and uses the same full
descent/clamp/lift targets as GRASP. With the gripper closed it folds, returns to
the recorded base pose, restores the measured aligned arm pose, lowers to the saved
grasp height, releases, lifts, and folds. No intermediate review clicks are needed.
There is no force-based grasp detection. Failures stop subsequent actions; a
persisted held-item record remains until placement or explicit gripper release.

### Dual-camera Fetch calibration

All items use the same LLM comparison of saved GOAL and current images. The arm
stage measures the same visible target fragment independently in both images;
there is no object-specific detector or separate measurement/confirmation step.
ALIGNMENT ROUNDS shows the returned bounds over GOAL and CURRENT. Logical Y+
moves the visible target upward, so a target above GOAL commands Y−. One stationary
frame must match within 0.5% top boundary and 2% horizontal/bottom position, with
visible width within 10% and area within 15%. Near GOAL, 0.25 mm corrections reduce
overshoot. Missing or ambiguous targets stop the task.

The arm base mounts facing chassis left; J1=90 faces forward. The arm camera
mounts 90 degrees clockwise, with the target near the bottom of its image.
Its image X controls logical arm X (chassis forward/back). Front camera image X
controls logical arm Y (chassis right/left). A target right of the alignment point
commands positive motion on either logical axis. The SDK receives native +Y for
logical X+, and native -X for logical Y+; images retain their mounted orientation.

Import and photo/item editing open as a child page inside OPERATE. The upper-left
back arrow returns to its main controls. First manually grasp the item and use
Save current grasp Z to record only the measured arm Z coordinate, in millimeters,
as `grasp_z_mm` on the item. Then release the item in place, return to OPERATE and
raise the arm above that height to the alignment position before saving both
camera views. The item editor explains this clearance step between the buttons.
New goal capture requires a recorded grasp height. Both recordings work while
TELEOP stays open, or with an idle, homed WebUI arm. Map-pose confirmation is not
required for these recordings. Clicking either recording button asks TELEOP to
stop motion and pause movement keys for the acquisition. TELEOP's main loop reads
the arm and supplies fresh, request-specific feedback; WebUI never shares its
serial port. Keys resume when acquisition finishes (or its 12-second hold expires).
ARM GOAL stores measured `arm_z_mm` alongside the
image, with joint and Z stability checks around acquisition. The item exposes that
photo's height as `calibration_z_mm`. ALIGN and vision-guided Fetch restore this Z
before any alignment observations or X/Y corrections, preserving current X/Y.
Measured Z must reach within 1 mm; STOP, invalid feedback or failure to arrive
abort alignment. Preview remains motionless. Legacy GOAL photos without Z must be
recaptured; grasp height and current height are never used as substitutes. New
items are saved before acquisition. Recording either height does not move the arm.
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
homed arm at J1=90 and a selected item with saved GOAL views. These local tests
need no map-pose confirmation or station association. Fresh sensors, stationary
acquisition, current map/no-go revisions and rotation clearance checks still
apply. Normal Fetch/navigation still requires confirmed localization.

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
  "max_step_mm": 2
}
```

Alignment permits up to 33 observations across both stages and at most 32
corrections, without a cumulative arm travel cap. Each nonzero X/Y axis correction
sends one complete absolute coordinate target at speed 30, capped by
`max_step_mm` (up to 10 mm). X and Y run in order with measured arrival before the
next axis and fresh photos before the next round. The arrival timeout is 3 seconds
per millimeter, with a 3-second minimum. Manual adjustments still default to 1 mm.
Grasp and possession still require the existing operator confirmations. Web STOP
sends M15 but may not interrupt an absolute move; physical stopping remains a
supervised acceptance check.

**Manual Item Update** is the supported item-map update workflow. Drive to each
viewing position manually and stop before capturing. The configured LLM
automatically assesses the current photo and up to six earlier views together,
using each acquisition's measured base pose and the occupancy-map image. Raw
occupancy arrays are excluded from the prompt. Estimates are projected from the
model's chosen source view, including items visible only in an earlier view.
LLM ROUNDS shows this assessment's inputs, output, errors and Codex thread.
Select a confident item estimate to enable **SLOW APPROACH TEST**, subject to the
existing localization, sensor, no-go and stopped-controller checks.
Save the item's final GOAL front-camera photo before approaching. Each stopped
round compares that saved image with CURRENT. Rough arrival accepts target width
at 85–115% of GOAL, visible top/bottom within 150/1000 and an estimated remaining
range within 2 cm. A visual match completes approach even if the LLM suggests a
tiny final adjustment; ALIGN handles the residual distance and framing. The item must remain
visible; horizontal centering and heading are not completion gates. ALIGN handles
the final angle and fine positioning.
Clipping is retained in visible boxes; the model chooses the next bounded adjustment.
The coarse map stand-off distance is only a staging position, not this endpoint.
Each round permits at most 0.25 m forward or backward and +/-15 degrees of turning, enforced
by the prompt, response schema, result validation and chassis executor. Forward
requests beyond +/-0.25 m are clamped rather than rejected by runtime validation.
travel must also stay within half the estimated remaining GOAL distance derived
from the target width ratio. At GOAL width or larger, no forward travel is allowed.
An overshoot remains recoverable: a zero/forward suggestion requests another
stopped observation, while a negative `forward_m` retreats toward GOAL distance.
Retreat retains STOP, sensor and lease guards.
The base stops for a fresh photo after each adjustment. It executes at slow speed
against measured odometry and stops when it reaches or passes the target, including small
targets that move_base's arrival tolerance would otherwise skip. STOP, leases,
fresh sensors, map bounds and swept no-go checks stay active during travel.
Visual Approach, base/arm ALIGN and pickup-stage checks assume sensed obstacles
are clear, including at the current pose: the item itself may be in the LiDAR map.
Map bounds, swept no-go zones, STOP, leases, sensor freshness and arm workspace
checks remain enforced. These local phases have no sensed-obstacle collision stop.
After grasping and folding the arm, Fetch retreats 10 cm along the current backward
axis using the same bounded odometry-controlled approach executor. This retreat
must finish and stop before return navigation; a timeout aborts the return.
Before any Fetch motion, the bridge snapshots the accepted map plus lethal
obstacles in the current global costmap. It disables both live obstacle layers
and feeds this fixed snapshot to the static layers for the entire Fetch. Inflation
is applied once; inflation costs are not copied back as obstacles. Existing
obstacles and no-go zones remain, but new obstacles cannot enter the Fetch plan.
Both costmaps must confirm the snapshot before motion. STOP, leases and fresh
LiDAR/odometry checks remain active. Completion, failure or cancellation restores
the normal map and live obstacle layers; bridge restart also restores live layers.
The local planner cannot add automatic reverse escape movements to the retreat.
After a failed Fetch, Go can return with the item held only when measured arm joints
match the recorded transport pose within one degree. It preserves the gripper and
fourth axis; normal navigation guards still apply.
Navigation to the observation point and back retains normal obstacle checks.
The default collision radius is 0.21 m; the configured hardware radius is applied
to both ROS costmaps and application clearance checks.
Only an explicit LLM turn produces angular velocity. Forward travel never adds
endpoint steering or heading corrections; its current forward path is checked for no-go zones live.
Necessary translations use at least 0.01 m/s, the driver's representable speed
step, so millimeter corrections do not truncate to a zero motor command.
During approach or ALIGN, stale LiDAR/odometry pauses with zero velocity while keeping the
task and Codex session. Fresh sensors and measured standstill allow resumption;
paused time does not consume movement or ALIGN inference timeouts. STOP, lease loss,
map/no-go changes and driver faults still cancel. A changed base pose after a
photo discards that adjustment and takes a fresh photo for the next round.
ALIGN request failures, including HTTP 409, retry up to five times after the first
attempt. Local session recovery resends identity, GOAL and retained metric context;
no movement is issued until one request succeeds. STOP/task changes cancel retries.
Codex keeps one session for the approach. The first turn sends identity and GOAL;
later turns send only CURRENT and actual movement feedback. Its compact six-field
JSON omits repeated confidence, uncertainty and arrival flags. Location estimates
retain their prior confidence and uncertainty while visual range/bearing updates.
The standalone test stops after matching GOAL. FETCH proceeds to arm ALIGN and
then GRASP only after that match; it never treats a coarse safety stop as arrival.

Each revisit replaces its latest unlabelled photo. Confirmed old photos stay as
historical evidence; new images never inherit a moving object's old identity.
Fetch uses a current confident map estimate and checks the named item again in
the front camera before alignment. This refreshes scene evidence, not measured
object coordinates or automatic identity tracking. Capture/import failure stops updating and preserves
the source frame for recovery; a 20-frame handoff backlog pauses updates.

Live captures bind the received camera frame to TF at that frame's host receipt
time, map identity and camera identity. The robot must be stopped. Arm captures
also require stable arm feedback. These USB cameras expose no calibrated exposure
clock; metadata labels the timestamp `host_frame_receipt`. Captures are not
precision visual-servo measurements. During active tasks the camera panels show
live frames, even if a stored photo was selected previously.

## STOP and reconnect behavior

P340 responses and coordinate motion completion default to 3 seconds.
Joint-pose travel (including folding) allows 30 seconds; homing allows 60 seconds. Alignment feedback reads allow two retries; reads during
motion share the remaining completion budget. Motion commands are never resent.
Terminal jogging retains its shorter `--arm-timeout` to enforce the key deadman.
Moves that exceed their completion budget stop with a timeout.

Starting `teleop_control.py` requests a local hardware handoff before opening
the P340 or publishing chassis commands. WebUI releases the arm serial port and
pauses its chassis output while continuing camera/map monitoring. Active WebUI
tasks must be stopped first. The terminal owns the hardware until TELEOP exits;
browser blur or disconnect does not stop terminal control. An explicit WebUI
STOP terminates TELEOP and leaves motion locked pending measured stop recovery.
Normal TELEOP exit returns the serial port, verifies stopped chassis and measured
arm feedback, then restores idle WebUI control. TELEOP's homing state transfers
only after that verification. Failed return or unexpected disconnect still locks
motion for stop recovery. Reconfirm localization before navigation. TELEOP also
opens the arm with exclusive serial access.

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

For a wheel overcurrent latch, the same confirmed button requests the installed
driver's native motor reset with zero speed, waits for the reset acknowledgement,
then requires new stationary odometry and arm feedback. Missing odometry alone
never triggers a motor reset. STOP, lease expiry or timeout revokes the request;
the interrupted navigation or item update never resumes. Recovery tolerates absent
sensor feedback while restoring the stopped driver, but motion still requires
fresh sensors, the driver watchdog and exclusive output ownership.

The manufacturer's [P340 protocol](https://docs.elephantrobotics.com/docs/ultraArm-en/3-HowToUseultraArm/2-SoftwareControl/6-SerialPort/6-SerialPort.html)
documents M15 as jog stop, not a guaranteed emergency stop for homing or queued
absolute moves. Transport and Fetch arm pose changes send
one complete joint target at speed 30 for continuous firmware interpolation,
then require stable measured arrival within 30 seconds. STOP interrupts the
controller's wait and sends M15; the full absolute move or gripper command may
still finish if firmware does not interrupt it. Homing cancellation is reported
unconfirmed and requires re-homing. Web STOP is not a substitute for physical
power/emergency control.
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

For direct OpenAI API access, run in Bash on the robot:

```bash
set +x
read -rsp 'OpenAI API key: ' OPENAI_API_KEY
printf '\n'
export OPENAI_API_KEY
pm2 restart myagv-api --update-env
unset OPENAI_API_KEY
```

Paste only at the hidden prompt. Do not run `pm2 save` or print process environments.
After restart, Settings > Model automatically lists the key's OpenAI catalog.
Select a model supporting Chat Completions, structured JSON, and image inputs for
vision tasks. See [OpenAI API key](WEBUI.md#openai-api-key) for storage and catalog
limitations. Existing OpenRouter and Local Codex configuration remains independent.

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
**Resolve** can also select this approach. **Fetch** automatically navigates to
the approach, refines with the front camera, aligns, grasps, returns and places.
For both outbound and return navigation, Fetch advances after move_base reports
success and measured standstill is confirmed, even outside the 15 cm / 30 degree
arrival tolerance. The measured error is logged. Camera approach and alignment
still gate pickup; placement uses the actual return position. Navigation failures,
timeouts, STOP, lease loss and sensor checks still stop the task. Standalone Go
keeps its arrival tolerance check.

Approach refinement remains part of Fetch and **SLOW APPROACH TEST** in OPERATE;
there is no Refine Approach action in MAP. The item must be visible in the front camera. This
base-only task makes at most 16 stopped camera checks, with at most 15 adjustments:
each moves forward or backward by at most 0.25 m and turns by at most 15 degrees.
Every step uses fresh localization, LiDAR and odometry, and checks the swept path
against no-go zones and map bounds. Sensed occupancy is assumed clear during these
local corrections, base/arm ALIGN and the measured 10 cm retreat before return.

Refinement shares Fetch's compressed-camera acquisition, vision request/response
channel, provider and **LLM ROUNDS** log. It sends the original item observation, all executed movement records,
measured before/after poses, the latest two previous images and the current image.
Each accepted answer saves a new map estimate on that round's acquisition photo;
the item pin and preview update automatically. Arrival needs a geometric check of
distance and heading in addition to the LLM's answer. Lost/ambiguous items, unsafe
suggestions, lack of progress, timeout, stale sensors or exhausted rounds stop the
task. STOP, disconnect, map/no-go changes and pose drift invalidate pending moves;
late answers after cancellation are discarded. The standalone approach test
sends no arm or gripper commands; Fetch continues to ALIGN only after arrival.

**Manual Item Update** captures a fresh view to retry an estimate.
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
position. Refinement uses viewpoint history to ask the LLM for a better estimate;
it does not perform calibrated geometric triangulation or prove grasp readiness.

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
.venv/bin/python -m unittest discover -s tests -t . -p 'test_*.py'
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
