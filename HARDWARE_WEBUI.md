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

After reset, the label becomes **Live SLAM map**. Fresh scans update occupancy
and the WebUI raster about once per second; map identity stays fixed during one
SLAM session, even as its bounds grow. Existing no-go zones keep their world
coordinates. Verify **Confirm map pose** before navigating; SLAM determines the
pose, so **Set pose from goal** is disabled. The live mode survives service
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
   bounded 1 mm targets. PICKUP allows X/Z adjustment with J1 near zero.
5. Fold the arm and measure a circular clearance radius covering the
   base, folded arm and carried object. Check the measurement box and **Record
   transport pose**. This updates both ROS planner footprints. No transport pose
   is guessed. Fetch and survey remain disabled until this is done.
   Transport keeps the measured joint angles, including J1 rotation. Only side
   pickup station poses require J1 near zero.
6. Teach a named pickup station at its real base/arm pose (J1=0).
   Existing names require overwrite confirmation; changed station links must be
   reconfirmed. Capture a photo, edit its item, and explicitly associate that station.

Manual BASE driving is supervised and bypasses no-go planning. It uses at most
0.1 m/s and 0.3 rad/s; PICKUP base motion uses 0.03 m/s and 0.05 rad/s. Releasing
buttons stops the task. Fetch and survey fold the arm before travel.

## Navigation, no-go zones and Fetch

**Go** sends the selected map-frame goal to move_base. A goal requires current
localization, live sensors, planner footprint clearance and both costmaps applying
the current no-go revision. Go does not require an online or homed arm or a
recorded transport pose, and does not move the arm. Unknown/occupied footprint
clearance is rejected.
No-go rectangles are rasterized into a derived map. Both global and rolling
local StaticLayers consume it; published full costmaps are checked for virtual
obstacles. The original map is preserved. Recovery rotations are disabled.

**Fetch** requires an explicitly selected item linked to an unchanged taught
station on the current map. It captures the startup base/arm poses, folds,
approaches the station, opens the gripper and reaches the taught arm pose.
It then pauses for live-camera alignment review; bounded X/Z adjustments are
available. After confirmation it closes, folds, and pauses again for the user
to confirm possession. Only then does it return and restore the startup pose.
There is no automatic vision calibration or force-based grasp detection.
A five-minute review timeout cancels the task.

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

The controlling tab must answer fresh server challenges every 150 ms. Its lease
expires within 450 ms, and the ROS gate sends zero velocity. Stale/duplicate
heartbeats cannot extend a lease. Another tab cannot renew or take over an
active task. Tab blur, hiding, disconnect, API shutdown, stale sensors and STOP
cancel the task. STOP uses an independent socket request and invalidates queued
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

Defaults: front `/dev/video0`, arm `/dev/video1`, P340 `/dev/ttyUSB0` at 115200.
Verify these roles. Override camera devices when restarting the bridge:

```bash
MYAGV_FRONT_DEVICE=/dev/video0 MYAGV_ARM_DEVICE=/dev/video1 \
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
