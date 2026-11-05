# myAGV + P340

WebUI for the myAGV chassis, P340 arm and gripper: live ROS map, cameras,
manual movement, no-go zones, navigation and camera-guided Fetch.

## Start or update the robot WebUI

Run on the robot. Requires ROS Noetic, the `~/myagv_ros` workspace, Python
3.11+, uv, Node.js and PM2. **First installation:** follow
[hardware setup](HARDWARE_WEBUI.md#install-and-start), including the driver
watchdog and navigation patches/builds. Repeat those builds when their patches change.

Stop any running task before updating. Close the vendor UI, keyboard teleop and
other serial controllers; only one controller may own the hardware.

```bash
cd ~/myAGV
git pull --ff-only
uv sync --locked --extra web
npm --prefix web ci
npm --prefix web run build
pm2 startOrRestart ecosystem.robot.config.cjs
MYAGV_HARDWARE_DIR="$HOME/myAGV/web_runtime/ros" \
MYAGV_WEB_ORIGIN="http://172.28.13.26:5173" \
pm2 startOrRestart ecosystem.config.cjs --only myagv-api,myagv-web --update-env
pm2 status
```

Open **http://172.28.13.26:5173** on the same trusted network. Replace that IP
in both the browser URL and `MYAGV_WEB_ORIGIN` for another robot.
The frontend proxies the local API on port 8791. Do not expose it publicly.
Starting services does not home the arm or start a navigation task.

## Operate

1. **Check connection:** verify live map, robot pose, front camera and arm camera.
   Open **OPERATE → Hardware controls**; check driver, sensor and controller status.
2. **Localize:** in saved-map mode, enter the robot's actual position and heading
   in **GOAL**, click **Set pose from goal**, verify the marker, then **Confirm map
   pose**. Live SLAM establishes its pose automatically.
3. **Home arm:** clear its travel area, click **Home arm** and confirm. Use
   **Already homed** only if homing completed since the last power-on. Controller
   restarts clear homing and localization confirmations; recheck both.
4. **Move manually:** select **BASE**, **ARM** or **PICKUP**. Hold a direction
   button to move; release to stop. PICKUP combines slow base movement and arm
   positioning. Use **Open gripper** / **Close gripper** while stopped.
   Manual base driving bypasses no-go planning: watch the robot.
5. **Prepare transport:** fold the arm manually, measure a radius enclosing the
   base, arm and payload, enter it, tick the measurement checkbox, then **Record
   transport pose**. **Transport pose** later returns to those measured joints.

Use the red **STOP** button to interrupt motion. Keep the physical stop accessible.
If controls are disabled, read the displayed reason before continuing.

## Map, navigation and Fetch

- **No-go zones:** MAP → More → No-go zone. Select opposite corners and save.
- **Go:** set a map goal, then click **Go**. Requires confirmed localization,
  fresh sensors and valid obstacle/clearance checks. Go does not fold the arm;
  put it in a safe transport pose before travel.
- **Manual Item Update:** position the robot manually, stop, then click this
  MAP action. With named reference items and a configured vision model, it
  captures the front camera and estimates item locations without moving.
- **Fetch:** select an item with a reference image and current-map location.
  Configure the vision model, record grasp height and arm/camera GOAL poses,
  and record transport clearance first. Fetch approaches, aligns, grasps,
  returns to its starting base pose and places the item. Check the displayed
  prerequisites; see [Fetch calibration](HARDWARE_WEBUI.md#dual-camera-fetch-calibration)
  for the recording procedure.

## Service controls

```bash
# Inspect recent controller/API logs
pm2 logs myagv-bridge --lines 30 --nostream
pm2 logs myagv-api --lines 30 --nostream

# Restart only the web services
pm2 restart myagv-api myagv-web

# After stopping motion in the UI, shut down this project's services
pm2 stop myagv-api myagv-web myagv-bridge myagv-localization myagv-ros
```

If hardware is unavailable, check power, USB/cameras and bridge logs. If TELEOP
owns control, exit it before using WebUI motion. These commands do not configure
automatic startup after reboot.

## Other entry points

- [Robot setup and keyboard quick start](QUICKSTART.md)
- [Hardware setup, safety and calibration](HARDWARE_WEBUI.md)
- [Local simulation and model configuration](WEBUI.md)
- [Combined keyboard control](TELEOP.md) · [Taught-station navigation](NAVIGATION.md)
- [Test commands](tests/README.md)

For the standalone direct-serial console (with other controllers stopped):

```bash
uv run python command_control.py --list-ports
uv run python command_control.py --p340-port /dev/ttyUSB0
```

Commands include `where`, `zero`, `reach X Y Z`, `coord Z VALUE`, `clamp`,
`release`, `move forward 10 1`, `stop` and `quit`. Coordinates are millimeters.
The first arm command homes automatically unless `--no-zero` is supplied.
