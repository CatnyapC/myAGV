# Read-only robot WebUI

ROS1 Noetic runs with Ubuntu's `/usr/bin/python3`. The web API keeps its Python
3.11+ environment. `ros_web_bridge.py` passes atomic snapshots between them;
it subscribes to `/map` and map-to-base TF and reads cameras. It never publishes
motion commands, opens arm serial, or starts navigation/fetch. Live observation
does not enable hardware control. The UI displays **HARDWARE · READ ONLY**.

## Start on the robot

Close `myAGV_UI` and its ROS terminals first; it resets GPIO when closing.
Do not run another chassis driver alongside these PM2 processes.

```bash
cd ~/myAGV
git pull --ff-only
uv sync --locked --extra web
npm --prefix web ci
npm --prefix web run build
pm2 start ecosystem.robot.config.cjs
```

The configuration starts chassis/LiDAR, loads `~/maps/room.yaml` with AMCL, then
starts the read-only bridge. It does not start `move_base` or issue goals.
The API may restart while waiting for the first real map snapshot:

```bash
MYAGV_HARDWARE_DIR="$HOME/myAGV/web_runtime/ros" \
MYAGV_WEB_ORIGIN="http://172.28.13.26:5173" \
pm2 restart myagv-api --update-env
```

Reload the UI yourself. **Connected** confirms the UI/API connection, not correct
localization. Set the robot's actual pose using RViz's **2D Pose Estimate** and
check scan alignment yourself. In a separate desktop terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/myagv_ros/devel/setup.bash
rviz -d ~/myagv_ros/src/myagv_navigation/rviz/teb.rviz
```

Pose is an AMCL/TF estimate. Missing/stale TF clears the pose and chassis marker.
The outline remains nominal 0.32 x 0.28 m; it is not measured collision geometry.
The saved map remains visible when ROS feedback becomes unavailable. Display
rotation affects only the WebUI raster, never ROS map coordinates.

## Cameras and map selection

Defaults on this Pi: front `/dev/video0` (camera0), arm `/dev/video1` (USB camera).
Verify camera roles yourself. Set device paths when starting/restarting the bridge:

```bash
MYAGV_FRONT_DEVICE=/dev/video0 MYAGV_ARM_DEVICE=/dev/video1 \
pm2 startOrRestart ecosystem.robot.config.cjs --only myagv-bridge --update-env
```

These are USB/V4L2 preview frames, not synchronized grasp observations. A failed
camera retries; snapshots expire after two seconds. Both previews use 640 x 480
where supported. Selecting a stored photo shows it instead of the corresponding
live preview. No frame is sent to OpenRouter. Real photo/pose capture is disabled
until acquisition-time pose binding is implemented.

To use another saved map:

```bash
MYAGV_MAP_FILE="$HOME/maps/other-room.yaml" \
pm2 restart myagv-localization --update-env
```

Map identity derives from grid contents and geometry; changing maps clears stale
pose bindings. The display normalizes occupancy values to free/occupied/unknown;
ROS data is unchanged. Maps above two million cells are rejected by this bridge.

## LLM key in Bash

```bash
read -rsp 'OpenRouter key: ' OPENROUTER_API_KEY
printf '\n'
export OPENROUTER_API_KEY
pm2 restart myagv-api --update-env
unset OPENROUTER_API_KEY
```

PM2 retains the backend environment in memory. Do not run `pm2 save` with the key
loaded: it serializes process environments to disk. No reboot autostart is set up.
The key enables text item selection only. It does not analyze camera frames,
start movement or enable Fetch. `configured` means loaded, not provider-verified.

## Limits and checks

Web motion, arm control, no-go enforcement and hardware STOP remain unavailable.
STOP is disabled in read-only mode; its API reports **unconfirmed**, never a false
hardware-stop acknowledgement. Use the physical stop or the controller responsible
for any movement. Do not mistake demo zones for ROS navigation obstacles.

```bash
pm2 logs myagv-ros --lines 20 --nostream
pm2 logs myagv-localization --lines 20 --nostream
pm2 logs myagv-bridge --lines 20 --nostream
source /opt/ros/noetic/setup.bash
rostopic hz /scan
rostopic hz /odom
```

Code checks use snapshots only and do not move hardware:

```bash
.venv/bin/python -m unittest test_web_backend test_web_simulation test_web_photos test_web_resolve test_web_hardware
npm --prefix web run build
npm --prefix web test
```
