# AGV quick start

For the official myAGV PI ROS1 Noetic image. Commands assume `~/myAGV`; replace the workspace, map path and P340 serial port with your actual values.

## 1. Update and install

```bash
cd ~/myAGV
git pull --ff-only
```

First time only; skip steps already completed:

```bash
/usr/bin/python3 -m pip install --user --upgrade 'pymycobot==4.0.5'
git clone https://github.com/ros-teleop/teleop_twist_keyboard.git vendor/teleop_twist_keyboard
git -C vendor/teleop_twist_keyboard checkout 8e1e14fdebd31b8e37ec1a453fe7c7fcf03e7648
```

Skip clone if the directory exists. If pip is missing, run `sudo apt install python3-pip`. The environment setting below makes the cloned teleop package visible to ROS; no separate teleop package installation is needed.

## 2. Prepare every new terminal

Run each line separately. Do not join `setup.bash` and `roslaunch` into one command.

```bash
cd ~/myAGV
source /opt/ros/noetic/setup.bash
source ~/myagv_ros/devel/setup.bash
export ROS_PACKAGE_PATH="$HOME/myAGV/vendor:$ROS_PACKAGE_PATH"
```

The last line fixes `ResourceNotFound: teleop_twist_keyboard`. Use system Python below, not `uv run`.

## 3. Start chassis and LiDAR (terminal 1)

For command-line operation, first close `myAGV_UI` and stop its launched ROS
terminals. Do not run two copies of the chassis driver or keyboard controller.
The UI resets GPIO when it closes, so close it **before** the command below.

On this myAGV PI, opening the UI sets BCM GPIO 21 HIGH. Its **Open LiDAR**
button then sets BCM GPIO 20 HIGH before starting the driver. Both steps are
needed to reproduce the UI initialization; setting only GPIO 20 omits startup.
Run the same GPIO sequence on the AGV:

```bash
/usr/bin/python3 -c 'import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); GPIO.setup(21, GPIO.OUT); GPIO.output(21, GPIO.HIGH); GPIO.setup(20, GPIO.OUT); GPIO.output(20, GPIO.HIGH)'
roslaunch myagv_odometry myagv_active.launch
```

Leave this terminal running. Skip starting another copy if this sequence is already running.
Verified against `/home/er/AGV_UI/operations.py`, `myAGV_windows.__init__()` and
`radar_open()` on the robot. An "already in use, continuing anyway" GPIO warning
does not abort the command; a pin may retain its output configuration from a
previous run. Keep the UI closed so it cannot reset the pins afterward.

## 4. Build or load a map (terminal 2)

### Build and save a map, if needed

In another prepared terminal, start the same mapping launch as the UI's
**Build Map** button:

```bash
roslaunch myagv_navigation myagv_slam_laser.launch
```

This also opens RViz; use a terminal in the AGV's remote desktop for the display.
Use teleop (section 5) to explore. Keep mapping running while saving from another
prepared terminal:

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/room
```

Keep both `room.pgm` (map image) and `room.yaml` (map parameters). A `.rviz` file
stores display settings, not the map. [Official map-saving guide](https://docs.elephantrobotics.com/docs/myagv_pi23_en/6-SDKDevelopment/6.2-ApplicationBaseROS1/6.2.5-Real-time_Mapping_with_Gmapping.html).

If saving remains at this message, `/map` has not arrived:

```text
[ INFO] [1789557607.777167610]: Waiting for the map
```

Check `rostopic hz /scan` for live scans and keep the mapping launch running.
Once saved, stop **only the mapping launch** with Ctrl-C before loading the map.
Keep terminal 1 running.

### Load the saved map for fetching

In terminal 2, after stopping SLAM:

```bash
roslaunch ./navigation_fetch.launch map_file:=$HOME/maps/room.yaml
```

In RViz, use **2D Pose Estimate** to set the robot's actual position and heading. Check that the scan aligns with the map.

Manual driving needs no map. Recording with `p` needs either SLAM or saved-map
localization. Fetching needs this navigation launch; building a map alone does
not start `move_base`.

## 5. Keyboard control (terminal 3)

Use the P340's port, which may differ from `/dev/ttyUSB0`.

```bash
/usr/bin/python3 teleop_control.py --p340-port /dev/ttyUSB0
```

Arm feedback gets 0.4 seconds; polls near key expiry are skipped. A feedback
timeout stops the arm without closing teleop. If needed, add `--arm-timeout 0.5`;
it must remain below `--key-timeout` (default 0.6 seconds).

| Key | Action |
| --- | --- |
| Tab | Switch BASE / ARM mode |
| BASE: `i` / `,`, `j` / `l`, `J` / `L` | Forward/back, turn, strafe |
| ARM: `w` / `s`, `a` / `d` | Vehicle forward/back (Y-/Y+), left/right (X-/X+); arrows match |
| ARM: `k` / `j` | Raise/lower |
| ARM: `h` | Home before the first arm movement |
| `g` / `r` | Close/open gripper |
| `p` | Stop, show pose, enter item name to save; Enter cancels |
| Space / Ctrl-C | Stop / stop and exit |

## 6. Record an item and calibrate once

Arm keys account for the 90-degree counterclockwise mounting. JSON stores native arm joint angles.

- Drive to the pickup position and align the arm for grasping. Press `p`, enter `red_cup`, and save to `stations.json`.
- Once: move the empty arm into a folded transport pose. Press `p`, copy `arm_angles_deg`, and leave the name blank. Replace `TRANSPORT_ANGLES = None` in `fetch_demo.py` with those measured angles.
- To edit item records manually: `nano stations.json`.
- When restarting teleop after homing without power loss, append `--arm-homed`.

## 7. Fetch and return

Drive to the desired placement position, set the arm to the placement pose, and leave the gripper empty. **Exit teleop with Ctrl-C.** Keep terminals 1 and 2 running.

```bash
/usr/bin/python3 fetch_demo.py red_cup --p340-port /dev/ttyUSB0 --arm-homed
```

The robot fetches the item, returns to **the base position, heading and arm pose captured at this demo's startup**, then opens the gripper. No dropoff record is needed.

For a chassis-only round trip, keep the arm in its transport pose, exit teleop, then run:

```bash
/usr/bin/python3 navigation.py roundtrip red_cup
```

Check paths without an object first. Full guide: [NAVIGATION.md](NAVIGATION.md).
