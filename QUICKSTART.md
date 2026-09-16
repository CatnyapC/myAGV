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

### Save the map shown in RViz, if needed

Keep SLAM running. In a prepared terminal:

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/room
```

Keep both `room.pgm` (map image) and `room.yaml` (map parameters). A `.rviz` file stores display settings, not the map. [Official map-saving guide](https://docs.elephantrobotics.com/docs/myagv_pi23_en/6-SDKDevelopment/6.2-ApplicationBaseROS1/6.2.5-Real-time_Mapping_with_Gmapping.html).

If it stays at `Waiting for the map`, check that SLAM is running; LiDAR scan points alone do not provide `/map`. Once saved, stop SLAM before starting navigation.

## 3. Start three terminals

Terminal 1: chassis and LiDAR. Skip if already running.

```bash
roslaunch myagv_odometry myagv_active.launch
```

Terminal 2: load the map. Optional for manual driving; required for recording stations with `p`.

```bash
roslaunch ./navigation_fetch.launch map_file:=$HOME/maps/room.yaml
```

In RViz, use **2D Pose Estimate** to set the robot's actual position and heading. Check that the scan aligns with the map.

Terminal 3: keyboard control. Use the P340's port, which may differ from `/dev/ttyUSB0`.

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

## 4. Record an item and calibrate once

Arm keys account for the 90-degree counterclockwise mounting. JSON stores native arm joint angles.

- Drive to the pickup position and align the arm for grasping. Press `p`, enter `red_cup`, and save to `stations.json`.
- Once: move the empty arm into a folded transport pose. Press `p`, copy `arm_angles_deg`, and leave the name blank. Replace `TRANSPORT_ANGLES = None` in `fetch_demo.py` with those measured angles.
- To edit item records manually: `nano stations.json`.
- When restarting teleop after homing without power loss, append `--arm-homed`.

## 5. Fetch and return

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
