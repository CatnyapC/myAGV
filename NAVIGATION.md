# Record an item and fetch it

Entry points: `teleop_control.py`, `stations.json`, `navigation.py`, and `fetch_demo.py`.
Run with the AGV's ROS1 system Python. Follow [QUICKSTART.md](QUICKSTART.md) for setup and [TELEOP.md](TELEOP.md) for keyboard details.

## 1. Map and localization

Save the LiDAR map displayed in RViz first. A `.rviz` file contains display settings. Keep SLAM running and execute in a terminal with the ROS environment loaded:

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/room
```

Keep both `room.pgm` and `room.yaml`. If only scan points are available and `/map` is missing, start SLAM first. Stop SLAM after saving, before starting navigation. [Official map-saving guide](https://docs.elephantrobotics.com/docs/myagv_pi23_en/6-SDKDevelopment/6.2-ApplicationBaseROS1/6.2.5-Real-time_Mapping_with_Gmapping.html).

Use the official `myagv_ros_2023Pi` workspace. Start the chassis and LiDAR, then launch navigation from this project's root:

```bash
source /opt/ros/noetic/setup.bash
source ~/myagv_ros/devel/setup.bash
export ROS_PACKAGE_PATH="$HOME/myAGV/vendor:$ROS_PACKAGE_PATH"
roslaunch ./navigation_fetch.launch map_file:=$HOME/maps/room.yaml
```

`navigation_fetch.launch` includes the official launch file and tightens planner tolerances to **3 cm / 0.05 rad (about 2.9 degrees)**, inside the client's 5 cm / 5-degree arrival check. It also tightens stopped-velocity thresholds. Do not start a second navigation instance.

Use the robot's existing chassis/LiDAR startup procedure without duplicating drivers. In RViz, use **2D Pose Estimate** to set the actual starting position and heading, then check scan alignment.
The scripts use `map -> base_footprint` TF, `odom`, `move_base`, and `cmd_vel`. Do not send simultaneous navigation goals from RViz or another client.

## 2. Record an item with teleop

Use the actual P340 serial port; it may differ from the example:

```bash
/usr/bin/python3 teleop_control.py --p340-port /dev/ttyUSB0
```

- In ARM mode, press `h` to home. If already homed without power loss, use `--arm-homed`.
- Park beside the item and move the arm to its grasp pose.
- Press `p`: stop, wait for standstill, read the map pose and all arm joint angles, and display the measurements.
- Enter an item name, such as `red_cup`, to save. The same name overwrites that item; an empty name cancels.
- Recording works in either mode. Missing localization or invalid arm feedback prevents saving.

Records go into `stations.json` beside the scripts. The initial file is empty so example coordinates cannot be used accidentally. Manual edits use the same format:

```json
{
  "red_cup": {
    "base": {"x_m": 1.2, "y_m": 0.5, "yaw_deg": 90.0},
    "arm_angles_deg": [0.0, 30.0, 20.0]
  }
}
```

These numbers only illustrate the format. Base coordinates use meters in the map frame; heading uses degrees (-180..180). Arm joint angles use degrees. Include the fourth value when a fourth axis is present; all poses must have the same joint count.
Record again after changing maps. The file stores items only, with no dropoff or startup pose.

## 3. Set one transport pose

Move the empty arm into a folded pose suitable for driving. Press `p` to see `arm_angles_deg`, then leave the name blank to avoid creating a station.
Set `TRANSPORT_ANGLES` near the top of `fetch_demo.py` to these measured angles. This is one shared transport pose; the default `None` prevents execution.

Alternatively, pass measured values using `--transport-angles J1 J2 J3 [J4]`. No extra configuration file is needed.
Check the paths between transport, grasp and placement poses at low speed without an object. The program does not plan collision-free arm motion.

## 4. Fetch and return

Use teleop to park at the desired placement location and set the arm to the placement pose with an empty gripper. Exit teleop; keep the arm powered and homed.

```bash
/usr/bin/python3 fetch_demo.py red_cup --p340-port /dev/ttyUSB0 --arm-homed
```

Before any movement, the script captures this run's base and arm poses:

```text
Capture startup poses -> transport pose -> navigate to item -> open gripper
-> grasp pose -> close gripper -> transport pose -> return to startup base pose
-> restore startup arm pose -> open gripper
```

The script does not home automatically. Each run captures its own placement position and arm pose without writing them into JSON.
Options: `--arm-speed` (default 30), `--grip-speed` (500), `--grip-wait` (1.5 seconds), `--clamp` (0), `--release` (100), and `--nav-timeout` (120 seconds per leg).

## Checks and limits

For chassis-only checks, exit teleop first:

```bash
/usr/bin/python3 navigation.py pose
/usr/bin/python3 navigation.py go red_cup
/usr/bin/python3 navigation.py roundtrip red_cup
```

`roundtrip` returns to that command's starting position and heading without moving the arm. Driving speed comes from the official navigation configuration, not the teleop speed setting.
After navigation reports success, the client checks standstill and measured arrival error (at most 5 cm / 5 degrees). This coarse gate does not guarantee grasp alignment; test repeated docking on the robot.

Timeouts, failures and interruptions cancel the current navigation goal and stop subsequent grasp/place steps. Unconfirmed cancellation is reported. The fetch demo bounds P340 calls to 3 seconds and arm movement waits to 30 seconds. Stop commands are best effort and may not interrupt firmware-queued moves; keep the hardware stop accessible.
Gripper completion uses a configurable delay and does not detect whether an object was grasped. Software checks do not replace hardware testing.

Local tests without hardware:

```bash
.venv/bin/python -m unittest test_navigation test_teleop_control test_command_control
```
