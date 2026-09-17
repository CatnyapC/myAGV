# Record an item and fetch it

Entry points: `teleop_control.py`, `stations.json`, `navigation.py`, and `fetch_demo.py`.
Run with the AGV's ROS1 system Python. Follow [QUICKSTART.md](QUICKSTART.md) for setup and [TELEOP.md](TELEOP.md) for keyboard details.

## 1. Map and localization

Start chassis/LiDAR using [QUICKSTART.md](QUICKSTART.md), including the BCM GPIO 21
then GPIO 20 enable sequence on this myAGV PI. In another prepared terminal, start the UI's mapping
launch: `roslaunch myagv_navigation myagv_slam_laser.launch`.

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

Turn speeds are capped at 0.1 rad/s with a 0.03 rad/s minimum for final heading
alignment, replacing the installed 0.6 rad/s minimum. The planner keeps the
position accepted while aligning heading; the client still checks final position
and heading before grasping. These starting values need hardware verification.
After updating this launch file, stop and restart only the navigation launch;
keep chassis/LiDAR running, then set **2D Pose Estimate** again. Updating ROS
parameters alone does not reload all final-alignment settings in this planner.

Use the robot's existing chassis/LiDAR startup procedure without duplicating drivers. In RViz, use **2D Pose Estimate** to set the actual starting position and heading, then check scan alignment.
The scripts use `map -> base_footprint` TF, `odom`, `move_base`, and `cmd_vel`. Do not send simultaneous navigation goals from RViz or another client.

## 2. Record an item with teleop

Use the actual P340 serial port; it may differ from the example:

```bash
/usr/bin/python3 teleop_control.py --p340-port /dev/ttyUSB0
```

- In ARM mode, press `h` to home. If already homed without power loss, use `--arm-homed`.
- Park with the item on the left. Press `v` for PICKUP teaching. Home with `h` if needed: J1 must be near zero.
- Use `w/s` to move the base forward/backward at 3 cm/s, `a/d` to extend/retract the arm, and `k/j` for height. `q/e` turns the base slowly.
- Align the item with the arm base along the vehicle's forward/back axis. Arm reach handles lateral distance; height handles vertical distance.
- Press `p`: stop, wait for standstill, read the map pose and all arm joint angles, and display the measurements.
- Enter an item name, such as `red_cup`, to save. The same name overwrites that item; an empty name cancels.
- Recording works in all modes, but named pickup records require J1 within 1 degree of zero. Missing localization or invalid arm feedback prevents saving.

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
Re-teach old stations with nonzero J1; changing only that JSON number is not a valid conversion. Record again after changing maps. The file stores items only, with no dropoff or startup pose.

## 3. Set one transport pose

Move the empty arm into a folded pose suitable for driving, keeping J1 at zero. Re-measure any old transport pose that used nonzero J1. Press `p` to see `arm_angles_deg`, then leave the name blank to avoid creating a station.
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
Capture startup poses -> transport pose -> travel to staging -> align beside item -> open gripper
-> grasp pose -> close gripper -> transport pose -> return to startup base pose
-> restore startup arm pose -> open gripper
```

The script does not home automatically. Each run captures its own placement position and arm pose without writing them into JSON.
Pickup and transport commands use J1=0. The startup placement pose can use any valid J1 and is restored unchanged.
Options: `--arm-speed` (default 30), `--grip-speed` (500), `--grip-wait` (1.5 seconds), `--clamp` (0), `--release` (100), and `--nav-timeout` (120 seconds per leg).

## Checks and limits

For chassis-only checks, exit teleop first:

```bash
/usr/bin/python3 navigation.py pose
/usr/bin/python3 navigation.py go red_cup
/usr/bin/python3 navigation.py roundtrip red_cup
```

`roundtrip` returns to that command's starting position and heading without moving the arm. Travel speed comes from the navigation configuration. Final pickup alignment defaults to 3 cm/s; teleop speed settings do not affect it.
The closest staging candidate is 30 cm ahead of or behind the taught base pose,
with the same heading. After move_base finishes there, final alignment moves
forward or backward along that heading. Small overshoots can reverse. Heading
errors get in-place corrections at 0.03–0.05 rad/s before translation resumes.
More than 2 cm lateral error or 15 degrees heading error stops the approach;
the controller does not search around the object or issue large recovery turns.
The arm stays folded during all automatic base movement.

Options shared by `navigation.py` and `fetch_demo.py`:

- `--approach-distance`: 0.3 m by default (allowed 0.1–0.6).
- `--approach-speed`: 0.03 m/s by default (allowed 0.01–0.05).
- `--approach-clearance`: 0.25 m clear radius around the folded robot/load (allowed 0.15–1.0). Measure the actual envelope; this circle also protects small rotations.

Unknown or occupied costmap cells inside the checked envelope stop motion.
Fresh full local costmaps, LiDAR and odometry are required. Restart the updated
navigation launch to enable full costmap publication at 5 Hz. Close teleop and
do not send other action goals while the demo controls the base.

Final control settles within 1 cm longitudinal / 1 degree heading error, then
checks standstill within 2 cm / 2 degrees. No measured improvement for 3 seconds
or exceeding the bounded approach duration stops the attempt. These values need
hardware validation; map accuracy still limits actual grasp accuracy.
The arm replays taught reach/height; it does not sense or compensate for a moved
object. Re-teach moved objects. Task list: [PICKUP_TASKS.md](PICKUP_TASKS.md).

Timeouts, failures and interruptions cancel the current navigation goal and stop subsequent grasp/place steps. Unconfirmed cancellation is reported. The fetch demo bounds P340 calls to 3 seconds and arm movement waits to 30 seconds. Stop commands are best effort and may not interrupt firmware-queued moves; keep the hardware stop accessible.
Gripper completion uses a configurable delay and does not detect whether an object was grasped. Software checks do not replace hardware testing.

Local tests without hardware:

```bash
.venv/bin/python -m unittest test_pickup_alignment test_navigation test_teleop_control test_command_control
```
