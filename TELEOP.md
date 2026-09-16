# AGV + P340 keyboard control (ROS1)

`teleop_control.py` reuses the publisher and chassis key mappings from
`ros-teleop/teleop_twist_keyboard`, plus the existing P340 jog helpers.
Tab selects chassis or arm mode. Gripper keys work in either mode.

## Setup on the AGV

Keep this program in the system ROS Python environment. The project's `uv`
environment uses a different Python version and does not provide ROS modules.
For the standard Noetic image:

```bash
source /opt/ros/noetic/setup.bash
source ~/myagv_ros/devel/setup.bash
sudo apt install ros-noetic-teleop-twist-keyboard python3-pip
/usr/bin/python3 -m pip install --user --upgrade 'pymycobot==4.0.5'
/usr/bin/python3 -c "from pymycobot.ultraArmP340 import ultraArmP340; print('P340 SDK OK')"
```

From this project's root, clone the source dependency if it is absent:

```bash
git clone https://github.com/ros-teleop/teleop_twist_keyboard.git vendor/teleop_twist_keyboard
git -C vendor/teleop_twist_keyboard checkout 8e1e14fdebd31b8e37ec1a453fe7c7fcf03e7648
```

The ROS package supplies `roslib.load_manifest` package discovery; our controller
imports the pinned clone. The clone has no local modifications.

## Run

On the AGV (or through an interactive SSH session), start the official chassis
driver in one terminal:

```bash
roslaunch myagv_odometry myagv_active.launch
```

In another terminal, source the same ROS setup files, enter this project, and run:

```bash
/usr/bin/python3 teleop_control.py --p340-port /dev/ttyUSB0
```

Use the actual P340 port. The LiDAR can also appear as a USB serial device.
Only one process may control the P340 port. Close myBlockly and other arm scripts.
Stop other teleop/navigation velocity publishers before using this controller;
do not run the old direct-serial chassis `move` commands alongside the ROS driver.

The program starts in BASE mode without moving or homing the arm. It waits for
a `cmd_vel` subscriber. Use `cmd_vel:=/your/topic` if the chassis topic differs.

## Keys

| Mode | Keys | Action |
| --- | --- | --- |
| Both | Tab | Stop motion, switch BASE / ARM |
| BASE | `i` / `,` | Forward / backward |
| BASE | `j` / `l` | Turn left / right |
| BASE | `J` / `L` (Shift) | Strafe left / right |
| BASE | `u o m .` | Forward/reverse arcs |
| ARM | `w` / `s` or up/down arrows | Y- / Y+ (vehicle forward/back) |
| ARM | `a` / `d` or left/right arrows | X- / X+ (vehicle left/right) |
| ARM | `k` / `j` | Z+ / Z- |
| ARM | `h` | Home arm; wait until completion |
| Both | `g` / `r` | Close / open gripper |
| Both | `+` / `-` | Stop motion, adjust active mode speed |
| Both | Space | Stop chassis and arm jogging; retain gripper position |
| Both | Ctrl-C | Stop motion and exit; retain gripper position |
| Both | `p` | Stop, read map pose + arm angles, save named item; Enter cancels |

The arm XY mapping accounts for the P340 mounted 90 degrees counterclockwise
relative to the base. Recorded joint angles stay in the arm's native coordinates.

Jogging requires homing. If already homed without power loss, pass `--arm-homed`
to acknowledge that state. Homing is a blocking SDK operation, not keyboard jog.
It has a 60-second software timeout; timeout does not prove physical homing stopped.

Recording also requires homing, current map localization and fresh `odom` feedback.
Press lowercase `p` without Enter. It immediately prints `Recording station`,
then reports each check: ROS connection, base standstill, map position, arm angles.
Wait for `Item name`, type a name, and press Enter. An empty name cancels.
If recording fails, `Not saved` shows the reason; the JSON stays unchanged.
Idle arm snapshots use four stable angle samples spanning about 0.6 seconds,
without requiring a `Moving end` reply after jogging. The arm settling timeout
is 5 seconds; individual SDK reads retain their own 3-second deadline.
Map recording requires SLAM or AMCL localization, not just the chassis driver.
Same-name saves overwrite that item in `stations.json`; `--stations PATH` changes
the file. See [NAVIGATION.md](NAVIGATION.md) for map setup and automatic fetching
back to the base and arm pose captured at demo startup. Exit teleop before running
navigation: its repeated zero velocity commands also compete with `move_base`.

Hold/repeat motion keys to keep moving. By default, absence of motion keys for
0.6 seconds stops chassis and arm jogging. Releasing a key is detected through
this timeout, not immediately. Adjust with `--key-timeout` (0.1–2 seconds);
allow for your keyboard's initial repeat delay.

Jog reads/commands and stop acknowledgements use `--arm-timeout` (default 0.4
seconds), which must be below `--key-timeout`. Feedback polls are skipped when
there isn't enough time before the motion-key deadline; that deadline still stops
motion. A feedback timeout stops the jog and keeps teleop open for the next key.
Command or stop-acknowledgement failures exit the controller. Gripper calls and
arm connection have 3-second deadlines; homing retains its 60-second deadline.
These bounds prevent a blocked serial read from freezing the software stop loop;
they cannot force an unresponsive controller to stop physically.

Default chassis speed: 0.05 m/s, turn: 0.3 rad/s; caps: 0.2 m/s and 1 rad/s.
Default arm speed: 30; gripper speed: 500; close/open: 0/100.
Options: `--speed`, `--turn`, `--arm-speed`, `--grip-speed`, `--clamp`, `--release`.

Arm coordinate limits reuse the existing P340 limits with a 5 mm margin and
polling every 0.25 seconds. These are software checks, not collision avoidance.
Jogging has no fixed duration while motion keys keep repeating. Releasing the
key stops through the key timeout; pressing the same direction can start again.
Unchanged coordinate feedback alone is not treated as proof of a stalled jog.
The XYZ bounds do not detect every physical joint or workspace limit.
Gripper keys send a chassis stop and stop arm jogging before sending the grip command;
they do not verify physical standstill or successful grasping.

## First hardware check

1. With a clear working area, test each chassis direction at default low speed.
2. Release keys: confirm stopping after the timeout. Check Space and Ctrl-C.
3. Switch to ARM, home with `h`, then check all six axis directions in short taps.
4. Check `g`/`r`, initially without an object. Confirm BASE motion does not resume.
5. Check mode switches while moving, then gripping a suitable test object.

Software timeout/cleanup cannot handle a killed or frozen process or loss of robot
power. Verify the chassis driver's independent command watchdog and keep the
hardware stop accessible. This controller has not yet been tested on the robot.

## Local checks (no hardware)

```bash
uv run python -m unittest test_teleop_control test_command_control
```

Tests cover mode switching, motion timeout, command ordering, limits, missing
feedback, EOF, and serial failures. ROS transport and physical behavior require
the hardware checks above.
