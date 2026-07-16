# P340 Command Control

## Run

```bash
uv run python P340/command_control.py --port /dev/cu.usbserial-140
```

Use `P340_PORT=/dev/cu.usbserial-140` to avoid passing `--port`.

The script runs `go_zero()` on startup. Use `--no-zero` only when you need to skip homing.

## Coordinate Scale

P340 Cartesian coordinates are millimeters.

| Axis | Official range |
| --- | --- |
| X | -360..365.55 mm |
| Y | -365.55..365.55 mm |
| Z | -140..130 mm |

The script rejects out-of-range coordinates by default. Pass `--no-range-check` only for testing.

## Command Set

Commands keep close to the `ultraArmP340` Python API names, with a few short aliases.

| Command | API | Example |
| --- | --- | --- |
| `go X Y Z [speed]` | `set_coords([x,y,z], speed)` | `go 180 0 80 30` |
| `coord AXIS VALUE [speed]` | `set_coord(axis, value, speed)` | `coord Z 60` |
| `where` | `get_coords_info()` | `where` |
| `zero` | `go_zero()` | `zero` |
| `speed VALUE` | default move speed | `speed 40` |
| `mode abs\|rel` | `set_mode(0/1)` | `mode abs` |
| `speedmode const\|accel` | `set_speed_mode(0/2)` | `speedmode accel` |
| `angle ID DEGREE [speed]` | `set_angle(id, degree, speed)` | `angle 1 30` |
| `joints J1 J2 J3 [speed]` | `set_angles([j1,j2,j3], speed)` | `joints 0 30 30 20` |
| `joints4 J1 J2 J3 J4 [speed]` | `set_angles([j1,j2,j3,j4], speed)` | `joints4 0 30 30 0 20` |
| `angles` | `get_angles_info()` | `angles` |
| `clamp [value] [speed]` | `set_gripper_state(value, speed)` | `clamp` |
| `release [value] [speed]` | `set_gripper_state(value, speed)` | `release` |
| `grip VALUE [speed]` | `set_gripper_state(value, speed)` | `grip 50 500` |
| `gripstop` | `set_gripper_release()` | `gripstop` |
| `power` | `power_on()` | `power` |
| `servorelease` | `release_all_servos()` | `servorelease` |
| `wait` | `is_moving_end()` loop | `wait` |
| `quit` | exit | `quit` |

## Notes

- Default motion speed is `30`.
- Motion speed is checked as `1..200`.
- Gripper value is `0..100`: `0` close, `100` open.
- Gripper speed is `0..1500`.
- Use `--wait` if you want each move command to block until movement ends.
