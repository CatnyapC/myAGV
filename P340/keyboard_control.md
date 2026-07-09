# P340 Keyboard Control

## 🚀 Run

```bash
uv run python P340/keyboard_control.py --port /dev/cu.usbserial-140 --zero
```

## 🕹️ Move

Press once. The arm keeps moving.

| Key | Move |
| --- | --- |
| `w` / `↑` | X- |
| `s` / `↓` | X+ |
| `a` / `←` | Y+ |
| `d` / `→` | Y- |
| `k` | Z+ |
| `j` | Z- |
| `space` | Stop arm + gripper |

## ✊ Gripper

| Key | Action |
| --- | --- |
| `n` | Clamp |
| `m` | Release |

## ⚡ Speed

| Key | Action |
| --- | --- |
| `q` | Slower |
| `e` | Faster |
| `r` | Reset speed |

## 🏠 Other

| Key | Action |
| --- | --- |
| `h` | Go zero |
| `x` | Quit |

After stop or direction change, current `x y z` prints.
