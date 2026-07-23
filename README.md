# myAGV P340 Command Control

Language: [English](#english) | [日本語](#日本語)

## English

Use `command_control.py` from the repository root for myAGV chassis movement and P340 arm operation.

## Initial setup on myAGV Raspberry Pi

Install Git and curl:

```bash
sudo apt update
sudo apt install -y git curl
```

If `apt update` hangs on `ports.ubuntu.com` IPv6, use `sudo apt -o Acquire::ForceIPv4=true update`.

Install `uv`:

```bash
export UV_INSTALL_DIR="$HOME/.local/uv"
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$UV_INSTALL_DIR/env"
```

Clone this repository and install the Python environment:

```bash
git clone https://github.com/CatnyapC/myAGV.git
cd myAGV
uv sync
```

`git clone` is preferred over downloading one file with `curl` because `uv sync` uses `pyproject.toml` and `uv.lock`.

## Connect and run

Connect the P340 by USB serial. The myAGV chassis uses `/dev/ttyAMA2` by default. List ports:

```bash
uv run python command_control.py --list-ports
```

On Raspberry Pi the port is usually `/dev/ttyUSB0`:

```bash
uv run python command_control.py --p340-port /dev/ttyUSB0
```

On macOS it may look like:

```bash
uv run python command_control.py --p340-port /dev/cu.usbserial-140
```

The first P340 command runs `go_zero()`. To skip homing:

```bash
uv run python command_control.py --p340-port /dev/ttyUSB0 --no-zero
```

The script waits for each P340 coordinate move to finish before showing the next `myagv>` prompt.

If a move prints `write failed: [Errno 5] Input/output error`, handle it as a dropped USB serial connection:

1. Stop the script with `Ctrl+C`.
2. Replug the USB serial cable.
3. Run `uv run python command_control.py --list-ports` again.
4. Restart with `uv run python command_control.py --p340-port /dev/ttyUSB0`.

If it keeps happening, use a powered USB hub, direct USB port, or shorter USB cable.

## Update later

```bash
cd ~/myAGV
git pull
uv sync
```

## Basic commands

Type commands at the `myagv>` prompt.

| Command | Meaning | Example |
| --- | --- | --- |
| `move DIRECTION [speed] [seconds]` | Manual chassis movement pulse | `move forward 10 1` |
| `stop` | Stop chassis movement | `stop` |
| `reach X Y Z [speed]` | Move to absolute XYZ coordinate in mm | `reach 180 0 80 30` |
| `coord AXIS VALUE [speed]` | Move one axis only | `coord Z 60` |
| `where` | Print current coordinates | `where` |
| `zero` | Run `go_zero()` | `zero` |
| `speed reach VALUE` | Set default reach speed, `1..200` | `speed reach 40` |
| `speed grip VALUE` | Set default gripper speed, `0..1500` | `speed grip 500` |
| `speed move VALUE` | Set default manual move speed, `1..127` | `speed move 10` |
| `speed go VALUE` | Set future navigation speed | `speed go 0.3` |
| `clamp [value] [speed]` | Close gripper, default value `0` | `clamp` |
| `release [value] [speed]` | Open gripper, default value `100` | `release` |
| `grip VALUE [speed]` | Set gripper value, `0..100` | `grip 50 500` |
| `go X Y YAW` | Reserved for navigation, not implemented yet | `go 1.2 0.5 90` |
| `quit` | Exit | `quit` |

`move` directions: `forward`, `back`, `left`, `right`, `cw`, `ccw`.

## Coordinate limits

Coordinates are millimeters. The script rejects out-of-range coordinates by default.

| Axis | Range |
| --- | --- |
| X | `-360..365.55 mm` |
| Y | `-365.55..365.55 mm` |
| Z | `-140..130 mm` |

## 日本語

リポジトリ直下の `command_control.py` で myAGV 車体移動と P340 アーム操作を行います。

## myAGV Raspberry Pi 初回セットアップ

Git と curl をインストールします。

```bash
sudo apt update
sudo apt install -y git curl
```

`apt update` が `ports.ubuntu.com` の IPv6 接続で止まる場合は、`sudo apt -o Acquire::ForceIPv4=true update` を使います。

`uv` をインストールします。

```bash
export UV_INSTALL_DIR="$HOME/.local/uv"
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$UV_INSTALL_DIR/env"
```

このリポジトリを取得し、Python 環境を作ります。

```bash
git clone https://github.com/CatnyapC/myAGV.git
cd myAGV
uv sync
```

`curl` で単体ファイルだけを取得する方法もありますが、`uv sync` は `pyproject.toml` と `uv.lock` を使うため、`git clone` を推奨します。

## 接続と起動

P340 を USB シリアル接続します。myAGV 車体は標準で `/dev/ttyAMA2` を使います。ポートを確認します。

```bash
uv run python command_control.py --list-ports
```

Raspberry Pi では通常 `/dev/ttyUSB0` です。

```bash
uv run python command_control.py --p340-port /dev/ttyUSB0
```

macOS から操作する場合は、次のようなポート名になります。

```bash
uv run python command_control.py --p340-port /dev/cu.usbserial-140
```

最初の P340 コマンド実行時に標準で `go_zero()` を実行します。原点復帰を省略する場合だけ、次のように `--no-zero` を付けます。

```bash
uv run python command_control.py --p340-port /dev/ttyUSB0 --no-zero
```

スクリプトは P340 座標移動が終わってから次の `myagv>` プロンプトを出します。

移動時に `write failed: [Errno 5] Input/output error` が出る場合は、USB シリアル接続が切断またはリセットされたものとして扱います。

1. `Ctrl+C` でスクリプトを止めます。
2. USB シリアルケーブルを挿し直します。
3. `uv run python command_control.py --list-ports` を再実行します。
4. `uv run python command_control.py --p340-port /dev/ttyUSB0` で起動し直します。

何度も起きる場合は、電源付き USB ハブ、直結 USB ポート、短い USB ケーブルを使ってください。

## 2 回目以降の更新

```bash
cd ~/myAGV
git pull
uv sync
```

## 基本コマンド

`myagv>` プロンプトで入力します。

| コマンド | 意味 | 例 |
| --- | --- | --- |
| `move DIRECTION [speed] [seconds]` | 手動の車体移動パルス | `move forward 10 1` |
| `stop` | 車体移動を停止 | `stop` |
| `reach X Y Z [speed]` | XYZ 絶対座標へ移動。単位は mm | `reach 180 0 80 30` |
| `coord AXIS VALUE [speed]` | 1 軸だけ移動 | `coord Z 60` |
| `where` | 現在座標を表示 | `where` |
| `zero` | `go_zero()` を実行 | `zero` |
| `speed reach VALUE` | 標準 reach 速度を設定。`1..200` | `speed reach 40` |
| `speed grip VALUE` | 標準グリッパー速度を設定。`0..1500` | `speed grip 500` |
| `speed move VALUE` | 標準手動移動速度を設定。`1..127` | `speed move 10` |
| `speed go VALUE` | 将来のナビゲーション速度を設定 | `speed go 0.3` |
| `clamp [value] [speed]` | クランパーを閉じる。標準 value は `0` | `clamp` |
| `release [value] [speed]` | クランパーを開く。標準 value は `100` | `release` |
| `grip VALUE [speed]` | クランパー開度を指定。`0..100` | `grip 50 500` |
| `go X Y YAW` | ナビゲーション予約。未実装 | `go 1.2 0.5 90` |
| `quit` | 終了 | `quit` |

`move` の方向: `forward`, `back`, `left`, `right`, `cw`, `ccw`。

## 座標範囲

座標単位は mm です。スクリプトは標準で範囲外の座標を拒否します。

| 軸 | 範囲 |
| --- | --- |
| X | `-360..365.55 mm` |
| Y | `-365.55..365.55 mm` |
| Z | `-140..130 mm` |
