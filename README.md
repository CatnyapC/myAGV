# myAGV P340 Command Control

Language: [English](#english) | [日本語](#日本語)

## English

Control scripts for the ultraArm P340 on myAGV live in `P340/`.

Use `P340/command_control.py` for normal coordinate-based operation. The keyboard/WASD script is only a test helper.

## Initial setup on myAGV Raspberry Pi

Install Git and curl:

```bash
sudo apt update
sudo apt install -y git curl
```

Install `uv`:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"
```

Clone this repository and install the Python environment:

```bash
git clone https://github.com/CatnyapC/myAGV.git
cd myAGV
uv sync
```

`git clone` is preferred over downloading one file with `curl` because `uv sync` uses `pyproject.toml` and `uv.lock`.

## Connect and run

Connect the P340 to the myAGV Raspberry Pi by USB serial, then list ports:

```bash
uv run python P340/command_control.py --list-ports
```

On Raspberry Pi the port is usually `/dev/ttyUSB0`:

```bash
uv run python P340/command_control.py --port /dev/ttyUSB0
```

On macOS it may look like:

```bash
uv run python P340/command_control.py --port /dev/cu.usbserial-140
```

The script runs `go_zero()` on startup. To skip startup homing:

```bash
uv run python P340/command_control.py --port /dev/ttyUSB0 --no-zero
```

## Update later

```bash
cd ~/myAGV
git pull
uv sync
```

## Basic commands

Type commands at the `p340>` prompt.

| Command | Meaning | Example |
| --- | --- | --- |
| `go X Y Z [speed]` | Move to absolute XYZ coordinate in mm | `go 180 0 80 30` |
| `coord AXIS VALUE [speed]` | Move one axis only | `coord Z 60` |
| `where` | Print current coordinates | `where` |
| `zero` | Run `go_zero()` | `zero` |
| `speed VALUE` | Set default move speed, `1..200` | `speed 40` |
| `clamp [value] [speed]` | Close gripper, default value `0` | `clamp` |
| `release [value] [speed]` | Open gripper, default value `100` | `release` |
| `grip VALUE [speed]` | Set gripper value, `0..100` | `grip 50 500` |
| `wait` | Wait until movement ends | `wait` |
| `quit` | Exit | `quit` |

## Coordinate limits

Coordinates are millimeters. The script rejects out-of-range coordinates by default.

| Axis | Range |
| --- | --- |
| X | `-360..365.55 mm` |
| Y | `-365.55..365.55 mm` |
| Z | `-140..130 mm` |

## Safety

- Start with low speed, for example `speed 20`.
- Test with no payload first.
- Keep power within reach before moving the arm.
- Real usable coordinates depend on the arm mount, gripper length, and workpiece height.

## 日本語

myAGV 上の ultraArm P340 制御スクリプトは `P340/` にあります。

通常の座標指定操作では `P340/command_control.py` を使います。キーボード/WASD 操作用スクリプトは動作確認用です。

## myAGV Raspberry Pi 初回セットアップ

Git と curl をインストールします。

```bash
sudo apt update
sudo apt install -y git curl
```

`uv` をインストールします。

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"
```

このリポジトリを取得し、Python 環境を作ります。

```bash
git clone https://github.com/CatnyapC/myAGV.git
cd myAGV
uv sync
```

`curl` で単体ファイルだけを取得する方法もありますが、`uv sync` は `pyproject.toml` と `uv.lock` を使うため、`git clone` を推奨します。

## 接続と起動

P340 を myAGV Raspberry Pi に USB シリアル接続し、ポートを確認します。

```bash
uv run python P340/command_control.py --list-ports
```

Raspberry Pi では通常 `/dev/ttyUSB0` です。

```bash
uv run python P340/command_control.py --port /dev/ttyUSB0
```

macOS から操作する場合は、次のようなポート名になります。

```bash
uv run python P340/command_control.py --port /dev/cu.usbserial-140
```

スクリプト起動時は標準で `go_zero()` を実行します。起動時の原点復帰を省略する場合だけ、次のように `--no-zero` を付けます。

```bash
uv run python P340/command_control.py --port /dev/ttyUSB0 --no-zero
```

## 2 回目以降の更新

```bash
cd ~/myAGV
git pull
uv sync
```

## 基本コマンド

`p340>` プロンプトで入力します。

| コマンド | 意味 | 例 |
| --- | --- | --- |
| `go X Y Z [speed]` | XYZ 絶対座標へ移動。単位は mm | `go 180 0 80 30` |
| `coord AXIS VALUE [speed]` | 1 軸だけ移動 | `coord Z 60` |
| `where` | 現在座標を表示 | `where` |
| `zero` | `go_zero()` を実行 | `zero` |
| `speed VALUE` | 標準移動速度を設定。`1..200` | `speed 40` |
| `clamp [value] [speed]` | クランパーを閉じる。標準 value は `0` | `clamp` |
| `release [value] [speed]` | クランパーを開く。標準 value は `100` | `release` |
| `grip VALUE [speed]` | クランパー開度を指定。`0..100` | `grip 50 500` |
| `wait` | 動作完了まで待機 | `wait` |
| `quit` | 終了 | `quit` |

## 座標範囲

座標単位は mm です。スクリプトは標準で範囲外の座標を拒否します。

| 軸 | 範囲 |
| --- | --- |
| X | `-360..365.55 mm` |
| Y | `-365.55..365.55 mm` |
| Z | `-140..130 mm` |

## 安全メモ

- 最初は `speed 20` など低速で試してください。
- 最初はワークを持たせず、空荷重で確認してください。
- アームを動かす前に、すぐ電源を切れる状態にしてください。
- 実際に使える座標は、アームの取り付け位置、クランパー長、ワーク高さによって変わります。
