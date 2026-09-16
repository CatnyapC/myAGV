# 记录物品 → 自动取回

实现入口：`teleop_control.py`、`stations.json`、`navigation.py`、`fetch_demo.py`。
在 AGV 的 ROS1 系统 Python 下运行；沿用 [TELEOP.md](TELEOP.md) 的安装步骤。

## 1. 地图与定位

使用官方 `myagv_ros_2023Pi` 工作区。启动底盘、LiDAR，再启动已有地图的导航：

```bash
source /opt/ros/noetic/setup.bash
source ~/myagv_ros/devel/setup.bash
roslaunch ./navigation_fetch.launch map_file:=/absolute/path/map.yaml
```

在本项目根目录运行。`navigation_fetch.launch` 复用官方启动文件，并在启动前把规划器到达容差收紧到 **3 cm / 0.05 rad（约 2.9°）**，留在代码的 5 cm / 5° 检查范围内；同时收紧停稳速度阈值。不要再另外启动官方导航文件。

底盘与 LiDAR 使用机器上原有的启动方式，避免重复启动驱动。在 RViz 用 **2D Pose Estimate** 设置实际初始位置，确认激光与地图重合。
接口使用 `map → base_footprint` TF、`odom`、`move_base`、`cmd_vel`。不要同时向 RViz 或其他客户端发送导航目标。

## 2. 遥控记录物品

```bash
/usr/bin/python3 teleop_control.py --p340-port /dev/ttyUSB0
```

- 在 ARM 模式按 `h` 完成回零；已回零且未断电可用 `--arm-homed`。
- 把底盘停到物品旁，机械臂调到实际抓取姿态。
- 按 `p`：停止、等待停稳、读取地图位姿和全部关节角，显示测量结果。
- 输入物品名（如 `red_cup`）并回车保存；同名覆盖，空名字取消。
- `p` 在两种模式都可用。缺少定位或有效机械臂反馈时不会保存。

默认保存到脚本目录的 `stations.json`。初始文件为空，避免把示例坐标用于实机。也可手动编辑：

```json
{
  "red_cup": {
    "base": {"x_m": 1.2, "y_m": 0.5, "yaw_deg": 90.0},
    "arm_angles_deg": [0.0, 30.0, 20.0]
  }
}
```

上面仅为格式示例。位置为地图坐标、米；底盘朝向为度（-180..180）；机械臂为关节角、度。有第 4 轴时保留第 4 个值；所有动作必须使用相同关节数。
换地图后重新记录。文件只存物品，既不存 dropoff，也不存每次运行的起点。

## 3. 一次性设置运输姿态

把空载机械臂调到适合底盘行驶的收拢姿态。按 `p` 查看 `arm_angles_deg`，名字留空，不保存站点。
把这组角度填入 `fetch_demo.py` 顶部的 `TRANSPORT_ANGLES`。这是唯一一组共用运输角度；默认 `None` 会拒绝运行。

也可通过 `--transport-angles J1 J2 J3 [J4]` 传入实测角度，不需要新建配置。
先低速空载检查运输姿态与抓取/放置姿态之间的运动路径；本程序没有机械臂碰撞规划。

## 4. 自动取回

先用遥控把底盘停到希望放物的位置，机械臂调到希望放物的姿态，夹爪空载。退出遥控，保持供电与已回零状态。

```bash
/usr/bin/python3 fetch_demo.py red_cup --p340-port /dev/ttyUSB0 --arm-homed
```

脚本在任何运动前读取本次底盘和机械臂姿态，流程为：

```text
保存启动姿态 → 运输姿态 → 去物品点 → 张爪 → 抓取姿态 → 夹紧
→ 运输姿态 → 回启动位置和朝向 → 恢复启动机械臂姿态 → 松爪
```

脚本不自动回零。更换启动位置/机械臂姿态后，下次运行直接使用新姿态，不写回 JSON。
可调参数：`--arm-speed`（默认 30）、`--grip-speed`（500）、`--grip-wait`（1.5 秒）、`--clamp`（0）、`--release`（100）、`--nav-timeout`（每段 120 秒）。

## 验证与限制

先仅验证底盘导航；运行前退出遥控：

```bash
/usr/bin/python3 navigation.py pose
/usr/bin/python3 navigation.py go red_cup
/usr/bin/python3 navigation.py roundtrip red_cup
```

`roundtrip` 返回该命令启动时的位置和朝向，不动机械臂。底盘速度由官方导航配置决定，不继承遥控速度。
成功到达后还会检查停稳和实测误差（不超过 5 cm / 5°）。这只是粗略门槛，不保证夹爪能对准；需现场验证重复停靠精度。

超时、失败或中断会取消当前导航目标并中止后续抓放；取消未确认会明确报错。P340 调用超时为 3 秒，运动等待为 30 秒；停止命令为尽力处理，不能保证中断固件已排队的动作，仍需硬件急停。
夹爪按可调时间等待，不检测物体是否夹住。软件检查不等于实机验证。

无硬件测试：

```bash
.venv/bin/python -m unittest test_navigation test_teleop_control test_command_control
```
