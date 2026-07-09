# ultraArm P340 与夹爪 API 速查

更新时间：2026-07-02

## 结论

- P340 对应 Elephant Robotics 官方名：`ultraArm P340`。
- 推荐控制方式：Python SDK `pymycobot`，类名 `ultraArmP340`。
- 通信方式：USB 串口，SDK 默认 `115200` baud。
- P340 配套夹爪文档名：`Adaptive gripper and Quick-change servo`。
- 常用夹爪 API：`set_gripper_state(value, speed)`，`value` 取 `0..100`，`speed` 取 `0..1500` RPM/s。
- 注意：官方串口协议页有一处写 `11520`，但 SDK 源码、Python 示例均使用 `115200`。

## 官方资料

- 产品介绍：<https://docs.elephantrobotics.com/docs/ultraArm-en/1-BriefIntroduction/2-Product/1-UltraArmP340/1-UltraArm_P340.html>
- 参数页：<https://docs.elephantrobotics.com/docs/ultraArm-en/1-BriefIntroduction/2-Product/1-UltraArmP340/1-IntroductionToProductParameters.html>
- Python API：<https://docs.elephantrobotics.com/docs/ultraArm-en/3-HowToUseultraArm/2-SoftwareControl/4-Python/2-PythonAPI.html>
- 夹爪 + quick-change servo：<https://docs.elephantrobotics.com/docs/ultraArm-en/1-BriefIntroduction/2-Product/1-UltraArmP340/5-IntroductionToEndEffector/7-gripper_servo.html>
- 串口 G/M 命令：<https://docs.elephantrobotics.com/docs/ultraArm-en/3-HowToUseultraArm/2-SoftwareControl/6-SerialPort/6-SerialPort.html>
- SDK 仓库：<https://github.com/elephantrobotics/pymycobot>
- P340 SDK API 文档：<https://github.com/elephantrobotics/pymycobot/blob/main/docs/ultraArm_P340_en.md>
- SDK 源码：<https://github.com/elephantrobotics/pymycobot/blob/main/pymycobot/ultraArmP340.py>

## 硬件参数

| 项 | 值 |
| --- | --- |
| 型号 | ultraArm P340 |
| DOF | 3/4 |
| 重复/定位精度 | ±0.1 mm |
| 负载 | 650 g |
| 工作半径 | 340 mm |
| 重量 | 2.9 kg |
| 控制核心 | Mega2560 |
| 通信 | RS485 / USB serial |

关节范围：

| 关节 | 范围 |
| --- | --- |
| J1 | -150°..+170° |
| J2 | -20°..+90° |
| J3 | -5°..+70° 参数页；API 页写到 +110° |
| J4 accessory | API 控制范围 -179°..+179° |

坐标范围：

| 轴 | 范围 |
| --- | --- |
| X | -360..365.55 mm |
| Y | -365.55..365.55 mm |
| Z | -140..130 mm |

## 安装与连接

```bash
pip install pymycobot --upgrade
```

```python
from pymycobot.ultraArmP340 import ultraArmP340

ua = ultraArmP340("/dev/ttyUSB0", 115200)  # Linux
# ua = ultraArmP340("COM6", 115200)        # Windows

ua.go_zero()
```

P340 每次运动前先 `go_zero()`。官方搬运示例明确说明：未回零时角度/坐标读取不可靠。

## P340 Python API

### 状态

| API | 说明 |
| --- | --- |
| `go_zero()` | 回零 |
| `power_on()` | 所有关节上电 |
| `release_all_servos()` | 所有关节掉电/释放 |
| `is_moving_end()` | `1` 运动结束，`0` 未结束 |
| `get_system_version()` | 读固件版本 |
| `get_modify_version()` | 读固件修订版本 |

### 运动

| API | 参数 | 说明 |
| --- | --- | --- |
| `get_angles_info()` | 无 | 返回当前关节角列表 |
| `set_angle(id, degree, speed)` | `id=1..4`, `speed=0..200` | 单关节角度运动 |
| `set_angles(degrees, speed)` | `[j1,j2,j3]` 或 `[j1,j2,j3,j4]` | 多关节角度运动 |
| `get_coords_info()` | 无 | 返回 `[x,y,z,theta]` |
| `set_coord(id, coord, speed)` | `id="X"/"Y"/"Z"` | 单坐标运动 |
| `set_coords(coords, speed)` | `[x,y,z]` 或 `[x,y,z,e]` | 坐标运动 |
| `set_radians(radians, speed)` | radians list | 弧度运动，SDK 内部转角度 |
| `set_mode(mode)` | `0` 绝对，`1` 相对 | 坐标模式 |
| `set_speed_mode(mode)` | `0` 匀速，`2` 加减速 | 速度模式 |
| `set_jog_angle(id, direction, speed)` | `direction=0/1` | 关节点动 |
| `set_jog_coord(axis, direction, speed)` | `axis=1(X),2(Y),3(Z)` | 坐标点动 |
| `set_jog_stop()` | 无 | 停止点动 |

### IO 与夹爪

| API | 参数 | 说明 |
| --- | --- | --- |
| `set_gpio_state(state)` | `0` 开泵，`1` 关泵 | 吸泵控制 |
| `set_gripper_zero()` | 无 | 设置夹爪零位。出厂已设，非必要别改 |
| `set_gripper_state(value, speed)` | `value=0..100`, `speed=0..1500` | 夹爪开合位置 |
| `get_gripper_angle()` | 无 | 读夹爪角度 |
| `set_gripper_release()` | 无 | 释放夹爪 |
| `set_fan_state(state)` | `0` 关，`1` 开 | 风扇控制 |

`set_gripper_state` 的串口等价命令为 `M25 A{value} F{speed}\r`。串口协议页标注 `A=0` close，`A=100` open。

## 夹爪安装

官方 P340 adaptive gripper + quick-change servo 流程：

1. gripper 接 Lego connector。
2. connector 接 quick-change servo。
3. gripper cable 接 quick-change servo 任意端口。
4. quick-change servo cable 接另一侧端口，再接机器人底座 gripper socket。
5. quick-change servo 装到机械臂末端。

旧版 quick-change servo 使用 3-pin cable 和 adapter board。

## 最小抓取示例

```python
import time
from pymycobot.ultraArmP340 import ultraArmP340

ua = ultraArmP340("/dev/ttyUSB0", 115200)
ua.go_zero()
time.sleep(0.5)

# 打开夹爪，移动到抓取点上方
ua.set_gripper_state(100, 500)
time.sleep(0.5)
ua.set_coords([180, 0, 80], 50)
time.sleep(1)

# 下探，夹紧，抬起
ua.set_coords([180, 0, 25], 30)
time.sleep(1)
ua.set_gripper_state(0, 500)
time.sleep(0.8)
ua.set_coords([180, 0, 100], 50)
time.sleep(1)

# 放置
ua.set_coords([120, 80, 40], 50)
time.sleep(1)
ua.set_gripper_state(100, 500)
time.sleep(0.5)
```

实际坐标必须现场标定。先低速、空载、手边急停。

## 串口命令速查

命令都是字符串，末尾带 `\r`。

| 命令 | 说明 | 示例 |
| --- | --- | --- |
| `G0 X.. Y.. Z.. F..` | 坐标运动 | `G0 X200 Y0 Z0 F100\r` |
| `G4 S..` | 等待秒数 | `G4 S1\r` |
| `G28` | 回零 | `G28\r` |
| `G90` | 绝对坐标模式 | `G90\r` |
| `G91` | 相对坐标模式 | `G91\r` |
| `G92 X.. Y.. Z..` | 设置当前位置坐标 | `G92 X0 Y0 Z0\r` |
| `M10 J.. A.. F..` | 单关节角度 | `M10 J1 A90 F100\r` |
| `M11 X.. Y.. Z.. F..` | 多关节角度 | `M11 X90 Y30 Z30 F100\r` |
| `M12` | 读关节角 | `M12\r` |
| `M13 J.. D.. F..` | 关节点动 | `M13 J1 D0 F10\r` |
| `M14 J.. D.. F..` | 坐标点动 | `M14 J1 D0 F10\r` |
| `M15` | 停止点动 | `M15\r` |
| `M17` | 释放所有电机 | `M17\r` |
| `M18` | 锁定/上电所有电机 | `M18\r` |
| `M21 P..` | PWM | `M21 P200\r` |
| `M22` | 关吸泵 | `M22\r` |
| `M23` | 开吸泵 | `M23\r` |
| `M24` | 夹爪校准/设零 | `M24\r` |
| `M25 A.. F..` | 夹爪位置 | `M25 A50 F200\r` |
| `M26` | 释放夹爪 | `M26\r` |
| `M50` | 读夹爪角度 | `M50\r` |
| `M114` | 读当前坐标 | `M114\r` |
| `M119` | 读回零开关 | `M119\r` |

## 如果夹爪实际是 F100 / Pro 力控夹爪

这不是 P340 页面里的默认 adaptive gripper。若实物标签是 `myGripper F100` 或 `myCobot Pro Force Control Gripper`，参考：

- F100 文档：<https://docs.elephantrobotics.com/docs/acc-en/2-serialproduct/F100_Gripper/gripper_en.html>
- Pro gripper API：<https://github.com/elephantrobotics/myCobot320-docs/blob/gitbook-en/1-ProductIntroduction/1.4-AccessoriesTools/1.4.1-Gripper/jiazhua_pi_en.md>

常用 API：

| API | 说明 |
| --- | --- |
| `set_pro_gripper_angle(gripper_id, angle)` | 设置角度，`angle=0..100` |
| `get_pro_gripper_angle(gripper_id)` | 读角度 |
| `set_pro_gripper_open(gripper_id)` | 打开 |
| `set_pro_gripper_close(gripper_id)` | 关闭 |
| `set_pro_gripper_calibration(gripper_id)` | 首次零位校准 |
| `get_pro_gripper_status(gripper_id)` | `0` 移动中，`1` 停止无物体，`2` 检测到物体，`3` 物体掉落 |
| `set_pro_gripper_torque(gripper_id, torque)` | 设置力矩，`1..100` |
| `set_pro_gripper_speed(gripper_id, speed)` | 设置速度，`1..100` |
| `set_pro_gripper_stop(gripper_id)` | 停止 |

默认 `gripper_id=14`。官方提示该类夹爪指令间隔需大于 `1.5s`，且部分固件/驱动曾处于内测发布状态。接线和供电也不同，别和 P340 adaptive gripper API 混用。
