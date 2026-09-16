# 遥控记录与自动取物：最小实现计划

软件实现已完成；操作步骤见 [NAVIGATION.md](NAVIGATION.md)。运输角度需现场标定，实机验收待完成。

目标：遥控记录物品的底盘停靠位置和机械臂抓取角度，也可直接修改 JSON。运行 demo 后自动取物，返回**启动本次脚本时的底盘位置、朝向和机械臂姿态**，放下物品。

## 使用流程

1. 加载已建好的地图，完成定位；P340 提前回零。
2. 用 `teleop_control.py` 遥控到底盘抓取位置，再把机械臂调到夹爪能夹住物品的姿态。
3. 按新增的 `p` 键：停止底盘和机械臂，等停稳，输入物品名，保存当前底盘位姿及机械臂关节角。同名覆盖。
4. 遥控到本次希望放下物品的位置，把机械臂调到希望放下物品的姿态，夹爪保持空载。
5. 设置实测运输角度，退出遥控，启动 `python3 fetch_demo.py red_cup --p340-port /dev/ttyUSB0 --arm-homed`。
6. demo 自动保存启动姿态，去取物，返回，恢复启动时的机械臂姿态，松爪。

每次运行都重新读取返回姿态。无需记录第二个 dropoff 点，也不使用上次运行的起点。

## 文件与职责

- `teleop_control.py`：增加一个记录按键；保留现有底盘、机械臂、夹爪遥控。
- `stations.json`：单份物品记录，按名称查询；不做多套 station profiles。
- `navigation.py`：读取地图位姿、发送导航目标、等待结果、取消目标；提供简单的 JSON 读写函数供遥控和 demo 共用。
- `fetch_demo.py`：串行执行“取物并返回”；复用现有 P340 控制及官方 SDK。

## JSON 格式

```json
{
  "red_cup": {
    "base": {"x_m": 1.2, "y_m": 0.5, "yaw_deg": 90.0},
    "arm_angles_deg": [0.0, 30.0, 20.0]
  }
}
```

数值仅示意，不能直接用于实机。底盘为 `map` 坐标系，位置单位米、朝向单位度；机械臂为 P340 关节角，单位度，记录实际可读、可控的全部关节。已启用的末端旋转轴也需包含，不能漏掉抓取方向。

所有条目使用同一张地图。换地图后重新记录。手动修改与遥控保存使用完全相同的格式；不存路径、dropoff 或启动姿态。

## 遥控记录

- `p` 在 BASE / ARM 模式都有效；先停，再读取，最后输入名字和写文件。
- 底盘读取最新 `map → base_footprint` TF；机械臂读取 `get_angles_info()`。保存实际反馈，不保存最后一次命令值。
- 定位缺失、反馈无效或机械臂未完成回零时不保存。名字为空取消。
- 只更新指定物品，保留其他条目；临时文件加原子替换，避免写坏 JSON。
- 记录时需要地图定位正常；`move_base` 不得同时执行导航目标。

## demo 顺序

1. 校验物品记录，连接导航和机械臂。**第一次运动前**读取 `start_base` 和 `start_arm_angles`，仅保存在内存。读取失败直接退出。
2. 空载机械臂进入统一运输姿态；导航到物品对应的 `base`。
3. 确认导航成功、底盘停稳；打开夹爪，移动到记录的抓取关节角。
4. 等机械臂到位，夹紧，等待夹爪动作完成，再回到运输姿态。
5. 导航回 `start_base`，包括启动时的朝向；成功并停稳后恢复 `start_arm_angles`。
6. 等机械臂到位，松爪，结束。

运输姿态只用一组现场验证过的共用关节角，作为 demo 常量；它不是第二个站点，也不是放置位置。首版使用固定关节动作序列，现场空载验证运输姿态与抓取/放置姿态之间的路径。夹爪关闭只表示动作已执行，不宣称已验证抓取成功。

P340 必须在启动 demo 前完成回零，随后调好放置姿态；demo 不自动回零，否则会改变要保存的初始姿态。

## 导航复用

沿用官方 ROS1 `myagv_ros_2023Pi` 的地图、AMCL 和 `move_base`。参考官方 Python `MapNavigation.moveToGoal()`，仅提取导航部分；机械臂使用本项目的 P340 控制。

- `get_pose()`：读取地图中的底盘位置和朝向。
- `go_to(pose, timeout)`：发送 `MoveBaseGoal`，坐标系固定 `map`；将 JSON 中的角度转换为四元数。
- `cancel()`：取消本客户端当前目标，等待取消结果。

导航失败、超时或 Ctrl-C 时取消目标并中止后续动作；取消未确认时明确报错。机械臂反馈失败也中止；不继续放下物品。遥控与 demo 不同时运行，避免争用 `/cmd_vel` 和机械臂串口。

参考：[官方 Python 示例](https://github.com/elephantrobotics/myagv_ros/blob/myagv_ros_2023Pi/simple_navigation_goals/scripts/agv_socket_server.py)、[官方导航启动文件](https://github.com/elephantrobotics/myagv_ros/blob/myagv_ros_2023Pi/myagv_navigation/launch/navigation_active.launch)、[ROS ActionClient](https://github.com/ros/actionlib/blob/noetic-devel/actionlib/src/actionlib/simple_action_client.py)。官方示例超时后没有取消目标，需要补上。

## 实现与验收顺序

1. 完成 `navigation.py`、JSON 读写和遥控 `p` 记录。验证记录/覆盖/手改后读取；无效反馈不覆盖文件。
2. 验证仅底盘的“去物品点 → 返回本次起点”，保留朝向；超时必须取消且不触发抓取。
3. 接入固定关节动作和夹爪，先空载，再用一个固定物品验证完整流程。
4. 换一个启动位置及机械臂姿态再次运行，确认返回新的启动姿态，JSON 不出现 dropoff 条目。

导航到达容差必须满足实际抓取需求。首版先实测重复停靠；不足时再补近距离对准。当前不加入视觉识别、LLM、巡逻或任务框架。
