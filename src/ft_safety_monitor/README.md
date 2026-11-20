# ft_safety_monitor

KWR75 力/力矩**辅助**安全监控节点（ROS 1 Noetic）。

> **这不是安全级保护。**
> 本节点没有任何安全认证、冗余通道或确定的反应时间，依赖 ROS、以太网、
> 工控机和传感器。它不能替代急停、安全继电器/安全 PLC 或机器人本体的安全
> 功能。涉及人身安全的场合，唯一可靠的保护是硬件急停和安全级回路。
>
> 节点初始化和参数变化都应在机器人停止、工作区清空的情况下进行。
>
> **【待真机验证】** = 仅由代码/仿真推断，尚未在真机确认。真机第一天必须按
> [`docs/first_day_checklist.md`](docs/first_day_checklist.md) 逐项通过，
> 不能把下面的停止链路当成既定流程。

## 功能

- 订阅 `/ft_sensor/wrench_raw`（`geometry_msgs/WrenchStamped`），**未清零**的原始读数
  - 不能订 `/ft_sensor/wrench`：清零后的值会把工具自重/静态偏置减掉，
    低估实际载荷；过载保护必须按绝对载荷判断
- 力与力矩**分别**按额定量程的百分比判断（默认报警 50%、停机 75%）
- 去抖：连续 N 帧超限才触发，N 可配置
- 看门狗：超过 X ms 没有新帧，按故障处理并停机
- 故障**锁存**，不自动恢复；必须调用 `/ft_safety/reset` 手动复位
- 发布 `/ft_safety/state`（JSON，latched）
- 停机动作（真机行为 **【待真机验证】**）：
  1. 发布 `/trajectory_execution_event` `"stop"`（清 `aubo_driver` 队列的关键路径）
  2. 发布 `/aubo_driver/cancel_trajectory` `1`（清 Python simulator 的 `motion_buffer`）
  3. `MoveGroupCommander.stop()`
  4. 取消 `/aubo_i10_controller/follow_joint_trajectory` 的 goal

停止链路的代码分析和真机必须实测的项目见
[`docs/stop_path_analysis.md`](docs/stop_path_analysis.md)。真机第一天按
[`docs/first_day_checklist.md`](docs/first_day_checklist.md) 执行。

另外两份文档：

- [`docs/motion_entry_points.md`](docs/motion_entry_points.md)：所有能让机械臂
  动的入口，以及联锁覆盖/不覆盖哪些。**联锁只覆盖 `moveit_pose_executor`。**
- [`docs/contact_fault_recovery.md`](docs/contact_fault_recovery.md)：接触状态下
  进入 FAULT 的示教器恢复流程，以及“只沿接触反方向小位移”退让模式的评估
  （未实现）。

## 状态机

| 状态 | 含义 | 退出条件 |
|---|---|---|
| `STARTING` | 等待第一帧 | 收到数据后进入 `NORMAL`；超过 grace 仍无数据则 `FAULT` |
| `NORMAL` | 正常 | 连续 N 帧超过报警阈值 → `WARNING` |
| `WARNING` | 报警，不自动停机 | 回落到报警阈值以下并持续 `warn_clear_frames` → `NORMAL`；达到停机阈值 → `FAULT` |
| `FAULT` | 锁存故障，已发送停机动作 | 只能通过 `/ft_safety/reset` 复位 |

复位条件（全部满足才成功）：

1. 当前处于 `FAULT`；
2. 传感器数据新鲜（未超过看门狗时间）；
3. 当前力和力矩都低于**报警**阈值。

复位**不会**自动恢复运动，需要上层重新下发目标。

## 接口

| 名称 | 类型 | 说明 |
|---|---|---|
| `/ft_sensor/wrench_raw` | `geometry_msgs/WrenchStamped` | 输入（未清零） |
| `/ft_safety/state` | `std_msgs/String` | JSON 状态，latched |
| `/ft_safety/reset` | `std_srvs/Trigger` | 手动复位 |
| `/trajectory_execution_event` | `std_msgs/String` | 故障时发布 `"stop"` |
| `/aubo_driver/cancel_trajectory` | `std_msgs/UInt8` | 故障时发布 `1` |

`/ft_safety/state` 示例：

```json
{
  "state": "FAULT",
  "latched": true,
  "reason": "wrench above stop threshold for 5 frames (...)",
  "sensor_age": 0.004,
  "force": {"x": 160.0, "y": 1.2, "z": 0.4},
  "torque": {"x": 0.01, "y": 0.0, "z": 0.0},
  "force_peak": 160.0,
  "force_axis": "fx",
  "torque_peak": 0.01,
  "torque_axis": "mx",
  "counters": {"warn": 6, "stop": 5, "clear": 0},
  "thresholds": {
    "force_warn": 100.0, "force_stop": 150.0,
    "torque_warn": 4.0, "torque_stop": 6.0
  },
  "rated": {"force": 200.0, "torque": 8.0},
  "stamp": 1790000000.0
}
```

## 参数

见 `config/ft_safety.yaml`。关键项：

| 参数 | 默认 | 说明 |
|---|---|---|
| `~wrench_topic` | `/ft_sensor/wrench_raw` | 必须是未清零话题 |
| `~rated_force_n` | `200.0` | KWR75B 量程 |
| `~rated_torque_nm` | `8.0` | KWR75B 量程 |
| `~warn_ratio_force` / `~warn_ratio_torque` | `0.50` | 报警比例 |
| `~stop_ratio_force` / `~stop_ratio_torque` | `0.75` | 停机比例 |
| `~debounce_frames` | `5` | 连续超限帧数（250 Hz 下约 20 ms） |
| `~warn_clear_frames` | `25` | 回落到报警以下多少帧才解除 `WARNING` |
| `~watchdog_timeout` | `0.20` | 无数据超时（秒） |
| `~startup_grace_period` | `2.0` | 启动宽限（秒） |
| `~baseline_frames` | `50` | 启动时采集多少帧 raw 基线；0 关闭 |
| `~use_driver_stop` | `true` | 发布 `/trajectory_execution_event` |
| `~use_cancel_trajectory` | `true` | 发布 `/aubo_driver/cancel_trajectory` |
| `~use_move_group_stop` | `true` | 调用 `MoveGroupCommander.stop()` |
| `~use_action_cancel` | `true` | 取消 `follow_joint_trajectory` goal |
| `~group_name` | `manipulator_i10` | MoveIt group |
| `~action_namespace` | `/aubo_i10_controller/follow_joint_trajectory` | 控制器 action |
| `~expected_frame_id` | `ft_sensor_link` | 不一致时只告警，不故障 |

**注意**：输入是原始读数，包含工具自重。阈值必须在完成 tare / 重力补偿后
根据实测的静态偏置来设定；否则工具自重本身就可能占用大部分量程。

启动时监控节点会采集 `~baseline_frames` 帧 raw 数据并打印基线。如果基线
已经超过 warn/stop，日志会明确写“这是静态载荷（工具自重/预载），不是
传感器或接触故障”。基线采集完成后才启动 warn/stop 状态机。

接触状态下进入 FAULT 后如何退出，见
[`docs/contact_fault_recovery.md`](docs/contact_fault_recovery.md)。

## 编译

```bash
source /opt/ros/noetic/setup.bash
cd ~/aubo_ws
catkin_make --pkg ft_safety_monitor
source devel/setup.bash
```

## 仿真 / fake execution 测试

`launch/ft_safety_test.launch` 只启动 MoveIt fake execution、假力发布器和
本节点，**不启动 `aubo_driver`、不连接真机、不启动 Gazebo**。

```bash
roslaunch ft_safety_monitor ft_safety_test.launch
```

另一个终端：

```bash
# 正常状态
rostopic echo -n1 /ft_safety/state

# 报警
rostopic pub -1 /fake_wrench/command std_msgs/String "data: 'warn_force'"

# 停机（锁存）
rostopic pub -1 /fake_wrench/command std_msgs/String "data: 'stop_force'"
rostopic echo -n1 /ft_safety/state

# 复位（仍超阈值时会拒绝）
rosservice call /ft_safety/reset

# 回到正常，等待 1 s 后再复位
rostopic pub -1 /fake_wrench/command std_msgs/String "data: 'normal'"
sleep 1
rosservice call /ft_safety/reset

# 看门狗
rostopic pub -1 /fake_wrench/command std_msgs/String "data: 'dropout'"
sleep 1
rostopic echo -n1 /ft_safety/state

# 去抖：4 帧超限 + 1 帧正常，不应触发 FAULT
rostopic pub -1 /fake_wrench/command std_msgs/String "data: 'burst_force'"
```

预期：

- `warn_force` → `WARNING`
- `stop_force` / `stop_torque` → `FAULT`，且 `/trajectory_execution_event`
  收到 `"stop"`，mock action server 打印 `MOCK_CANCEL_RECEIVED`
- 超阈值时 `/ft_safety/reset` 返回失败
- `dropout` 超过 `watchdog_timeout` → `FAULT`
- `burst_force`（4 帧超限 + 1 帧正常）在 `debounce_frames=5` 时不触发
  `FAULT`

也可以运行 `test/run_fake_test.sh` 自动执行上述检查。

### 已验证结果（2026-10-21，fake execution）

`test/run_fake_test.sh` 在只启动 MoveIt fake execution、mock action server 和
假力发布器（不启动 `aubo_driver`、不连接真机、不启动 Gazebo）的条件下通过：

```
PASS: initial state NORMAL
PASS: warn_force -> WARNING
PASS: stop_force -> FAULT
PASS: trajectory_execution_event stop published
PASS: reset refused while wrench is high
PASS: follow_joint_trajectory cancel observed
PASS: reset accepted when wrench is normal
PASS: state NORMAL after reset
PASS: watchdog -> FAULT
PASS: reset accepted after watchdog and fresh data
PASS: burst_force did not latch FAULT (state=NORMAL)
ALL CHECKS PASSED
```

测试中 `MoveGroupCommander.stop()` 和 action cancel 都已实际执行；这不代表
真机停止延迟已经验证。

启动基线日志也验证过：让假力发布器以 `warn_force` 启动，日志会打印
`EXCEEDS THE WARNING THRESHOLD ... STATIC LOAD (tool weight / fixture
preload), NOT a sensor fault`，随后进入 `WARNING`。

### 安全联锁测试（fake execution）

`test/run_interlock_test.sh` 同时启动 MoveIt fake execution、本节点、
`moveit_pose_executor` 和假力发布器，验证执行层在以下情况拒绝动作：

```
PASS: safety state NORMAL
PASS: stop_force -> FAULT
PASS: FAULT rejects execution with safety interlock
PASS: fault reset
PASS: state NORMAL after reset
PASS: NORMAL state passed the interlock
PASS: missing/stale safety state rejects execution
ALL INTERLOCK CHECKS PASSED
```

第三种情况是杀掉监控节点后等待超过 `safety_state_timeout`，执行层返回
`safety interlock: safety state is stale`。

### Gazebo 插件测试

`sensor` 包的 `kwr75_ft_sensor_gazebo_test.launch` 用
`libgazebo_ros_ft_sensor.so` 在 Gazebo 里发布 `/ft_sensor/wrench_raw`。
2026-10-21 在 Gazebo 11 下验证（**仿真，不是真机**）：空载 `force.z ≈ -9.8 N`（1 kg 测试块），施加 -160 N 后约 `-169.8 N`，本节点进入 `FAULT`。细节见
`kwr75_ft_sensor/README.md` 的 Gazebo 仿真一节。

## 真机使用前的检查 【待真机验证】

> 以下为待验证的检查项；未全部通过前，不要把本节点接入自动流程。

1. 先完成传感器 tare / 重力补偿，确认空载时各轴远低于报警阈值。
2. 用已知载荷核对符号和量程。
3. 在低速、空工作区、手按急停的条件下，单独验证
   `/trajectory_execution_event=stop` 能否让机器人停下，并测量停止延迟。
4. 详见 `docs/stop_path_analysis.md` 第 7 节。未完成这些验证前，不要把
   本节点接入自动流程。

## 已知限制

- 不是安全级保护；无认证、无冗余、无确定反应时间。
- 看门狗只能发现“没有数据”，不能发现“数据错误但看起来合理”。
- 原始读数含工具自重和姿态相关的重力分量；阈值会随姿态变化。
- 停止动作是异步的：`aubo_driver` 的队列清理最多约 100 ms 启动 +
  约 200 ms 延迟清理，机器人控制器 RIB 缓冲区内的点是否继续执行仍未
  验证。**【待真机验证】**
- `MoveGroupCommander.stop()` 依赖当前节点命名空间；本节点同时直接发布
  绝对话题 `/trajectory_execution_event`，以降低命名空间风险。
- 复位只清故障锁存，不会恢复运动。
