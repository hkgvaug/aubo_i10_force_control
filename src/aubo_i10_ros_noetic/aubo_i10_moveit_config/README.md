# 按时间控制 AUBO i10 运动

> **本文已按 2026-11-20 的实测结论重写。** 旧版本（"7.5 点/秒"那一套）已失效：
> 驱动现在走 **bulk 批量下发**，控制器按自己的固定节拍回放，吞吐量提高约 20–40 倍。

> 真机运动前请确认：工作区无人、急停在手边、示教器无人操作。
> 硬件急停永远优先于这里的所有软件功能。

---

## 1. 启动（每次开机跑一次）

```bash
# 终端 1：力/力矩传感器
roslaunch kwr75_ft_sensor kwr75_ft_sensor.launch port:=/dev/ttyUSB0

# 终端 2：MoveIt + 真机驱动
roslaunch aubo_i10_moveit_config moveit_planning_execution.launch \
    robot_ip:=192.168.100.11 \
    motion_update_rate:=250
```

启动成功的标志（`~/.ros/log/` 下对应节点日志）：

```
Joint step fuse: 0.00200 rad (0.114 deg) per point ENABLED
login success.
Switches to ros-controller successfully
Real-time joint state push enabled
```

> ⚠️ 驱动会调用 `robotServiceEnterTcp2CanbusMode()`，**控制权从示教器转到 ROS**。
> **必须用 Ctrl+C 正常退出**（驱动析构里会 `LeaveTcp2CanbusMode`）。
> 用 `kill` / 关终端窗口会**把控制器锁在透传模式**，示教器无法点动 ——
> 这种情况用 `~/sdk_bench/release` 或重启控制柜恢复。

---

## 2. 现在的时间模型（关键）

```
请求时长 T ──(motion_update_rate)──▶ N ≈ rate × T 个点 ──(控制器 5.09 ms/点)──▶ 实际时长 ≈ N × dt_ctrl
```

也就是：

```
实际时长 ≈ 请求时长 × motion_update_rate × dt_ctrl
```

**实测 `dt_ctrl = 5.09 ms/点`（196.6 Hz）** —— 2026-11-20 用 600 点闭环标定得出
（600 点走了 3.052 s）。控制柜底层就是标准的 **200 Hz / 5 ms** 透传周期。

> ⚠️ 之前用 `ribData` 反算出的「3.6 ms / 278 Hz」是**错的**：
> `ribData`（`macTargetPosDataSize`）单位并不严格等于 1 ms，实测约 0.71 ms 真实时间/单位。
> 一切以第 5 节的墙钟标定为准。

| 请求 | `motion_update_rate` | 生成点数 | 实测实际时长 | 说明 |
|---|---|---|---|---|
| 2 s | 50 | ~100 | 0.51 s | 太稀，会被保险丝拒绝 |
| 2 s | 200 | ~400 | **≈ 2.04 s** | **推荐：实际 ≈ 请求** |
| 2 s | 250 | ~500 | ≈ 2.55 s | 比请求快 27% |

要让 **实际 ≈ 请求**，取：

```
motion_update_rate ≈ 1 / dt_ctrl ≈ 1 / 0.00509 ≈ 197 Hz   →  取整用 200
```

---

## 3. 单步硬限幅（软件保险丝）

驱动在调用 `robotServiceSetRobotPosData2Canbus()` **之前**，会逐点检查：

> 相邻两点**任一关节**差值 > `~max_joint_step_rad`（默认 **0.002 rad ≈ 0.114°**）
> → **拒绝下发 + 打 ERROR + 清队列停机**

为什么必须有它：控制器按 ~3.6 ms/点回放，**相邻两点之间的大跳变 = 阶跃冲击**
（谐波减速器齿隙撞击 + 伺服猛刹），就是之前听到的那声"咔"。

命中时的日志：

```
[ERROR] [step-fuse] trajectory chunk: joint 1 would jump 0.5236 rad (30.00 deg)
        in one point, limit 0.0020 rad (0.11 deg). Point REJECTED, motion stopped.
```

遥测里能看到累计次数：

```
SDK timing: ... rib_buf=42 ms  bulk_max=48 pts  fuse_trips=1
```

覆盖范围：**轨迹分块、latest-wins 伺服流、OTG 减速路径**三条下发通路全部检查，
任何一条通路都不可能绕过。

### ⚠️ 它同时是一道「速度上限」

因为限幅是按**相邻下发点**判断的，它等价于：

```
允许的最大关节速度 = max_joint_step_rad × motion_update_rate
```

| `motion_update_rate` | 等效最大关节速度 |
|---|---|
| 50 Hz | 0.1 rad/s ≈ **5.7 °/s** ← 很慢，正常轨迹容易被拒 |
| 200 Hz | 0.4 rad/s ≈ **22.9 °/s** ← 默认配置 |
| 250 Hz | 0.5 rad/s ≈ 28.6 °/s |

**所以 `motion_update_rate` 不要低于 200**（默认已是 200），否则正常 MoveIt 轨迹会被保险丝拒绝
（拒绝是安全的：机械臂停下并报错，不会跳）。

需要更快时才提高 `~max_joint_step_rad`，请清楚代价：

```bash
roslaunch ... max_joint_step_rad:=0.006    # 约 1.7 倍速度上限
```

`~max_joint_step_rad:=0` 会**关闭保险丝**（不推荐，仅在明确知道自己在做什么时用）。

---

## 4. 按时间运动的三种方式

### 方式 A：脚本（推荐）

```bash
# upperArm 抬高 10°，请求 2 s
rosrun aubo_i10_moveit_config move_by_time.py --joint 1 --delta-deg -10 --duration 2

# 6 关节绝对目标（度）
rosrun aubo_i10_moveit_config move_by_time.py \
    --target-deg 166.8,58.0,88.4,157.4,108.7,13.8 --duration 3

# 只规划不执行
rosrun aubo_i10_moveit_config move_by_time.py --joint 1 --delta-deg -5 --duration 2 --plan-only
```

关节编号：`0`=shoulder，`1`=upperArm，`2`=foreArm，`3`=wrist1，`4`=wrist2，`5`=wrist3。
`--delta-deg` 为负时，对 upperArm 就是"抬高"。

输出示例（实测）：

```
current (deg): 147.94, 71.44, 142.91, 168.70, 111.83, 35.54
target  (deg): 147.94, 61.44, 142.91, 168.70, 111.83, 35.54
planned 0.83 s, retimed to 2.00 s, 4 points
simulator will emit about 100 points at 50 Hz; the controller plays them back at
~5.09 ms/point, so expect roughly 0.5 s
executed in 0.9 s (requested 2.0 s), result=True
final   (deg): 147.93, 61.45, 142.91, 168.70, 111.82, 35.53
error   (deg): -0.00, +0.00, +0.00, -0.00, -0.00, -0.00
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--velocity` / `--acceleration` | 0.2 | 规划时的速度/加速度缩放 |
| `--dt-per-point` | 0.00509 | 控制器每点回放时间（秒），本机实测值 |
| `--group` | `manipulator_i10` | MoveIt 规划组 |
| `--plan-only` | — | 只规划不执行 |

### 方式 B：直接给 MoveIt 轨迹（不重排时间）

时长由 `motion_update_rate` 与规划速度共同决定，**不可精确控时**。

### 方式 C：让「请求 ≈ 实际」

```bash
roslaunch aubo_i10_moveit_config moveit_planning_execution.launch \
    robot_ip:=192.168.100.11 motion_update_rate:=197
```

---

## 5. 首次标定（建议手动做一次，约 10 分钟）

`dt_ctrl` 不是手册值，必须自己测；否则"按时间"永远对不上。

**已在本机做过（2026-11-20），结果 5.09 ms/点。** 重测方法：

```bash
# 裸 SDK 标定工具：wrist3 平滑 0→2°→0（净位移 0，无归位跳变），600 点
cd ~/aubo_ws/src/aubo_i10_ros_noetic/aubo_driver/config
LD_LIBRARY_PATH=../lib/lib64/aubocontroller:../lib/lib64/log4cplus:../lib/lib64/config \
    ~/sdk_bench/calib_dt 600 2.0
```

实测输出：

```
built 600 points ... fuse rejects = 0
bulk calls=13 over 3.052 s
dt_ctrl = 5.0864 ms/point  (196.6 Hz)
wrist3 end = 12.0846 deg (should equal start 12.1168, delta -0.0322 deg)
```

三条自检：① `fuse rejects = 0`；② 末位置 ≈ 起始位置（残差 <0.05°）；
③ `dt_ctrl` 重复测两次误差应 <5%。

---

## 6. 运行中观察

```bash
rostopic echo -n1 /joint_states/position      # 当前关节角（rad）
rostopic echo -n1 /aubo_driver/rib_status     # data[0] = 控制器缓冲（毫秒）
rostopic echo -n1 /ft_safety/state            # 联锁状态，应为 NORMAL
```

驱动每 2 s 打印一行：

```
SDK timing: waypoint=0.0 ms  diag=95.0 ms  canbus_max=179.0 ms  push_gap_max=162.2 ms
            moving=0  rib_buf=42 ms  bulk_max=60 pts  fuse_trips=0
```

| 字段 | 含义 |
|---|---|
| `rib_buf` | 控制器里还压着多少**毫秒**的运动（= 无法撤回的死区时间） |
| `bulk_max` | 单次 bulk 最多发了多少点 |
| `canbus_max` | 单次 bulk 调用最大耗时（正常 30–60 ms） |
| `fuse_trips` | 保险丝累计拦截次数，**应保持 0** |

---

## 7. 急停 / 停止

```bash
# 软停：清空队列，机械臂停在当前点
rostopic pub -1 /trajectory_execution_event std_msgs/String "stop"

# 取消当前 MoveIt 目标
rostopic pub -1 /aubo_i10_controller/follow_joint_trajectory/cancel actionlib_msgs/GoalID "{}"
```

---

## 8. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `[step-fuse] ... Point REJECTED` | 轨迹太快 / 点太稀。三个办法：① `move_by_time.py --velocity` 降到 **0.15 以下**；② 提高 `motion_update_rate`；③ 调大 `~max_joint_step_rad` |
| 实际时长 ≠ 请求时长 | 见第 2 节；`dt_ctrl` 未标定或 `motion_update_rate` 不匹配 |
| 示教器无法点动 | 驱动没正常退出，控制器仍在透传模式 → `~/sdk_bench/release` 或重启控制柜 |
| `canbus_max` 突然几百 ms | 控制器缓冲已满；检查 `~rib_low_watermark` / `~bulk_chunk_points` |
| `/ft_safety/state` 变 `FAULT` | 传感器断流或超阈值；排除后 `rosservice call /ft_safety/reset` |
| 运动有"咔"声 | 保险丝被关闭或阈值过大；确认 `fuse_trips=0` 且 `max_joint_step_rad ≤ 0.002` |
