# 力控算法集成说明（contact_rich_control + bim_aubo_adapter）

本文记录本次把 `aubo_force_control_deploy`（笛卡尔导纳 / L1 自适应力控）集成到
`~/aubo_ws` 的过程、**实际踩到的坑**以及每个坑的修法。已在真机 AUBO-i10
（`192.168.100.11`，控制器 V9.0.6，`I10_MIX`）上验证。

> **安全**：这不是安全级保护。软停依赖 ROS 与 SDK，硬件急停永远优先。
> 首次真机务必先 `mock_hardware:=true`。

---

## 1. 一键部署

```bash
cd ~/Downloads/aubo_force_control_deploy
bash deploy_to_ubuntu.sh          # 备份 → rsync → catkin_make
```

部署后位置：

| 内容 | 路径 |
|---|---|
| ROS 包 | `~/aubo_ws/src/{bim_aubo_adapter,kwr75_ft_sensor,ft_safety_monitor}` |
| 算法库 | `~/contact_rich_control/` |
| 旧版备份 | `~/aubo_ws_backup_<时间戳>/` |

离线自检（不连机器人）：

```bash
cd ~/contact_rich_control && python3 verify_deployment.py     # 19/19
python3 -m unittest test_cartesian_force_control -v           # 5 tests OK
python3 test_closed_loop_simulation.py                        # 闭环仿真
```

---

## 2. 启动顺序（真机）

```bash
# 终端 1：力传感器
roslaunch kwr75_ft_sensor kwr75_ft_sensor.launch port:=/dev/ttyUSB0

# 终端 2：MoveIt + aubo_driver（robot_connected=1）
roslaunch aubo_i10_moveit_config moveit_planning_execution.launch robot_ip:=192.168.100.11

# 终端 3：清零（机械臂静止、无接触时执行，只需一次）
rosservice call /ft_sensor/tare

# 终端 4：力控节点
roslaunch bim_aubo_adapter aubo_cartesian_force_servo.launch mock_hardware:=false
```

验证：

```bash
rostopic info /aubo_driver/servo_joint_cmd    # 必须有 /aubo_driver 订阅
rostopic echo -n1 /cartesian_force_servo/status
rostopic hz /aubo_driver/servo_joint_cmd      # 100 Hz
rostopic echo -n1 /aubo_driver/rib_status     # data[0] 必须保持 0
```

下发目标（`frame_id=base_link`，绝对位置；四元数全 0 表示保持当前姿态）：

```bash
rostopic pub -1 /bim/target_pose geometry_msgs/PoseStamped \
 '{header: {frame_id: "base_link"},
   pose: {position: {x: 0.221, y: 0.058, z: 0.634},
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
```

实测：目标 `z -10 mm` → 法兰实际 `-10.00 mm`，全程 `/ft_safety/state = NORMAL`。

---

## 3. 踩过的坑（按严重程度）

### 坑 1 ★★★ `servo_joint_cmd` 没有任何订阅者，力控节点等于空转

节点发布 `/aubo_driver/servo_joint_cmd`，但驱动只订阅
`moveItController_cmd`。也就是说 **100 Hz 的伺服指令全部喂进了空气**，
而唯一被消费的 `moveItController_cmd` 是**排队**语义（`buf_queue_`）。

**修法**：在驱动里新增 latest-wins 伺服通路
（`aubo_i10_ros_noetic/aubo_driver/src/aubo_driver.cpp`）：

- 新增订阅 `~servo_topic`（默认 `/aubo_driver/servo_joint_cmd`）
- `updateControlStatus()` 里若伺服点新鲜（`~servo_timeout`，默认 0.30 s），
  直接 `robotServiceSetRobotPosData2Canbus(latest)` **并跳过 `buf_queue_`**
- 伺服流停止后自动打印 `Servo stream stopped ... back to trajectory queue.`

实测：驱动 `rib_status[0]` 全程保持 0（无积压），停止后自动回到轨迹队列，
`SDK timing` 从 `diag=1250 ms` 恢复到 `diag≈90 ms`。

### 坑 2 ★★★ 100 Hz 流灌进 7.5 点/秒的队列 → 无限积压

`robotServiceSetRobotPosData2Canbus()` 单次阻塞 **0.13–0.7 s**，实测吞吐约
**7.5 点/秒**。把 100 Hz 指令发到 `moveItController_cmd`（`buf_queue_` 队列），
队列会一直涨，手臂会延迟几分钟才执行完，力控完全失效。

**修法**：节点默认 **不再**发布 `moveItController_cmd`
（新参数 `~publish_moveit_points`，默认 `false`），只走坑 1 的 latest-wins 通路。
确需兼容旧驱动时才打开。

### 坑 3 ★★★ 没有关节状态就开始发指令 → 手臂乱窜

节点用固定种子位 `[0, -0.3, 1.0, -0.7, 1.5708, 0]` 算 FK/IK。如果
`aubo_driver` 没起来、`/joint_states` 没数据，它仍会按这个**虚构位姿**解 IK 并
持续下发 → 手臂朝任意构型运动。

**修法**：`joint_states_received` 为假且非 mock 时，直接 hold，不发任何指令。

### 坑 4 ★★★ 安全联锁在"监控节点不存在"时失效（fail-open）

`_on_safety_state` 只在收到消息时更新；`ft_safety_monitor` 没启动时
`self.safety_state` 一直是 `"NORMAL"` → 联锁被**静默关闭**。

**修法**：记录 `last_safety_time`，超过 `~safety_state_age_limit`（默认 1.0 s）
没有新状态就 hold（fail-closed）。

实测：杀掉 `ft_safety_monitor` 后 `/aubo_driver/servo_joint_cmd`
立即停止发布、关节角不变；重启监控后自动恢复 100 Hz。

### 坑 5 ★★ 重力补偿与传感器清零重复计算 → 漂移

节点订阅的是 `/ft_sensor/wrench`（**已清零**），却又调用
`enable_gravity_comp=True` 再减一次工具重力（0.95 kg ≈ 9.3 N）。
零接触时 `w_contact ≈ 9.3 N`，超过 1.5 N 死区 → 稳态漂移约
`(9.3-1.5)/250 ≈ 31 mm`（方向朝上）。

**修法**：新增 `~enable_gravity_comp`，**默认 false**。只有在明确改为订阅
未清零的 `wrench_raw` 时才打开。

同时注意：`/ft_sensor/tare` **不会自动执行**，未清零时静态读数约
`(2.6, -1.1, -1.9) N`，同样会造成约 8 mm 漂移 → 上线前必须手动 tare 一次。

### 坑 6 ★★ `JointTrajectoryPoint` 缺 accelerations → 越界读取

驱动 `moveItPosCallback()` 无条件 `memcpy(..., &msg->accelerations[0], ...)`，
而节点只填了 `velocities`。对空 vector 取 `&v[0]` 是未定义行为，
会让队列里混入垃圾加速度（OTG 急停分支会用到）。

**修法**：驱动侧改成"长度不足就填 0"；节点侧补上 `accelerations`。

### 坑 7 ★★ `import contact_rich_control` 直接失败

`__init__.py` 无条件 import `diffusion_policy / dataset_generator /
statistical_evaluator`，但这三个模块**本次部署没有提供**：

```
ModuleNotFoundError: No module named 'contact_rich_control.diffusion_policy'
```

→ 任何 `import contact_rich_control`（包括 `from contact_rich_control import
L1AdaptiveController`）都会崩。单元测试能跑只是因为它们按文件名直接 import。

**修法**：改为可选导入，缺失时跳过；`__all__` 只导出实际存在的符号。

### 坑 8 ★★ IK 构型跳变被速度限幅"抹平"成快速扫掠

`ik_nearest()` 在奇异位形附近可能返回另一支解。原来的速度限幅
（2.5 rad/s → 0.025 rad/10 ms）会把几 rad 的跳变**平滑地**执行出来，
反而变成一次快速扫掠。

**修法**：单周期关节变化超过 `~max_joint_jump_rad`（默认 0.15 rad）
**直接拒绝并保持**，不再用限幅掩盖。

### 坑 9 ★ `aubo_streaming_servo.launch` 默认 IP 写错网段

默认 `robot_ip=192.168.1.100`，而现场是 `192.168.100.11`。
已改为 `192.168.100.11`。

### 坑 10 ★ L1 节点是空壳：只发当前关节，不动手臂

`aubo_canbus_stream_node.py` 里写的是注释占位：

```python
# In real AUBO: solve IK or differential IK -> joint targets
servo_msg.position = self.current_joints.tolist()   # 原样回发
```

且 `current_flange_pos` 是硬编码 `[0.5, 0, 0.3]`。它还会**长期占用伺服通路**
（持续发当前位姿），导致 MoveIt 轨迹被丢弃、手臂"锁死"。

**修法**：接入 `AuboI10Kinematics` —— 由 `/joint_states` 做 FK 得到实际位姿，
`u_cmd` 作为笛卡尔增量，`ik_nearest` 求关节角，并加 0.6 rad/s 关节速度限制后下发。

### 坑 11 ★ 诊断轮询拖慢 SDK 通道（上次已修，这里复核）

伺服流开启时 `diag` 会涨到 450–1250 ms（通道被 push 占满）。
`~diag_poll_rate`（默认 5 Hz）仍然有效，但伺服期间 `robot_status`
会明显滞后 —— 属预期，不要据此判断机器人状态。

---

## 4. 算法与真机的一致性验证

`aubo_i10_kinematics.py` 的解析 FK 与 URDF **完全一致**（不是近似）：

| 关节配置 | URDF FK 位置 | 算法 FK 位置 | 误差 |
|---|---|---|---|
| `[0,0,0,0,0,0]` | `(0, -0.2953, 1.5132)` | 同 | 0.0 m / 0.0 (R) |
| `[0.1,-0.5,0.8,-0.3,1.2,0.5]` | `(1.0073, -0.1355, 0.8011)` | 同 | 0.0 m / 0.0 (R) |
| 现场实机位姿 | `(0.0776, 0.1580, 1.1121)` | 同 | 0.0 m / 0.0 (R) |

真机闭环实测：

| 项目 | 结果 |
|---|---|
| 力控节点 → 100 Hz 伺服流 | 100.001 Hz，`rib_status[0]=0` |
| 目标 `z −10 mm` | 实际 **−10.00 mm** |
| 力控节点未启动时 MoveIt 1° 轨迹 | 正常，误差 ≤0.01°（无回归） |
| 关掉 `ft_safety_monitor` | 伺服指令立即停止（fail-closed） |
| mock 模式 | 状态/位姿正常发布，**不向机器人发指令** |

---

## 5. 已知限制

1. **控制频率 ≠ 伺服频率**：算法 100 Hz，但**单点**透传 API 只有 ~6 Hz；
   改走 bulk 批量下发后轨迹吞吐可达 190–300 Hz 等效（见第 6 节）。
   注意"轨迹吞吐"不等于"力控带宽"，bulk 分块会引入死区时间，见 6.3。
2. **`robot_ip` / C SDK 为占位**：`_try_init_c_sdk()` 只做库存在性检查，
   不建立连接；数据通路完全走 ROS 话题。
3. **扩散策略（Diffusion Policy）未包含**：本次只交付运动学 + 导纳 + L1。
   `ContactDiffusionPolicy` 等模块缺失，`__init__.py` 已做可选导入。
4. **传感器安装朝向按与法兰一致处理**：`transform_wrench_to_base()` 用的是
   法兰旋转 `R_b`，未单独建模传感器到法兰的静态旋转。
5. **`wrench_raw` 与 `wrench` 的分工**：安全监控必须用 raw，
   力控用已清零的 `wrench`，两者不要混用。

---

## 6. 下发通路实测（2026-11-20，裸 SDK，无 ROS）

### 6.1 先排除的嫌疑

| 测试 | 结果 | 结论 |
|---|---|---|
| `ping -i 0.2` ×100 | avg **0.179 ms**，max 0.29 ms，0% 丢包 | 网络没问题 |
| 裸 TCP 建连 8899 | p50 **0.28 ms** | 网络栈没问题 |
| TCP 重传率 | **0.048%** | 链路没有明显丢包 |
| 普通 RPC（`GetCurrentWaypointInfo` / `GetRobotDiagnosisInfo`） | p50 **~24 ms** | 控制柜 RPC 本身不慢 |
| 只 push、完全无其他 SDK 调用 | p50 **147 ms** | **与 ROS 1 / IO 轮询无关** |
| push + 诊断@5 Hz | p50 174 ms（+18%） | 轮询只是加重，不是主因 |
| push + IO@2 Hz | p50 177 ms（+20%） | 同上 |

> 结论：**换 ROS 2 不会改善**，IO/诊断轮询也不是根因（但它确实额外增加 ~20%，
> 且 `GetBoardIOStatus` 单次高达 **923 ms**，力控场景应直接关掉 IO 发布）。

### 6.2 真正的根因：单点 API 的背压

| 场景 | 单次耗时 |
|---|---|
| 单点 push，控制器缓冲**空** | **17.5 ms** |
| 单点 push，控制器缓冲**非空** | **163 ms**（→ 6 Hz 上限） |
| 单点 push，按 20 / 100 / 200 Hz 节拍发 | 仍然 **163–183 ms**（节拍再慢也没用） |
| **bulk `vector<wayPoint_S>` 48–120 点** | **33–48 ms** |
| bulk 分块流式（48–120 点/块） | **186–308 Hz 等效** |

`ribData`（`macTargetPosDataSize`）不是点数，单位也不严格等于 1 ms：
一次 bulk 塞 80 点后读到 288 → 表面上是 3.6 ms/点，但**墙钟标定推翻了它**。

**2026-11-20 闭环标定（600 点、wrist3 平滑 0→2°→0、净位移 0）：**

```
bulk calls=13 over 3.052 s
dt_ctrl = 5.0864 ms/point  (196.6 Hz)
wrist3 end = 12.0846 deg (start 12.1168, delta -0.0322 deg)
```

即控制柜底层是标准的 **200 Hz / 5 ms 透传周期**；
`ribData` 一个单位 ≈ **0.71 ms** 真实时间。

> 因此 6.2 表里「186–308 Hz 等效」应理解为「≥190 Hz 可达」，
> 上限就是 200 Hz，超过它的数字来自 `ribData` 估算误差。
> **时间相关的计算一律用 `dt_ctrl = 5.09 ms`。**

实际收益：

| 动作 | 改前（单点） | 改后（bulk） |
|---|---|---|
| upperArm 10° | **20.5 s** | **0.9 s**（快 22 倍） |
| 到位误差 | ±0.01° | ±0.01° |

### 6.3 ⚠️ 死区时间：轨迹吞吐 ≠ 力控带宽

这是 bulk 方案最大的代价，做接触力控时必须算清楚：

- 一次 bulk 塞 100 点 ≈ **510 ms 的运动**已经进了控制器缓冲（100 × 5.09 ms）；
- 这 510 ms 内机械臂**闭着眼睛走完**，期间传感器撞到木头、力飙到 50 N，
  新算出的退让点也要**排在缓冲末尾**才能生效；
- 所以 **力控闭环延迟 ≈ 缓冲水位（`rib_buf`）+ 一次 bulk 调用（33–48 ms）+ 传感/计算**。

| 缓冲水位 `rib_buf` | 力控可实现的响应延迟 |
|---|---|
| 500 ms | ~550 ms（只能跑轨迹） |
| 150 ms | ~200 ms |
| **≤ 50 ms** | **~100 ms（柔顺装配够用）** |

按 5.09 ms/点换算：**水位 50 ms ≈ 10 个点/块**。
即 `~bulk_chunk_points:=14` 配合 `~rib_low_watermark:=30` 左右，
就能在保留 278 Hz 内部插补的同时把力控延迟压到 ~100 ms。

代价是 bulk 调用次数变多（每次 33–48 ms），如果块太小会掉回 163 ms 背压 ——
**"最小可行块大小"必须实测**（下一步待做，见 6.5）。

这与 Diffusion Policy 的 Action Chunking 天然契合：策略本来就 10–20 Hz
输出一段动作块，直接把这一块转成 bulk 就能下发。

### 6.4 单步硬限幅（软件保险丝）

`aubo_driver` 在三条下发通路（轨迹分块、latest-wins 伺服流、OTG 减速）**之前**
统一做检查：

```
相邻两点任一关节 |Δq| > ~max_joint_step_rad（默认 0.002 rad = 0.114°）
  → 拒绝该点 + ROS_ERROR + 清空队列停机
```

原因：控制器以 ~3.6 ms/点回放，**大跳变会被压缩成阶跃冲击**
（谐波减速器齿隙撞击 + 伺服猛刹，即现场的"咔"声）。

`~max_joint_step_rad` 的默认值等价于速度上限
`0.002 × motion_update_rate`：

| `motion_update_rate` | 等效最大关节速度 |
|---|---|
| 50 Hz | 5.7 °/s |
| 200 Hz（默认） | 22.9 °/s |
| 250 Hz | 28.6 °/s |

**因此力控/轨迹场景都应把 `motion_update_rate` 提到 ≥200**，否则正常轨迹会被保险丝拒绝。
`fuse_trips` 应恒为 0；一旦非 0 说明有人在塞跳变点。

### 6.5 待做（按顺序，人工触发）

1. ~~标定真实 `dt_ctrl`~~ **已完成**：`dt_ctrl = 5.09 ms/点（196.6 Hz）`，
   工具 `~/sdk_bench/calib_dt`，结果已写入 `move_by_time.py` 与 launch 默认值。
2. **测最小可行块大小**：扫 5 / 8 / 10 / 15 / 20 点每块，
   找到既不触发 163 ms 背压、又不会让缓冲饿死（饿死会咔哒顿挫）的最小值。
   目标：块 ≈ 10 点（≈51 ms 运动）、单次调用 ≈ 25–40 ms、水位稳定在 40–60 ms
   → 力控延迟 ~100 ms。
3. **首次接触测试**：换海绵/泡沫、低增益、手放急停，
   确认接触力阶跃响应平滑不振荡后，再上木料。
