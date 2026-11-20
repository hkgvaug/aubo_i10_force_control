# aubo_i10_force_control

AUBO-i10（ROS 1 Noetic）**笛卡尔导纳力控 + 轨迹运动** 的完整工作空间与实测记录。

包含三部分：

| 目录 | 内容 |
|---|---|
| `src/` | catkin 工作空间源码：`aubo_i10_ros_noetic`（驱动/MoveIt）、`bim_aubo_adapter`（力控节点）、`kwr75_ft_sensor`（六维力驱动）、`ft_safety_monitor`（安全联锁） |
| `contact_rich_control/` | 算法库：解析运动学、6-DoF 导纳控制器、L1 自适应控制器 |
| `tools/sdk_bench/` | 裸 SDK 诊断工具（无 ROS）：下发通路基准、`dt_ctrl` 标定、只读体检、控制权释放 |

---

## 已验证环境

| 项 | 值 |
|---|---|
| 机械臂 | AUBO-i10，控制器 `V9.0.6`，自报型号 `I10_MIX` |
| 机械臂 IP | `192.168.100.11`（本机 `192.168.100.101/24`，网线直连） |
| ROS | Noetic（Ubuntu 20.04） |
| 力传感器 | Kunwei KWR75B，RS422 @ 460800 8N1，`/dev/ttyUSB0` |
| MoveIt 规划组 | `manipulator_i10` |

---

## 快速开始

```bash
# 0) 部署（把 ROS 包同步进工作空间、算法库放到 ~/contact_rich_control）
bash deploy_to_ubuntu.sh

# 1) 力/力矩传感器
roslaunch kwr75_ft_sensor kwr75_ft_sensor.launch port:=/dev/ttyUSB0

# 2) MoveIt + 真机驱动（会接管控制权，示教器暂时失效）
roslaunch aubo_i10_moveit_config moveit_planning_execution.launch \
    robot_ip:=192.168.100.11 motion_update_rate:=200

# 3) 传感器清零（机械臂静止、无接触）
rosservice call /ft_sensor/tare

# 4) 力控节点
roslaunch bim_aubo_adapter aubo_cartesian_force_servo.launch mock_hardware:=false

# 5) 下发目标位姿
rostopic pub -1 /bim/target_pose geometry_msgs/PoseStamped \
 '{header: {frame_id: "base_link"},
   pose: {position: {x: 0.221, y: 0.058, z: 0.634},
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
```

按时间运动：

```bash
rosrun aubo_i10_moveit_config move_by_time.py --joint 1 --delta-deg -10 --duration 2
```

---

## 关键工程结论（全部实测）

### 1. 卡顿的根因不是 ROS，也不是 IO 轮询

裸 SDK（无 ROS、无驱动）只循环调用 `SetRobotPosData2Canbus` 下发**同一个位置**，
单次仍要 **147 ms**（p50），所以与 ROS 1/2 无关。

| 场景 | 单次耗时 |
|---|---|
| 单点 push，控制器缓冲空 | 17.5 ms |
| 单点 push，控制器缓冲非空 | **163 ms → 6 Hz 上限** |
| **bulk `vector<wayPoint_S>` 48–120 点** | **33–48 ms** |
| 网络 ping / TCP 建连 | 0.18 ms / 0.28 ms |

修法：驱动改用 **bulk 分块下发**。upperArm 转 10°：**20.5 s → 0.9 s**。

### 2. 控制柜节拍 = 200 Hz

600 点闭环标定（净位移 0）：

```
dt_ctrl = 5.0864 ms/point  (196.6 Hz)
```

所以想让「请求时长 ≈ 实际时长」，取 `motion_update_rate ≈ 200`。
`ribData`（`macTargetPosDataSize`）单位不是 1 ms，约 **0.71 ms/单位**，不能用它反算节拍。

### 3. 单步硬限幅（防止"咔"的一声）

驱动在三条下发通路（轨迹分块 / 伺服流 / OTG 减速）**之前**逐点检查：

```
相邻两点任一关节 |Δq| > ~max_joint_step_rad（默认 0.002 rad = 0.114°）
  → 拒绝 + ROS_ERROR + 清队列停机
```

因为控制器按 5 ms/点回放，相邻两点的大跳变会被压缩成**阶跃冲击**
（谐波减速器齿隙撞击 + 伺服猛刹）。单测：`src/.../aubo_driver/test/test_joint_step_fuse.cpp`。

> 该阈值同时是速度上限：`0.002 × motion_update_rate`，200 Hz 时约 **22.9 °/s**。
> 所以 `motion_update_rate` 不要低于 200。

### 4. 轨迹吞吐 ≠ 力控带宽（死区时间）

bulk 分块会把 **N × 5 ms** 的运动预先塞进控制器，这段时间内**无法撤回**：

| 缓冲水位 | 力控响应延迟 |
|---|---|
| 500 ms | ~550 ms（只适合跑轨迹） |
| 150 ms | ~200 ms |
| ≤ 50 ms（约 10 点/块） | **~100 ms（柔顺装配够用）** |

做接触力控时用 `~bulk_chunk_points:=10`、`~rib_low_watermark:=30` 左右。

---

## 安全说明

- **本仓库不提供安全级保护**。软件停止依赖 ROS 与 SDK，不能替代急停/安全继电器。
- 驱动调用 `robotServiceEnterTcp2CanbusMode()` 后会**接管控制权，示教器失效**。
  **必须用 Ctrl+C 正常退出**（析构里才 `LeaveTcp2CanbusMode`）；
  用 `kill` 会**把控制器锁死在透传模式**。恢复：`tools/sdk_bench/release` 或重启控制柜。
- 首次接触测试请用海绵/泡沫、低增益、手放急停，确认力响应平滑后再上刚性工件。

## 已知限制

1. 等效伺服带宽受 200 Hz 控制柜节拍限制，不可能更高。
2. `aubo_cartesian_force_servo_node` 的 `robot_ip`/C SDK 部分是占位实现，数据全走 ROS 话题。
3. 扩散策略（Diffusion Policy）模块未包含，`contact_rich_control/__init__.py` 已做可选导入。
4. 传感器到法兰的静态旋转未单独建模（假设与法兰同向）。
5. 驱动源码中 `"aubo" / "123456"` 是 **AUBO 厂家默认登录口令**（非个人凭证），
   如你的控制器改过密码，需要自行修改 `aubo_driver.cpp` 的 `robotServiceLogin` 调用。
