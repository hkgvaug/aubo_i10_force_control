# bim_aubo_adapter

BIM2ROS 与 AUBO-i10（ROS1 Noetic + MoveIt）之间的**解耦适配层**。

> **【待真机验证】** = 仅由代码/仿真推断，尚未在真机确认。联锁在 fake
> execution 下已通过测试，但真机行为仍需按
> [`ft_safety_monitor/docs/first_day_checklist.md`](../ft_safety_monitor/docs/first_day_checklist.md)
> 验证。

```
BIM2ROS
  │  /bim/components   std_msgs/String(JSON)
  ▼
bim_target_bridge
  │  /bim/target_pose  geometry_msgs/PoseStamped (frame=base_link)
  ▼
moveit_pose_executor
  │
  ▼
MoveIt (manipulator_i10) → AUBO-i10 仿真
```

## 节点

| 节点 | 订阅 | 发布 |
|------|------|------|
| `bim_target_bridge.py` | `/bim/components` | `/bim/target_pose`, `/bim/target_info`, `/bim/marker`, `/bim/adapter_state` |
| `moveit_pose_executor.py` | `/bim/target_pose`, `/bim/target_info`, `/ft_safety/state` | `/bim/execution_result`, `/bim/executor_state`, `/bim_target_coords` |
| `publish_components_json.py` | — | `/bim/components`（测试用） |

## 参数

见 `config/bim_aubo.yaml`。最常改的两组：

- `bim_target_bridge.bim_x_range / bim_y_range / bim_z_range`
- `bim_target_bridge.robot_x_range / robot_y_range / robot_z_range`

## 启动

```bash
source /opt/ros/noetic/setup.bash
source ~/aubo_ws/devel/setup.bash

# 1) MoveIt + AUBO 执行接口
roslaunch aubo_i10_moveit_config moveit_planning_execution.launch robot_ip:=127.0.0.1

# 2) Gazebo（可选，需要仿真画面/物理时）
roslaunch aubo_gazebo aubo_i10_gazebo_control.launch gui:=false

# 3) 适配层
roslaunch bim_aubo_adapter bim_aubo_adapter.launch

# 4) 测试数据
rosrun bim_aubo_adapter publish_components_json.py \
  --file /media/sf_ubuntutowin/bimros/components_preview.json --interval 2.0
```

也可以只启动 `bim_aubo_demo.launch` 做 MoveIt fake execution 最小验证：

```bash
roslaunch bim_aubo_adapter bim_aubo_demo.launch
```

## 力安全联锁 【fake execution 已验证；真机待验证】

`moveit_pose_executor` 在执行前和 `group.execute()` 之前各检查一次
`/ft_safety/state`：

- 状态必须是 `NORMAL`；
- 状态必须是新鲜的（默认 `safety_state_timeout: 1.0` s）；
- 没收到状态、JSON 解析失败、`WARNING`、`FAULT` 一律拒绝；
- 拒绝时不规划、不执行，`/bim/execution_result` 返回 `success: false`，
  `error` 以 `safety interlock:` 开头，`/bim/executor_state` 为 `REJECTED`。

这意味着**监控节点没启动或挂了，执行层也会拒绝动作**。这一点已在 fake
 execution 下通过“杀掉监控节点后拒绝执行”测试；真机行为
 **【待真机验证】**。演示时可把 `require_safety_state` 设为 `false`，但真机
不要这样做。

## 设计约束

- 不修改 BIM2ROS 的 JSON 格式与 BIM 数据转换逻辑。
- 不修改 AUBO 的 URDF/SRDF/坐标系。
- 换真实 AUBO 时只替换 `moveit_pose_executor` 之后的执行层。
- 第一阶段 `use_orientation: false`，只约束位置；工具姿态标定后再打开。

## 力控集成（本次新增）

笛卡尔导纳 / L1 自适应力控的集成方式、启动顺序与**踩坑记录**见
[`docs/force_control_integration.md`](docs/force_control_integration.md)。

新增节点与话题：

| 名称 | 类型 | 说明 |
|---|---|---|
| `aubo_cartesian_force_servo_node.py` | 节点 | 100 Hz SE(3) 导纳 + 解析 IK，输出关节伺服点 |
| `aubo_canbus_stream_node.py` | 节点 | 10 Hz L1 自适应 + 100 Hz 微插值 |
| `/bim/target_pose` | `geometry_msgs/PoseStamped` | 输入：目标位姿（base_link） |
| `/ft_sensor/wrench` | `geometry_msgs/WrenchStamped` | 输入：**已清零**的力/力矩 |
| `/ft_safety/state` | `std_msgs/String` | 输入：安全联锁（缺失即 hold） |
| `/aubo_driver/servo_joint_cmd` | `sensor_msgs/JointState` | 输出：latest-wins 伺服点 |
| `/cartesian_force_servo/status` | `std_msgs/String` | 输出：JSON 诊断 |
