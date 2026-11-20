# kwr75_ft_sensor — ROS1 Noetic 驱动

Kunwei KWR75 系列六维力/力矩传感器（RS422 接口）的 ROS1 驱动。

> **【待真机验证】** = 仅由手册/代码/仿真推断，尚未在实机确认。真机第一天按
> [`ft_safety_monitor/docs/first_day_checklist.md`](../ft_safety_monitor/docs/first_day_checklist.md)
> 逐项执行；仿真/Gazebo 结果不等于真机验证。

## 功能

- 1 kHz 串口读取放在**独立线程**，ROS 发布频率可配（默认 250 Hz，建议 200–500 Hz）
- 启动时先发停止指令、清空输入缓冲，再发启动指令；退出时再次停止
- 按 `48/49 AA` 帧头同步，校验 28 字节帧和末尾 `0D 0A`，坏帧丢 1 字节重找
- 输出 `geometry_msgs/WrenchStamped`，**力单位 N、力矩单位 N·m**
- 同时发布清零后话题 `/ft_sensor/wrench` 和未清零原始话题
  `/ft_sensor/wrench_raw`；过载保护必须用 raw
- `/ft_sensor/tare` 软件清零（`std_srvs/Trigger`），对最近若干帧求平均
- 80% 量程警告、100% 量程报错（默认按 KWR75B：200 N / 8 Nm）
- 可选一阶低通滤波，`frame_id` 可配置以匹配 URDF
- 附带不依赖 ROS 的串口探针 `kwr75_probe.py`

## 协议速查

串口：`460800, 8N1, 无校验、无流控`。

| 指令 | 含义 |
|---|---|
| `43 AA 0D 0A` | 停止转换和发送 |
| `48 AA 0D 0A` | 连续发送，1 kHz，帧头 `0x48` |
| `49 AA 0D 0A` | 单次触发，回复帧头 `0x49` |

28 字节帧（小端 IEEE-754 float）：

| 偏移 | 内容 |
|---|---|
| 0 | `0x48` 或 `0x49` |
| 1 | `0xAA` |
| 2–5 | Fx（kgf） |
| 6–9 | Fy（kgf） |
| 10–13 | Fz（kgf） |
| 14–17 | Mx（kgf·m） |
| 18–21 | My（kgf·m） |
| 22–25 | Mz（kgf·m） |
| 26–27 | `0x0D 0x0A` |

换算：`N = kgf × 9.80665`，`N·m = kgf·m × 9.80665`。手册没有标定系数，也没有硬件清零指令。

## 目录

```
kwr75_ft_sensor/
├── config/kwr75_ft_sensor.yaml      参数
├── config/99-kwr75-force-sensor.rules  udev 规则模板
├── docs/wiring_and_power.md         上电/接线检查
├── docs/*.png / KWR75-RS422.pdf     手册关键页
├── launch/kwr75_ft_sensor.launch
├── scripts/kwr75_ft_sensor_node.py  ROS 节点
├── scripts/kwr75_probe.py           无 ROS 串口探针
├── src/kwr75_ft_sensor/protocol.py  纯 Python 帧解析
├── test/test_protocol.py            解析单测
└── urdf/kwr75_ft_sensor.urdf.xacro  传感器 link
```

## 依赖与编译

```bash
sudo apt install python3-serial
source /opt/ros/noetic/setup.bash
```

共享目录里的源码需要同步到虚拟机工作空间（在虚拟机里执行）：

```bash
bash /media/sf_ubuntutowin/bimros/ros1_ws/sync_to_vm.sh
```

脚本会把包复制到 `~/aubo_ws/src/kwr75_ft_sensor`、补执行权限并 `catkin_make --pkg kwr75_ft_sensor`。

手工方式：

```bash
rm -rf ~/aubo_ws/src/kwr75_ft_sensor
cp -r /media/sf_ubuntutowin/bimros/ros1_ws/src/kwr75_ft_sensor ~/aubo_ws/src/
chmod +x ~/aubo_ws/src/kwr75_ft_sensor/scripts/*.py
cd ~/aubo_ws
catkin_make --pkg kwr75_ft_sensor
source devel/setup.bash
```

## udev（建议）

找到转换器序列号：

```bash
udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep serial
```

复制 `config/99-kwr75-force-sensor.rules` 到 `/etc/udev/rules.d/`，替换序列号，然后：

```bash
sudo udevadm control --reload && sudo udevadm trigger
```

规则会创建 `/dev/robot/kwr75_force_sensor`，并把 `latency_timer` 设为 1 ms。把用户加入 `dialout` 组后重新登录：

```bash
sudo usermod -aG dialout $USER
```

## 运行 【待真机验证】

> 以下是示教/调试步骤，尚未在实机执行；先完成第一天清单第 1 项。

先跑探针确认数据：

```bash
rosrun kwr75_ft_sensor kwr75_probe.py --port /dev/ttyUSB0 --duration 10 --raw
```

启动节点：

```bash
roslaunch kwr75_ft_sensor kwr75_ft_sensor.launch \
  port:=/dev/robot/kwr75_force_sensor frame_id:=ft_sensor_link
```

另一个终端：

```bash
rostopic echo /ft_sensor/wrench       # 清零后
rostopic echo /ft_sensor/wrench_raw   # 未清零，过载/安全监控用这个
rosservice call /ft_sensor/tare       # 无载、静止时调用
```

## 接口

| 名称 | 类型 | 说明 |
|---|---|---|
| `/ft_sensor/wrench` | `geometry_msgs/WrenchStamped` | 清零后的力 N、力矩 N·m |
| `/ft_sensor/wrench_raw` | `geometry_msgs/WrenchStamped` | **未清零**（低通后）的力/力矩，含工具自重 |
| `/ft_sensor/tare` | `std_srvs/Trigger` | 以最近 `tare_frames` 帧均值作为偏置 |

### 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `~port` | `/dev/robot/kwr75_force_sensor` | 串口设备 |
| `~baudrate` | `460800` | 不要改 |
| `~publish_rate` | `250.0` | 发布频率 Hz |
| `~frame_id` | `ft_sensor_link` | 必须与 URDF 一致 |
| `~raw_topic` | `/ft_sensor/wrench_raw` | 未清零输出话题 |
| `~publish_raw` | `true` | 是否发布 raw 话题 |
| `~rated_force_n` | `200.0` | B 型量程 |
| `~rated_torque_nm` | `8.0` | B 型量程 |
| `~warn_ratio` | `0.8` | 80% 警告 |
| `~lowpass_tau` | `0.01` | 低通时间常数 s，0 关闭 |
| `~tare_frames` | `200` | 200 帧 = 1 kHz 下 0.2 s |
| `~stale_timeout` | `0.5` | 超过该时间无有效帧则暂停发布并告警 |

完整参数见 `config/kwr75_ft_sensor.yaml`。

## 清零与坐标方向 【待真机验证】

> 以下为计划步骤；tare、符号和坐标方向尚未在实机核对。

1. 安装好工具、预热后（手册建议预热 1 小时），保持**无外力、静止**；
2. `rosservice call /ft_sensor/tare`；
3. 注意 `/ft_sensor/wrench` 会减掉工具自重，`/ft_sensor/wrench_raw`
   不会。过载/安全监控必须订阅 raw，否则静态载荷会被低估。
4. 挂已知质量做符号检查：沿 +Z 加载时 `force.z` 应为正，其它轴接近 0；
5. 若方向不对，优先在 URDF 的 fixed joint `rpy` 里修正，而不是改驱动代码。

传感器坐标系按手册第 1 页：Fx 向右、Fy 向上、Fz 沿轴向，力矩右手定则。

## URDF

### 实物参数（实测后填写）【待真机验证】

| 项目 | 值 |
|---|---|
| 高度（安装面到工具面） | `__` mm |
| 重量 | `__` kg |
| 是否带转接板 | `__` |

在填写之前，宏默认按 KWR75B 手册值：`height=0.0315` m、`mass=0.28` kg、
`com_z=height/2`。带转接板时必须实测并同时修改 `height`、`mass`、`com_z`。

在 AUBO 的 xacro 中：

```xml
<xacro:include filename="$(find kwr75_ft_sensor)/urdf/kwr75_ft_sensor.urdf.xacro"/>
<xacro:kwr75_ft_sensor parent="wrist3_Link" xyz="0 0 0" rpy="0 0 0"/>
```

宏生成两级 frame：

```
wrist3_Link
  └─ ft_sensor_link_base   安装面（mounting side）
      └─ ft_sensor_link    工具面/测量原点（tool side）
```

- `xyz` / `rpy`：机器人法兰到传感器**安装面**的变换，默认 `0 0 0`，需实测。
- `height`：安装面到工具面的距离，默认 `0.0315`（KWR75B 手册）。
- `mass` / `com_z`：默认 `0.28` kg / `-1`（负数表示 `height/2`）。
- `radius`：默认 `0.0375`。
- 若加转接板，**实测**后同时改 `height`、`mass`、`com_z`，不要用默认值。
- 工具必须挂在 `ft_sensor_link` 下；驱动的 `~frame_id` 也设为 `ft_sensor_link`。
- 默认 `use_mesh_visual:=false`，视觉和碰撞都用圆柱。`meshes/kwr75.dae` 是 ROS2 包里的 CAD 模型，包围盒约 95 × 95 × 37.5 mm，与手册 31.5 mm 不一致，只在 `use_mesh_visual:=true` 时用于显示；碰撞仍用圆柱。
- 碰撞体是保守圆柱，MoveIt 规划时请把工具和构件也加入 planning scene。

## Gazebo 仿真

`sensor` 包提供一个只依赖 Gazebo 的测试模型，不启动 AUBO 驱动、不连接真机：

```bash
roslaunch kwr75_ft_sensor kwr75_ft_sensor_gazebo_test.launch gui:=false
rostopic echo -n1 /ft_sensor/wrench_raw
```

模型把传感器安装面固定在 world，工具面挂一个 1 kg 测试块，插件是
`libgazebo_ros_ft_sensor.so`，发布 `/ft_sensor/wrench_raw`（`WrenchStamped`，
`frame_id=ft_sensor_link`）。

以下为仿真验证（2026-10-21，Gazebo 11），**不是真机验证**：

- 空载时读数约 `force.z = -9.8 N`（1 kg 测试块，方向为负）；
- 对 `kwr75_gazebo_test::ft_sensor_link` 施加 `-160 N`（world Z）：

```bash
rosservice call /gazebo/apply_body_wrench \
  "{body_name: 'kwr75_gazebo_test::ft_sensor_link', \
    reference_frame: 'world', \
    wrench: {force: {x: 0, y: 0, z: -160}, torque: {x: 0, y: 0, z: 0}}, \
    start_time: {secs: 0, nsecs: 0}, duration: {secs: 5, nsecs: 0}}"
```

  读数约 `-169.8 N`，`ft_safety_monitor` 进入 `FAULT`。
- 固定关节必须加 `<preserveFixedJoint>true</preserveFixedJoint>`，否则
  Gazebo 会把 fixed joint 合并掉，插件报 `jointName does not exist`。
- 测试块的 fixed joint 被合并进 `ft_sensor_link`，所以 body 名是
  `kwr75_gazebo_test::ft_sensor_link`，不是 `test_tool`。

## 上电前 【待真机验证】

先阅读 `docs/wiring_and_power.md`：供电 9–24 VDC / <3 W、蓝正白负、反接会烧；接线需要全双工 RS422（转换器必须有 TX）。供电/接线检查来自手册，尚未在实机执行；按第一天清单第 1 项逐项确认。

## 故障排查

| 现象 | 排查 |
|---|---|
| 完全无帧 | 电源、TX 方向、黑/灰对调、波特率、USB 透传 |
| 有帧但大量 `bad_tail` | 波特率不对/线序干扰/转换器缓冲；检查接地和屏蔽 |
| 发布值全是 0 | 确认帧头、`rostopic hz`、是否调用了 tare 后无负载 |
| 数值约为 1/10 | 误把 kgf 当 N 或反过来 |
| `rate` 明显低于 1000 Hz | udev `latency_timer=1`、USB 供电/虚拟化负载 |
| 重启后前半帧残留 | 节点启动/退出已发送停止并清缓冲；若仍出现，重启转换器 |
