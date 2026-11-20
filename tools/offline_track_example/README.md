# AUBO-i10 SDK 最小示例：离线轨迹（offline track）

用 AUBO C++ SDK 的 `robotServiceOfflineTrack*` 系列把**一整条点列**交给控制柜，
由控制柜内部规划和插补，而不是像 `Tcp2Canbus` 透传那样每 5 ms 喂一个点。

## 区别（实测）

| | 离线轨迹 `robotServiceOfflineTrack*` | 透传 `SetRobotPosData2Canbus` |
|---|---|---|
| 谁做插补 | **控制柜** | 上位机（你） |
| 下发频率 | 一次调用给一整条 | 需要持续喂点 |
| 需要透传模式 | **不需要**（必须退出） | 需要 `EnterTcp2CanbusMode` |
| 力控闭环 | ❌ 不能中途改 | ✅ 可随时改（受缓冲死区限制） |
| 适合 | 已知轨迹、点胶/搬运 | 导纳/力控、需要边跑边改 |

**结论：离线轨迹适合"一次下发、走完为止"，不适合接触力控。**

## 编译

```bash
cd ~/sdk_examples/offline_track
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

如果 `aubo_driver` 不在默认位置：

```bash
cmake .. -DAUBO_DRIVER_DIR=/path/to/aubo_driver
```

CMake 已经把厂家库路径写进 RPATH，**不需要设置 `LD_LIBRARY_PATH`**。

## 运行

```bash
# 默认 dry-run：只 append + clear，机械臂不动
./offline_track

# 指定参数
./offline_track --host 192.168.100.11 --deg 2 --points 200 --vel-deg 10

# 真正执行（会动）
./offline_track --execute
```

## 内置的安全设计

1. **默认 dry-run**，必须显式 `--execute` 才会运动
2. 生成的轨迹**闭合**（首尾都是当前位置），净位移为 0
3. 相邻点步长用 `--max-step-rad`（默认 0.002 rad）硬检查，超了直接退出
4. 运动前检查 `armPowerStatus / softEmergency / remoteEmergency / jointErrorStatus`，
   任一异常直接 abort
5. 退出时调用 `_Exit()`，绕开厂家 SDK 静态析构时的 `stack smashing` 误报

## 实测输出（dry-run）

```
login(192.168.100.11:8899) = 0
leaveTcp2CanbusMode = 10023  (was not in passthrough mode - good)
realRobot=1 power=1 softEmg=0 remoteEmg=0 jointErr=0
start joints(deg):  170.460   71.036  131.475  149.123  113.176   11.082
built 200 points on joint 5, +-2.00 deg, max step 0.00035 rad (0.0201 deg)
path is CLOSED: last point == start point
set joint max vel/acc = 10.0 deg/s -> ret=0,0
offlineTrack: clear=0 append=0

DRY RUN: trajectory appended then cleared, robot did NOT move.
done (exit 0)
```

## 关键 API

| 方法 | 说明 |
|---|---|
| `robotServiceInitGlobalMoveProfile()` | 初始化全局运动属性（关节默认 25°/s） |
| `robotServiceSetGlobalMoveJointMaxVelc(JointVelcAccParam)` | 关节最大速度（离线轨迹按这个限速） |
| `robotServiceSetGlobalMoveJointMaxAcc(JointVelcAccParam)` | 关节最大加速度 |
| `robotServiceOfflineTrackWaypointClear()` | 清空离线轨迹缓存 |
| `robotServiceOfflineTrackWaypointAppend(vector<wayPoint_S>)` | 追加点列（也可传文件名） |
| `robotServiceOfflineTrackMoveStartup(bool IsBolck)` | 开始执行，`true` = 阻塞到走完 |
| `robotServiceOfflineTrackMoveStop()` | 停止 |

`wayPoint_S` 的 `jointpos[6]` 是弧度。离线轨迹下 `cartPos`/`orientation` 不用填，
程序里直接从上一次的 `wayPoint_S` 复制。

## 注意

- 运行前**必须**已退出透传模式（程序会自动 `LeaveTcp2CanbusMode`，返回 `10023` 表示本来就不在）
- 真正的执行时长由控制柜速度限制决定，不是点的数量
- 中途要停：另开终端调 `robotServiceOfflineTrackMoveStop()`，或直接硬件急停
