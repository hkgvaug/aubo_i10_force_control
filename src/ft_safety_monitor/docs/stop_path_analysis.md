# Stop path and command chain analysis (ROS 1 / AUBO i10)

Date: 2026-10-21

Scope: analysis only.  No changes were made to `aubo_driver`, and no command
that could move a real robot was executed.  Line numbers refer to the
checkout in `~/aubo_ws/src/aubo_i10_ros_noetic`.

> **【待真机验证】** = inferred from source/binary inspection or simulation,
> not measured on hardware.  Except for section 7, the runtime behaviour in
> this document is not an established procedure.  First-day checks:
> [first_day_checklist.md](first_day_checklist.md).

## 1. The live chain 【待真机验证】

The executable launched as `aubo_robot_simulator` is the **Python** script,
not the C++ file with the same name:

- `aubo_controller/CMakeLists.txt:53-57` comments out the C++ target
  (`src/aubo_robot_simulator.cpp`).
- `aubo_controller/CMakeLists.txt:59` installs
  `script/aubo_controller/aubo_robot_simulator` with `catkin_install_python`.
- `devel/lib/aubo_controller/aubo_robot_simulator` is therefore the
  generated Python script.

Live chain:

```
move_group
  └─ action: /aubo_i10_controller/follow_joint_trajectory
       └─ aubo_joint_trajectory_action (C++)
            ├─ subscribes /feedback_states            <- aubo_driver
            ├─ subscribes /robot_status               <- aubo_driver
            ├─ subscribes /trajectory_execution_event
            └─ publishes  /joint_path_command (trajectory_msgs/JointTrajectory)
                 └─ aubo_robot_simulator (Python)
                      ├─ subscribes /aubo_driver/rib_status
                      ├─ subscribes /aubo_driver/real_pose
                      ├─ subscribes /aubo_driver/cancel_trajectory
                      ├─ publishes  /moveItController_cmd
                      │             (trajectory_msgs/JointTrajectoryPoint, ~200 Hz)
                      └─ publishes  /feedback_states
                           └─ aubo_driver (C++)
                                ├─ subscribes /moveItController_cmd (queue 2000)
                                ├─ 500 Hz loop, robotServiceSetRobotPosData2Canbus
                                ├─ publishes /joint_states, /feedback_states,
                                │   /robot_status, /aubo_driver/rib_status,
                                │   /aubo_driver/real_pose
                                └─ /trajectory_execution_event "stop"
                                     -> normal_stopped_ -> decelerate + clear queue
```

Evidence:

- Simulator publisher: `aubo_controller/script/aubo_controller/aubo_robot_simulator:111`
  (`moveItController_cmd`, `JointTrajectoryPoint`).
- Simulator subscriptions: same file, lines 114-115
  (`/aubo_driver/real_pose`, `aubo_driver/cancel_trajectory`).
- Driver subscription: `aubo_driver/src/aubo_driver.cpp:81`
  (`moveItController_cmd`, `JointTrajectoryPoint`).
- Driver publications: `aubo_driver/src/aubo_driver.cpp:67-73`.
- Action server: `aubo_controller/src/joint_trajectory_action.cpp:63,66`.
- Driver 500 Hz loop: `aubo_driver/src/driver_node.cpp:60`,
  `aubo_driver/include/aubo_driver/aubo_driver.h:137`.
- Real-robot launch starts both nodes:
  `aubo_i10_moveit_config/launch/moveit_planning_execution.launch`.

### The `moveit_controller_cmd` vs `moveItController_cmd` question

There is no live mismatch:

- The C++ `aubo_robot_simulator.cpp:116` publishes `moveit_controller_cmd`
  (`sensor_msgs/JointState`) and is **not built** (CMake target commented out).
- The running Python node publishes `moveItController_cmd`
  (`trajectory_msgs/JointTrajectoryPoint`) at line 111, which matches the
  driver.

So the case/type mismatch is dead code.  This must still be confirmed on the
real robot with `rostopic info`, because the installed executable depends on
the build/install step.

## 2. What `MoveGroupCommander.stop()` actually does 【待真机验证】

In MoveIt 1.1.16 (Noetic), `MoveGroupInterface::stop()` publishes a
`std_msgs/String` with data `"stop"` on
`trajectory_execution_manager::TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC`,
which is `trajectory_execution_event`.

Verified from the installed binary:

- `libmoveit_move_group_interface.so.1.1.16`,
  `moveit::planning_interface::MoveGroupInterface::stop()`
- `libmoveit_trajectory_execution_manager.so.1.1.16`,
  `EXECUTION_EVENT_TOPIC` string.

Consequences for this stack:

- The topic is resolved in the calling node's namespace.  Launch the safety
  node in the global namespace (or remap), otherwise a namespaced
  `MoveGroupCommander.stop()` will publish `/ns/trajectory_execution_event`
  and the driver will not see it.  The safety node also publishes the
  absolute topic itself, so this is a belt-and-braces measure.
- The move_group server's own `TrajectoryExecutionManager` also subscribes
  to this topic and stops its execution, sending an action cancel.
- The AUBO `joint_trajectory_action` and `aubo_driver` both subscribe to the
  same topic.

## 3. What happens to queued points 【待真机验证】

There are three queues:

1. `aubo_robot_simulator` (Python) `motion_buffer`
2. `aubo_driver` `buf_queue_`
3. the robot controller's CAN/RIB target buffer
   (`rib_status.data[0]` = `macTargetPosDataSize`)

### 3.1 Driver `buf_queue_`

On `/trajectory_execution_event == "stop"`:

- `aubo_driver.cpp:406-412` sets `normal_stopped_ = true`.  It does **not**
  clear the queue in the callback.
- The next `setRobotJointsByMoveIt()` call (`aubo_driver.cpp:255`) enters the
  `normal_stopped_` branch at lines 270-306:
  - publishes `aubo_driver/cancel_trajectory` with `1` (line 280),
  - runs the OTG velocity-mode deceleration loop (lines 285-303),
  - clears `buf_queue_` and sets `start_move_ = false` (lines 304-306),
  - resets `normal_stopped_` and sets `delay_clear_times = 100`
    (line 306; constant at `aubo_driver.h:78`).
- `updateControlStatus()` (`aubo_driver.cpp:436-461`) clears the queue for
  `delay_clear_times` cycles at 500 Hz, i.e. about **200 ms**, discarding
  points that arrive immediately after the stop.

Caveats:

- The branch only runs when `setRobotJointsByMoveIt()` is called, which
  requires `start_move_ == true` (`updateControlStatus` line 457).  If the
  driver is idle, there is nothing to stop; if points are queued but
  `start_move_` is still false, the clear happens after the 100 ms startup
  condition (`MAXALLOWEDDELAY`, line 439) makes `start_move_` true.
- The whole `setRobotJointsByMoveIt()` body is inside
  `if(controller_connected_flag_)`.  With no robot connection nothing is
  sent anyway.

### 3.2 Simulator `motion_buffer`

- The driver publishes `aubo_driver/cancel_trajectory = 1` (line 280).
- The Python simulator subscribes at line 115 and clears its buffer at
  lines 138-151 (`_clear_buffer`, line 178).
- The interpolation loop also checks `cancle_trajectory` as a stop condition
  (`_motion_worker`, line 226 onwards).

So stopping via `/trajectory_execution_event` clears both the driver queue
and the simulator queue, **provided the driver is connected and reaches the
`normal_stopped_` branch**.

### 3.3 Robot controller CAN/RIB buffer 【待真机验证】

This code does not call any API to flush the robot controller's target
buffer.  The driver only stops feeding it and sends a deceleration stream.
Points already accepted by the robot controller may therefore still execute.
The driver does not have a service/topic to clear that buffer.

This is the most important unknown for real hardware.  Measure it with the
robot at low speed, a clear workspace, and the emergency stop in hand:

- `rostopic echo /aubo_driver/rib_status` before and after a stop;
  `data[0]` is the controller buffer depth.
- Time from `trajectory_execution_event=stop` to actual standstill.
- Whether the AUBO API exposes a buffer-flush/service call (check the AUBO
  SDK documentation; nothing in this driver uses one).

## 4. Action-cancel path is not sufficient by itself 【待真机验证】

`joint_trajectory_action.cpp:214-227` (`cancelCB`) publishes an **empty**
`JointTrajectory` on `joint_path_command` and marks the goal canceled.

The Python simulator's `trajectory_callback` ignores empty trajectories
(`aubo_robot_simulator:467-468`).  Therefore:

- canceling the `follow_joint_trajectory` goal aborts the action, but it
  does **not** clear the simulator's `motion_buffer`;
- it also does not clear the driver's `buf_queue_`.

The reliable order is:

1. publish `/trajectory_execution_event` `"stop"` (or call
   `MoveGroupCommander.stop()`, which does the same),
2. publish `/aubo_driver/cancel_trajectory` `1` (the driver does this too,
   but publishing it directly also covers the case where the driver is not
   in the deceleration branch),
3. cancel the `follow_joint_trajectory` goal as a secondary measure.

## 5. Streaming-control limits (analysis only) 【待真机验证】

`moveItPosCallback` (`aubo_driver.cpp:374-398`) is batch-oriented:

- every accepted new point resets `data_count_ = 0` (line 387);
- `start_move_` becomes true when `data_count_ == 50` (100 ms without a new
  point) or when `buf_queue_.size() > buffer_size_` (400, constructor
  line 38);
- once running, the 500 Hz loop pops one point every 2 ms while
  `rib_buffer_size_ < 300` (`MINIMUM_BUFFER_SIZE`, header line 67).

Consequences:

- Streaming a few points at 100-200 Hz will usually **not** start motion:
  the 100 ms idle condition is never reached and the queue never reaches
  400 points.
- Streaming slower than 10 Hz can move one point at a time, with ~100 ms of
  startup latency.
- A complete MoveIt trajectory (hundreds of points) fills the queue and
  plays out normally.
- This is fine for trajectory execution, but not for a 50-200 Hz
  admittance/force loop.  A real streaming mode requires changing the
  driver (not done here) or bypassing it with the AUBO API.
- `roadPointCompare` uses a 1e-6 rad threshold
  (`aubo_driver.h:139`), so small corrections are not filtered out.
- The driver expects `velocities` and `accelerations` to be present in the
  `JointTrajectoryPoint` (`aubo_driver.cpp:390-392`).

## 6. End-to-end stop latency (theoretical upper bound) 【待真机验证】

Everything in this section is a **calculation from code constants and simple
models, not a measurement** 【待真机验证】.  The two dominant terms (OTG deceleration and
the robot controller RIB buffer) must be measured on hardware.

Assumptions:

- sensor publishes at 250 Hz, low-pass `tau = 0.01 s`;
- debounce = 5 frames;
- the fault is a force step of 160–300 N (the filter crossing time depends on
  the step size);
- the robot is moving at the maximum joint velocity when the fault occurs;
- the robot controller RIB buffer can hold 300 points and consumes them at
  500 Hz.

| Stage | Time | Basis |
|---|---|---|
| Sensor publish period | 4 ms | 250 Hz |
| Low-pass crossing (tau = 10 ms) | ≈7–28 ms | first-order filter, 160–300 N step estimate |
| Debounce, 5 frames | 20 ms | 250 Hz |
| Monitor callback + fault handling | 1–5 ms | Python, estimate |
| `trajectory_execution_event` to driver | 1–5 ms | localhost, estimate |
| Driver 500 Hz loop | 2 ms | `UPDATE_RATE_ = 500` |
| OTG deceleration from max joint velocity | ≈550 ms | `VMAX/AMAX/JMAX` constants, 200 Hz profile, estimate |
| Robot controller RIB buffer drain | ≤600 ms | 300 points at 500 Hz, code constant, estimate |
| **Motion to standstill** | **≈1.2 s** | sum of the above, upper bound |
| Post-stop drop window (`delay_clear_times`) | 200 ms | 100 × 2 ms; not motion, but no new point is accepted |

Notes:

- The OTG estimate uses the per-joint limits:
  `VMAX=3000`, `AMAX=10000`, `JMAX=40000`, with the big/small module ratios
  in `aubo_driver.h:71-77`.  Both module types give a jerk-limited stop time
  of about 0.55 s from their maximum joint velocity.  The profile is sampled
  at 1/200 s (`jti(ARM_DOF, 1.0/200)`), but the driver loop sends without an
  explicit sleep, so the wall-clock value depends on the Canbus/API rate and
  must be measured.
- The RIB buffer estimate is the worst case: the driver only sends the
  deceleration stream when `rib_buffer_size_ < 300`.  If the controller is
  already at 300 points, the deceleration command waits for the buffer to
  drain.  If the controller consumes slower than 500 Hz, the real time is
  longer.
- The safety monitor evaluates thresholds in the `WrenchStamped` callback,
  not in its 20 Hz state timer, so `publish_rate` is not in the detection
  path.
- `MoveGroupCommander` construction can block for ~1–2 s on the first fault
  (observed in the fake-execution test).  The monitor publishes the driver
  stop event **synchronously in the fault callback**, before starting the
  MoveGroup thread, so this delay is not on the driver queue-clear path.
  Do not rely on `MoveGroupCommander.stop()` alone for a fast stop.
- For a sensor dropout, add `watchdog_timeout` (default 200 ms) before the
  same stop path; the theoretical upper bound becomes roughly 1.4 s.
- These numbers are not a safety claim.  Measure the real standstill time
  and the RIB buffer depth before setting any operating speed.

## 7. Must be tested on the real robot

Do these only with the robot at low speed, a clear workspace, and the
emergency stop in hand.  This document does not claim any of them is safe.

> The ordered day-one subset is in
> [first_day_checklist.md](first_day_checklist.md).  Each item must pass
> before the next one; none of the code-inferred behaviour above is an
> established procedure until then.

1. `rosnode info /aubo_robot_simulator` — confirm it is the Python node and
   that it publishes `moveItController_cmd` (`JointTrajectoryPoint`).
2. `rostopic info /moveItController_cmd` — one publisher
   (`/aubo_robot_simulator`), one subscriber (`/aubo_driver`), matching type.
3. `rostopic hz /moveItController_cmd` during a trajectory (~200 Hz).
4. `rostopic echo /aubo_driver/rib_status` — `data[1] == 1` means RosMoveIt
   mode; `data[0]` is the robot controller queue depth.
5. Publish `/trajectory_execution_event` `"stop"` during a slow motion and
   measure standstill latency.  Check that `buf_queue_` and the simulator
   buffer are cleared (log messages and `rib_status`).
6. Call `MoveGroupCommander.stop()` from a separate process (global
   namespace) and verify the same behaviour.
7. Verify whether the robot controller still executes points that were
   already in its RIB buffer; if yes, measure how many and how long.
8. Verify that the first points arriving after a stop are discarded for
   ~200 ms (`STOP_DELAY_CLEAR_TIMES`), and decide whether that is acceptable
   for the next motion command.
9. Verify that `/aubo_driver/cancel_trajectory` is received by the Python
   simulator.
10. Verify the driver's `normal_stopped_` branch is reached when a stop is
    issued while the driver is still in its 100 ms startup phase.

## 8. Safety statement

Everything above is best-effort software supervision.  It has no
certification, no redundant channel, no guaranteed reaction time, and it
depends on ROS, Ethernet, the PC, and the sensor.  It must not replace the
robot's emergency stop, safety-rated inputs, or a safety PLC.
