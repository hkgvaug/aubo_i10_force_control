# Motion entry points and interlock coverage

Date: 2026-10-21.  Scope: current `aubo_ws` checkout.  This is an
inventory, not a safety claim.

The interlock in `moveit_pose_executor` is **not** a global robot interlock.
It only gates the BIM target path.  Everything else below can still move the
robot.

> **【待真机验证】** = from source inspection, not measured on hardware.
> The inventory is a code audit; the actual behaviour of the driver topics
> and controller switching must be checked on the real robot.  Day-one
> checks: [first_day_checklist.md](first_day_checklist.md).

| Entry | Type | Moves robot? | Covered by the executor interlock? | Notes |
|---|---|---|---|---|
| `/bim/target_pose` → `moveit_pose_executor` | `geometry_msgs/PoseStamped` | yes | **YES** | checks `/ft_safety/state` before planning and again before `group.execute()` |
| RViz MotionPlanning `Plan & Execute` / `Execute` | MoveGroup action/service | yes | **NO** | talks directly to `move_group`, bypasses `moveit_pose_executor` |
| Any `moveit_commander` / `MoveGroupInterface` client (including `aubo_demo`) | MoveGroup action | yes | **NO** | same MoveGroup path, no safety check |
| `/move_group/move_group` action, `/move_group/execute_trajectory` action, MoveGroup services | action/service | yes | **NO** | direct MoveIt entry |
| `/joint_path_command` | `trajectory_msgs/JointTrajectory` | yes (Python simulator → driver) | **NO** | anyone can inject a trajectory; monitor can only stop after a fault |
| `/moveItController_cmd` | `trajectory_msgs/JointTrajectoryPoint` | yes (driver stream) | **NO** | direct driver injection; a publisher can keep sending after a stop |
| `/moveAPI_cmd` | `std_msgs/Float32MultiArray`, `type=0` | yes, direct AUBO API `robotServiceJointMove(..., true)` | **NO** | **【待真机验证】** bypasses both `buf_queue_` and the `trajectory_execution_event` stop path |
| `/teach_cmd` | `std_msgs/Float32MultiArray` | effectively disabled (`control_mode_` defaults to SendTargetGoal; most code commented) | **NO** | do not rely on it being inactive forever |
| `/aubo_driver/controller_switch` | `std_msgs/Int32` | changes control mode | **NO** | **【待真机验证】** switches between RosMoveIt and AuboAPI; can hand control to the robot controller |
| `/robot_control` `"powerOn"` | `std_msgs/String` | enables/powers the robot | **NO** | not a motion command, but safety-relevant |
| `/aubo_driver/set_io` / tool IO | service | tool/gripper, not the arm | **NO** | can create contact/load indirectly |
| Teach pendant / AUBO controller | hardware | yes | **NO** | outside ROS; only the E-stop/safety PLC protects |
| Gazebo position controllers / `aubo_gazebo_driver` | topics | simulation only | **NO** | not relevant to the real robot, but useful for tests |
| `/trajectory_execution_event` `"stop"` | `std_msgs/String` | stop only | N/A | stop path used by the monitor, MoveIt and the driver |
| `/aubo_driver/cancel_trajectory` | `std_msgs/UInt8` | clears the Python simulator buffer | N/A | published by the driver; an external `1` also clears the simulator buffer |

## What the monitor actually stops 【待真机验证】

On a fault, the monitor publishes `/trajectory_execution_event` `"stop"` and
`/aubo_driver/cancel_trajectory` `1`, then calls
`MoveGroupCommander.stop()` and cancels the `follow_joint_trajectory` goal.
That covers trajectories that are already in:

- `move_group` / `joint_trajectory_action` → `/joint_path_command` →
  Python simulator → `/moveItController_cmd` → driver queue.

It does **not** prevent a new trajectory from being injected, and it does
not cover:

- `moveAPI_cmd` direct joint moves (`robotServiceJointMove`);
- teach-pendant / AUBO controller motion;
- a node that keeps publishing to `/moveItController_cmd` or
  `/joint_path_command` after the stop.

## Practical rules until a global interlock exists

1. For automated assembly, use only `/bim/target_pose` →
   `moveit_pose_executor`; do not use RViz "Execute" during production.
2. Do not run `aubo_demo` or other MoveGroup clients on the real robot while
   the safety function is required.
3. Do not publish to `/moveAPI_cmd`, `/teach_cmd`, or
   `/moveItController_cmd` from any other node.
4. Treat the teach pendant and the AUBO controller as outside the ROS
   interlock; the E-stop is the protection there.
5. If a global interlock is required, put it in a place every path passes
   through (a MoveGroup capability/plugin, a driver-level guard, or a relay
   that owns the only publisher of the driver command topics).  This is a
   design change, not a parameter.

## Related code

- Executor check: `bim_aubo_adapter/scripts/moveit_pose_executor.py`
  (`_safety_ok`, `_execute`).
- Driver entries: `aubo_driver.cpp:79-84` (subscribers),
  `:374` `moveItPosCallback`, `:489` `AuboAPICallback`, `:415`
  `robotControlCallback`, `:329` `controllerSwitchCallback`.
- Monitor stop path: `ft_safety_monitor/scripts/ft_safety_monitor.py`
  (`_enter_fault`, `_driver_stop_worker`, `_action_cancel_worker`,
  `_move_group_stop_worker`).
- Driver queue clear: `aubo_driver.cpp:255-320`, `:406-412`,
  `:436-461`; analysis in `docs/stop_path_analysis.md`.
