# Contact-state FAULT recovery

Applies to `ft_safety_monitor` + `moveit_pose_executor` on the AUBO i10.

> **Not safety-rated.**  Teach-pendant motion is not covered by the ROS
> interlock.  The only safety-rated protection is the robot emergency stop /
> safety PLC.  Keep the E-stop within reach and the workspace clear.
>
> **【待真机验证】** = only inferred from code/manual, not yet confirmed on
> hardware.  The controller-switch and teach-pendant steps below are a
> proposed recovery procedure, not an established one.  Complete the
> [first-day checklist](first_day_checklist.md) items 3–4 first.

## 1. The deadlock

When the robot is pressing on a component:

1. the raw force exceeds the stop threshold;
2. the monitor latches `FAULT` and stops the ROS motion;
3. `/ft_safety/reset` refuses because the current wrench is still above the
   **warning** threshold;
4. the `moveit_pose_executor` interlock rejects every ROS target because the
   state is not `NORMAL`.

The robot cannot move through ROS, so the force cannot be reduced through
ROS.  This is intentional: the monitor must not allow motion while the load
is high.  Recovery therefore has to happen **outside** the ROS motion path,
with the teach pendant, under manual supervision.

## 2. Immediate actions

1. Stop.  Do not repeatedly call `/ft_safety/reset`; it will keep refusing.
2. Do not disable the monitor, do not set `require_safety_state:=false`, and
   do not switch to the AUBO API just to "make it move".
3. If the structure is unstable, the tool is slipping, or the load is still
   rising, press the emergency stop.
4. Keep the monitor running: it continues to publish
   `/ft_sensor/wrench_raw` and `/ft_safety/state`, which you need for the
   next step.

## 3. Teach-pendant recovery procedure 【待真机验证】

> This whole section is a proposed procedure.  The controller-switch path
> and the exact teach-pendant behaviour have not been tested on hardware.
> Do not present it as an established recovery recipe until the checklist
> items 3–4 pass.

1. **Take control away from ROS. 【待真机验证】**  The driver starts in
   RosMoveIt mode (`robotServiceEnterTcp2CanbusMode`).  Switch it back to the
   robot controller:
   - publish `std_msgs/Int32` `0` on `/aubo_driver/controller_switch`, or
   - use the teach pendant's mode switch.
   The driver refuses the switch while `start_move_` is true (code-inferred,
   not measured); make sure the ROS motion has actually stopped first.
2. Select manual / low-speed mode on the teach pendant.  Verify the robot is
   in the mode you expect, not automatic.
3. Watch the **raw** force, not the tared value:
   ```bash
   rostopic echo /ft_sensor/wrench_raw
   ```
   The monitor state stays `FAULT` (latched).  That is expected; it does not
   block teach-pendant motion.
4. Jog in the direction that **reduces** the dominant raw force:
   - axial contact: retract along the tool axis;
   - side contact: move away from the dominant `fx`/`fy` direction;
   - if the force increases, stop immediately and reverse.
   Use small increments: 0.5–1 mm or 0.1–0.2 degrees, then pause and read the
   force.  Do not make a large move based on one reading.
5. Stop when the raw force/torque is below the **warning** threshold and
   stable.  Do not try to bring it exactly to zero.
6. Switch back to RosMoveIt mode (`/aubo_driver/controller_switch` `1`).
   **【待真机验证】**
7. Call `/ft_safety/reset`.  If it still refuses, the force is still high,
   the data is stale, or the wrong topic is being monitored; do not bypass
   the check.
8. Verify `/ft_safety/state` is `NORMAL`.  Re-approach from a known safe pose;
   do not resume the interrupted trajectory from the contact pose.

If the component is jammed and small retracts do not reduce the force, stop
and disassemble mechanically.  Increasing the force is not a recovery
strategy.

## 4. Information to record

- fault timestamp and `reason` from `/ft_safety/state`;
- raw force/torque at the fault and before/after each jog;
- jog direction and amount;
- whether the controller switch succeeded;
- reset result and the state after reset.

## 5. "Retreat mode" evaluation (not implemented) 【未实现 / 待评估】

Idea: after an operator-confirmed service call, allow a small bounded motion
**only opposite the contact force**, so the robot can unload itself without
a full teach-pendant session.

### 5.1 Proposed scheme (step-and-settle)

1. Operator calls `/ft_safety/retreat` with a signed confirmation and a
   maximum total displacement (e.g., 5 mm) and speed (e.g., 2 mm/s).
2. The node reads the raw wrench, finds the dominant force vector, transforms
   it to `base_link` with TF, and defines the retreat direction as `-F`
   (or the tool-axis projection if the contact is known to be axial).
3. It asks MoveIt for a short Cartesian straight-line move (1 mm), executes
   it at low speed, waits for the robot to settle, and reads the raw force.
4. If the force decreased, repeat up to the total limit.  If the force did
   not decrease or increased, stop and latch `FAULT` again.
5. The normal stop thresholds remain armed; the retreat mode only relaxes the
   "no motion" interlock for this bounded service, with a timeout and a
   dead-man requirement.

### 5.2 Why it is risky in this stack

- **Direction ambiguity.**  The raw wrench contains tool weight and fixture
  preload.  `-F` may point in the wrong direction.  Without gravity
  compensation, the retreat could push further into the part.
- **Friction / jamming.**  The contact force is not necessarily the direction
  needed to release a jammed tenon; a straight retract may not work.
- **Gravity and configuration changes.**  As the arm moves, the gravity
  component changes; a fixed direction becomes invalid.
- **Driver granularity.**  The current driver is batch/stream-limited
  (400-point / 100 ms startup, 500 Hz Canbus).  A 1 mm Cartesian step may not
  execute reliably without changing the driver or using MoveIt Servo.
- **Interlock bypass.**  The retreat mode deliberately allows motion while
  the state is `FAULT`.  That is exactly the condition the interlock exists
  to prevent; a bug in the direction or limit logic can drive into the part.
- **Collision in the retreat direction.**  The straight-line retreat may hit
  another object or the table; the planning scene must include the fixture.
- **No safety rating.**  The mode would still depend on ROS, the PC, the
  sensor, and the operator.

### 5.3 Recommendation

Do not implement the retreat mode yet.  First:

1. validate the real-robot stop latency and the RIB buffer behavior;
2. implement and validate gravity compensation (tool mass/COM);
3. make the Cartesian step reliable (driver streaming or MoveIt Servo);
4. require an operator-held dead-man, a hard displacement/speed/force cap,
   a timeout, and full raw-data logging.

Until then, the teach-pendant procedure in section 3 is the supported
recovery path.
