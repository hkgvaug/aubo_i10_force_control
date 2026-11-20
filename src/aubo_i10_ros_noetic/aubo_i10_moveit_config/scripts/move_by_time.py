#!/usr/bin/env python3
"""Move the AUBO i10 along a joint trajectory stretched to a requested duration.

The planner produces a trajectory with its own timing; this script rescales
every point's ``time_from_start`` so the last point lands exactly at
``--duration`` seconds, then executes it.  Velocities and accelerations are
rescaled with the same factor so the controller-side interpolation stays
consistent.

Note: the real arm is fed by the driver at only a few position points per
second, so the physically achieved duration is usually *longer* than the
requested one.  Both numbers are printed; see README.md for the rule of thumb.

Examples:
    # raise the arm 10 deg over a requested 5 s
    rosrun aubo_i10_moveit_config move_by_time.py --joint 1 --delta-deg -10 --duration 5

    # absolute target in degrees, joint order: shoulder, upperArm, foreArm,
    # wrist1, wrist2, wrist3
    rosrun aubo_i10_moveit_config move_by_time.py \
        --target-deg 166.8,58.0,88.4,157.4,108.7,13.8 --duration 8

    # only validate / show the plan, never move the robot
    rosrun aubo_i10_moveit_config move_by_time.py --joint 1 --delta-deg -10 \
        --duration 5 --plan-only
"""

import argparse
import math
import sys
import time

import moveit_commander
import rospy

JOINT_NAMES = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
               "wrist1_joint", "wrist2_joint", "wrist3_joint"]


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--delta-deg", type=float,
                        help="relative move of --joint, in degrees (negative = up for upperArm)")
    target.add_argument("--target-deg", type=str,
                        help="absolute target, comma separated, 6 values in degrees")
    parser.add_argument("--joint", type=int, default=1, choices=range(6),
                        help="joint index for --delta-deg: 0..5, default 1 (upperArm)")
    parser.add_argument("--duration", type=float, required=True,
                        help="requested motion time of the trajectory, in seconds")
    parser.add_argument("--velocity", type=float, default=0.2,
                        help="planning velocity scaling used before retiming, default 0.2")
    parser.add_argument("--acceleration", type=float, default=0.2,
                        help="planning acceleration scaling used before retiming, default 0.2")
    parser.add_argument("--group", default="manipulator_i10", help="MoveIt planning group")
    parser.add_argument("--dt-per-point", type=float, default=0.00509,
                        help="controller playback time per queued point, seconds "
                             "(default 0.00509 = measured on this controller, see README)")
    parser.add_argument("--plan-only", action="store_true",
                        help="plan and retime, but do not execute")
    args = parser.parse_args()

    if args.duration <= 0.0:
        parser.error("--duration must be > 0")

    if args.target_deg:
        try:
            values = [float(v) for v in args.target_deg.split(",")]
        except ValueError:
            parser.error("--target-deg needs 6 comma separated numbers")
        if len(values) != 6:
            parser.error("--target-deg needs exactly 6 values")
    return args


def retime(trajectory, duration):
    """Stretch the trajectory so it lasts exactly `duration` seconds."""
    points = trajectory.joint_trajectory.points
    if not points:
        return 0.0
    original = points[-1].time_from_start.to_sec()
    if original <= 0.0:
        return 0.0
    factor = duration / original
    for point in points:
        point.time_from_start = rospy.Duration.from_sec(
            point.time_from_start.to_sec() * factor)
        if point.velocities:
            point.velocities = [v / factor for v in point.velocities]
        if point.accelerations:
            point.accelerations = [a / (factor * factor) for a in point.accelerations]
    return original


def main():
    args = parse_args()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_by_time", anonymous=True)

    group = moveit_commander.MoveGroupCommander(args.group)
    group.set_planning_time(5.0)
    group.set_max_velocity_scaling_factor(args.velocity)
    group.set_max_acceleration_scaling_factor(args.acceleration)

    current = list(group.get_current_joint_values())
    if args.target_deg:
        target = [math.radians(float(v)) for v in args.target_deg.split(",")]
    else:
        target = list(current)
        target[args.joint] += math.radians(args.delta_deg)

    print("current (deg): " + ", ".join("%.2f" % math.degrees(v) for v in current), flush=True)
    print("target  (deg): " + ", ".join("%.2f" % math.degrees(v) for v in target), flush=True)

    group.set_joint_value_target(target)
    result = group.plan()
    ok, plan = (result[0], result[1]) if isinstance(result, tuple) else (bool(result.joint_trajectory.points), result)
    points = plan.joint_trajectory.points
    if not ok or not points:
        print("PLAN FAILED - nothing was executed", flush=True)
        return 1

    original = retime(plan, args.duration)
    update_rate = rospy.get_param("/motion_update_rate", 0.0)
    print("planned %.2f s, retimed to %.2f s, %d points"
          % (original, args.duration, len(points)), flush=True)
    if update_rate > 0.0:
        points_gen = update_rate * args.duration
        print("simulator will emit about %.0f points at %.0f Hz; the controller plays "
              "them back at ~%.2f ms/point, so expect roughly %.1f s"
              % (points_gen, update_rate, args.dt_per_point * 1000.0,
                 points_gen * args.dt_per_point), flush=True)

    if args.plan_only:
        print("--plan-only: not executing", flush=True)
        return 0

    started = time.time()
    exec_result = group.execute(plan, wait=True)
    elapsed = time.time() - started

    after = list(group.get_current_joint_values())
    print("executed in %.1f s (requested %.1f s), result=%s"
          % (elapsed, args.duration, exec_result), flush=True)
    print("final   (deg): " + ", ".join("%.2f" % math.degrees(v) for v in after), flush=True)
    print("error   (deg): " + ", ".join("%+.2f" % math.degrees(a - t) for a, t in zip(after, target)), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
