#!/usr/bin/env python3
"""Send a long-running FollowJointTrajectory goal to the mock action server.

Used only by test/run_fake_test.sh so that the monitor's cancel_all_goals()
has an active goal to cancel.  It never commands a robot.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    rospy.init_node("mock_goal_sender")
    action_ns = rospy.get_param(
        "~action_namespace", "/aubo_i10_controller/follow_joint_trajectory")
    joint_names = rospy.get_param(
        "~joint_names",
        ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"])

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = [0.0] * len(joint_names)
    point.velocities = [0.0] * len(joint_names)
    point.accelerations = [0.0] * len(joint_names)
    point.time_from_start = rospy.Duration(60.0)
    goal.trajectory.points = [point]

    client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("Mock action server %s is not available", action_ns)
        return
    rospy.logwarn("Sending long mock goal to %s", action_ns)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(120.0))
    rospy.logwarn("Mock goal finished with state %s", client.get_state())


if __name__ == "__main__":
    main()
