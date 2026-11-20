#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mock FollowJointTrajectory action server used by the safety monitor test.

It accepts goals and logs cancellations.  It never commands a robot.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction


class MockTrajectoryActionServer(object):
    def __init__(self):
        rospy.init_node("mock_trajectory_action_server")
        action_ns = rospy.get_param(
            "~action_namespace", "/aubo_i10_controller/follow_joint_trajectory")
        self.event_file = rospy.get_param("~event_file", "")
        self.server = actionlib.SimpleActionServer(
            action_ns, FollowJointTrajectoryAction, self._execute,
            auto_start=False)
        self.server.start()
        rospy.logwarn("Mock FollowJointTrajectory server on %s", action_ns)

    def _record(self, text):
        rospy.logwarn("%s", text)
        if self.event_file:
            try:
                with open(self.event_file, "a", encoding="utf-8") as handle:
                    handle.write(text + "\n")
                    handle.flush()
            except OSError as exc:
                rospy.logwarn("Cannot write mock event file: %s", exc)

    def _execute(self, goal):
        self._record(
            "MOCK_GOAL points=%d joint_names=%s"
            % (len(goal.trajectory.points), ",".join(goal.trajectory.joint_names)))
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self._record("MOCK_CANCEL_RECEIVED")
                try:
                    self.server.set_preempted()
                except Exception as exc:  # noqa: BLE001 - shutdown race
                    rospy.logwarn("set_preempted failed: %s", exc)
                return
            rate.sleep()
        try:
            self.server.set_aborted()
        except Exception as exc:  # noqa: BLE001 - shutdown race
            rospy.logwarn("set_aborted failed: %s", exc)


def main():
    MockTrajectoryActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
