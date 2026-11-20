#!/usr/bin/env python3
"""Publish one BIM target with an explicit header.seq.

rostopic pub may overwrite header.seq, so the interlock test uses this
helper instead.  It never commands a robot by itself; the executor decides
whether to plan/execute.
"""

import sys

import rospy
from geometry_msgs.msg import PoseStamped


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: send_target.py SEQ [x y z]")
    seq = int(sys.argv[1])
    x = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    y = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    z = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5

    rospy.init_node("send_bim_target", anonymous=True)
    pub = rospy.Publisher("/bim/target_pose", PoseStamped, queue_size=1)

    deadline = rospy.Time.now() + rospy.Duration(5.0)
    while (pub.get_num_connections() == 0
           and not rospy.is_shutdown()
           and rospy.Time.now() < deadline):
        rospy.sleep(0.05)

    msg = PoseStamped()
    msg.header.seq = seq
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    pub.publish(msg)
    rospy.sleep(0.5)


if __name__ == "__main__":
    main()
