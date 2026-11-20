#!/usr/bin/env python3
"""Print the current /ft_safety/state 'state' field (latched topic)."""
import json
import sys

import rospy
from std_msgs.msg import String

RESULT = {"state": None}


def _callback(msg):
    try:
        RESULT["state"] = json.loads(msg.data).get("state")
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write("cannot parse state: %s\n" % exc)
    rospy.signal_shutdown("done")


def main():
    rospy.init_node("ft_safety_get_state", anonymous=True)
    rospy.Subscriber("/ft_safety/state", String, _callback)
    rospy.spin()
    print(RESULT["state"] if RESULT["state"] is not None else "")


if __name__ == "__main__":
    main()
