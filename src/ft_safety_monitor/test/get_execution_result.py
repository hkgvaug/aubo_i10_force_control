#!/usr/bin/env python3
"""Print seq / success / error / safety fields from the latched
/bim/execution_result message."""

import json
import sys

import rospy
from std_msgs.msg import String

RESULT = {}
MIN_STAMP = 0.0


def _callback(msg):
    try:
        data = json.loads(msg.data)
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write("cannot parse execution result: %s\n" % exc)
        return
    # rospy overwrites Header.seq with the publisher's sequence number, so
    # a fresh publisher always starts at 1.  Correlate on the result stamp
    # (wall clock) instead.
    if float(data.get("stamp", 0.0)) <= MIN_STAMP:
        return
    RESULT.update(data)
    rospy.signal_shutdown("done")


def main():
    global MIN_STAMP
    rospy.init_node("get_execution_result", anonymous=True)
    MIN_STAMP = float(rospy.get_param("~min_stamp", 0.0))
    rospy.Subscriber("/bim/execution_result", String, _callback)
    rospy.spin()
    print(json.dumps({
        "seq": RESULT.get("seq"),
        "success": RESULT.get("success"),
        "error": RESULT.get("error"),
        "safety": RESULT.get("safety"),
    }))


if __name__ == "__main__":
    main()
