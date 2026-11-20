#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
publish_components_json.py — 测试/联调工具

把 BIM2ROS 生成的 components JSON（例如 components_preview.json）
按原格式发布到 /bim/components，用于验证：

    BIM JSON → /bim/components → bim_target_bridge → /bim/target_pose → MoveIt

它不参与正式链路，只是替代“BIM2ROS 发送端”做单点测试。

用法：
  rosrun bim_aubo_adapter publish_components_json.py \
      --file /media/sf_ubuntutowin/bimros/components_preview.json \
      --interval 2.0
"""

import argparse
import json
import os
import sys
import time

import rospy
from std_msgs.msg import String


def load_components(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    if isinstance(data, dict):
        data = data.get("components", [])
    if not isinstance(data, list):
        raise ValueError("JSON 顶层必须是组件数组，或包含 components 数组")
    return data


def main():
    parser = argparse.ArgumentParser(description="发布 BIM components JSON 到 ROS1")
    parser.add_argument("--file", default="components_preview.json",
                        help="components JSON 文件路径")
    parser.add_argument("--topic", default="/bim/components",
                        help="目标 topic（默认 /bim/components）")
    parser.add_argument("--interval", type=float, default=2.0,
                        help="逐步模式的发送间隔（秒）")
    parser.add_argument("--batch", action="store_true",
                        help="一次性批量发送（默认逐个发送）")
    parser.add_argument("--loop", action="store_true",
                        help="循环发送，直到 Ctrl+C")
    args = parser.parse_args()

    if not os.path.isfile(args.file):
        print("找不到文件：%s" % args.file)
        return 1

    components = load_components(args.file)
    rospy.init_node("bim_components_publisher", anonymous=True)
    pub = rospy.Publisher(args.topic, String, queue_size=10)

    rospy.loginfo("已加载 %d 个组件：%s", len(components), args.file)
    rospy.loginfo("等待 %s 的订阅者...", args.topic)
    deadline = time.time() + 5.0
    while pub.get_num_connections() == 0 and time.time() < deadline \
            and not rospy.is_shutdown():
        time.sleep(0.1)

    if pub.get_num_connections() == 0:
        rospy.logwarn("5 秒内没有订阅者，仍继续发布")

    try:
        while not rospy.is_shutdown():
            if args.batch:
                payload = {"components": components}
                pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
                rospy.loginfo("批量发布 %d 个组件", len(components))
                rospy.sleep(1.0)
            else:
                total = len(components)
                for index, comp in enumerate(components):
                    payload = {
                        "index": index + 1,
                        "total": total,
                        "component": comp,
                    }
                    pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
                    rospy.loginfo(
                        "[%d/%d] %s %s",
                        index + 1, total,
                        comp.get("id", "?"), comp.get("name", ""))
                    rospy.sleep(args.interval)

            if not args.loop:
                break
            rospy.loginfo("循环发送下一轮...")
    except rospy.ROSInterruptException:
        pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
