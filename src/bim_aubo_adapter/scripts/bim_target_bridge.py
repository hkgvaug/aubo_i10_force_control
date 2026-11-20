#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bim_target_bridge.py — BIM2ROS 与 AUBO-i10/MoveIt 之间的适配层

数据流（与 BIM2ROS 完全解耦）：

    BIM2ROS
      └─ /bim/components   std_msgs/String(JSON)
             ↓
      bim_target_bridge   （本节点：BIM 坐标 → AUBO base_link 位姿）
             └─ /bim/target_pose   geometry_msgs/PoseStamped
                    ↓
              moveit_pose_executor
                    ↓
              MoveIt / AUBO-i10

设计原则：
  1. 不修改 BIM2ROS 的 BIM → ROS JSON 格式；
  2. 不修改 AUBO 的 URDF/SRDF/坐标系；
  3. 只把“BIM 坐标 → 机器人目标位姿”这一步封装在本适配层；
  4. 换真实 AUBO 时，只需替换 moveit_pose_executor 之后的执行层。

上游消息格式（两种都支持）：
  批量：{"components": [{...}, {...}]}
  逐步：{"index": 1, "total": N, "component": {...}}
"""

import json
import math

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker


def _as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _get_comp_value(comp, *keys, default=0.0):
    """兼容 X/x、Y/y、Z/z 字段。"""
    for key in keys:
        if key in comp and comp[key] is not None:
            return _as_float(comp[key], default)
    return default


def _clamp(value, low, high):
    return max(low, min(high, value))


def _linear_map(value, src_range, dst_range):
    s0, s1 = float(src_range[0]), float(src_range[1])
    d0, d1 = float(dst_range[0]), float(dst_range[1])
    span = s1 - s0
    if abs(span) < 1e-9:
        return (d0 + d1) / 2.0
    return d0 + (value - s0) / span * (d1 - d0)


class BimTargetBridge(object):
    def __init__(self):
        rospy.init_node("bim_target_bridge", anonymous=False)

        # ── 标准接口参数 ──────────────────────────────────────────────────
        self.components_topic = rospy.get_param(
            "~components_topic", "/bim/components")
        self.target_pose_topic = rospy.get_param(
            "~target_pose_topic", "/bim/target_pose")
        self.target_info_topic = rospy.get_param(
            "~target_info_topic", "/bim/target_info")
        self.marker_topic = rospy.get_param("~marker_topic", "/bim/marker")
        self.state_topic = rospy.get_param("~state_topic", "/bim/adapter_state")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        # ── 坐标映射参数 ──────────────────────────────────────────────────
        self.bim_x_range = rospy.get_param("~bim_x_range", [0.0, 50.0])
        self.bim_y_range = rospy.get_param("~bim_y_range", [0.0, 15.0])
        self.bim_z_range = rospy.get_param("~bim_z_range", [0.0, 6.0])

        self.robot_x_range = rospy.get_param("~robot_x_range", [0.40, 0.85])
        self.robot_y_range = rospy.get_param("~robot_y_range", [-0.35, 0.35])
        self.robot_z_range = rospy.get_param("~robot_z_range", [0.25, 0.80])

        self.offset = rospy.get_param("~target_offset_xyz", [0.0, 0.0, 0.0])
        self.orientation_xyzw = rospy.get_param("~orientation_xyzw", [])
        self.dedup_distance = float(rospy.get_param("~dedup_distance", 0.005))
        queue_size = int(rospy.get_param("~queue_size", 100))

        # ── 发布器 / 订阅器 ───────────────────────────────────────────────
        self.pose_pub = rospy.Publisher(
            self.target_pose_topic, PoseStamped, queue_size=10)
        self.info_pub = rospy.Publisher(
            self.target_info_topic, String, queue_size=10)
        self.marker_pub = rospy.Publisher(
            self.marker_topic, Marker, queue_size=10)
        self.state_pub = rospy.Publisher(
            self.state_topic, String, queue_size=10)

        self.seq = 0
        self.last_target = None
        self.last_marker_stamp = rospy.Time.now()

        self.sub = rospy.Subscriber(
            self.components_topic, String, self._on_components, queue_size=queue_size)

        rospy.loginfo("BIM → AUBO 适配层已启动")
        rospy.loginfo("  订阅: %s", self.components_topic)
        rospy.loginfo("  发布: %s (%s)", self.target_pose_topic, self.base_frame)
        rospy.loginfo(
            "  映射: BIM X%s→%s  Y%s→%s  Z%s→%s",
            self.bim_x_range, self.robot_x_range,
            self.bim_y_range, self.robot_y_range,
            self.bim_z_range, self.robot_z_range,
        )
        if self.orientation_xyzw and len(self.orientation_xyzw) == 4:
            rospy.loginfo("  固定姿态: %s", self.orientation_xyzw)
        else:
            rospy.loginfo("  固定姿态: 未设置（仅位置约束）")

    # ==========================================================================
    # 上游消息解析
    # ==========================================================================
    def _on_components(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn("收到非 JSON 消息，忽略: %s", exc)
            return

        components = []
        if isinstance(payload, dict) and "component" in payload:
            if isinstance(payload["component"], dict):
                components = [payload["component"]]
            rospy.loginfo(
                "收到逐步组件 [%s/%s]",
                payload.get("index", "?"), payload.get("total", "?"),
            )
        elif isinstance(payload, dict) and "components" in payload:
            if isinstance(payload["components"], list):
                components = payload["components"]
            rospy.loginfo("收到批量组件: %d 个", len(components))
        elif isinstance(payload, dict) and any(
                k in payload for k in ("X", "x", "id", "name")):
            components = [payload]
        elif isinstance(payload, list):
            components = payload
        else:
            rospy.logwarn("未知消息格式: %s", type(payload).__name__)
            return

        for comp in components:
            if not isinstance(comp, dict):
                rospy.logwarn("跳过非字典组件: %r", comp)
                continue
            try:
                self._publish_component(comp)
            except Exception as exc:
                rospy.logerr("组件处理失败: %s  comp=%s", exc, comp)

    # ==========================================================================
    # BIM → AUBO 坐标映射
    # ==========================================================================
    def bim_to_aubo(self, x_bim, y_bim, z_bim):
        """
        保持原项目坐标轴语义（X 前、Y 左、Z 上），只做范围缩放：
          BIM X(0~50m) → AUBO X(0.40~0.85m)
          BIM Y(0~15m) → AUBO Y(-0.35~0.35m)
          BIM Z(0~ 6m) → AUBO Z(0.25~0.80m)
        """
        rx = _linear_map(x_bim, self.bim_x_range, self.robot_x_range)
        ry = _linear_map(y_bim, self.bim_y_range, self.robot_y_range)
        rz = _linear_map(z_bim, self.bim_z_range, self.robot_z_range)

        rx = _clamp(rx, min(self.robot_x_range), max(self.robot_x_range))
        ry = _clamp(ry, min(self.robot_y_range), max(self.robot_y_range))
        rz = _clamp(rz, min(self.robot_z_range), max(self.robot_z_range))

        rx += _as_float(self.offset[0])
        ry += _as_float(self.offset[1])
        rz += _as_float(self.offset[2])
        return round(rx, 5), round(ry, 5), round(rz, 5)

    # ==========================================================================
    # 发布目标位姿
    # ==========================================================================
    def _publish_component(self, comp):
        x_bim = _get_comp_value(comp, "X", "x")
        y_bim = _get_comp_value(comp, "Y", "y")
        z_bim = _get_comp_value(comp, "Z", "z")

        rx, ry, rz = self.bim_to_aubo(x_bim, y_bim, z_bim)

        # 去重：连续完全相同的目标不重复发给 MoveIt
        if self.last_target is not None:
            dx = rx - self.last_target[0]
            dy = ry - self.last_target[1]
            dz = rz - self.last_target[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) < self.dedup_distance:
                rospy.loginfo("目标重复，跳过: (%.3f, %.3f, %.3f)", rx, ry, rz)
                self._set_state("SKIPPED")
                return

        self.seq += 1
        now = rospy.Time.now()

        pose = PoseStamped()
        pose.header.seq = self.seq
        pose.header.stamp = now
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = rx
        pose.pose.position.y = ry
        pose.pose.position.z = rz

        if self.orientation_xyzw and len(self.orientation_xyzw) == 4:
            pose.pose.orientation.x = _as_float(self.orientation_xyzw[0])
            pose.pose.orientation.y = _as_float(self.orientation_xyzw[1])
            pose.pose.orientation.z = _as_float(self.orientation_xyzw[2])
            pose.pose.orientation.w = _as_float(self.orientation_xyzw[3], 1.0)
        else:
            pose.pose.orientation.w = 1.0

        info = {
            "seq": self.seq,
            "id": comp.get("id", "?"),
            "name": comp.get("name", comp.get("type", "?")),
            "floor": comp.get("floor", ""),
            "bim": {"X": x_bim, "Y": y_bim, "Z": z_bim},
            "robot": {"x": rx, "y": ry, "z": rz},
            "frame_id": self.base_frame,
        }

        # 先发元数据再发位姿，确保执行层收到 Pose 时能按 seq 查到组件信息
        self.info_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))
        self.pose_pub.publish(pose)
        self._publish_marker(now, rx, ry, rz, info["id"])
        self._set_state("TARGET_PUBLISHED")

        self.last_target = (rx, ry, rz)
        rospy.loginfo(
            "[%d] %s %s | BIM(%.2f, %.2f, %.2f) → AUBO(%.3f, %.3f, %.3f)",
            self.seq, info["id"], info["name"],
            x_bim, y_bim, z_bim, rx, ry, rz,
        )

    def _publish_marker(self, stamp, x, y, z, comp_id):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.base_frame
        marker.ns = "bim_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.06
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.85
        marker.text = str(comp_id)
        self.marker_pub.publish(marker)

    def _set_state(self, state):
        self.state_pub.publish(String(data=state))


def main():
    try:
        BimTargetBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
