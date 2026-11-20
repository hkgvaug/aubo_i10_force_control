#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
moveit_pose_executor.py — 标准位姿目标 → MoveIt → AUBO-i10

职责边界：
  输入：/bim/target_pose  geometry_msgs/PoseStamped（frame_id 通常是 base_link）
  输出：调用 MoveIt（aubo_i10_moveit_config 的 move_group）规划并执行
        /bim/execution_result  std_msgs/String(JSON)
        /bim/executor_state    std_msgs/String
        /bim_target_coords     std_msgs/Float32MultiArray（兼容原反馈节点）

这一层不含任何 BIM 解析逻辑。以后换真实 AUBO 时，只需要替换本节点
或把 MoveIt 的 controller 从仿真切到真实驱动，BIM2ROS 不需要改。
"""

import json
import sys
import time
from collections import deque

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, String


class MoveItPoseExecutor(object):
    def __init__(self):
        rospy.init_node("moveit_pose_executor", anonymous=False)

        # ── 参数 ──────────────────────────────────────────────────────────
        self.target_pose_topic = rospy.get_param(
            "~target_pose_topic", "/bim/target_pose")
        self.target_info_topic = rospy.get_param(
            "~target_info_topic", "/bim/target_info")
        self.result_topic = rospy.get_param(
            "~result_topic", "/bim/execution_result")
        self.state_topic = rospy.get_param(
            "~state_topic", "/bim/executor_state")
        self.target_coords_topic = rospy.get_param(
            "~target_coords_topic", "/bim_target_coords")

        self.group_name = rospy.get_param("~group_name", "manipulator_i10")
        self.ee_link = rospy.get_param("~end_effector_link", "wrist3_Link")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.use_orientation = bool(rospy.get_param("~use_orientation", False))
        self.retry_count = int(rospy.get_param("~retry_count", 2))
        queue_size = int(rospy.get_param("~queue_size", 100))

        # Safety interlock.  Fail closed: no state, stale state, WARNING or
        # FAULT all reject the target.  The monitor is expected to publish
        # /ft_safety/state at 20 Hz.
        self.safety_state_topic = rospy.get_param(
            "~safety_state_topic", "/ft_safety/state")
        self.require_safety_state = bool(
            rospy.get_param("~require_safety_state", True))
        self.safety_state_timeout = float(
            rospy.get_param("~safety_state_timeout", 1.0))
        if self.safety_state_timeout <= 0.0:
            raise ValueError("~safety_state_timeout must be positive")

        # ── MoveIt 接口 ───────────────────────────────────────────────────
        self.group = self._wait_for_move_group()
        self.group.set_end_effector_link(self.ee_link)
        self.group.set_pose_reference_frame(self.base_frame)
        self.group.set_planning_time(float(rospy.get_param("~planning_time", 5.0)))
        self.group.set_num_planning_attempts(
            int(rospy.get_param("~num_planning_attempts", 10)))
        self.group.set_max_velocity_scaling_factor(
            float(rospy.get_param("~velocity_scaling", 0.3)))
        self.group.set_max_acceleration_scaling_factor(
            float(rospy.get_param("~acceleration_scaling", 0.3)))
        self.group.allow_replanning(True)

        # ── 状态 ──────────────────────────────────────────────────────────
        self.queue = deque()
        self.info_by_seq = {}
        self.busy = False
        self.safety_state = None
        self.safety_state_mono = 0.0
        self.safety_reason = "no safety state received"

        # ── 发布器 / 订阅器 ───────────────────────────────────────────────
        # latch=True：方便执行后随时 `rostopic echo /bim/execution_result` 查看最后一次结果
        self.result_pub = rospy.Publisher(
            self.result_topic, String, queue_size=10, latch=True)
        self.state_pub = rospy.Publisher(
            self.state_topic, String, queue_size=10)
        self.coords_pub = rospy.Publisher(
            self.target_coords_topic, Float32MultiArray, queue_size=10)

        self.pose_sub = rospy.Subscriber(
            self.target_pose_topic, PoseStamped, self._on_target_pose,
            queue_size=queue_size)
        self.info_sub = rospy.Subscriber(
            self.target_info_topic, String, self._on_target_info,
            queue_size=queue_size)
        self.safety_sub = rospy.Subscriber(
            self.safety_state_topic, String, self._on_safety_state,
            queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.1), self._drain_queue)
        self._set_state("IDLE")

        rospy.loginfo("MoveIt 执行层已启动")
        rospy.loginfo("  group=%s  ee=%s  frame=%s",
                      self.group_name, self.ee_link, self.base_frame)
        rospy.loginfo("  订阅目标: %s", self.target_pose_topic)
        rospy.loginfo("  结果输出: %s", self.result_topic)
        rospy.loginfo(
            "  安全联锁: %s  topic=%s  timeout=%.2f s",
            "ON" if self.require_safety_state else "OFF",
            self.safety_state_topic, self.safety_state_timeout)

    # ==========================================================================
    # 初始化 / 回调
    # ==========================================================================
    def _wait_for_move_group(self):
        """等待 move_group 上线；未启动时每 2 秒重试，方便与 MoveIt 并行启动。"""
        while not rospy.is_shutdown():
            try:
                return moveit_commander.MoveGroupCommander(self.group_name)
            except Exception as exc:
                rospy.logwarn(
                    "等待 MoveIt move_group / group '%s' ... (%s)",
                    self.group_name, exc)
                rospy.sleep(2.0)
        raise rospy.ROSInterruptException()

    def _on_safety_state(self, msg):
        now = time.monotonic()
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0, "收到无法解析的安全状态，按不安全处理: %s", exc)
            self.safety_state = None
            self.safety_reason = "invalid safety state JSON"
            self.safety_state_mono = now
            return
        self.safety_state = payload.get("state")
        self.safety_reason = payload.get("reason", "")
        self.safety_state_mono = now

    def _safety_ok(self):
        if not self.require_safety_state:
            return True, "interlock disabled"
        if self.safety_state is None:
            return False, (
                "no safety state received on %s" % self.safety_state_topic)
        age = time.monotonic() - self.safety_state_mono
        if age > self.safety_state_timeout:
            return False, (
                "safety state is stale: age %.2f s > %.2f s"
                % (age, self.safety_state_timeout))
        if self.safety_state != "NORMAL":
            return False, "safety state is %s (%s)" % (
                self.safety_state, self.safety_reason or "no reason")
        return True, "NORMAL"

    def _on_target_info(self, msg):
        try:
            info = json.loads(msg.data)
        except Exception:
            return
        seq = info.get("seq")
        if isinstance(seq, int):
            self.info_by_seq[seq] = info
            # 只保留最近 200 条，避免长时间运行内存增长
            if len(self.info_by_seq) > 200:
                for key in sorted(self.info_by_seq)[:-100]:
                    self.info_by_seq.pop(key, None)

    def _on_target_pose(self, msg):
        info = self.info_by_seq.get(msg.header.seq, {})
        self.queue.append((msg, info))
        rospy.loginfo(
            "收到目标 seq=%s %s → (%.3f, %.3f, %.3f) frame=%s  队列=%d",
            msg.header.seq, info.get("id", "?"),
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.header.frame_id or self.base_frame, len(self.queue))
        self._set_state("QUEUED")

    def _drain_queue(self, _event):
        if self.busy or not self.queue:
            return
        self.busy = True
        try:
            pose_msg, info = self.queue.popleft()
            self._execute(pose_msg, info)
        finally:
            self.busy = False

    # ==========================================================================
    # 规划 + 执行
    # ==========================================================================
    def _execute(self, pose_msg, info):
        # info 和 pose 是两个 topic，可能到达顺序略有差异；执行前再查一次
        info = self.info_by_seq.get(pose_msg.header.seq, info)
        frame_id = pose_msg.header.frame_id or self.base_frame
        self.group.set_pose_reference_frame(frame_id)

        target_xyz = [
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z,
        ]
        quat = pose_msg.pose.orientation
        has_orientation = any(abs(v) > 1e-9 for v in
                              (quat.x, quat.y, quat.z, quat.w))
        use_orientation = self.use_orientation and has_orientation

        ok = False
        error_text = ""
        safety_ok, safety_reason = self._safety_ok()
        if not safety_ok:
            error_text = "safety interlock: %s" % safety_reason
            rospy.logerr(
                "拒绝执行 %s: %s", info.get("id", "?"), error_text)
            self._set_state("REJECTED")
            # Empty iterable: do not plan or execute.
            attempts = []
        else:
            self._set_state("PLANNING")
            attempts = range(self.retry_count + 1)

        for attempt in attempts:
            try:
                # 每次规划前都重新读取当前关节状态，避免使用缓存/过期状态
                self.group.set_start_state_to_current_state()

                if use_orientation:
                    self.group.set_pose_target(pose_msg, self.ee_link)
                else:
                    # 第一阶段只约束位置，避免 AUBO 工具坐标系尚未标定时
                    # 固定姿态导致大量规划失败。
                    self.group.set_position_target(target_xyz, self.ee_link)

                rospy.loginfo(
                    "规划中... attempt=%d target=(%.3f, %.3f, %.3f) orientation=%s",
                    attempt + 1, target_xyz[0], target_xyz[1], target_xyz[2],
                    "fixed" if use_orientation else "free")

                plan_result = self.group.plan()
                if isinstance(plan_result, tuple):
                    success = plan_result[0]
                    plan = plan_result[1] if len(plan_result) > 1 else None
                    error_code = plan_result[3] if len(plan_result) > 3 else None
                else:
                    success = bool(plan_result)
                    plan = plan_result
                    error_code = None

                if not success or plan is None:
                    error_text = "规划失败 error_code=%s" % (error_code,)
                    rospy.logwarn(error_text)
                    self.group.clear_pose_targets()
                    rospy.sleep(0.5)
                    continue

                # Check again immediately before execution: a fault can
                # arrive while the planner is running.
                safety_ok, safety_reason = self._safety_ok()
                if not safety_ok:
                    error_text = (
                        "safety interlock before execution: %s"
                        % safety_reason)
                    rospy.logerr(
                        "执行前安全检查失败 %s: %s",
                        info.get("id", "?"), error_text)
                    self.group.clear_pose_targets()
                    break

                self._set_state("MOVING")
                exec_ok = self.group.execute(plan, wait=True)
                if exec_ok is False:
                    error_text = "轨迹执行失败"
                    rospy.logwarn(error_text)
                    self.group.clear_pose_targets()
                    # AUBO 仿真中执行失败后关节可能仍在收敛，
                    # 等状态稳定再重试，避免 “start point deviates” 连续失败。
                    rospy.sleep(1.5)
                    continue

                ok = True
                error_text = ""
                break
            except Exception as exc:
                error_text = "规划/执行异常: %s" % exc
                rospy.logerr(error_text)
                try:
                    self.group.clear_pose_targets()
                except Exception:
                    pass
                rospy.sleep(0.5)

        self.group.clear_pose_targets()

        # 兼容原 feedback.py：发布“已执行目标”的机器人坐标
        coords = Float32MultiArray()
        coords.data = target_xyz
        self.coords_pub.publish(coords)

        actual = None
        actual_frame = self.group.get_planning_frame()
        try:
            # get_current_pose 返回 planning frame（通常是 world）下的位姿，
            # 因此在结果里显式标注 actual_frame_id，避免和 base_link 目标混淆。
            cur = self.group.get_current_pose(self.ee_link)
            actual = [cur.pose.position.x, cur.pose.position.y, cur.pose.position.z]
        except Exception:
            pass

        result = {
            "success": bool(ok),
            "seq": pose_msg.header.seq,
            "id": info.get("id", "?"),
            "name": info.get("name", "?"),
            "frame_id": frame_id,
            "target": {"x": target_xyz[0], "y": target_xyz[1], "z": target_xyz[2]},
            "actual": actual,
            "actual_frame_id": actual_frame,
            "orientation_used": bool(use_orientation),
            "safety": {
                "required": bool(self.require_safety_state),
                "state": self.safety_state,
                "reason": safety_reason,
                "age": (time.monotonic() - self.safety_state_mono
                        if self.safety_state_mono else None),
            },
            "error": error_text,
            "stamp": rospy.Time.now().to_sec(),
        }
        self.result_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
        if ok:
            self._set_state("DONE")
        elif not safety_ok:
            self._set_state("REJECTED")
        else:
            self._set_state("FAILED")

        if ok:
            rospy.loginfo(
                "执行完成 %s (%.3f, %.3f, %.3f)",
                info.get("id", "?"), target_xyz[0], target_xyz[1], target_xyz[2])
        else:
            rospy.logerr("执行失败 %s: %s", info.get("id", "?"), error_text)

    def _set_state(self, state):
        self.state_pub.publish(String(data=state))


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    try:
        MoveItPoseExecutor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            moveit_commander.roscpp_shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
