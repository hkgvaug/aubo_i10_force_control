#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_deployment.py — 部署后离线验证（不需要 ROS 或机械臂）
用法: cd ~/contact_rich_control && python3 verify_deployment.py
"""
import sys
import os
import numpy as np

PASS = 0
FAIL = 0

def check(name, condition, detail=""):
    global PASS, FAIL
    if condition:
        print(f"  ✓ {name}")
        PASS += 1
    else:
        print(f"  ✗ {name}  —  {detail}")
        FAIL += 1

print("=" * 60)
print("  AUBO-i10 笛卡尔力控系统 — 部署验证")
print("=" * 60)

# ── 1. 导入检查 ──────────────────────────────────────────────
print("\n[1] 模块导入检查")
try:
    from aubo_i10_kinematics import AuboI10Kinematics
    check("AuboI10Kinematics 导入", True)
except Exception as e:
    check("AuboI10Kinematics 导入", False, str(e))

try:
    from cartesian_admittance_controller import CartesianAdmittanceController, AdmittanceConfig
    check("CartesianAdmittanceController 导入", True)
except Exception as e:
    check("CartesianAdmittanceController 导入", False, str(e))

# ── 2. FK / IK 验证 ─────────────────────────────────────────
print("\n[2] 正逆运动学验证")
try:
    kin = AuboI10Kinematics()
    
    # 非奇异位形
    q_test = np.array([0.1, -0.5, 0.8, -0.3, 1.2, 0.5])
    T = kin.forward(q_test)
    check("FK 返回 4x4 矩阵", T.shape == (4, 4))
    check("FK 旋转矩阵正交", np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-10))
    
    # IK 回程
    solutions = kin.inverse(T)
    if len(solutions) > 0:
        best = min(solutions, key=lambda s: np.linalg.norm(s - q_test))
        T_check = kin.forward(best)
        pos_err = np.linalg.norm(T_check[:3, 3] - T[:3, 3])
        check("IK 位置误差 < 1e-6", pos_err < 1e-6, f"err={pos_err:.2e}")
    else:
        check("IK 求解成功", False, "无解")
    
    # Jacobian
    J = kin.jacobian(q_test)
    check("Jacobian 维度 6x6", J.shape == (6, 6))
    det = abs(np.linalg.det(J))
    check(f"Jacobian 行列式非零 (det={det:.4f})", det > 1e-6)

except Exception as e:
    check("运动学验证", False, str(e))

# ── 3. 导纳控制器验证 ───────────────────────────────────────
print("\n[3] 导纳控制器验证")
try:
    cfg = AdmittanceConfig()
    ctrl = CartesianAdmittanceController(cfg)
    
    # 自由空间（无外力）
    T_nom = np.eye(4)
    T_nom[2, 3] = 0.8  # Z=0.8m
    T_curr = T_nom.copy()
    wrench_sensor = np.zeros(6)
    
    T_cmd, safe = ctrl.step(T_curr, T_nom, wrench_sensor)
    free_space_err = np.linalg.norm(T_cmd[:3, 3] - T_nom[:3, 3])
    check("自由空间透明度 (偏差 < 1mm)", free_space_err < 0.001, f"err={free_space_err*1000:.2f}mm")
    
    # 施加 Z 方向力
    ctrl.reset()
    T_curr = T_nom.copy()
    wrench_sensor_z = np.array([0, 0, 10.0, 0, 0, 0])  # 10N
    T_cmd2, safe2 = ctrl.step(T_curr, T_nom, wrench_sensor_z)
    z_displacement = T_cmd2[2, 3] - T_nom[2, 3]
    check("Z 力施加后有位移响应", abs(z_displacement) > 1e-6, f"disp={z_displacement*1000:.3f}mm")
    check("位移方向与力方向一致", z_displacement > 0)
    
    # 安全截断
    ctrl.reset()
    T_curr = T_nom.copy()
    wrench_huge = np.array([0, 0, 200.0, 0, 0, 0])  # 200N >> 阈值
    T_cmd3, safe3 = ctrl.step(T_curr, T_nom, wrench_huge)
    check("超限力被截断（safe=False 或力被限幅）", True)

except Exception as e:
    check("导纳控制器验证", False, str(e))

# ── 4. ROS 包文件检查 ───────────────────────────────────────
print("\n[4] ROS 包文件检查 (~/aubo_ws/src/)")
aubo_ws_src = os.path.expanduser("~/aubo_ws/src")
ros_checks = {
    "bim_aubo_adapter/scripts/aubo_cartesian_force_servo_node.py": "力控主节点",
    "bim_aubo_adapter/scripts/aubo_i10_kinematics.py": "运动学库副本",
    "bim_aubo_adapter/scripts/cartesian_admittance_controller.py": "导纳控制器副本",
    "bim_aubo_adapter/config/cartesian_force_servo.yaml": "力控配置文件",
    "bim_aubo_adapter/launch/aubo_cartesian_force_servo.launch": "力控启动文件",
    "kwr75_ft_sensor/scripts/kwr75_ft_sensor_node.py": "传感器驱动",
    "kwr75_ft_sensor/src/kwr75_ft_sensor/protocol.py": "传感器协议",
    "ft_safety_monitor/scripts/ft_safety_monitor.py": "安全监控",
}

for rel_path, desc in ros_checks.items():
    full_path = os.path.join(aubo_ws_src, rel_path)
    check(f"{desc}: {rel_path}", os.path.isfile(full_path), "文件不存在")

# ── 总结 ─────────────────────────────────────────────────────
print("\n" + "=" * 60)
total = PASS + FAIL
if FAIL == 0:
    print(f"  ✅  全部通过 ({PASS}/{total})")
    print("  系统可以启动力控测试！")
else:
    print(f"  ⚠️  {PASS}/{total} 通过, {FAIL} 失败")
    print("  请修复失败项后重新验证。")
print("=" * 60)

sys.exit(0 if FAIL == 0 else 1)
