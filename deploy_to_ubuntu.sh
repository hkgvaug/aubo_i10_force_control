#!/usr/bin/env bash
# ============================================================================
#  deploy_to_ubuntu.sh — 一键部署笛卡尔力控系统到 Ubuntu
#
#  用法（在 Ubuntu 上执行）：
#    1. 挂载包含此脚本的 Windows D: 分区（通常自动挂载到 /media/<user>/xxx）
#    2. cd 到此脚本所在目录
#    3. bash deploy_to_ubuntu.sh
#
#  或者把整个 ubuntutowin 文件夹拷贝到 Ubuntu 后执行：
#    bash /path/to/ubuntutowin/deploy_to_ubuntu.sh
# ============================================================================
set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUBO_WS="${HOME}/aubo_ws"

echo -e "${CYAN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║  AUBO-i10 笛卡尔力控系统 — Ubuntu 部署脚本          ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

# ── 检查前置条件 ──────────────────────────────────────────────────────────────
echo -e "${YELLOW}[1/6] 检查前置条件...${NC}"

if [[ ! -d "${AUBO_WS}" ]]; then
    echo -e "${RED}ERROR: ~/aubo_ws 不存在！${NC}"
    echo "请先确认 Aubo 工作空间已正确设置。"
    exit 1
fi

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    echo -e "${RED}ERROR: 未找到 ROS Noetic！${NC}"
    echo "请先安装 ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu"
    exit 1
fi

source /opt/ros/noetic/setup.bash
echo -e "${GREEN}  ✓ ROS Noetic 已找到${NC}"
echo -e "${GREEN}  ✓ ~/aubo_ws 已找到${NC}"

# ── 检查源文件 ────────────────────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[2/6] 检查源文件...${NC}"

ROS1_SRC="${SCRIPT_DIR}/bimros/ros1_ws/src"
CTRL_SRC="${SCRIPT_DIR}/contact_rich_control"

PACKAGES=(bim_aubo_adapter kwr75_ft_sensor ft_safety_monitor)
for pkg in "${PACKAGES[@]}"; do
    if [[ ! -d "${ROS1_SRC}/${pkg}" ]]; then
        echo -e "${RED}ERROR: 缺少 ROS 包: ${ROS1_SRC}/${pkg}${NC}"
        exit 1
    fi
    echo -e "${GREEN}  ✓ ${pkg}${NC}"
done

if [[ ! -f "${CTRL_SRC}/aubo_i10_kinematics.py" ]]; then
    echo -e "${RED}ERROR: 缺少 contact_rich_control/aubo_i10_kinematics.py${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓ contact_rich_control${NC}"

# ── 备份现有包 ────────────────────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[3/6] 备份现有包（如有）...${NC}"

BACKUP_DIR="${HOME}/aubo_ws_backup_$(date +%Y%m%d_%H%M%S)"
NEED_BACKUP=false
for pkg in "${PACKAGES[@]}"; do
    if [[ -d "${AUBO_WS}/src/${pkg}" ]]; then
        NEED_BACKUP=true
        break
    fi
done

if $NEED_BACKUP; then
    mkdir -p "${BACKUP_DIR}"
    for pkg in "${PACKAGES[@]}"; do
        if [[ -d "${AUBO_WS}/src/${pkg}" ]]; then
            cp -r "${AUBO_WS}/src/${pkg}" "${BACKUP_DIR}/"
            echo -e "${GREEN}  ✓ 备份 ${pkg} -> ${BACKUP_DIR}/${NC}"
        fi
    done
else
    echo -e "${GREEN}  ✓ 无需备份（首次部署）${NC}"
fi

# ── 同步 ROS 包 ──────────────────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[4/6] 同步 ROS 包到 ~/aubo_ws/src/ ...${NC}"

for pkg in "${PACKAGES[@]}"; do
    mkdir -p "${AUBO_WS}/src/${pkg}"
    rsync -a --delete \
        --exclude '__pycache__' --exclude '*.pyc' \
        "${ROS1_SRC}/${pkg}/" "${AUBO_WS}/src/${pkg}/"
    
    # 确保 Python 脚本有执行权限
    if [[ -d "${AUBO_WS}/src/${pkg}/scripts" ]]; then
        chmod +x "${AUBO_WS}/src/${pkg}"/scripts/*.py 2>/dev/null || true
    fi
    
    file_count=$(find "${AUBO_WS}/src/${pkg}" -type f | wc -l)
    echo -e "${GREEN}  ✓ ${pkg} (${file_count} files)${NC}"
done

# ── 部署 contact_rich_control ────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[5/6] 部署 contact_rich_control 到 ~/contact_rich_control/ ...${NC}"

CTRL_DST="${HOME}/contact_rich_control"
mkdir -p "${CTRL_DST}"
rsync -a --delete \
    --exclude '__pycache__' --exclude '*.pyc' \
    --exclude 'checkpoints' --exclude 'data' \
    "${CTRL_SRC}/" "${CTRL_DST}/"

file_count=$(find "${CTRL_DST}" -type f | wc -l)
echo -e "${GREEN}  ✓ contact_rich_control (${file_count} files)${NC}"

# ── 编译 ─────────────────────────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[6/6] 编译 ROS 包...${NC}"

cd "${AUBO_WS}"

# 先 source 现有的 devel（以获取 aubo_driver 等依赖）
if [[ -f "${AUBO_WS}/devel/setup.bash" ]]; then
    source "${AUBO_WS}/devel/setup.bash"
fi

catkin_make --pkg "${PACKAGES[@]}" 2>&1 | tail -20

echo ""
echo -e "${CYAN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                   部署完成！                          ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${GREEN}已部署的文件位置：${NC}"
echo "  ROS 包:     ~/aubo_ws/src/{bim_aubo_adapter, kwr75_ft_sensor, ft_safety_monitor}"
echo "  算法库:     ~/contact_rich_control/"
if $NEED_BACKUP; then
    echo -e "  ${YELLOW}旧版备份:   ${BACKUP_DIR}${NC}"
fi
echo ""
echo -e "${GREEN}下一步操作：${NC}"
echo ""
echo "  # 1. 打开新终端，source 环境"
echo "  source /opt/ros/noetic/setup.bash"
echo "  source ~/aubo_ws/devel/setup.bash"
echo ""
echo "  # 2. 检查网络连接 (机械臂 IP: 192.168.100.11)"
echo "  ping -c 3 192.168.100.11"
echo ""
echo "  # 3. 先用 mock 模式测试（不连真机）"
echo "  roslaunch bim_aubo_adapter aubo_cartesian_force_servo.launch mock_hardware:=true"
echo ""
echo "  # 4. 连真机测试（确保示教器在手！）"
echo "  # 4a. 启动传感器"
echo "  roslaunch kwr75_ft_sensor kwr75_ft_sensor.launch"
echo "  # 4b. 启动安全监控"
echo "  roslaunch ft_safety_monitor ft_safety_monitor.launch"
echo "  # 4c. 启动力控节点"
echo "  roslaunch bim_aubo_adapter aubo_cartesian_force_servo.launch mock_hardware:=false"
echo ""
echo "  # 5. 验证话题"
echo "  rostopic echo /ft_sensor/wrench           # 看传感器数据"
echo "  rostopic echo /cartesian_force_servo/status  # 看控制器状态"
echo ""
