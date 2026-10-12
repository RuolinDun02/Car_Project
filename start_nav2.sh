#!/usr/bin/env bash
set -eo pipefail

# ============================================================
# 颜色定义
# ============================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

log_info()    { echo -e "${CYAN}[$(date '+%H:%M:%S')]${NC} $*"; }
log_ok()      { echo -e "${GREEN}[$(date '+%H:%M:%S')] ✅ $*${NC}"; }
log_warn()    { echo -e "${YELLOW}[$(date '+%H:%M:%S')] ⚠️  $*${NC}"; }
log_err()     { echo -e "${RED}[$(date '+%H:%M:%S')] ❌ $*${NC}"; }
log_section() {
    echo ""
    echo -e "${BOLD}${BLUE}============================================================${NC}"
    echo -e "${BOLD}${BLUE}  $*${NC}"
    echo -e "${BOLD}${BLUE}============================================================${NC}"
}

# ============================================================
# 环境
# ============================================================
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER 2>/dev/null || true
unset PYTHONPATH PYTHONHOME 2>/dev/null || true

set +u
source /opt/ros/jazzy/setup.bash
source /home/jiayi/Car_Project/install/setup.bash
set -u

# ============================================================
# ROS 环境（避免 SSH 会话差异）
# ============================================================
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

log_info "===== ROS 环境 ====="
log_info "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
log_info "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
log_info "  ROS_DISTRO=$ROS_DISTRO"
log_info "  TTY=$(tty 2>/dev/null || echo 'not a tty')"
log_info "  USER=$(whoami)"
log_info "===================="

LOG_DIR="$HOME/Car_Project/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
log_info "日志目录: $LOG_DIR"

# ============================================================
# 状态
# ============================================================
HAS_CRITICAL_ERROR=0
HAS_WARNING=0

NAV2_PID=""
FOXGLOVE_PID=""
TRIGGER_PID=""
ARM_BRIDGE_PID=""       # ★ 新增

# ============================================================
# 工具函数
# ============================================================
cleanup_stale_processes() {
    log_info "清理可能残留的进程..."
    set +e
    pkill -9 -f "nav2.launch.py" 2>/dev/null
    pkill -9 -f "foxglove_bridge" 2>/dev/null
    pkill -9 -f "nav_trigger" 2>/dev/null
    pkill -9 -f "arm_nav_bridge" 2>/dev/null           # ★ 新增
    pkill -9 -f "motor_watchdog" 2>/dev/null
    pkill -9 -f "lifecycle_manager" 2>/dev/null
    pkill -9 -f "amcl" 2>/dev/null
    pkill -9 -f "planner_server" 2>/dev/null
    pkill -9 -f "controller_server" 2>/dev/null
    pkill -9 -f "bt_navigator" 2>/dev/null
    pkill -9 -f "map_server" 2>/dev/null
    pkill -9 -f "behavior_server" 2>/dev/null
    pkill -9 -f "velocity_smoother" 2>/dev/null
    pkill -9 -f "yhs_can_control" 2>/dev/null
    pkill -9 -f "TanwayLidar" 2>/dev/null
    pkill -9 -f "cmd_vel_to_ctrl_cmd_bridge" 2>/dev/null
    set -e
    sleep 3
    set +e
    ros2 daemon stop 2>/dev/null
    sleep 1
    ros2 daemon start 2>/dev/null
    set -e
    sleep 2
    log_ok "清理完成"
}

retry() {
    local max_attempts=$1
    local delay=$2
    shift 2
    local attempt=1
    while [ $attempt -le $max_attempts ]; do
        log_info "[重试 $attempt/$max_attempts] $*"
        if "$@"; then
            return 0
        fi
        if [ $attempt -lt $max_attempts ]; then
            log_warn "失败，${delay}s 后重试..."
            sleep "$delay"
        fi
        attempt=$((attempt + 1))
    done
    log_err "重试 $max_attempts 次后仍失败"
    return 1
}

wait_for_node() {
    local node=$1 timeout=${2:-30} start=$(date +%s)
    while [ $(( $(date +%s) - start )) -lt $timeout ]; do
        if ros2 node list 2>/dev/null | grep -q "^${node}$"; then
            return 0
        fi
        sleep 1
    done
    return 1
}

wait_for_lifecycle() {
    local node=$1 timeout=${2:-60} start=$(date +%s)
    while [ $(( $(date +%s) - start )) -lt $timeout ]; do
        local state
        state=$(ros2 lifecycle get "$node" 2>/dev/null | awk '{print $1}' || true)
        if [ "$state" = "active" ]; then
            return 0
        fi
        sleep 1
    done
    return 1
}

wait_for_topic_data() {
    local topic=$1 timeout=${2:-30} start=$(date +%s)
    while [ $(( $(date +%s) - start )) -lt "$timeout" ]; do
        if timeout 3 ros2 topic echo "$topic" --once \
            --qos-reliability best_effort >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    return 1
}

wait_for_port() {
    local port=$1 timeout=${2:-30} start=$(date +%s)
    while [ $(( $(date +%s) - start )) -lt $timeout ]; do
        if ss -tln 2>/dev/null | grep -q ":${port} "; then
            return 0
        fi
        sleep 1
    done
    return 1
}

check_pid_alive() {
    local pid=$1
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

# ============================================================
# 检查函数
# ============================================================
check_can() {
    if ip link show can0 &>/dev/null; then
        if ip link show can0 | grep -q "UP"; then
            log_ok "CAN 接口 can0: UP"; return 0
        else
            log_warn "CAN 接口 can0: 存在但未 UP"; HAS_WARNING=1; return 1
        fi
    else
        log_warn "CAN 接口 can0: 不存在"; HAS_WARNING=1; return 1
    fi
}

check_topic_publisher() {
    local topic=$1 desc=$2 timeout=${3:-3} count
    count=$(timeout "$timeout" ros2 topic info "$topic" 2>/dev/null | grep "Publisher count" | awk '{print $3}' || true)
    if [ -n "$count" ] && [ "$count" -gt 0 ] 2>/dev/null; then
        log_ok "$desc: $topic (发布者: $count)"; return 0
    else
        log_warn "$desc: $topic (无发布者)"; HAS_WARNING=1; return 1
    fi
}

check_topic_data() {
    local topic=$1 desc=$2 timeout=${3:-3} output
    output=$(timeout "$timeout" ros2 topic echo "$topic" --once \
        --qos-reliability best_effort 2>/dev/null | head -20 || true)
    if [ -n "$output" ]; then
        log_ok "$desc: $topic (有数据)"; return 0
    else
        log_warn "$desc: $topic (无数据)"; HAS_WARNING=1; return 1
    fi
}

check_node() {
    local node=$1 desc=$2 i=0
    while [ $i -lt 3 ]; do
        if ros2 node list 2>/dev/null | grep -q "^${node}$"; then
            log_ok "$desc: $node"; return 0
        fi
        i=$((i + 1))
        [ $i -lt 3 ] && sleep 2
    done
    log_err "$desc: $node (重试 3 次后仍不存在)"
    HAS_CRITICAL_ERROR=1
    return 1
}

check_lifecycle() {
    local node=$1 desc=$2 max_wait=${3:-30} i=0
    while [ $i -lt "$max_wait" ]; do
        local state
        state=$(ros2 lifecycle get "$node" 2>/dev/null | awk '{print $1}' || true)
        if [ "$state" = "active" ]; then
            log_ok "$desc: $node (active)"; return 0
        fi
        sleep 1; i=$((i + 1))
    done
    log_err "$desc: $node (等待 ${max_wait}s 后仍未 active)"
    HAS_CRITICAL_ERROR=1
    return 1
}

check_port() {
    local port=$1 desc=$2
    if ss -tlnp 2>/dev/null | grep -q ":${port} "; then
        log_ok "$desc: 端口 $port 监听中"; return 0
    else
        log_warn "$desc: 端口 $port 未监听"; HAS_WARNING=1; return 1
    fi
}

check_tf_system() {
    local tf_pub_count tf_static_pub_count
    tf_pub_count=$(timeout 3 ros2 topic info /tf 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    if [ -n "$tf_pub_count" ] && [ "$tf_pub_count" -gt 0 ] 2>/dev/null; then
        log_ok "/tf 发布者: $tf_pub_count 个"
    else
        log_err "/tf 无发布者"; HAS_CRITICAL_ERROR=1
    fi
    tf_static_pub_count=$(timeout 3 ros2 topic info /tf_static 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    if [ -n "$tf_static_pub_count" ] && [ "$tf_static_pub_count" -gt 0 ] 2>/dev/null; then
        log_ok "/tf_static 发布者: $tf_static_pub_count 个"
    else
        log_warn "/tf_static 无发布者"; HAS_WARNING=1
    fi
}

# ============================================================
# 阶段 0：清理
# ============================================================
log_section "阶段 0/6：清理残留进程"
cleanup_stale_processes

# ============================================================
# 阶段 1：CAN
# ============================================================
log_section "阶段 1/6：CAN 接口初始化"

setup_can() {
    if ! ip link show can0 &>/dev/null; then
        log_warn "can0 不存在，等待 3 秒..."
        sleep 3
        if ! ip link show can0 &>/dev/null; then
            return 1
        fi
    fi
    set +e
    sudo -n ip link set can0 down 2>/dev/null
    sudo -n ip link set can0 type can bitrate 500000 2>/dev/null
    sudo -n ip link set can0 up 2>/dev/null
    sudo -n ip link set can0 txqueuelen 1000 2>/dev/null
    set -e
    sleep 1
    if ip link show can0 | grep -q "UP"; then
        return 0
    fi
    return 1
}

if retry 3 2 setup_can; then
    log_ok "CAN 接口已就绪"
    check_can || true
else
    log_warn "CAN 初始化失败"
    HAS_WARNING=1
fi

# ============================================================
# 阶段 2：Nav2
# ============================================================
log_section "阶段 2/6：启动 Nav2"

start_nav2() {
    if check_pid_alive "$NAV2_PID"; then
        kill -INT "$NAV2_PID" 2>/dev/null || true
        sleep 3
    fi
    set +e
    pkill -9 -f "nav2.launch.py" 2>/dev/null
    set -e
    sleep 2
    log_info "启动 nav2.launch.py..."
    ros2 launch jy_tk nav2.launch.py \
        use_sim_time:=false use_motor:=true \
        > "$LOG_DIR/nav2.log" 2>&1 &
    NAV2_PID=$!
    log_info "nav2.launch.py PID=$NAV2_PID"
    sleep 8
    if ! check_pid_alive "$NAV2_PID"; then
        log_err "Nav2 启动后立即退出"
        return 1
    fi
    local lifecycle_nodes=("/map_server" "/amcl" "/planner_server" "/controller_server" "/bt_navigator")
    for node in "${lifecycle_nodes[@]}"; do
        log_info "等待 $node 就绪..."
        if ! wait_for_lifecycle "$node" 120; then
            log_err "$node 120 秒内未 active"
            return 1
        fi
        log_ok "$node 已 active"
    done
    log_info "等待 /scan 实际数据..."
    if ! wait_for_topic_data "/scan" 30; then
        log_err "/scan 30 秒内无数据，Nav2 不可用"
        return 1
    fi
    log_ok "/scan 数据流正常"
    return 0
}

if retry 3 5 start_nav2; then
    log_ok "Nav2 已就绪"
else
    log_err "Nav2 启动失败"
    HAS_CRITICAL_ERROR=1
fi

# ============================================================
# 阶段 3：Foxglove
# ============================================================
log_section "阶段 3/6：启动 Foxglove"

start_foxglove() {
    if check_pid_alive "$FOXGLOVE_PID"; then
        kill -INT "$FOXGLOVE_PID" 2>/dev/null || true
        sleep 2
    fi
    set +e
    pkill -9 -f "foxglove_bridge" 2>/dev/null
    set -e
    sleep 1
    log_info "启动 foxglove_bridge..."
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
        > "$LOG_DIR/foxglove.log" 2>&1 &
    FOXGLOVE_PID=$!
    log_info "Foxglove PID=$FOXGLOVE_PID"
    sleep 5
    if ! check_pid_alive "$FOXGLOVE_PID"; then
        log_err "Foxglove 启动后立即退出"
        return 1
    fi
    if ! wait_for_port 8765 30; then
        log_err "Foxglove 端口 8765 未监听"
        return 1
    fi
    return 0
}

if retry 3 3 start_foxglove; then
    log_ok "Foxglove 已就绪"
else
    log_warn "Foxglove 启动失败"
    HAS_WARNING=1
fi

# ============================================================
# 阶段 4：nav_trigger
# ============================================================
log_section "阶段 4/6：启动 nav_trigger"

start_nav_trigger() {
    if check_pid_alive "$TRIGGER_PID"; then
        kill -INT "$TRIGGER_PID" 2>/dev/null || true
        sleep 2
    fi
    set +e
    pkill -9 -f "nav_trigger" 2>/dev/null
    set -e
    sleep 1
    log_info "启动 nav_trigger.launch.py..."
    ros2 launch jy_tk nav_trigger.launch.py \
        > "$LOG_DIR/nav_trigger.log" 2>&1 &
    TRIGGER_PID=$!
    log_info "nav_trigger PID=$TRIGGER_PID"
    sleep 5
    if ! check_pid_alive "$TRIGGER_PID"; then
        log_err "nav_trigger 启动后立即退出"
        return 1
    fi
    if ! wait_for_node "/nav_trigger" 30; then
        log_err "nav_trigger 节点未出现"
        return 1
    fi
    return 0
}

if retry 3 3 start_nav_trigger; then
    log_ok "nav_trigger 已就绪"
else
    log_err "nav_trigger 启动失败"
    HAS_CRITICAL_ERROR=1
fi

# ============================================================
# ★ 阶段 5：arm_nav_bridge
# ============================================================
log_section "阶段 5/6：启动 arm_nav_bridge"

start_arm_bridge() {
    if check_pid_alive "$ARM_BRIDGE_PID"; then
        kill -INT "$ARM_BRIDGE_PID" 2>/dev/null || true
        sleep 2
    fi
    set +e
    pkill -9 -f "arm_nav_bridge" 2>/dev/null
    set -e
    sleep 1

    log_info "启动 arm_nav_bridge.py..."
    ros2 run jy_tk arm_nav_bridge.py \
        > "$LOG_DIR/arm_bridge.log" 2>&1 &
    ARM_BRIDGE_PID=$!
    log_info "arm_nav_bridge PID=$ARM_BRIDGE_PID"

    sleep 3
    if ! check_pid_alive "$ARM_BRIDGE_PID"; then
        log_err "arm_nav_bridge 启动后立即退出"
        return 1
    fi

    if ! wait_for_port 8189 15; then
        log_err "arm_nav_bridge HTTP 端口 8189 未监听"
        return 1
    fi

    if curl -sf http://127.0.0.1:8189/health > /dev/null 2>&1; then
        log_ok "arm_nav_bridge /health 正常"
    else
        log_warn "arm_nav_bridge /health 无响应"
    fi

    return 0
}

if retry 3 3 start_arm_bridge; then
    log_ok "arm_nav_bridge 已就绪"
else
    log_err "arm_nav_bridge 启动失败"
    HAS_WARNING=1
fi

# ============================================================
# 阶段 6：诊断
# ============================================================
log_section "阶段 6/6：系统诊断"
set +e

echo ""; echo -e "${BOLD}【硬件接口】${NC}"; check_can

echo ""; echo -e "${BOLD}【关键节点】${NC}"
check_node "/yhs_can_control_node" "底盘驱动"
check_node "/tanwaylidar_view/TanwayLidar_MainNode" "雷达驱动"
check_node "/ekf_filter_node" "EKF 融合"
check_node "/cmd_vel_to_ctrl_cmd_bridge" "速度桥接"
check_node "/map_server" "地图服务"
check_node "/amcl" "AMCL 定位"
check_node "/planner_server" "规划器"
check_node "/controller_server" "控制器"
check_node "/bt_navigator" "行为树导航"
check_node "/nav_trigger" "触发节点"
check_node "/arm_nav_bridge" "机械臂桥接"        # ★ 新增
check_node "/foxglove_bridge" "Foxglove"

echo ""; echo -e "${BOLD}【Lifecycle 状态】${NC}"
check_lifecycle "/map_server" "地图服务" 10
check_lifecycle "/amcl" "AMCL" 10
check_lifecycle "/planner_server" "规划器" 10
check_lifecycle "/controller_server" "控制器" 10
check_lifecycle "/bt_navigator" "行为树" 10

echo ""; echo -e "${BOLD}【话题发布者】${NC}"
check_topic_publisher "/scan" "2D 激光" 3
check_topic_publisher "/odom" "融合里程计" 3
check_topic_publisher "/chassis_info_fb" "底盘反馈" 3
check_topic_publisher "/map" "地图" 3

echo ""; echo -e "${BOLD}【话题数据流】${NC}"
check_topic_data "/scan" "2D 激光" 5
check_topic_data "/odom" "融合里程计" 5
check_topic_data "/map" "地图" 3

echo ""; echo -e "${BOLD}【TF 变换链】${NC}"; check_tf_system

echo ""; echo -e "${BOLD}【服务端口】${NC}"
check_port "8765" "Foxglove WebSocket"
check_port "8189" "arm_bridge HTTP"              # ★ 新增

echo ""; echo -e "${BOLD}【nav_trigger 状态】${NC}"
TRIGGER_STATE=$(timeout 3 ros2 topic echo /nav_trigger/state --once 2>/dev/null | grep "data:" | awk '{print $2}' | tr -d '"')
if [ -n "$TRIGGER_STATE" ]; then
    log_ok "nav_trigger 状态: $TRIGGER_STATE"
else
    log_warn "nav_trigger 状态: 无法读取"
fi

echo ""; echo -e "${BOLD}【arm_nav_bridge 健康】${NC}"
if curl -sf http://127.0.0.1:8189/health > /dev/null 2>&1; then
    log_ok "arm_nav_bridge /health: OK"
else
    log_warn "arm_nav_bridge /health: 无响应"
fi

# ============================================================
# 总结
# ============================================================
echo ""
echo -e "${BOLD}${BLUE}============================================================${NC}"
echo -e "${BOLD}${BLUE}  🚀 启动总结${NC}"
echo -e "${BOLD}${BLUE}============================================================${NC}"

if [ "$HAS_CRITICAL_ERROR" -eq 1 ]; then
    echo -e "${RED}${BOLD}"
    echo "  ❌ 存在重大错误，系统可能无法正常导航"
    echo -e "${NC}"
elif [ "$HAS_WARNING" -eq 1 ]; then
    echo -e "${YELLOW}${BOLD}"
    echo "  ⚠️  系统已启动，但存在警告项"
    echo -e "${NC}"
else
    echo -e "${GREEN}${BOLD}"
    echo "  ✅ 所有节点正常，系统就绪"
    echo -e "${NC}"
fi

echo ""
echo -e "${BOLD}【触发导航】${NC}"
echo -e "  ${GREEN}ros2 topic pub --once /nav_trigger std_msgs/Bool '{data: true}'${NC}"
echo ""
echo -e "${BOLD}【arm_bridge 健康检查】${NC}"
echo -e "  ${GREEN}curl http://127.0.0.1:8189/health${NC}"
echo ""
echo -e "${BOLD}【模拟导航到达（触发机械臂）】${NC}"
echo -e "  ${GREEN}ros2 topic pub --once /patrol/arrived std_msgs/String '{data: \"test\"}'${NC}"
echo ""
echo -e "${BOLD}【模拟机械臂完成（触发导航继续）】${NC}"
echo -e "  ${GREEN}curl -X POST http://127.0.0.1:8189/api/arm_done -H 'Content-Type: application/json' -d '{\"arm\":\"BOTH\",\"result\":\"ok\"}'${NC}"
echo ""
echo -e "${BOLD}【日志目录】${NC}"
echo -e "  $LOG_DIR"
echo ""
echo -e "${BOLD}${BLUE}============================================================${NC}"

set -e

# ============================================================
# 清理
# ============================================================
cleanup() {
    echo ""
    log_info "收到退出信号，停止所有节点..."

    check_pid_alive "$ARM_BRIDGE_PID" && kill -INT "$ARM_BRIDGE_PID" 2>/dev/null || true   # ★ 新增
    check_pid_alive "$TRIGGER_PID"    && kill -INT "$TRIGGER_PID"    2>/dev/null || true
    check_pid_alive "$FOXGLOVE_PID"   && kill -INT "$FOXGLOVE_PID"   2>/dev/null || true
    check_pid_alive "$NAV2_PID"       && kill -INT "$NAV2_PID"       2>/dev/null || true

    sleep 10

    set +e
    pkill -9 -f "arm_nav_bridge" 2>/dev/null     # ★ 新增
    pkill -9 -f "nav2.launch.py" 2>/dev/null
    pkill -9 -f "foxglove_bridge" 2>/dev/null
    pkill -9 -f "nav_trigger" 2>/dev/null
    pkill -9 -f "lifecycle_manager" 2>/dev/null
    pkill -9 -f "yhs_can_control" 2>/dev/null
    pkill -9 -f "TanwayLidar" 2>/dev/null
    set -e

    log_info "全部停止"
}
trap cleanup SIGINT SIGTERM

set +e
wait
log_info "所有子进程已退出"
