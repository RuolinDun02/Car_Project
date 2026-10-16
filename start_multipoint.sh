#!/usr/bin/env bash
set -eo pipefail

# 防止 systemd 与手工运行脚本时启动两套 ROS 节点。文件描述符随进程
# 退出自动释放，因此即使脚本被 SIGKILL，也不会留下“死锁”。
exec 9>/tmp/jy-tk-nav2.lock
if ! flock -n 9; then
    echo "jy-tk-nav2 已经在运行，拒绝重复启动" >&2
    exit 1
fi

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

# ============================================================
# 日志函数
# ============================================================
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
# 环境清理：退出 conda
# ============================================================
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER 2>/dev/null || true
unset PYTHONPATH PYTHONHOME 2>/dev/null || true

# ============================================================
# source ROS 2
# ============================================================
set +u
source /opt/ros/jazzy/setup.bash
source /home/jiayi/Car_Project/install/setup.bash
set -u

# ============================================================
# ★★★ 显式设置 ROS 环境（避免 SSH 会话差异）★★★
# ------------------------------------------------------------
# 背景：
#   - 交互式 SSH：加载 ~/.bashrc → ROS 环境变量完整
#   - 非交互式 SSH（ssh host "cmd"）：不加载 ~/.bashrc → 环境变量丢失
#   - 结果：ROS_DOMAIN_ID / RMW / ROS_LOCALHOST_ONLY 等不同
#          → 节点间发现失败 → 底盘不使能
#
# 解决：强制在这里 export，不依赖任何 shell 配置
# ============================================================
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# ★ 改回 FastDDS（FastRTPS）—— 不再使用 CycloneDDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ★ CycloneDDS 配置已废弃，注释保留备查：
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export CYCLONEDDS_URI=file://$HOME/.cyclonedds/config.xml

# 打印当前环境（调试用）
log_info "===== ROS 环境 ====="
log_info "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
log_info "  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
log_info "  ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"
log_info "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-未设置(默认fastrtps)}"
log_info "  ROS_DISTRO=$ROS_DISTRO"
log_info "  TTY=$(tty 2>/dev/null || echo 'not a tty')"
log_info "  USER=$(whoami)"
log_info "  PWD=$(pwd)"
log_info "===================="

# ============================================================
# 日志目录
# ============================================================
LOG_DIR="$HOME/Car_Project/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
log_info "日志目录: $LOG_DIR"
log_info "Python: $(which python3)"

# ============================================================
# 全局状态标记
# ============================================================
HAS_CRITICAL_ERROR=0
HAS_WARNING=0

# 进程 PID（清理时用）
NAV2_PID=""
FOXGLOVE_PID=""
PATROL_PID=""
ARM_BRIDGE_PID=""
CLEANUP_STARTED=0

# ============================================================
# ★ 工具函数
# ============================================================

# ---------- 清理残留进程 ----------
cleanup_stale_processes() {
    log_info "清理可能残留的进程..."

    set +e
    pkill -9 -f "nav2.launch.py" 2>/dev/null
    pkill -9 -f "foxglove_bridge" 2>/dev/null
    # 停止可能由旧启动脚本遗留的固定目标节点。
    pkill -9 -f "nav_trigger" 2>/dev/null
    pkill -9 -f "multi_point_patrol" 2>/dev/null
    pkill -9 -f "arm_nav_bridge" 2>/dev/null
    pkill -9 -f "motor_watchdog" 2>/dev/null
    pkill -9 -f "pose_saver" 2>/dev/null
    pkill -9 -f "lifecycle_manager" 2>/dev/null
    pkill -9 -f "amcl" 2>/dev/null
    pkill -9 -f "planner_server" 2>/dev/null
    pkill -9 -f "controller_server" 2>/dev/null
    pkill -9 -f "bt_navigator" 2>/dev/null
    pkill -9 -f "map_server" 2>/dev/null
    pkill -9 -f "behavior_server" 2>/dev/null
    pkill -9 -f "smoother_server" 2>/dev/null
    pkill -9 -f "waypoint_follower" 2>/dev/null
    pkill -9 -f "velocity_smoother" 2>/dev/null
    pkill -9 -f "ekf_node" 2>/dev/null
    pkill -9 -f "robot_state_publisher" 2>/dev/null
    pkill -9 -f "pointcloud_to_laserscan_node" 2>/dev/null
    pkill -9 -f "static_transform_publisher" 2>/dev/null
    pkill -9 -f "yhs_can_control" 2>/dev/null
    pkill -9 -f "TanwayLidar" 2>/dev/null
    pkill -9 -f "cmd_vel_to_ctrl_cmd_bridge" 2>/dev/null
    pkill -9 -f "docking_controller.py" 2>/dev/null
    pkill -9 -f "polygon_keepout_mask_server.py" 2>/dev/null
    set -e

    # 等内核释放资源
    sleep 3

    clean_dds_state

    # 启动阶段需要 daemon；退出阶段不会再次启动它。
    ros2 daemon start >/dev/null 2>&1 || true
    sleep 2

    # ★ 等 daemon 真正就绪（用 timeout 5 保护）
    log_info "等待 ROS 2 daemon 就绪..."
    for i in $(seq 1 10); do
        if timeout 3 ros2 daemon status 2>/dev/null | grep -q "running"; then
            log_ok "ROS 2 daemon 已就绪"
            break
        fi
        sleep 1
    done

    log_ok "清理完成"
}

# ---------- 重试执行 ----------
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

# ---------- 等待节点出现 ----------
wait_for_node() {
    local node=$1
    local timeout=${2:-30}
    local start=$(date +%s)

    while [ $(( $(date +%s) - start )) -lt $timeout ]; do
        if timeout 5 ros2 node list 2>/dev/null | grep -q "^${node}$"; then
            return 0
        fi
        sleep 1
    done
    return 1
}

# ---------- 等待 lifecycle active ----------
wait_for_lifecycle() {
    local node=$1
    local timeout=${2:-60}
    local start=$(date +%s)

    while [ $(( $(date +%s) - start )) -lt $timeout ]; do
        local state
        state=$(timeout 5 ros2 lifecycle get "$node" 2>/dev/null | awk '{print $1}' || true)
        if [ "$state" = "active" ]; then
            return 0
        fi
        sleep 1
    done
    return 1
}

# ---------- 等待端口监听 ----------
wait_for_port() {
    local port=$1
    local timeout=${2:-30}
    local start=$(date +%s)

    while [ $(( $(date +%s) - start )) -lt $timeout ]; do
        if ss -tln 2>/dev/null | grep -q ":${port} "; then
            return 0
        fi
        sleep 1
    done
    return 1
}

# ---------- 检查 PID 是否存活 ----------
check_pid_alive() {
    local pid=$1
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

# ---------- 进程组与 DDS 清理 ----------
# ros2 launch 会再创建许多子进程，只向 launch PID 发信号可能遗留孤儿节点。
# 下面所有启动命令都通过 setsid 成为独立进程组，退出时按整组发送信号。
process_group_alive() {
    local pid=${1:-}
    [ -n "$pid" ] && kill -0 -- "-$pid" 2>/dev/null
}

signal_process_group() {
    local signal=$1
    local pid=${2:-}
    [ -z "$pid" ] && return 0

    if process_group_alive "$pid"; then
        kill -s "$signal" -- "-$pid" 2>/dev/null || true
    elif check_pid_alive "$pid"; then
        kill -s "$signal" "$pid" 2>/dev/null || true
    fi
}

wait_for_process_group() {
    local pid=${1:-}
    local timeout=${2:-5}
    local deadline=$((SECONDS + timeout))
    [ -z "$pid" ] && return 0

    while [ "$SECONDS" -lt "$deadline" ]; do
        if ! process_group_alive "$pid" && ! check_pid_alive "$pid"; then
            return 0
        fi
        sleep 0.2
    done
    ! process_group_alive "$pid" && ! check_pid_alive "$pid"
}

stop_process_group() {
    local pid=${1:-}
    [ -z "$pid" ] && return 0

    signal_process_group INT "$pid"
    wait_for_process_group "$pid" 3 && return 0
    signal_process_group TERM "$pid"
    wait_for_process_group "$pid" 2 && return 0
    signal_process_group KILL "$pid"
    wait_for_process_group "$pid" 1 || true
}

all_process_groups_stopped() {
    local pid
    for pid in "$ARM_BRIDGE_PID" "$PATROL_PID" "$FOXGLOVE_PID" "$NAV2_PID"; do
        if process_group_alive "$pid" || check_pid_alive "$pid"; then
            return 1
        fi
    done
    return 0
}

wait_for_process_groups() {
    local timeout=${1:-10}
    local deadline=$((SECONDS + timeout))
    while [ "$SECONDS" -lt "$deadline" ]; do
        all_process_groups_stopped && return 0
        sleep 0.2
    done
    all_process_groups_stopped
}

signal_all_process_groups() {
    local signal=$1
    local pid
    for pid in "$ARM_BRIDGE_PID" "$PATROL_PID" "$FOXGLOVE_PID" "$NAV2_PID"; do
        signal_process_group "$signal" "$pid"
    done
}

clean_dds_state() {
    log_info "停止 ROS 2 daemon 并清理 Fast DDS 僵尸共享内存..."
    timeout 3 ros2 daemon stop >/dev/null 2>&1 || true
    sleep 1

    # fastdds shm clean 只清理 zombie 文件，比直接 rm /dev/shm/fastrtps_*
    # 安全；直接删除可能破坏本机仍在运行的其他 DDS participant。
    if command -v fastdds >/dev/null 2>&1; then
        timeout 5 fastdds shm clean >/dev/null 2>&1 || true
    fi
    log_ok "ROS daemon/DDS 清理完成"
}

cleanup() {
    local exit_code=$?

    # EXIT、SIGINT、SIGTERM 可能连续触发，确保只执行一次。
    if [ "$CLEANUP_STARTED" -eq 1 ]; then
        return "$exit_code"
    fi
    CLEANUP_STARTED=1
    trap - INT TERM EXIT
    set +e

    echo ""
    log_info "收到退出信号，停止所有节点..."

    # 在关闭控制链之前先发零速度；即使 DDS 已异常，后续底盘桥接自身的
    # cmd_timeout 仍会负责归零。
    timeout 2 ros2 topic pub --once /cmd_vel_smoothed geometry_msgs/msg/Twist \
        "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true

    signal_all_process_groups INT
    if ! wait_for_process_groups 12; then
        log_warn "部分节点未响应 SIGINT，升级为 SIGTERM"
        signal_all_process_groups TERM
        wait_for_process_groups 4 || true
    fi
    if ! all_process_groups_stopped; then
        log_warn "部分节点仍未退出，最后使用 SIGKILL"
        signal_all_process_groups KILL
        wait_for_process_groups 2 || true
    fi

    # 回收后台任务，避免 shell 留下僵尸子进程。
    local pid
    for pid in "$ARM_BRIDGE_PID" "$PATROL_PID" "$FOXGLOVE_PID" "$NAV2_PID"; do
        [ -n "$pid" ] && wait "$pid" 2>/dev/null || true
    done

    clean_dds_state
    log_info "全部停止"
    return "$exit_code"
}

# 必须在启动任何 ROS 节点之前安装 trap，启动过程中 Ctrl+C 也能清理。
trap 'exit 130' INT
trap 'exit 143' TERM
trap cleanup EXIT

# ============================================================
# 检查函数（诊断用）
# ============================================================
check_can() {
    if ip link show can0 &>/dev/null; then
        if ip link show can0 | grep -q "UP"; then
            log_ok "CAN 接口 can0: UP"
            return 0
        else
            log_warn "CAN 接口 can0: 存在但未 UP"
            HAS_WARNING=1
            return 1
        fi
    else
        log_warn "CAN 接口 can0: 不存在（底盘未连接？）"
        HAS_WARNING=1
        return 1
    fi
}

check_topic_publisher() {
    local topic=$1
    local desc=$2
    local timeout=${3:-3}
    local count
    count=$(timeout "$timeout" ros2 topic info "$topic" 2>/dev/null | grep "Publisher count" | awk '{print $3}' || true)
    if [ -n "$count" ] && [ "$count" -gt 0 ] 2>/dev/null; then
        log_ok "$desc: $topic (发布者: $count)"
        return 0
    else
        log_warn "$desc: $topic (无发布者)"
        HAS_WARNING=1
        return 1
    fi
}

check_topic_data() {
    local topic=$1
    local desc=$2
    local timeout=${3:-3}
    local output
    output=$(timeout "$timeout" ros2 topic echo "$topic" --once 2>/dev/null | head -20 || true)
    if [ -n "$output" ]; then
        log_ok "$desc: $topic (有数据)"
        return 0
    else
        log_warn "$desc: $topic (无数据)"
        HAS_WARNING=1
        return 1
    fi
}

check_node() {
    local node=$1
    local desc=$2
    # ★ 最多重试 3 次，每次间隔 2 秒（给慢启动节点时间）
    local i=0
    while [ $i -lt 3 ]; do
        if timeout 5 ros2 node list 2>/dev/null | grep -q "^${node}$"; then
            log_ok "$desc: $node"
            return 0
        fi
        i=$((i + 1))
        [ $i -lt 3 ] && sleep 2
    done
    log_err "$desc: $node (重试 3 次后仍不存在)"
    HAS_CRITICAL_ERROR=1
    return 1
}

check_lifecycle() {
    local node=$1
    local desc=$2
    local max_wait=${3:-30}
    local i=0
    while [ $i -lt "$max_wait" ]; do
        local state
        state=$(timeout 5 ros2 lifecycle get "$node" 2>/dev/null | awk '{print $1}' || true)
        if [ "$state" = "active" ]; then
            log_ok "$desc: $node (active)"
            return 0
        fi
        sleep 1
        i=$((i + 1))
    done
    log_err "$desc: $node (等待 ${max_wait}s 后仍未 active)"
    HAS_CRITICAL_ERROR=1
    return 1
}

check_port() {
    local port=$1
    local desc=$2
    if ss -tlnp 2>/dev/null | grep -q ":${port} "; then
        log_ok "$desc: 端口 $port 监听中"
        return 0
    else
        log_warn "$desc: 端口 $port 未监听"
        HAS_WARNING=1
        return 1
    fi
}

check_tf_system() {
    local tf_pub_count
    tf_pub_count=$(timeout 3 ros2 topic info /tf 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    if [ -n "$tf_pub_count" ] && [ "$tf_pub_count" -gt 0 ] 2>/dev/null; then
        log_ok "/tf 发布者: $tf_pub_count 个"
    else
        log_err "/tf 无发布者（TF 系统异常，导航无法工作）"
        HAS_CRITICAL_ERROR=1
    fi

    local tf_static_pub_count
    tf_static_pub_count=$(timeout 3 ros2 topic info /tf_static 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    if [ -n "$tf_static_pub_count" ] && [ "$tf_static_pub_count" -gt 0 ] 2>/dev/null; then
        log_ok "/tf_static 发布者: $tf_static_pub_count 个"
    else
        log_warn "/tf_static 无发布者（静态 TF 未启动）"
        HAS_WARNING=1
    fi

    echo -e "${CYAN}  /tf 发布者列表:${NC}"
    timeout 3 ros2 topic info /tf --verbose 2>/dev/null \
        | grep "Node name:" \
        | awk '{print "    - "$3}' \
        | sort -u
}

# ============================================================
# 阶段 0：清理残留进程
# ============================================================
log_section "阶段 0/6：清理残留进程"
cleanup_stale_processes

# ============================================================
# 阶段 1：CAN 接口初始化（带重试）
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
    log_warn "CAN 初始化失败（可能没接底盘 或 sudo 免密未配）"
    HAS_WARNING=1
fi

# ============================================================
# 阶段 2：启动 Nav2（带等待 + 重试）
# ============================================================
log_section "阶段 2/6：启动 Nav2（含硬件层）"

start_nav2() {
    if process_group_alive "$NAV2_PID" || check_pid_alive "$NAV2_PID"; then
        log_warn "上次 Nav2 进程还在，先停"
        stop_process_group "$NAV2_PID"
    fi
    sleep 2

    log_info "启动 nav2.launch.py..."
    setsid ros2 launch jy_tk nav2.launch.py \
        use_sim_time:=false use_motor:=true \
        > "$LOG_DIR/nav2.log" 2>&1 &
    NAV2_PID=$!
    log_info "nav2.launch.py PID=$NAV2_PID"

    sleep 8
    if ! check_pid_alive "$NAV2_PID"; then
        log_err "Nav2 进程启动后立即退出，看日志 $LOG_DIR/nav2.log"
        return 1
    fi

    local lifecycle_nodes=(
        "/map_server"
        "/amcl"
        "/planner_server"
        "/controller_server"
        "/bt_navigator"
    )

    for node in "${lifecycle_nodes[@]}"; do
        log_info "等待 $node 就绪..."
        if ! wait_for_lifecycle "$node" 120; then
            log_err "$node 120 秒内未 active"
            return 1
        fi
        log_ok "$node 已 active"
    done

    return 0
}

if retry 3 5 start_nav2; then
    log_ok "Nav2 已就绪"
else
    log_err "Nav2 启动失败（重试 3 次后放弃）"
    HAS_CRITICAL_ERROR=1
fi

# ============================================================
# 阶段 3：启动 Foxglove（带端口检查 + 重试）
# ============================================================
log_section "阶段 3/6：启动 Foxglove Bridge"

start_foxglove() {
    if process_group_alive "$FOXGLOVE_PID" || check_pid_alive "$FOXGLOVE_PID"; then
        stop_process_group "$FOXGLOVE_PID"
    fi
    sleep 1

    log_info "启动 foxglove_bridge..."
    setsid ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
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
    log_warn "Foxglove 启动失败（不影响导航）"
    HAS_WARNING=1
fi

# ============================================================
# 阶段 4：启动 multi_point_patrol（带节点检查 + 重试）
# ============================================================
log_section "阶段 4/6：启动 multi_point_patrol"

start_multi_point_patrol() {
    if process_group_alive "$PATROL_PID" || check_pid_alive "$PATROL_PID"; then
        stop_process_group "$PATROL_PID"
    fi
    sleep 1

    log_info "启动 multi_point_patrol.launch.py..."
    setsid ros2 launch jy_tk multi_point_patrol.launch.py \
        > "$LOG_DIR/multi_point_patrol.log" 2>&1 &
    PATROL_PID=$!
    log_info "multi_point_patrol PID=$PATROL_PID"

    sleep 5
    if ! check_pid_alive "$PATROL_PID"; then
        log_err "multi_point_patrol 启动后立即退出"
        return 1
    fi

    if ! wait_for_node "/multi_point_patrol" 30; then
        log_err "multi_point_patrol 节点未出现"
        return 1
    fi

    return 0
}

if retry 3 3 start_multi_point_patrol; then
    log_ok "multi_point_patrol 已就绪"
else
    log_err "multi_point_patrol 启动失败"
    HAS_CRITICAL_ERROR=1
fi

# ============================================================
# 阶段 5：启动 arm_nav_bridge（机械臂通信节点）
# ============================================================
log_section "阶段 5/6：启动 arm_nav_bridge"

start_arm_bridge() {
    if process_group_alive "$ARM_BRIDGE_PID" || check_pid_alive "$ARM_BRIDGE_PID"; then
        stop_process_group "$ARM_BRIDGE_PID"
    fi
    sleep 1

    log_info "启动 arm_nav_bridge.py..."
    setsid ros2 run jy_tk arm_nav_bridge.py \
        > "$LOG_DIR/arm_bridge.log" 2>&1 &
    ARM_BRIDGE_PID=$!
    log_info "arm_nav_bridge PID=$ARM_BRIDGE_PID"

    sleep 3
    if ! check_pid_alive "$ARM_BRIDGE_PID"; then
        log_err "arm_nav_bridge 启动后立即退出，看日志 $LOG_DIR/arm_bridge.log"
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
# 阶段 6：全面诊断
# ============================================================
log_section "阶段 6/6：系统诊断"

set +e

echo ""
echo -e "${BOLD}【硬件接口】${NC}"
check_can

echo ""
echo -e "${BOLD}【关键节点】${NC}"
check_node "/yhs_can_control_node" "底盘驱动"
check_node "/tanwaylidar_view/TanwayLidar_MainNode" "雷达驱动"
check_node "/ekf_filter_node" "EKF 融合"
check_node "/cmd_vel_to_ctrl_cmd_bridge" "速度桥接"
check_node "/map_server" "地图服务"
check_node "/amcl" "AMCL 定位"
check_node "/planner_server" "规划器"
check_node "/controller_server" "控制器"
check_node "/bt_navigator" "行为树导航"
check_node "/multi_point_patrol" "多点巡检"
check_node "/arm_nav_bridge" "机械臂桥接"
check_node "/foxglove_bridge" "Foxglove"

echo ""
echo -e "${BOLD}【Lifecycle 状态】${NC}"
check_lifecycle "/map_server" "地图服务" 10
check_lifecycle "/amcl" "AMCL" 10
check_lifecycle "/planner_server" "规划器" 10
check_lifecycle "/controller_server" "控制器" 10
check_lifecycle "/bt_navigator" "行为树" 10

echo ""
echo -e "${BOLD}【话题发布者】${NC}"
check_topic_publisher "/scan" "2D 激光" 3
check_topic_publisher "/odom" "融合里程计" 3
check_topic_publisher "/odom_base" "底盘里程计" 3
check_topic_publisher "/tanwaylidar_pointcloud" "3D 点云" 3
check_topic_publisher "/tanwaylidar_imu" "IMU" 3
check_topic_publisher "/chassis_info_fb" "底盘反馈" 3
check_topic_publisher "/map" "地图" 3

echo ""
echo -e "${BOLD}【话题数据流】${NC}"
check_topic_data "/scan" "2D 激光" 5
check_topic_data "/odom" "融合里程计" 5
check_topic_data "/map" "地图" 3

echo ""
echo -e "${BOLD}【TF 变换链】${NC}"
check_tf_system

echo ""
echo -e "${BOLD}【服务端口】${NC}"
check_port "8765" "Foxglove WebSocket"
check_port "8189" "arm_bridge HTTP"

echo ""
echo -e "${BOLD}【multi_point_patrol 状态】${NC}"
PATROL_STATE=$(timeout 3 ros2 topic echo /patrol/state --once 2>/dev/null | grep "data:" | awk '{print $2}' | tr -d '"')
if [ -n "$PATROL_STATE" ]; then
    log_ok "multi_point_patrol 状态: $PATROL_STATE"
else
    log_warn "multi_point_patrol 状态: 无法读取"
fi

echo ""
echo -e "${BOLD}【arm_nav_bridge 健康】${NC}"
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
    echo "     请检查上方红色标记的项"
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
echo -e "${BOLD}【启动多点巡检】${NC}"
echo -e "  ${GREEN}curl -X POST http://127.0.0.1:8189/api/trigger_nav -H 'Content-Type: application/json' -d '{}'${NC}"
echo ""
echo -e "${BOLD}【日志目录】${NC}"
echo -e "  $LOG_DIR"
echo ""
echo -e "${BOLD}${BLUE}============================================================${NC}"

set -e

# ============================================================
# 等待
# ============================================================
set +e
wait
log_info "所有子进程已退出"
