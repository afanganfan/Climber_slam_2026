#!/usr/bin/env bash
# 关闭严格变量检查，防止加载 ROS 环境时因 AMENT_TRACE_SETUP_FILES 等变量报错
set +u

# 3v3 看门狗启动脚本（适配开机自启动）。

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT="${SCRIPT_DIR}"
LOG_DIR="${PROJECT_ROOT}/logs"
LOG_FILE="${LOG_DIR}/3v3_watchdog.log"
PID_FILE="${LOG_DIR}/3v3_watchdog.pid"

RESTART_DELAY_SEC=5
ACTION="${1:-start}"
CHILD_PIDS=()

# 可按实际 ROS 发行版修改。
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${PROJECT_ROOT}/install/setup.bash"

# 3v3 任务清单（按顺序拉起）。
CMDS=(
    "ros2 launch rm_bringup bringup.launch.py"
    "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
    "ros2 launch linefit_ground_segmentation_ros segmentation.launch.py"
    "ros2 launch fast_lio mapping.launch.py"
    "ros2 launch imu_complementary_filter complementary_filter.launch.py"
    "ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
    "ros2 launch icp_registration icp.launch.py"
    "ros2 launch rm_navigation bringup_launch.py"
    "ros2 run rm_communication talker --ros-args -p data_type:=three"
)

mkdir -p "${LOG_DIR}"
cd "${PROJECT_ROOT}"

is_running() {
    if [[ -f "${PID_FILE}" ]]; then
        read -r pid < "${PID_FILE}"
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            echo "${pid}"
            return 0
        fi
    fi
    return 1
}

stop_watchdog() {
    if pid=$(is_running); then
        kill "${pid}" 2>/dev/null || true
        sleep 1
        kill -9 "${pid}" 2>/dev/null || true
        rm -f "${PID_FILE}"
        echo "[$(date -Is)] watchdog stop" >> "${LOG_FILE}"
        echo "已停止，PID=${pid}"
    else
        echo "未在运行"
    fi
}

status_watchdog() {
    if pid=$(is_running); then
        echo "运行中，PID=${pid}"
        return 0
    fi
    echo "未在运行"
    return 1
}

kill_children() {
    echo "[$(date -Is)] 触发全量清理..." >> "${LOG_FILE}"
    # 物理超度：通过关键字杀掉所有潜在残留，防止“进程堆叠”
    pkill -9 -f "rm_bringup|livox_ros_driver2|linefit_ground_segmentation|fast_lio|imu_complementary_filter|pointcloud_to_laserscan|icp_registration|rm_navigation|talker|robot_state_publisher"
    
    sleep 1
    CHILD_PIDS=()
}

cleanup() {
    kill_children
    rm -f "${PID_FILE}"
    echo "[$(date -Is)] watchdog exit" >> "${LOG_FILE}"
    exit 0
}

# 修正后的 start_all
start_all() {
    CHILD_PIDS=()
    local cmd
    
    echo "[$(date -Is)] 启动所有节点..." >> "${LOG_FILE}"
    
    for cmd in "${CMDS[@]}"; do
        # 这里的子 shell 会继承 set +u，确保 source 成功
        (
            source "${ROS_SETUP}"
            source "${WORKSPACE_SETUP}"
            exec ${cmd} 
        ) >> "${LOG_FILE}" 2>&1 &
        
        CHILD_PIDS+=("$!")
        sleep 0.8
    done
    
    sleep 2
}

check_any_exited() {
    local pid
    if [[ ${#CHILD_PIDS[@]} -eq 0 ]]; then return 1; fi
    
    for pid in "${CHILD_PIDS[@]}"; do
        if ! kill -0 "${pid}" 2>/dev/null; then
            echo "[$(date -Is)] 监测到异常退出 (PID: ${pid})，触发全系统重启..." >> "${LOG_FILE}"
            return 0
        fi
    done
    return 1
}

# 参数解析逻辑
case "${ACTION}" in
    start)
        if is_running >/dev/null; then
            echo "已在运行，PID=$(is_running)"
            exit 0
        fi
        ;;
    stop)
        stop_watchdog
        exit 0
        ;;
    status)
        status_watchdog
        exit $?
        ;;
    restart)
        stop_watchdog
        ;;
    *)
        echo "用法: $0 {start|stop|status|restart}"
        exit 2
        ;;
esac

# 环境预检查
if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
    echo "错误: 未找到 ${WORKSPACE_SETUP}，请先执行 colcon build。"
    exit 1
fi

# 写入当前脚本 PID 并设置清理钩子
echo $$ > "${PID_FILE}"
trap cleanup INT TERM

echo "[$(date -Is)] watchdog start" >> "${LOG_FILE}"

# 主循环逻辑
while true; do
    start_all

    while true; do
        if check_any_exited; then
            echo "[$(date -Is)] child exited, restarting all in ${RESTART_DELAY_SEC}s" >> "${LOG_FILE}"
            kill_children
            sleep "${RESTART_DELAY_SEC}"
            break
        fi
        sleep 1
    done
done






















