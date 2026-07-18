#!/usr/bin/env bash
set -Ee
set -o pipefail

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
unset ROS_LOCALHOST_ONLY
unset RMW_IMPLEMENTATION

source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash

LOG_DIR="/home/sunrise/robot/logs/auto_grasp"
mkdir -p "${LOG_DIR}"
rm -f "${LOG_DIR}"/*.log 2>/dev/null || true

PIDS=()

start_process() {
    local name="$1"
    shift

    local log_file="${LOG_DIR}/${name}.log"

    echo "============================================================"
    echo "[START] ${name}"
    echo "[LOG]   ${log_file}"
    echo "[CMD]   $*"
    echo "============================================================"

    stdbuf -oL -eL "$@" > "${log_file}" 2>&1 &
    local pid=$!
    PIDS+=("${pid}")

    echo "[PID] ${name}: ${pid}"
    sleep 1
}

cleanup() {
    echo
    echo "============================================================"
    echo "[STOP] Stopping auto grasp processes..."
    echo "============================================================"

    for pid in "${PIDS[@]:-}"; do
        if kill -0 "${pid}" >/dev/null 2>&1; then
            kill "${pid}" >/dev/null 2>&1 || true
        fi
    done

    sleep 2

    for pid in "${PIDS[@]:-}"; do
        if kill -0 "${pid}" >/dev/null 2>&1; then
            kill -9 "${pid}" >/dev/null 2>&1 || true
        fi
    done

    echo "[STOP] Done."
}

trap cleanup SIGINT SIGTERM EXIT

echo "============================================================"
echo "AIRBOT Auto Grasp System"
echo "============================================================"
echo "Please make sure airbot_server is already running in another terminal:"
echo "  bash /home/sunrise/robot/start_airbot_can1.sh"
echo
echo "This script uses default ROS2 RMW."
echo "It does NOT force rmw_cyclonedds_cpp."
echo
echo "Important:"
echo "  camera_to_base_transform.py publishes /visual_target_base directly."
echo "  open_loop_grasp.launch.py consumes /visual_target_base as robot_msgs/msg/VisualTarget."
echo "============================================================"
echo

start_process "orbbec_camera" \
    ros2 launch orbbec_camera gemini2.launch.py

echo "[WAIT] Waiting for camera startup..."
sleep 6

start_process "duck_detector" \
    ros2 run detector duck_detector_node

echo "[WAIT] Waiting for detector startup..."
sleep 2

start_process "robot_bringup_open_loop_grasp" \
    ros2 launch robot_bringup open_loop_grasp.launch.py

echo "[WAIT] Waiting for robot bringup startup..."
sleep 4

start_process "camera_to_base_transform" \
    python3 /home/sunrise/robot/hand_to_eye/camera_to_base_transform.py

echo "[WAIT] Waiting for hand-eye transform startup..."
sleep 2

echo
echo "============================================================"
echo "[READY] Auto grasp system is running."
echo "============================================================"
echo "Expected topics:"
echo "  /duck_position"
echo "  /visual_target_base"
echo "  /robot_arm/executor_status"
echo
echo "Logs are saved in:"
echo "  ${LOG_DIR}"
echo
echo "Press Ctrl+C to stop all processes."
echo "============================================================"
echo

tail -n +1 -F "${LOG_DIR}"/*.log &
TAIL_PID=$!
PIDS+=("${TAIL_PID}")

wait
