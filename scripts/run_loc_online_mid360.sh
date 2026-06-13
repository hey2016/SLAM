#!/usr/bin/env bash
set -euo pipefail

PROGRAM="run_loc_online"
DEFAULT_CONFIG="/home/gitee/lightning_ws/src/lightning/config/default_livox.yaml"
CONFIG="${CONFIG:-$DEFAULT_CONFIG}"
LIVOX_LIDAR_TOPIC="${LIVOX_LIDAR_TOPIC:-/livox/lidar}"
IMU_TOPIC="${IMU_TOPIC:-/livox/imu}"
CHASSIS_FEEDBACK_TOPIC="${CHASSIS_FEEDBACK_TOPIC:-/chassis_feedback}"
WORKSPACE="/home/gitee/lightning_ws"

usage() {
    echo "Usage: $0 [--config CONFIG]"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --config)
            CONFIG="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WORKSPACE/logs/runtime"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/${PROGRAM}_${STAMP}.log"

exec > >(tee -a "$LOG_FILE") 2>&1

echo "[$(date '+%F %T')] program=$PROGRAM"
echo "config=$CONFIG"
echo "livox_lidar_topic=$LIVOX_LIDAR_TOPIC"
echo "imu_topic=$IMU_TOPIC"
echo "reserved_chassis_feedback_topic=$CHASSIS_FEEDBACK_TOPIC"
echo "log_file=$LOG_FILE"

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

rosrun lightning "$PROGRAM" --config="$CONFIG"
