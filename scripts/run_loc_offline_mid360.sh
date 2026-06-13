#!/usr/bin/env bash
set -euo pipefail

PROGRAM="run_loc_offline"
DEFAULT_CONFIG="/home/gitee/lightning_ws/src/lightning/config/default_livox.yaml"
CONFIG="${CONFIG:-$DEFAULT_CONFIG}"
INPUT_BAG="${INPUT_BAG:-}"
MAP_PATH="${MAP_PATH:-./data/new_map/}"
OUTPUT_DIR="${OUTPUT_DIR:-}"
BAG_ODOM_TOPIC="${BAG_ODOM_TOPIC:-/odom}"
SLEEP_USEC="${SLEEP_USEC:-0}"
LIVOX_LIDAR_TOPIC="${LIVOX_LIDAR_TOPIC:-/livox/lidar}"
IMU_TOPIC="${IMU_TOPIC:-/livox/imu}"
CHASSIS_FEEDBACK_TOPIC="${CHASSIS_FEEDBACK_TOPIC:-/chassis_feedback}"
WORKSPACE="/home/gitee/lightning_ws"

usage() {
    echo "Usage: $0 --input_bag BAG [--config CONFIG] [--map_path MAP_PATH] [--output_dir DIR] [--bag_odom_topic TOPIC] [--sleep_usec USEC]"
    echo "       $0 BAG [--config CONFIG] [--map_path MAP_PATH] [--output_dir DIR] [--bag_odom_topic TOPIC] [--sleep_usec USEC]"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --config)
            CONFIG="$2"
            shift 2
            ;;
        --input_bag|--bag)
            INPUT_BAG="$2"
            shift 2
            ;;
        --map_path)
            MAP_PATH="$2"
            shift 2
            ;;
        --output_dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --bag_odom_topic)
            BAG_ODOM_TOPIC="$2"
            shift 2
            ;;
        --sleep_usec)
            SLEEP_USEC="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            if [[ -z "$INPUT_BAG" ]]; then
                INPUT_BAG="$1"
                shift
            else
                echo "Unknown argument: $1" >&2
                usage >&2
                exit 2
            fi
            ;;
    esac
done

if [[ -z "$INPUT_BAG" ]]; then
    echo "Missing required input bag." >&2
    usage >&2
    exit 2
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WORKSPACE/logs/runtime"
mkdir -p "$LOG_DIR"
LOG_FILE="${RUN_LOC_LOG_FILE:-$LOG_DIR/${PROGRAM}_${STAMP}.log}"
mkdir -p "$(dirname "$LOG_FILE")"

exec > >(tee -a "$LOG_FILE") 2>&1

echo "[$(date '+%F %T')] program=$PROGRAM"
echo "config=$CONFIG"
echo "input_bag=$INPUT_BAG"
echo "map_path=$MAP_PATH"
echo "output_dir=${OUTPUT_DIR:-<none>}"
echo "bag_odom_topic=$BAG_ODOM_TOPIC"
echo "sleep_usec=$SLEEP_USEC"
echo "livox_lidar_topic=$LIVOX_LIDAR_TOPIC"
echo "imu_topic=$IMU_TOPIC"
echo "reserved_chassis_feedback_topic=$CHASSIS_FEEDBACK_TOPIC"
echo "log_file=$LOG_FILE"

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

RUN_ARGS=(
    --config="$CONFIG"
    --input_bag="$INPUT_BAG"
    --map_path="$MAP_PATH"
    --bag_odom_topic="$BAG_ODOM_TOPIC"
    --sleep_usec="$SLEEP_USEC"
)
if [[ -n "$OUTPUT_DIR" ]]; then
    RUN_ARGS+=(--output_dir="$OUTPUT_DIR")
fi

rosrun lightning "$PROGRAM" "${RUN_ARGS[@]}"
