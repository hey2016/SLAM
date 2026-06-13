#!/usr/bin/env bash
set -uo pipefail

WORKSPACE="/home/gitee/lightning_ws"
SCRIPT_DIR="$WORKSPACE/src/lightning/scripts"
SINGLE_RUN_SCRIPT="$SCRIPT_DIR/run_loc_offline_mid360.sh"
DEFAULT_CONFIG="$WORKSPACE/src/lightning/config/default_livox.yaml"
CONFIG="${CONFIG:-$DEFAULT_CONFIG}"
MAP_PATH="${MAP_PATH:-$WORKSPACE/data/new_map/}"
BAG_DIR="${BAG_DIR:-/home/ZNY_data/test}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$WORKSPACE/data/loc_outputs}"
BAG_ODOM_TOPIC="${BAG_ODOM_TOPIC:-/odom}"
SLEEP_USEC="${SLEEP_USEC:-0}"
PLOT_SCRIPT="$SCRIPT_DIR/plot_offline_loc_trajectories.py"

usage() {
    echo "Usage: $0 [--bag_dir DIR] [--config CONFIG] [--map_path MAP_PATH] [--output_root DIR] [--bag_odom_topic TOPIC] [--sleep_usec USEC]"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --bag_dir)
            BAG_DIR="$2"
            shift 2
            ;;
        --config)
            CONFIG="$2"
            shift 2
            ;;
        --map_path)
            MAP_PATH="$2"
            shift 2
            ;;
        --output_root)
            OUTPUT_ROOT="$2"
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
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ ! -x "$SINGLE_RUN_SCRIPT" ]]; then
    echo "single run script is not executable: $SINGLE_RUN_SCRIPT" >&2
    exit 2
fi

if [[ ! -d "$BAG_DIR" ]]; then
    echo "bag dir does not exist: $BAG_DIR" >&2
    exit 2
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
BATCH_DIR="$OUTPUT_ROOT/batch_${STAMP}"
mkdir -p "$BATCH_DIR"
BATCH_LOG_FILE="$BATCH_DIR/batch.log"
SUMMARY_CSV="$BATCH_DIR/summary.csv"
RUN_CONFIG_FILE="$BATCH_DIR/run_config.env"
CONFIG_SNAPSHOT="$BATCH_DIR/config_snapshot.yaml"
BAG_LIST_FILE="$BATCH_DIR/bag_list.txt"
REPRODUCE_COMMAND="$BATCH_DIR/reproduce_command.sh"

write_env_value() {
    local key="$1"
    local value="$2"
    printf '%s=' "$key" >> "$RUN_CONFIG_FILE"
    printf '%q' "$value" >> "$RUN_CONFIG_FILE"
    printf '\n' >> "$RUN_CONFIG_FILE"
}

write_batch_run_config() {
    local repo_dir="$WORKSPACE/src/lightning"
    local git_commit="<not-a-git-repo>"
    local git_status_short="<not-a-git-repo>"
    local config_exists=0

    if [[ -f "$CONFIG" ]]; then
        config_exists=1
        cp "$CONFIG" "$CONFIG_SNAPSHOT"
    else
        echo "config snapshot skipped, config file does not exist: $CONFIG"
    fi

    : > "$BAG_LIST_FILE"
    if [[ "${#BAGS[@]}" -gt 0 ]]; then
        printf '%s\n' "${BAGS[@]}" > "$BAG_LIST_FILE"
    fi

    if git -C "$repo_dir" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        git_commit="$(git -C "$repo_dir" rev-parse HEAD 2>/dev/null || true)"
        git_status_short="$(git -C "$repo_dir" status --short 2>/dev/null || true)"
    fi

    : > "$RUN_CONFIG_FILE"
    write_env_value "timestamp" "$STAMP"
    write_env_value "workspace" "$WORKSPACE"
    write_env_value "script" "$0"
    write_env_value "single_run_script" "$SINGLE_RUN_SCRIPT"
    write_env_value "config" "$CONFIG"
    write_env_value "config_exists" "$config_exists"
    write_env_value "config_snapshot" "$CONFIG_SNAPSHOT"
    write_env_value "map_path" "$MAP_PATH"
    write_env_value "bag_dir" "$BAG_DIR"
    write_env_value "output_root" "$OUTPUT_ROOT"
    write_env_value "batch_dir" "$BATCH_DIR"
    write_env_value "bag_odom_topic" "$BAG_ODOM_TOPIC"
    write_env_value "sleep_usec" "$SLEEP_USEC"
    write_env_value "plot_script" "$PLOT_SCRIPT"
    write_env_value "bag_count" "${#BAGS[@]}"
    write_env_value "batch_log_file" "$BATCH_LOG_FILE"
    write_env_value "summary_csv" "$SUMMARY_CSV"
    write_env_value "bag_list_file" "$BAG_LIST_FILE"
    write_env_value "reproduce_command" "$REPRODUCE_COMMAND"
    write_env_value "hostname" "$(hostname 2>/dev/null || true)"
    write_env_value "user" "${USER:-}"
    write_env_value "pwd" "$(pwd)"
    write_env_value "git_commit" "$git_commit"
    write_env_value "git_status_short" "$git_status_short"

    {
        printf '#!/usr/bin/env bash\n'
        printf 'set -euo pipefail\n\n'
        printf '%q \\\n' "$0"
        printf '  --bag_dir %q \\\n' "$BAG_DIR"
        printf '  --config %q \\\n' "$CONFIG"
        printf '  --map_path %q \\\n' "$MAP_PATH"
        printf '  --output_root %q \\\n' "$OUTPUT_ROOT"
        printf '  --bag_odom_topic %q \\\n' "$BAG_ODOM_TOPIC"
        printf '  --sleep_usec %q\n' "$SLEEP_USEC"
    } > "$REPRODUCE_COMMAND"
    chmod +x "$REPRODUCE_COMMAND"
}

exec > >(tee -a "$BATCH_LOG_FILE") 2>&1

mapfile -t BAGS < <(find "$BAG_DIR" -maxdepth 1 -type f -name '*.bag' | sort)
write_batch_run_config

echo "[$(date '+%F %T')] batch_loc_offline"
echo "bag_dir=$BAG_DIR"
echo "config=$CONFIG"
echo "map_path=$MAP_PATH"
echo "output_root=$OUTPUT_ROOT"
echo "batch_dir=$BATCH_DIR"
echo "bag_odom_topic=$BAG_ODOM_TOPIC"
echo "sleep_usec=$SLEEP_USEC"
echo "batch_log_file=$BATCH_LOG_FILE"
echo "summary_csv=$SUMMARY_CSV"
echo "run_config_file=$RUN_CONFIG_FILE"
echo "config_snapshot=$CONFIG_SNAPSHOT"
echo "bag_list_file=$BAG_LIST_FILE"
echo "reproduce_command=$REPRODUCE_COMMAND"
echo "bag_count=${#BAGS[@]}"

if [[ "${#BAGS[@]}" -eq 0 ]]; then
    echo "no .bag files found under $BAG_DIR" >&2
    exit 2
fi

echo "index,bag,bag_name,start_time,end_time,duration_sec,exit_code,status,result_dir,log_file,global_loc_tum,lio_local_tum,bag_odom_tum,trajectory_png" > "$SUMMARY_CSV"

ok_count=0
warning_count=0
failed_count=0
idx=0

csv_append() {
    local index="$1"
    local bag="$2"
    local bag_name="$3"
    local start_time="$4"
    local end_time="$5"
    local duration="$6"
    local rc="$7"
    local status="$8"
    local result_dir="$9"
    local log_file="${10}"
    local global_loc_tum="${11}"
    local lio_local_tum="${12}"
    local bag_odom_tum="${13}"
    local trajectory_png="${14}"
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
        "$index" "$bag" "$bag_name" "$start_time" "$end_time" "$duration" "$rc" "$status" \
        "$result_dir" "$log_file" "$global_loc_tum" "$lio_local_tum" "$bag_odom_tum" "$trajectory_png" >> "$SUMMARY_CSV"
}

for bag in "${BAGS[@]}"; do
    idx=$((idx + 1))
    bag_name="$(basename "$bag")"
    bag_stem="${bag_name%.bag}"
    safe_stem="$(printf '%s' "$bag_stem" | sed 's/[^A-Za-z0-9._-]/_/g')"
    result_dir="$BATCH_DIR/$(printf '%02d' "$idx")_${safe_stem}"
    mkdir -p "$result_dir"
    run_log="$result_dir/run.log"
    global_loc_tum="$result_dir/global_loc.tum"
    lio_local_tum="$result_dir/lio_local.tum"
    bag_odom_tum="$result_dir/bag_odom.tum"
    trajectory_png="$result_dir/trajectory_xy.png"

    start_epoch="$(date +%s)"
    start_time="$(date '+%F %T')"

    echo
    echo "================================================================================"
    echo "[$start_time] start [$idx/${#BAGS[@]}] $bag"
    echo "result_dir=$result_dir"
    echo "run_log=$run_log"

    RUN_LOC_LOG_FILE="$run_log" "$SINGLE_RUN_SCRIPT" \
        --input_bag "$bag" \
        --config "$CONFIG" \
        --map_path "$MAP_PATH" \
        --output_dir "$result_dir" \
        --bag_odom_topic "$BAG_ODOM_TOPIC" \
        --sleep_usec "$SLEEP_USEC"
    rc=$?

    plot_status="not_run"
    if [[ -f "$PLOT_SCRIPT" ]]; then
        if python3 "$PLOT_SCRIPT" "$result_dir" --output "$trajectory_png"; then
            plot_status="ok"
        else
            plot_status="failed"
        fi
    else
        plot_status="missing_script"
        echo "plot script missing: $PLOT_SCRIPT"
    fi

    end_epoch="$(date +%s)"
    end_time="$(date '+%F %T')"
    duration=$((end_epoch - start_epoch))

    if [[ "$rc" -eq 0 ]]; then
        status="ok"
        ok_count=$((ok_count + 1))
    elif [[ -f "$run_log" ]] && grep -q "done" "$run_log"; then
        status="ui_exit_warning"
        warning_count=$((warning_count + 1))
    else
        status="failed"
        failed_count=$((failed_count + 1))
    fi

    csv_append "$idx" "$bag" "$bag_name" "$start_time" "$end_time" "$duration" "$rc" "$status" \
        "$result_dir" "$run_log" "$global_loc_tum" "$lio_local_tum" "$bag_odom_tum" "$trajectory_png"

    echo "[$end_time] finish [$idx/${#BAGS[@]}] status=$status exit_code=$rc duration_sec=$duration"
    echo "plot_status=$plot_status"
    echo "summary_csv=$SUMMARY_CSV"
done

echo
echo "================================================================================"
echo "batch finished: ok=$ok_count ui_exit_warning=$warning_count failed=$failed_count total=${#BAGS[@]}"
echo "summary_csv=$SUMMARY_CSV"
echo "batch_log_file=$BATCH_LOG_FILE"

if [[ "$failed_count" -gt 0 ]]; then
    exit 1
fi

exit 0
