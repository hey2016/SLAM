#!/usr/bin/env bash
set -euo pipefail

PROGRAM="run_slam_offline"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$(cd "$PACKAGE_DIR/../.." && pwd)"
DEFAULT_CONFIG="$PACKAGE_DIR/config/default_mid360_slam.yaml"
CONFIG="${CONFIG:-$DEFAULT_CONFIG}"
INPUT_BAG="${INPUT_BAG:-}"
DRY_RUN=0
WITH_UI="${WITH_UI:-0}"
LIVE_PLOT="${LIVE_PLOT:-1}"
LIVE_PLOT_INTERVAL="${LIVE_PLOT_INTERVAL:-5}"
LIVOX_LIDAR_TOPIC="${LIVOX_LIDAR_TOPIC:-/livox/lidar}"
IMU_TOPIC="${IMU_TOPIC:-/livox/imu}"
CHASSIS_FEEDBACK_TOPIC="${CHASSIS_FEEDBACK_TOPIC:-/chassis_feedback}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RESULT_DIR="${RESULT_DIR:-}"
MAP_OUTPUT_DIR="${OUTPUT_DIR:-$WORKSPACE/data/new_map}"
PLOT_SCRIPT="$PACKAGE_DIR/scripts/plot_offline_slam_trajectory.py"
LOOP_ALIGNMENT_PLOT_SCRIPT="$PACKAGE_DIR/scripts/plot_loop_alignment_live.py"
LIVE_PLOT_PID=""
LIVE_ALIGNMENT_PLOT_PID=""
LIVE_TRAJECTORY_PNG=""
LOOP_ALIGNMENT_LIVE_DIR=""

usage() {
    echo "Usage: $0 --input_bag BAG [--config CONFIG] [--output_dir MAP_DIR] [--result_dir RESULT_DIR] [--with_ui] [--no_live_plot] [--dry_run]"
    echo "       $0 BAG [--config CONFIG] [--output_dir MAP_DIR] [--result_dir RESULT_DIR] [--with_ui] [--no_live_plot] [--dry_run]"
}

abspath() {
    python3 - "$1" <<'PY'
import os
import sys
print(os.path.abspath(sys.argv[1]))
PY
}

bag_stem() {
    python3 - "$1" <<'PY'
import os
import re
import sys

path = sys.argv[1]
name = "dry_run" if not path else os.path.basename(path.rstrip("/"))
if name.endswith(".bag"):
    name = name[:-4]
if not name:
    name = "bag"
name = re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("._-")
print(name or "bag")
PY
}

git_value() {
    local args="$1"
    git -C "$PACKAGE_DIR" $args 2>/dev/null || echo "unknown"
}

map_outputs_exist() {
    [[ -f "$MAP_OUTPUT_DIR/global.pcd" || -f "$MAP_OUTPUT_DIR/map.pgm" || -f "$MAP_OUTPUT_DIR/map.yaml" ]]
}

write_effective_config() {
    python3 - "$CONFIG" "$EFFECTIVE_CONFIG" "$EVAL_DIR" "$WHEEL_ODOM_MAPPING_DIR" "$WITH_UI" <<'PY'
import os
import sys
import yaml

input_config, output_config, eval_dir, wheel_odom_mapping_dir, with_ui_value = sys.argv[1:6]
with open(input_config, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}

with_ui = str(with_ui_value).strip().lower() in {"1", "true", "yes", "on"}

system = data.setdefault("system", {})
system["evaluation"] = True
system["evaluation_output_dir"] = eval_dir
system["with_ui"] = with_ui
system["with_2dui"] = False

health = data.setdefault("health_check", {})
health_log = health.setdefault("input_health_log", {})
health_log["output_dir"] = os.path.join(eval_dir, "input_health")
health_log["event_output_dir"] = os.path.join(eval_dir, "input_health_events")

# Kept for compatibility with the reference script. The raw lightning-lm
# checkout currently has no wheel_odom_mapping consumer.
wheel = data.setdefault("wheel_odom_mapping", {})
debug = wheel.setdefault("debug", {})
debug["output_dir"] = wheel_odom_mapping_dir

os.makedirs(os.path.dirname(output_config), exist_ok=True)
with open(output_config, "w", encoding="utf-8") as f:
    yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
PY
}

copy_run_configs() {
    local dst_dir="$1"
    mkdir -p "$dst_dir"
    cp "$CONFIG" "$dst_dir/input_config.yaml"
    cp "$EFFECTIVE_CONFIG" "$dst_dir/effective_config.yaml"
}

write_dry_run_outputs() {
    mkdir -p "$EVAL_DIR"

    local eval_prefix="slam"
    local csv_file="$EVAL_DIR/${eval_prefix}_evaluation.csv"
    local tum_file="$EVAL_DIR/${eval_prefix}_trajectory.tum"
    local map_quality_file="$EVAL_DIR/${eval_prefix}_map_quality_input.yaml"
    local report_file="$EVAL_DIR/${eval_prefix}_evaluation_report.md"

    echo "timestamp,pose_x,pose_y,pose_z,roll,pitch,yaw,lidar_odom_score,keyframe_id,loop_candidate_id,loop_accepted,loop_score,pgo_before_error,pgo_after_error,loop_status,loop_reject_reason,loop_chi2,loop_robust_delta" > "$csv_file"
    : > "$tum_file"
    {
        echo "global_pcd: $MAP_OUTPUT_DIR/global.pcd"
        echo "map_pgm: $MAP_OUTPUT_DIR/map.pgm"
        echo "trajectory_tum: $tum_file"
    } > "$map_quality_file"
    {
        echo "# Lightning Evaluation Report"
        echo
        echo "dry_run: true"
        echo "rtk_ground_truth: unavailable"
        echo "ate: not_computed"
        echo "rpe: not_computed"
        echo "reason: no RTK or external ground-truth trajectory was provided for this run."
    } > "$report_file"

    echo "dry_run=true"
    echo "result_dir=$RESULT_DIR"
    echo "map_output_dir=$MAP_OUTPUT_DIR"
    echo "evaluation_dir=$EVAL_DIR"
    echo "csv_file=$csv_file"
    echo "tum_file=$tum_file"
    echo "map_quality_file=$map_quality_file"
    echo "report_file=$report_file"
}

plot_slam_trajectory() {
    local trajectory_png="$RESULT_DIR/trajectory_loop_events.png"
    local trajectory_accepted_png="$RESULT_DIR/trajectory_loop_events_accepted.png"
    local trajectory_rejected_png="$RESULT_DIR/trajectory_loop_events_rejected.png"
    if python3 "$PLOT_SCRIPT" "$EVAL_DIR" --output "$trajectory_png"; then
        echo "trajectory_plot=$trajectory_png"
    else
        echo "trajectory_plot_failed=$trajectory_png"
    fi
    if python3 "$PLOT_SCRIPT" "$EVAL_DIR" --output "$trajectory_accepted_png" --loop-filter accepted; then
        echo "trajectory_plot_accepted=$trajectory_accepted_png"
    else
        echo "trajectory_plot_accepted_failed=$trajectory_accepted_png"
    fi
    if python3 "$PLOT_SCRIPT" "$EVAL_DIR" --output "$trajectory_rejected_png" --loop-filter rejected; then
        echo "trajectory_plot_rejected=$trajectory_rejected_png"
    else
        echo "trajectory_plot_rejected_failed=$trajectory_rejected_png"
    fi
}

start_live_plotter() {
    if [[ "$LIVE_PLOT" -ne 1 ]]; then
        return
    fi
    LIVE_TRAJECTORY_PNG="$RESULT_DIR/trajectory_loop_events_live.png"
    echo "trajectory_live=$LIVE_TRAJECTORY_PNG"
    (
        while true; do
            python3 "$PLOT_SCRIPT" "$EVAL_DIR" --output "$LIVE_TRAJECTORY_PNG" --live --max-table-rows 260 || true
            sleep "$LIVE_PLOT_INTERVAL" || exit 0
        done
    ) &
    LIVE_PLOT_PID=$!
}

stop_live_plotter() {
    if [[ -n "${LIVE_PLOT_PID:-}" ]] && kill -0 "$LIVE_PLOT_PID" 2>/dev/null; then
        kill "$LIVE_PLOT_PID" 2>/dev/null || true
        wait "$LIVE_PLOT_PID" 2>/dev/null || true
    fi
    LIVE_PLOT_PID=""
}

read_loop_alignment_live_dir() {
    python3 - "$EFFECTIVE_CONFIG" "$EVAL_DIR" <<'PY'
import os
import sys
import yaml

config_path, eval_dir = sys.argv[1:3]
try:
    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
except Exception:
    data = {}
loop = data.get("loop_closing", {}) or {}
def as_bool(value, default=False):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on", "y"}

enabled = as_bool(loop.get("debug_loop_alignment_live_enable", False), False)
save_png = as_bool(loop.get("debug_loop_alignment_live_save_png", True), True)
debug_dir = str(loop.get("debug_log_dir", "") or "")
if not debug_dir:
    debug_dir = os.path.join(eval_dir, "loop_debug")
subdir = str(loop.get("debug_loop_alignment_live_dir", "loop_alignment_live") or "loop_alignment_live")
alignment_dir = subdir if os.path.isabs(subdir) else os.path.join(debug_dir, subdir)
print(("1" if enabled and save_png else "0") + "|" + alignment_dir)
PY
}

start_loop_alignment_live_plotter() {
    if [[ "$LIVE_PLOT" -ne 1 ]]; then
        return
    fi
    local cfg
    cfg="$(read_loop_alignment_live_dir)"
    local enabled="${cfg%%|*}"
    local alignment_dir="${cfg#*|}"
    if [[ "$enabled" != "1" ]]; then
        return
    fi
    LOOP_ALIGNMENT_LIVE_DIR="$alignment_dir"
    mkdir -p "$LOOP_ALIGNMENT_LIVE_DIR"
    echo "loop_alignment_live_dir=$LOOP_ALIGNMENT_LIVE_DIR"
    (
        python3 "$LOOP_ALIGNMENT_PLOT_SCRIPT" --alignment-dir "$LOOP_ALIGNMENT_LIVE_DIR" \
            --interval "$LIVE_PLOT_INTERVAL" --quiet || true
    ) &
    LIVE_ALIGNMENT_PLOT_PID=$!
}

stop_loop_alignment_live_plotter() {
    if [[ -n "${LIVE_ALIGNMENT_PLOT_PID:-}" ]] && kill -0 "$LIVE_ALIGNMENT_PLOT_PID" 2>/dev/null; then
        kill "$LIVE_ALIGNMENT_PLOT_PID" 2>/dev/null || true
        wait "$LIVE_ALIGNMENT_PLOT_PID" 2>/dev/null || true
    fi
    LIVE_ALIGNMENT_PLOT_PID=""
}

plot_loop_alignment_once() {
    if [[ -n "${LOOP_ALIGNMENT_LIVE_DIR:-}" && -d "$LOOP_ALIGNMENT_LIVE_DIR" ]]; then
        python3 "$LOOP_ALIGNMENT_PLOT_SCRIPT" --alignment-dir "$LOOP_ALIGNMENT_LIVE_DIR" --once --quiet || true
    fi
}

stop_background_plotters() {
    stop_live_plotter
    stop_loop_alignment_live_plotter
}

filter_console_output() {
    awk '
        BEGIN {
            start_time = systime();
            progress_active = 0;
            last_progress_len = 0;
        }

        function fmt_duration(seconds,    h, m, s) {
            if (seconds < 0) {
                return "--:--:--";
            }
            seconds = int(seconds);
            h = int(seconds / 3600);
            m = int((seconds % 3600) / 60);
            s = seconds % 60;
            return sprintf("%02d:%02d:%02d", h, m, s);
        }

        function finish_progress_line() {
            if (progress_active) {
                printf "\n";
                progress_active = 0;
                last_progress_len = 0;
            }
        }

        function emit_progress(line,    i, value, parts, processed, total, percent, elapsed, eta, text, pad_len, pad) {
            processed = "";
            total = "";
            percent = "";
            for (i = 1; i <= NF; i++) {
                if ($i ~ /^processed=/) {
                    value = $i;
                    sub(/^processed=/, "", value);
                    split(value, parts, "/");
                    processed = parts[1];
                    total = parts[2];
                } else if ($i ~ /^percent=/) {
                    value = $i;
                    sub(/^percent=/, "", value);
                    sub(/%$/, "", value);
                    percent = value;
                }
            }

            elapsed = systime() - start_time;
            eta = -1;
            if ((processed + 0) > 0 && (total + 0) >= (processed + 0)) {
                eta = elapsed * ((total + 0) - (processed + 0)) / (processed + 0);
            }
            text = sprintf("[slam_progress] processed=%s/%s percent=%s%% elapsed=%s eta=%s",
                           processed, total, percent, fmt_duration(elapsed), fmt_duration(eta));
            pad = "";
            pad_len = last_progress_len - length(text);
            while (pad_len-- > 0) {
                pad = pad " ";
            }
            printf "\r%s%s", text, pad;
            fflush();
            progress_active = 1;
            last_progress_len = length(text);
            if ((processed + 0) >= (total + 0) || (percent + 0) >= 100.0) {
                finish_progress_line();
            }
        }

        function emit_line(line) {
            finish_progress_line();
            print line;
            fflush();
        }

        /^\[slam_progress\]/ {
            emit_progress($0);
            next;
        }

        /^\[trajectory_plot\]/ || /^trajectory_plot=/ {
            next;
        }

        /\[WARN\]/ ||
        /\[WARNING\]/ ||
        /\[ERROR\]/ ||
        /\[FATAL\]/ ||
        /^W[0-9][0-9][0-9][0-9]/ ||
        /^E[0-9][0-9][0-9][0-9]/ ||
        /^F[0-9][0-9][0-9][0-9]/ ||
        /W[0-9][0-9][0-9][0-9]/ ||
        /E[0-9][0-9][0-9][0-9]/ ||
        /F[0-9][0-9][0-9][0-9]/ ||
        /WARN/ ||
        /WARNING/ ||
        /ERROR/ ||
        /FATAL/ ||
        /warning:/ ||
        /error:/ ||
        /Warning:/ ||
        /Error:/ ||
        /terminate called/ ||
        /Exception/ ||
        /exception/ ||
        /Aborted/ ||
        /core dumped/ ||
        /Segmentation/ ||
        /^dry_run=/ ||
        /^result_dir=/ ||
        /^map_output_dir=/ ||
        /^evaluation_dir=/ ||
        /^csv_file=/ ||
        /^tum_file=/ ||
        /^map_quality_file=/ ||
        /^report_file=/ ||
        /^trajectory_tum=/ ||
        /^trajectory_plot_failed=/ ||
        /^trajectory_live=/ ||
        /^loop_alignment_live_dir=/ ||
        /^loop_debug_dir=/ ||
        /^loop_candidates_csv=/ ||
        /^loop_candidate_clusters_csv=/ ||
        /^loop_init_to_ndt_debug_csv=/ ||
        /^loop_matches_csv=/ ||
        /^loop_gate_decisions_csv=/ ||
        /^loop_edges_csv=/ ||
        /^loop_pgo_impact_csv=/ ||
        /^loop_suspects_csv=/ ||
        /^loop_metadata_json=/ {
            emit_line($0);
        }

        END {
            finish_progress_line();
        }
    '
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
        --output_dir)
            MAP_OUTPUT_DIR="$2"
            shift 2
            ;;
        --result_dir)
            RESULT_DIR="$2"
            shift 2
            ;;
        --dry_run)
            DRY_RUN=1
            shift
            ;;
        --with_ui)
            WITH_UI=1
            shift
            ;;
        --no_live_plot)
            LIVE_PLOT=0
            shift
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

if [[ "$DRY_RUN" -eq 0 && -z "$INPUT_BAG" ]]; then
    echo "Missing required input bag." >&2
    usage >&2
    exit 2
fi

BAG_STEM="$(bag_stem "$INPUT_BAG")"
if [[ -z "$RESULT_DIR" ]]; then
    RESULT_DIR="$WORKSPACE/data/slam_outputs/${STAMP}_${BAG_STEM}_slam_offline"
fi

CONFIG="$(abspath "$CONFIG")"
RESULT_DIR="$(abspath "$RESULT_DIR")"
MAP_OUTPUT_DIR="$(abspath "$MAP_OUTPUT_DIR")"
EVAL_DIR="$RESULT_DIR/evaluation"
WHEEL_ODOM_MAPPING_DIR="$RESULT_DIR/wheel_odom_mapping"
CONFIG_DIR="$RESULT_DIR/config"
EFFECTIVE_CONFIG="$CONFIG_DIR/effective_config.yaml"
LOG_DIR="$WORKSPACE/logs/runtime"
LOG_FILE="$RESULT_DIR/run.log"
RUNTIME_LOG_FILE="$LOG_DIR/${PROGRAM}_${STAMP}.log"

mkdir -p "$RESULT_DIR" "$CONFIG_DIR" "$EVAL_DIR" "$LOG_DIR"
cp "$CONFIG" "$CONFIG_DIR/input_config.yaml"
write_effective_config

exec > >(tee -a "$LOG_FILE" "$RUNTIME_LOG_FILE" | filter_console_output) 2>&1

echo "[$(date '+%F %T')] program=$PROGRAM"
echo "package_dir=$PACKAGE_DIR"
echo "workspace=$WORKSPACE"
echo "config=$CONFIG"
echo "effective_config=$EFFECTIVE_CONFIG"
echo "input_bag=${INPUT_BAG:-<dry-run>}"
echo "bag_stem=$BAG_STEM"
echo "result_dir=$RESULT_DIR"
echo "map_output_dir=$MAP_OUTPUT_DIR"
echo "evaluation_dir=$EVAL_DIR"
echo "wheel_odom_mapping_dir=$WHEEL_ODOM_MAPPING_DIR"
echo "with_ui=$WITH_UI"
echo "live_plot=$LIVE_PLOT"
echo "live_plot_interval_sec=$LIVE_PLOT_INTERVAL"
echo "livox_lidar_topic=$LIVOX_LIDAR_TOPIC"
echo "imu_topic=$IMU_TOPIC"
echo "reserved_chassis_feedback_topic=$CHASSIS_FEEDBACK_TOPIC"
echo "log_file=$LOG_FILE"
echo "runtime_log_file=$RUNTIME_LOG_FILE"

if [[ "$DRY_RUN" -eq 1 ]]; then
    write_dry_run_outputs
    plot_slam_trajectory
    exit 0
fi

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

export LIGHTNING_BAG_PATH="$INPUT_BAG"
export LIGHTNING_INPUT_CONFIG_PATH="$CONFIG"
export LIGHTNING_EFFECTIVE_CONFIG_PATH="$EFFECTIVE_CONFIG"
export LIGHTNING_WHEEL_ODOM_MAPPING_DEBUG_DIR="$WHEEL_ODOM_MAPPING_DIR"
export LIGHTNING_GIT_COMMIT="$(git_value 'rev-parse --short HEAD')"
export LIGHTNING_GIT_BRANCH="$(git_value 'rev-parse --abbrev-ref HEAD')"

trap stop_background_plotters EXIT
start_live_plotter
start_loop_alignment_live_plotter

set +e
rosrun lightning "$PROGRAM" --config="$EFFECTIVE_CONFIG" --input_bag="$INPUT_BAG" --output_dir="$MAP_OUTPUT_DIR"
RUN_RC=$?
set -e
stop_background_plotters
plot_loop_alignment_once

if [[ "$RUN_RC" -ne 0 ]]; then
    echo "warning: $PROGRAM exited with code $RUN_RC"
    if grep -q "map saved" "$LOG_FILE" && map_outputs_exist; then
        echo "warning: map appears saved; continuing post-processing after non-zero exit"
    else
        echo "error: map save was not confirmed; continuing best-effort post-processing then exiting with $RUN_RC"
    fi
fi

copy_run_configs "$MAP_OUTPUT_DIR/config"
plot_slam_trajectory

LOOP_DEBUG_DIR="$EVAL_DIR/loop_debug"
if [[ -d "$LOOP_DEBUG_DIR" ]]; then
    echo "loop_debug_dir=$LOOP_DEBUG_DIR"
    [[ -f "$LOOP_DEBUG_DIR/loop_candidates.csv" ]] && echo "loop_candidates_csv=$LOOP_DEBUG_DIR/loop_candidates.csv"
    [[ -f "$LOOP_DEBUG_DIR/loop_candidate_clusters.csv" ]] && echo "loop_candidate_clusters_csv=$LOOP_DEBUG_DIR/loop_candidate_clusters.csv"
    [[ -f "$LOOP_DEBUG_DIR/loop_init_to_ndt_debug.csv" ]] && echo "loop_init_to_ndt_debug_csv=$LOOP_DEBUG_DIR/loop_init_to_ndt_debug.csv"
    [[ -f "$LOOP_DEBUG_DIR/loop_matches.csv" ]] && echo "loop_matches_csv=$LOOP_DEBUG_DIR/loop_matches.csv"
    [[ -f "$LOOP_DEBUG_DIR/loop_gate_decisions.csv" ]] && echo "loop_gate_decisions_csv=$LOOP_DEBUG_DIR/loop_gate_decisions.csv"
    [[ -f "$LOOP_DEBUG_DIR/loop_edges.csv" ]] && echo "loop_edges_csv=$LOOP_DEBUG_DIR/loop_edges.csv"
    [[ -f "$LOOP_DEBUG_DIR/loop_pgo_impact.csv" ]] && echo "loop_pgo_impact_csv=$LOOP_DEBUG_DIR/loop_pgo_impact.csv"
    [[ -f "$LOOP_DEBUG_DIR/loop_suspects.csv" ]] && echo "loop_suspects_csv=$LOOP_DEBUG_DIR/loop_suspects.csv"
    [[ -f "$LOOP_DEBUG_DIR/run_metadata.json" ]] && echo "loop_metadata_json=$LOOP_DEBUG_DIR/run_metadata.json"
    [[ -d "$LOOP_DEBUG_DIR/loop_alignment_live" ]] && echo "loop_alignment_live_dir=$LOOP_DEBUG_DIR/loop_alignment_live"
fi

if [[ "$RUN_RC" -ne 0 ]]; then
    if grep -q "map saved" "$LOG_FILE" && map_outputs_exist; then
        exit 0
    fi
    exit "$RUN_RC"
fi
