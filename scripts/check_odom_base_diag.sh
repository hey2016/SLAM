#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/gitee/lightning_ws}"
CONFIG="${CONFIG:-$WORKSPACE/src/lightning/config/default_livox.yaml}"
BAG="${BAG:-}"
MAP_PATH="${MAP_PATH:-$WORKSPACE/data/new_map}"
OUTPUT_DIR="${OUTPUT_DIR:-$WORKSPACE/data/odom_base_check_$(date +%Y%m%d_%H%M%S)}"
PROGRAM="${PROGRAM:-run_loc_offline}"
BUILD_CMD="${BUILD_CMD:-catkin build lightning --cmake-args -DCMAKE_BUILD_TYPE=Release}"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

echo "workspace=$WORKSPACE"
echo "config=$CONFIG"
echo "output_dir=$OUTPUT_DIR"

cd "$WORKSPACE"
eval "$BUILD_CMD"

mkdir -p "$OUTPUT_DIR"

if [[ -z "$BAG" ]]; then
  BAG="$(find /home/ZNY_data/test -maxdepth 1 -name '*.bag' 2>/dev/null | sort | head -n 1 || true)"
fi

echo "bag=${BAG:-<none>}"

if [[ -n "$BAG" && -f "$BAG" ]]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE/devel/setup.bash" 2>/dev/null || true
  timeout "${RUN_TIMEOUT_SEC:-90}s" rosrun lightning "$PROGRAM" \
    --config="$CONFIG" \
    --input_bag="$BAG" \
    --map_path="$MAP_PATH" \
    --output_dir="$OUTPUT_DIR/run" \
    --sleep_usec=0 >/tmp/odom_base_diag_check.log 2>&1 || {
      tail -n 120 /tmp/odom_base_diag_check.log >&2 || true
      fail "offline run failed"
    }
else
  echo "No bag found. Static checks only; runtime log checks will fail without prior outputs."
fi

CSV="$(find data/loc_reports "$OUTPUT_DIR" -name 'odom_base_trace_*.csv' -print 2>/dev/null | sort | tail -n 1 || true)"
JSONL="$(find data/loc_health "$OUTPUT_DIR" -name 'odom_base_events_*.jsonl' -print 2>/dev/null | sort | tail -n 1 || true)"
SUMMARY="$(find data/loc_reports "$OUTPUT_DIR" -name 'odom_base_summary_*.md' -print 2>/dev/null | sort | tail -n 1 || true)"

[[ -n "$CSV" && -f "$CSV" ]] || fail "odom_base_trace_*.csv not found"
[[ -n "$JSONL" && -f "$JSONL" ]] || fail "odom_base_events_*.jsonl not found"
[[ -n "$SUMMARY" && -f "$SUMMARY" ]] || fail "odom_base_summary_*.md not found"

HEADER="$(head -n 1 "$CSV")"
for field in source_selected source_switch tf_lookup_ok tf_error_type selected_delta_xy selected_speed_mps motion_trans_error_th_m odom_vs_lio_delta_xy diagnosis_status; do
  grep -Eq "(^|,)$field(,|$)" <<<"$HEADER" || fail "CSV header missing $field"
done

grep -q "vehicle_max_speed_mps = 4.0" "$SUMMARY" || fail "summary missing vehicle_max_speed_mps"
grep -q "CHASSIS_ODOM_JUMP count" "$SUMMARY" || fail "summary missing CHASSIS_ODOM_JUMP count"
grep -q "TF_SOURCE_SWITCH_JUMP count" "$SUMMARY" || fail "summary missing TF_SOURCE_SWITCH_JUMP count"
grep -q "ODOM_NORMAL_MAP_ODOM_JUMP count" "$SUMMARY" || fail "summary missing ODOM_NORMAL_MAP_ODOM_JUMP count"
grep -q "需要运行日志/测试结果进一步确认" "$SUMMARY" || fail "summary missing confirmation note"

echo "PASS: odom base diagnostics generated"
echo "csv=$CSV"
echo "jsonl=$JSONL"
echo "summary=$SUMMARY"
