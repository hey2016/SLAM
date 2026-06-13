#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/gitee/lightning_ws}"
CONFIG="${CONFIG:-$WORKSPACE/src/lightning/config/default_livox.yaml}"
BAG="${BAG:-}"
MAP_PATH="${MAP_PATH:-$WORKSPACE/data/new_map}"
OUTPUT_DIR="${OUTPUT_DIR:-$WORKSPACE/data/input_health_check_$(date +%Y%m%d_%H%M%S)}"
PROGRAM="${PROGRAM:-run_loc_offline}"

CSV_HEADER_FIELDS=(
  run_id wall_time ros_time topic frame_id child_frame_id msg_stamp arrival_time age_sec
  dt_msg_sec dt_arrival_sec hz_inst_msg hz_inst_arrival hz_ema hz_basis min_hz
  max_age_sec max_gap_sec max_delay_sec count_total count_ok count_warn count_error
  non_monotonic_count gap_count timeout_count low_rate_count delay_count status reason
  severity raw_queue_size extra_json
)

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

pass_msg() {
  echo "PASS: $*"
}

mkdir -p "$OUTPUT_DIR"
cd "$WORKSPACE"

if [[ -z "$BAG" ]]; then
  BAG="$(find /home/ZNY_data/test -maxdepth 1 -name '*.bag' 2>/dev/null | sort | head -n 1 || true)"
fi

echo "workspace=$WORKSPACE"
echo "config=$CONFIG"
echo "bag=${BAG:-<none>}"
echo "output_dir=$OUTPUT_DIR"

if [[ -n "$BAG" && -f "$BAG" ]]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE/devel/setup.bash" 2>/dev/null || true
  rosrun lightning "$PROGRAM" \
    --config="$CONFIG" \
    --input_bag="$BAG" \
    --map_path="$MAP_PATH" \
    --output_dir="$OUTPUT_DIR/run" \
    --sleep_usec=0 >/tmp/input_health_check.log 2>&1 || {
      tail -n 80 /tmp/input_health_check.log >&2 || true
      fail "offline run failed"
    }
else
  echo "No bag found. Running static checks only."
fi

CSV="$(find data/loc_reports "$OUTPUT_DIR" -name 'input_health_*.csv' -print 2>/dev/null | sort | tail -n 1 || true)"
JSONL="$(find data/loc_health "$OUTPUT_DIR" -name 'input_health_events_*.jsonl' -print 2>/dev/null | sort | tail -n 1 || true)"
SUMMARY="$(find data/loc_reports "$OUTPUT_DIR" -name 'input_health_summary_*.md' -print 2>/dev/null | sort | tail -n 1 || true)"

[[ -n "$CSV" && -f "$CSV" ]] || fail "input_health_*.csv not found"
[[ -n "$JSONL" && -f "$JSONL" ]] || fail "input_health_events_*.jsonl not found"
[[ -n "$SUMMARY" && -f "$SUMMARY" ]] || fail "input_health_summary_*.md not found"

HEADER="$(head -n 1 "$CSV")"
for field in "${CSV_HEADER_FIELDS[@]}"; do
  grep -Eq "(^|,)$field(,|$)" <<<"$HEADER" || fail "CSV header missing $field"
done

grep -q '/livox/imu' "$CSV" || fail "CSV does not contain /livox/imu"
grep -q '/livox/lidar' "$CSV" || fail "CSV does not contain /livox/lidar"
if ! grep -q '/odom' "$CSV" && ! grep -q '/tf:odom->base_link' "$CSV"; then
  fail "CSV does not contain /odom or /tf:odom->base_link"
fi

if grep -q '/odom' "$CSV" && ! awk -F, '$4=="/odom" && $29!="NOT_RECEIVED" {found=1} END{exit found?0:1}' "$CSV"; then
  grep -q 'ODOM_TIMEOUT\|NOT_RECEIVED\|/odom' "$JSONL" "$SUMMARY" || \
    fail "odom missing but JSONL/summary does not mention timeout or not received"
fi

pass_msg "input health logs generated"
echo "csv=$CSV"
echo "jsonl=$JSONL"
echo "summary=$SUMMARY"
