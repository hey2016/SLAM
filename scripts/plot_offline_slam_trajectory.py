#!/usr/bin/env python3
import argparse
import csv
import math
import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_tum(path: Path):
    points = []
    if not path.exists() or path.stat().st_size == 0:
        return points
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 4:
                continue
            try:
                points.append((float(parts[1]), float(parts[2])))
            except ValueError:
                continue
    return points


def select_trajectory(evaluation_dir: Path):
    optimized = evaluation_dir / "slam_trajectory_optimized.tum"
    raw = evaluation_dir / "slam_trajectory.tum"
    optimized_points = read_tum(optimized)
    if optimized_points:
        return optimized, optimized_points, "optimized"
    return raw, read_tum(raw), "raw"


def parse_float(value, default=math.nan):
    try:
        if value is None or value == "":
            return default
        return float(value)
    except ValueError:
        return default


def parse_int(value, default=None):
    try:
        if value is None or value == "":
            return default
        return int(float(value))
    except ValueError:
        return default


def format_value(value, digits=2):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return "-"
    if not math.isfinite(v):
        return "-"
    return f"{v:.{digits}f}"


def parse_bool(value):
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y"}:
        return True
    if text in {"0", "false", "no", "n"}:
        return False
    return None


def format_pass(value):
    parsed = parse_bool(value)
    if parsed is True:
        return "Y"
    if parsed is False:
        return "N"
    return "-"


def threshold_pass(value, threshold, mode):
    v = parse_float(value)
    th = parse_float(threshold)
    if not math.isfinite(v) or not math.isfinite(th):
        return None
    if mode == "ge":
        return v >= th
    if mode == "le":
        return v <= th
    return None


def format_threshold(value, threshold, mode, digits=2, pass_override=None):
    v_text = format_value(value, digits)
    th_text = format_value(threshold, digits)
    passed = parse_bool(pass_override)
    if passed is None:
        passed = threshold_pass(value, threshold, mode)
    if v_text == "-" and th_text == "-":
        return format_pass(passed)
    return f"{v_text}/{th_text} {format_pass(passed)}"


def format_init_to_ndt_delta(event):
    parts = [
        format_value(event.get("init_to_ndt_xy"), 2),
        format_value(event.get("init_to_ndt_yaw_deg"), 2),
        format_value(event.get("init_to_ndt_z"), 2),
    ]
    if all(part == "-" for part in parts):
        return "-"
    return "/".join(parts)


def first_text(*values):
    for value in values:
        if value is None:
            continue
        text = str(value)
        if text and text.lower() != "nan":
            return text
    return ""


def truncate_text(value, max_len):
    text = first_text(value)
    if len(text) > max_len:
        return text[: max_len - 3] + "..."
    return text


def read_pair_detail_file(path: Path, pair_set, field_names):
    details = {}
    if not path.exists() or path.stat().st_size == 0:
        return details
    max_curr_id = max((pair[0] for pair in pair_set if pair[0] is not None), default=None)
    with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        header = f.readline().strip().split(",")
        field_index = {name: idx for idx, name in enumerate(header)}
        curr_idx = field_index.get("curr_kf_id")
        hist_idx = field_index.get("hist_kf_id")
        if curr_idx is None or hist_idx is None:
            return details
        wanted_indices = [(field, field_index[field]) for field in field_names if field in field_index]
        for line in f:
            parts = line.rstrip("\n").split(",")
            if len(parts) <= max(curr_idx, hist_idx):
                continue
            curr = parse_int(parts[curr_idx])
            if max_curr_id is not None and curr is not None and curr > max_curr_id:
                break
            hist = parse_int(parts[hist_idx])
            pair = (curr, hist)
            if curr is None or hist is None or pair not in pair_set:
                continue
            entry = details.setdefault(pair, {})
            for field, idx in wanted_indices:
                if idx < len(parts) and parts[idx] != "":
                    entry[field] = parts[idx]
    return details


def merge_detail_maps(*maps):
    merged = {}
    for detail_map in maps:
        for pair, values in detail_map.items():
            merged.setdefault(pair, {}).update(values)
    return merged


FINAL_EVENT_FIELDS = [
    "event_id",
    "candidate_rank",
    "curr_kf_candidate_count",
    "final_status",
    "reject_reason_primary",
    "reject_reason_secondary",
    "selected_for_pgo_trial",
    "suppressed_by_same_curr_kf_nms",
    "committed",
    "pose_writeback",
    "edge_committed",
    "ndt_score",
    "ndt_score_threshold",
    "inlier_ratio",
    "inlier_ratio_threshold",
    "source_type",
    "source_scan_count",
    "source_time_span_sec",
    "init_to_ndt_xy",
    "init_to_ndt_yaw_deg",
    "init_to_ndt_z",
    "loop_chi2",
    "rk_loop_th",
    "adjacent_pose_gate_result",
    "adjacent_pose_gate_reject_reason",
    "shape_gate_result",
    "shape_gate_reject_reason",
    "shape_local_max_delta_max_m",
    "shape_local_max_delta_p95_m",
    "shape_local_max_delta_mean_m",
]


def read_loop_final_event_details(loop_debug_dir: Path, pair_set):
    return read_pair_detail_file(loop_debug_dir / "loop_final_events.csv", pair_set, FINAL_EVENT_FIELDS)


def apply_final_event_status(events, final_event_details):
    for event in events:
        pair = (event["keyframe_id"], event["candidate_id"])
        details = final_event_details.get(pair, {})
        if not details:
            continue
        event.update(details)
        if event.get("final_status"):
            event["status"] = event["final_status"]
        committed = parse_bool(event.get("committed"))
        if committed is not None:
            event["accepted"] = committed


def read_loop_debug_details(loop_debug_dir: Path, pair_set):
    if not loop_debug_dir.exists() or not pair_set:
        return {}
    return merge_detail_maps(
        read_pair_detail_file(
            loop_debug_dir / "loop_matches.csv",
            pair_set,
            ["ndt_score", "ndt_score_threshold", "correction_trans_m", "correction_yaw_deg", "match_time_ms"],
        ),
        read_pair_detail_file(
            loop_debug_dir / "loop_init_to_ndt_debug.csv",
            pair_set,
            ["init_to_ndt_xy", "init_to_ndt_yaw_deg", "init_to_ndt_z"],
        ),
        read_pair_detail_file(
            loop_debug_dir / "loop_edges.csv",
            pair_set,
            ["loop_chi2", "rk_loop_th"],
        ),
        read_pair_detail_file(
            loop_debug_dir / "loop_gate_decisions.csv",
            pair_set,
            [
                "gate_id_gap_pass",
                "gate_range_pass",
                "gate_yaw_pass",
                "gate_z_pass",
                "gate_ndt_score_pass",
                "gate_correction_pass",
                "gate_nms_pass",
                "gate_pgo_chi2_pass",
                "final_status",
                "reject_reason_primary",
                "reject_reason_secondary",
                "risk_flags",
                "risk_score",
            ],
        ),
        read_loop_final_event_details(loop_debug_dir, pair_set),
    )


def read_live_loop_debug_details(loop_debug_dir: Path, pair_set):
    if not loop_debug_dir.exists() or not pair_set:
        return {}
    return merge_detail_maps(
        read_pair_detail_file(
            loop_debug_dir / "loop_matches.csv",
            pair_set,
            ["ndt_score", "ndt_score_threshold", "correction_trans_m", "correction_yaw_deg"],
        ),
        read_pair_detail_file(
            loop_debug_dir / "loop_init_to_ndt_debug.csv",
            pair_set,
            ["init_to_ndt_xy", "init_to_ndt_yaw_deg", "init_to_ndt_z"],
        ),
        read_pair_detail_file(
            loop_debug_dir / "loop_edges.csv",
            pair_set,
            ["loop_chi2", "rk_loop_th"],
        ),
        read_loop_final_event_details(loop_debug_dir, pair_set),
    )


def read_events(path: Path):
    keyframe_xy = {}
    events = []
    if not path.exists() or path.stat().st_size == 0:
        return keyframe_xy, events

    with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            keyframe_id = parse_int(row.get("keyframe_id"))
            x = parse_float(row.get("pose_x"))
            y = parse_float(row.get("pose_y"))
            if keyframe_id is not None and math.isfinite(x) and math.isfinite(y):
                keyframe_xy[keyframe_id] = (x, y)

            candidate_id = parse_int(row.get("loop_candidate_id"))
            if candidate_id is None:
                continue

            accepted = parse_int(row.get("loop_accepted"), -1)
            raw_status = row.get("loop_status")
            status = raw_status or ("accepted" if accepted == 1 else "rejected")
            is_accepted = status in ("accepted", "accepted_high_risk") or (not raw_status and accepted == 1)
            events.append(
                {
                    "loop_index": len(events) + 1,
                    "keyframe_id": keyframe_id,
                    "candidate_id": candidate_id,
                    "x": x,
                    "y": y,
                    "accepted": is_accepted,
                    "status": status,
                    "score": parse_float(row.get("loop_score")),
                    "loop_reject_reason": row.get("loop_reject_reason"),
                    "loop_chi2": parse_float(row.get("loop_chi2")),
                    "loop_robust_delta": parse_float(row.get("loop_robust_delta")),
                }
            )
    return keyframe_xy, events


def gate_summary(event):
    labels = [
        ("id", "gate_id_gap_pass"),
        ("rng", "gate_range_pass"),
        ("yaw", "gate_yaw_pass"),
        ("z", "gate_z_pass"),
        ("ndt", "gate_ndt_score_pass"),
        ("corr", "gate_correction_pass"),
        ("nms", "gate_nms_pass"),
        ("pgo", "gate_pgo_chi2_pass"),
    ]
    parts = []
    for label, field in labels:
        mark = format_pass(event.get(field))
        if mark != "-":
            parts.append(f"{label}:{mark}")
    if parts:
        return " ".join(parts)

    live_parts = []
    ndt_pass = threshold_pass(first_text(event.get("score"), event.get("ndt_score")), event.get("ndt_score_threshold"), "ge")
    pgo_pass = threshold_pass(event.get("loop_chi2"), first_text(event.get("rk_loop_th"), event.get("loop_robust_delta")), "le")
    if ndt_pass is not None:
        live_parts.append(f"ndt:{format_pass(ndt_pass)}")
    if pgo_pass is not None:
        live_parts.append(f"pgo:{format_pass(pgo_pass)}")
    return " ".join(live_parts) if live_parts else "-"


def main():
    parser = argparse.ArgumentParser(description="Plot offline SLAM trajectory with loop-closure events.")
    parser.add_argument("evaluation_dir", help="Directory containing slam_trajectory.tum and slam_evaluation.csv")
    parser.add_argument("--output", default="", help="Output PNG path. Default: evaluation_dir/trajectory_loop_events.png")
    parser.add_argument("--live", action="store_true", help="Use lightweight inputs for periodic live refresh.")
    parser.add_argument("--max-table-rows", type=int, default=260, help="Maximum loop rows to draw in the right table.")
    parser.add_argument(
        "--loop-filter",
        choices=("all", "accepted", "rejected"),
        default="all",
        help="Loop events to draw. Default: all.",
    )
    args = parser.parse_args()

    evaluation_dir = Path(args.evaluation_dir)
    output = Path(args.output) if args.output else evaluation_dir / "trajectory_loop_events.png"
    csv_path = evaluation_dir / "slam_evaluation.csv"

    tum_path, trajectory, trajectory_source = select_trajectory(evaluation_dir)
    if not trajectory:
        print(f"[trajectory_plot] waiting_for_trajectory={tum_path}")
        return 1

    csv_keyframe_xy, all_events = read_events(csv_path)
    keyframe_xy = {idx: point for idx, point in enumerate(trajectory)}
    keyframe_xy.update({k: v for k, v in csv_keyframe_xy.items() if k not in keyframe_xy})
    for event in all_events:
        opt_xy = keyframe_xy.get(event["keyframe_id"])
        if opt_xy:
            event["x"], event["y"] = opt_xy

    all_pair_set = {
        (e["keyframe_id"], e["candidate_id"])
        for e in all_events
        if e["keyframe_id"] is not None and e["candidate_id"] is not None
    }
    apply_final_event_status(
        all_events,
        read_loop_final_event_details(evaluation_dir / "loop_debug", all_pair_set),
    )

    if args.loop_filter == "accepted":
        events = [e for e in all_events if e["accepted"]]
    elif args.loop_filter == "rejected":
        events = [e for e in all_events if not e["accepted"]]
    else:
        events = all_events

    table_events = events[-args.max_table_rows :] if args.max_table_rows > 0 else events
    pair_set = {
        (e["keyframe_id"], e["candidate_id"])
        for e in table_events
        if e["keyframe_id"] is not None and e["candidate_id"] is not None
    }
    if args.live:
        loop_debug_details = read_live_loop_debug_details(evaluation_dir / "loop_debug", pair_set)
    else:
        loop_debug_details = read_loop_debug_details(evaluation_dir / "loop_debug", pair_set)
    for event in table_events:
        pair = (event["keyframe_id"], event["candidate_id"])
        event.update(loop_debug_details.get(pair, {}))
    apply_final_event_status(table_events, {pair: loop_debug_details.get(pair, {}) for pair in pair_set})
    accepted_events = [e for e in events if e["accepted"]]
    rejected_events = [e for e in events if not e["accepted"]]

    fig_height = max(9.0, 2.0 + 0.18 * max(1, len(table_events)))
    fig, (ax, ax_table) = plt.subplots(
        1,
        2,
        figsize=(24, fig_height),
        gridspec_kw={"width_ratios": [2.4, 3.2]},
    )
    xs = [p[0] for p in trajectory]
    ys = [p[1] for p in trajectory]
    ax.plot(xs, ys, label=f"slam keyframes ({len(trajectory)}, {trajectory_source})", linewidth=1.2, color="#1f77b4")
    ax.scatter(xs[0], ys[0], marker="o", s=30, color="#1f77b4", label="start")
    ax.scatter(xs[-1], ys[-1], marker="x", s=36, color="#111111", label="end")

    label_offsets = [(4, 5), (5, -5), (-5, 5), (-5, -5), (0, 8), (0, -8)]

    def annotate_events(event_group, color):
        for event in event_group:
            if not math.isfinite(event["x"]) or not math.isfinite(event["y"]):
                continue
            offset = label_offsets[(event["loop_index"] - 1) % len(label_offsets)]
            ax.annotate(
                str(event["loop_index"]),
                xy=(event["x"], event["y"]),
                xytext=offset,
                textcoords="offset points",
                fontsize=5.2,
                color="#111111",
                ha="center",
                va="center",
                bbox={"boxstyle": "round,pad=0.12", "fc": "white", "ec": color, "alpha": 0.74, "lw": 0.45},
                zorder=6,
            )

    def draw_events(event_group, color, marker, label, linestyle):
        if not event_group:
            return
        ex = [e["x"] for e in event_group if math.isfinite(e["x"]) and math.isfinite(e["y"])]
        ey = [e["y"] for e in event_group if math.isfinite(e["x"]) and math.isfinite(e["y"])]
        if ex:
            ax.scatter(ex, ey, marker=marker, s=48, color=color, edgecolors="#111111", linewidths=0.4, label=label)

        for event in event_group:
            src = (event["x"], event["y"])
            dst = keyframe_xy.get(event["candidate_id"])
            if not dst or not math.isfinite(src[0]) or not math.isfinite(src[1]):
                continue
            ax.plot([src[0], dst[0]], [src[1], dst[1]], color=color, linewidth=0.8, alpha=0.55, linestyle=linestyle)

    draw_events(accepted_events, "#2ca02c", "^", f"loop accepted ({len(accepted_events)})", "-")
    draw_events(rejected_events, "#d62728", "v", f"loop rejected ({len(rejected_events)})", "--")
    annotate_events(accepted_events, "#2ca02c")
    annotate_events(rejected_events, "#d62728")

    ax.axis("equal")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(evaluation_dir.parent.name)
    ax.legend(loc="upper right")

    ax_table.axis("off")
    columns = ["#", "pair", "status", "reason", "initΔ xy/yaw/z", "range<=th", "ndt>=th", "pgo<=th", "gates"]
    table_rows = []
    for event in table_events:
        reason = first_text(event.get("loop_reject_reason"), event.get("reject_reason_primary"), event.get("status"))
        ndt_value = first_text(event.get("score"), event.get("ndt_score"))
        pgo_threshold = first_text(event.get("rk_loop_th"), event.get("loop_robust_delta"))
        table_rows.append(
            [
                str(event["loop_index"]),
                f"{event['keyframe_id']}->{event['candidate_id']}",
                truncate_text(event.get("status"), 16),
                truncate_text(reason, 22),
                format_init_to_ndt_delta(event),
                format_threshold(
                    event.get("xy_dist_m"),
                    event.get("range_threshold_m"),
                    "le",
                    1,
                    first_text(event.get("gate_range_pass"), event.get("pass_range")),
                ),
                format_threshold(ndt_value, event.get("ndt_score_threshold"), "ge", 2, event.get("gate_ndt_score_pass")),
                format_threshold(event.get("loop_chi2"), pgo_threshold, "le", 3, event.get("gate_pgo_chi2_pass")),
                truncate_text(gate_summary(event), 34),
            ]
        )
    draw_table_rows = table_rows if table_rows else [["-", "-", "waiting", "no loop events yet", "-", "-", "-", "-", "-"]]
    table = ax_table.table(
        cellText=draw_table_rows,
        colLabels=columns,
        loc="upper left",
        cellLoc="left",
        colLoc="left",
        colWidths=[0.04, 0.11, 0.125, 0.17, 0.12, 0.11, 0.105, 0.105, 0.115],
        bbox=[0.0, 0.0, 1.0, 0.96],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(4.5 if len(table_rows) > 100 else 5.5 if len(table_rows) > 70 else 6.0)
    for (row, col), cell in table.get_celld().items():
        if row == 0:
            cell.set_facecolor("#e9eef5")
            cell.set_text_props(weight="bold")
        else:
            cell.set_facecolor("#ffffff" if row % 2 else "#f7f7f7")
            if col == 0:
                event = table_events[row - 1] if row - 1 < len(table_events) else {}
                cell.set_facecolor("#bfe5bf" if event.get("accepted") else "#f2b8b5")
                cell.set_text_props(weight="bold")
        cell.set_edgecolor("#c9c9c9")
        cell.set_linewidth(0.35)
    hidden_rows = max(0, len(events) - len(table_events))
    suffix = f", last {len(table_events)} shown" if hidden_rows else ""
    filter_title = "" if args.loop_filter == "all" else f", {args.loop_filter} only"
    total_suffix = "" if len(events) == len(all_events) else f", {len(all_events)} total"
    ax_table.set_title(
        f"Loop details{filter_title} ({len(events)} shown{total_suffix}, "
        f"{len(accepted_events)} accepted, {len(rejected_events)} rejected{suffix})",
        loc="left",
        fontsize=10,
    )

    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    tmp_output = output.with_name(output.name + ".tmp.png")
    fig.savefig(tmp_output, dpi=150)
    os.replace(tmp_output, output)
    plt.close(fig)
    print(
        f"[trajectory_plot] refreshed={output} keyframes={len(trajectory)} loops={len(events)} "
        f"accepted={len(accepted_events)} rejected={len(rejected_events)} "
        f"filter={args.loop_filter} source={trajectory_source}"
    )
    print(f"trajectory_plot={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
