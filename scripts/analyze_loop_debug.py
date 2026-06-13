#!/usr/bin/env python3
import argparse
import json
import math
import sys
from pathlib import Path

PAIR_COLS = ["curr_kf_id", "hist_kf_id"]
PAIR_COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b",
    "#e377c2", "#7f7f7f", "#bcbd22", "#17becf", "#393b79", "#637939",
    "#8c6d31", "#843c39", "#7b4173", "#3182bd", "#31a354", "#756bb1",
    "#e6550d", "#969696", "#6baed6", "#74c476", "#9e9ac8", "#fd8d3c",
]
PAIR_LINESTYLES = ["-", "--", "-.", ":"]
LABEL_OFFSETS_PT = [
    (0, 14), (14, 0), (0, -14), (-14, 0),
    (14, 14), (14, -14), (-14, 14), (-14, -14),
    (0, 26), (26, 0), (0, -26), (-26, 0),
    (24, 24), (24, -24), (-24, 24), (-24, -24),
    (0, 40), (40, 0), (0, -40), (-40, 0),
    (36, 36), (36, -36), (-36, 36), (-36, -36),
]


def require_deps():
    try:
        import pandas as pd  # noqa: F401
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # noqa: F401
    except ImportError as exc:
        print(
            "ERROR: analyze_loop_debug.py requires pandas and matplotlib. "
            "Install them with: python3 -m pip install pandas matplotlib",
            file=sys.stderr,
        )
        print(f"missing_dependency={exc}", file=sys.stderr)
        sys.exit(2)


def read_csv(pd, path, required=False):
    if not path.exists():
        if required:
            raise FileNotFoundError(str(path))
        return pd.DataFrame()
    return pd.read_csv(path)


def ensure_pair_cols(pd, df):
    if df is None or df.empty:
        return pd.DataFrame(columns=PAIR_COLS)
    out = df.copy()
    for col in PAIR_COLS:
        if col not in out.columns:
            out[col] = pd.Series(dtype="float64")
    return out


def merge_missing_pair_cols(pd, base, source, wanted_cols):
    base = ensure_pair_cols(pd, base).copy()
    source = ensure_pair_cols(pd, source)
    if source.empty:
        return base
    source_cols = [c for c in wanted_cols if c in source.columns]
    if not source_cols:
        return base

    source = source[PAIR_COLS + source_cols].drop_duplicates(PAIR_COLS)
    add_cols = [c for c in source_cols if c not in base.columns]
    fill_cols = [c for c in source_cols if c in base.columns]

    if add_cols:
        base = base.merge(source[PAIR_COLS + add_cols], on=PAIR_COLS, how="left")

    if fill_cols:
        suffix = "__fill"
        fill_source = source[PAIR_COLS + fill_cols]
        merged = base.merge(fill_source, on=PAIR_COLS, how="left", suffixes=("", suffix))
        for col in fill_cols:
            fill_col = col + suffix
            if fill_col in merged.columns:
                merged[col] = merged[col].combine_first(merged[fill_col])
                merged = merged.drop(columns=[fill_col])
        base = merged

    return base


def first_numeric(pd, df, col, default=math.nan):
    if df is None or col not in df:
        return default
    values = pd.to_numeric(df[col], errors="coerce").dropna()
    if values.empty:
        return default
    return float(values.iloc[0])


def latest_gate_per_pair(gates):
    if gates.empty or not set(PAIR_COLS).issubset(gates.columns):
        return gates
    return gates.dropna(subset=PAIR_COLS).groupby(PAIR_COLS, as_index=False).tail(1)


def safe_num(series):
    return series.dropna() if series is not None else []


def save_hist(plt, data, path, title, xlabel):
    plt.figure(figsize=(8, 4.5))
    values = safe_num(data)
    if len(values) > 0:
        plt.hist(values, bins=40)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel("count")
    plt.tight_layout()
    plt.savefig(path)
    plt.close()


def save_bar(plt, series, path, title, xlabel):
    plt.figure(figsize=(9, 4.8))
    if series is not None and len(series) > 0:
        series.plot(kind="bar")
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel("count")
    plt.tight_layout()
    plt.savefig(path)
    plt.close()


def build_suspects(pd, candidates, matches, gates_final, edges, pgo, metadata):
    if gates_final.empty or "final_status" not in gates_final.columns:
        return pd.DataFrame()
    candidates = ensure_pair_cols(pd, candidates)
    matches = ensure_pair_cols(pd, matches)
    edges = ensure_pair_cols(pd, edges)
    pgo = ensure_pair_cols(pd, pgo)
    accepted = gates_final[gates_final["final_status"].isin(["accepted", "accepted_high_risk"])].copy()
    if accepted.empty:
        return accepted

    out = accepted.merge(matches, on=PAIR_COLS, how="left")
    out = out.merge(candidates, on=PAIR_COLS, how="left", suffixes=("", "_candidate"))
    out = out.merge(edges, on=PAIR_COLS, how="left", suffixes=("", "_edge"))
    out = out.merge(pgo, on=PAIR_COLS, how="left", suffixes=("", "_pgo"))

    loop_cfg = metadata.get("loop_closing", {}) if isinstance(metadata, dict) else {}
    ndt_th = float(loop_cfg.get("ndt_score_th", first_numeric(pd, out, "ndt_score_threshold")))
    max_range = float(loop_cfg.get("max_range", first_numeric(pd, out, "range_threshold_m")))
    rk_loop_th = float(loop_cfg.get("rk_loop_th", first_numeric(pd, out, "rk_loop_th")))

    def flags(row):
        fs = set(str(row.get("risk_flags", "")).split(";")) if str(row.get("risk_flags", "")) != "nan" else set()
        score = row.get("ndt_score")
        xy = row.get("xy_dist_m")
        corr_t = row.get("correction_trans_m")
        corr_yaw = row.get("correction_yaw_deg")
        chi2 = row.get("loop_chi2")
        near_delta = row.get("max_pose_delta_near_loop_m")
        before = row.get("pgo_error_before")
        after = row.get("pgo_error_after")
        try:
            if pd.notna(score) and score - ndt_th < 0.3:
                fs.add("low_score_margin")
            if pd.notna(xy) and pd.notna(max_range) and xy > 0.8 * max_range:
                fs.add("near_max_range")
            if (pd.notna(corr_t) and corr_t > 0.8) or (pd.notna(corr_yaw) and abs(corr_yaw) > 10.0):
                fs.add("large_correction")
            if pd.notna(chi2) and pd.notna(rk_loop_th) and chi2 > 0.8 * rk_loop_th:
                fs.add("high_chi2_margin")
            if pd.notna(near_delta) and near_delta > 0.3:
                fs.add("local_pose_delta_large")
            if pd.notna(before) and pd.notna(after) and after >= before:
                fs.add("pgo_error_not_reduced")
        except Exception:
            pass
        return ";".join(sorted(f for f in fs if f))

    per_curr = accepted.groupby("curr_kf_id").size()
    out["risk_flags"] = out.apply(flags, axis=1)
    out.loc[out["curr_kf_id"].map(per_curr).fillna(0) > 2, "risk_flags"] = out.apply(
        lambda r: ";".join(sorted(set([x for x in str(r.get("risk_flags", "")).split(";") if x] + ["multi_loop_same_kf"]))),
        axis=1,
    )
    out["risk_score"] = out["risk_flags"].apply(lambda s: 0 if not s or s == "nan" else len([x for x in s.split(";") if x]))
    out = out[out["risk_score"] > 0].sort_values(["risk_score", "curr_kf_id"], ascending=[False, True])
    keep = [
        "curr_kf_id",
        "hist_kf_id",
        "risk_score",
        "risk_flags",
        "final_status",
        "ndt_score",
        "ndt_score_threshold",
        "xy_dist_m",
        "range_threshold_m",
        "correction_trans_m",
        "correction_yaw_deg",
        "loop_chi2",
        "rk_loop_th",
        "pgo_error_before",
        "pgo_error_after",
        "max_pose_delta_near_loop_m",
    ]
    return out[[c for c in keep if c in out.columns]]


def parse_review_pairs(value):
    pairs = []
    if not value:
        return pairs
    for token in value.split(","):
        item = token.strip()
        if not item:
            continue
        item = item.replace("->", ":").replace("-", ":")
        parts = item.split(":")
        if len(parts) != 2:
            raise ValueError(f"invalid review pair '{token}', expected curr:hist")
        pairs.append((int(parts[0]), int(parts[1])))
    return pairs


def format_value(value, digits=2):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return "-"
    if not math.isfinite(v):
        return "-"
    return f"{v:.{digits}f}"


def first_text(*values):
    for value in values:
        if value is None:
            continue
        text = str(value)
        if text and text.lower() != "nan":
            return text
    return ""


def format_flags(value, max_len=42):
    flags = first_text(value)
    if len(flags) > max_len:
        return flags[: max_len - 3] + "..."
    return flags


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in {"1", "true", "yes", "y", "on"}


def safe_int(value, default=None):
    try:
        if value is None:
            return default
        return int(float(value))
    except (TypeError, ValueError):
        return default


def build_curr_color_map(rows):
    curr_ids = []
    seen = set()
    for curr, _, _ in rows:
        if curr not in seen:
            seen.add(curr)
            curr_ids.append(curr)
    return {curr: PAIR_COLORS[i % len(PAIR_COLORS)] for i, curr in enumerate(curr_ids)}


def build_review_rows(pd, review_pairs, review_top_n, suspects, gates_final, matches, candidates, edges, pgo):
    if review_pairs:
        base = pd.DataFrame(review_pairs, columns=PAIR_COLS)
        base["_review_order"] = range(len(base))
    elif not suspects.empty and set(PAIR_COLS).issubset(suspects.columns):
        base = suspects.head(review_top_n)[PAIR_COLS].copy()
        base["_review_order"] = range(len(base))
    elif not gates_final.empty and "final_status" in gates_final.columns:
        accepted = gates_final[gates_final["final_status"].isin(["accepted", "accepted_high_risk"])].copy()
        if set(PAIR_COLS).issubset(accepted.columns):
            base = accepted.head(review_top_n)[PAIR_COLS].copy()
        else:
            base = pd.DataFrame(columns=PAIR_COLS)
        base["_review_order"] = range(len(base))
    else:
        return pd.DataFrame(columns=PAIR_COLS)

    base = merge_missing_pair_cols(pd, base, gates_final, [
        "final_status", "reject_reason_primary", "reject_reason_secondary", "risk_flags", "risk_score",
    ])
    base = merge_missing_pair_cols(pd, base, suspects, [
        "risk_score", "risk_flags", "final_status", "ndt_score", "ndt_score_threshold", "xy_dist_m",
        "range_threshold_m", "correction_trans_m", "correction_yaw_deg", "loop_chi2", "rk_loop_th",
        "pgo_error_before", "pgo_error_after", "max_pose_delta_near_loop_m",
    ])
    base = merge_missing_pair_cols(pd, base, matches, [
        "ndt_score", "ndt_score_threshold", "fitness_score", "correction_trans_m", "correction_yaw_deg",
        "match_time_ms",
    ])
    base = merge_missing_pair_cols(pd, base, candidates, [
        "xy_dist_m", "z_diff_m", "yaw_diff_deg", "id_gap", "range_threshold_m", "candidate_rank",
    ])
    base = merge_missing_pair_cols(pd, base, edges, ["loop_chi2", "rk_loop_th", "accepted"])
    base = merge_missing_pair_cols(pd, base, pgo, [
        "pgo_error_before", "pgo_error_after", "pgo_error_delta", "max_pose_delta_near_loop_m",
    ])
    if "_review_order" in base:
        base = base.sort_values("_review_order")
    return base


def make_review_plot_rows(pd, keyframes, review_rows):
    required = {"kf_id", "x", "y"}
    if keyframes.empty or not required.issubset(keyframes.columns):
        return [], None, "skipped: keyframes.csv missing kf_id/x/y"
    if review_rows.empty:
        return [], None, "skipped: no review pairs"

    keyframes = keyframes.dropna(subset=["kf_id", "x", "y"]).copy()
    keyframes["kf_id"] = keyframes["kf_id"].astype(int)
    pose_by_id = keyframes.set_index("kf_id")[["x", "y"]].to_dict("index")

    rows = []
    for _, row in review_rows.iterrows():
        try:
            curr = int(row["curr_kf_id"])
            hist = int(row["hist_kf_id"])
        except (TypeError, ValueError):
            continue
        if curr not in pose_by_id or hist not in pose_by_id:
            continue
        rows.append((curr, hist, row))
    if not rows:
        return [], keyframes, "skipped: review pair keyframes not found in keyframes.csv"
    return rows, keyframes, ""


def review_color(plt, pd, row, idx, curr, color_by, curr_color_map, max_risk):
    if color_by == "risk":
        risk = pd.to_numeric(pd.Series([row.get("risk_score", 0)]), errors="coerce").fillna(0).iloc[0]
        cmap = plt.get_cmap("autumn_r")
        return cmap(min(1.0, float(risk) / max_risk)) if max_risk > 0 else "#ff7f0e"
    if color_by == "curr":
        return curr_color_map.get(curr, PAIR_COLORS[(idx - 1) % len(PAIR_COLORS)])
    return PAIR_COLORS[(idx - 1) % len(PAIR_COLORS)]


def bbox_overlap(a, b, pad=0.0):
    return not (
        a[2] + pad < b[0]
        or a[0] - pad > b[2]
        or a[3] + pad < b[1]
        or a[1] - pad > b[3]
    )


def estimate_label_bbox(ax, text, xy_data, offset_pt, fontsize):
    fig = ax.figure
    anchor_px = ax.transData.transform(xy_data)
    px_per_pt = fig.dpi / 72.0
    center_x = anchor_px[0] + offset_pt[0] * px_per_pt
    center_y = anchor_px[1] + offset_pt[1] * px_per_pt
    lines = str(text).splitlines() or [""]
    max_chars = max(len(line) for line in lines)
    width = max(16.0, max_chars * fontsize * px_per_pt * 0.62 + 10.0)
    height = max(14.0, len(lines) * fontsize * px_per_pt * 1.25 + 8.0)
    return (
        center_x - width * 0.5,
        center_y - height * 0.5,
        center_x + width * 0.5,
        center_y + height * 0.5,
    )


def obstacle_boxes_from_points(ax, obstacle_points, box_px=9.0):
    boxes = []
    if not obstacle_points:
        return boxes
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    for x, y in obstacle_points:
        if not (xlim[0] <= x <= xlim[1] and ylim[0] <= y <= ylim[1]):
            continue
        px = ax.transData.transform((x, y))
        boxes.append((px[0] - box_px, px[1] - box_px, px[0] + box_px, px[1] + box_px))
    return boxes


def choose_label_offset(ax, text, xy_data, occupied_boxes, fontsize):
    best_offset = LABEL_OFFSETS_PT[0]
    best_bbox = estimate_label_bbox(ax, text, xy_data, best_offset, fontsize)
    best_score = math.inf
    for offset in LABEL_OFFSETS_PT:
        bbox = estimate_label_bbox(ax, text, xy_data, offset, fontsize)
        overlap_count = sum(1 for occupied in occupied_boxes if bbox_overlap(bbox, occupied, pad=2.0))
        distance_penalty = (abs(offset[0]) + abs(offset[1])) * 0.01
        score = overlap_count * 1000.0 + distance_penalty
        if score < best_score:
            best_score = score
            best_offset = offset
            best_bbox = bbox
            if overlap_count == 0:
                break
    return best_offset, best_bbox


def draw_review_lines(ax, plt, pd, rows, pose_by_id, label_mode, label_max, color_by,
                      avoid_label_overlap=True, obstacle_points=None):
    max_risk = 1.0
    risk_series = pd.to_numeric(pd.Series([row.get("risk_score", 0) for _, _, row in rows]), errors="coerce").dropna()
    if not risk_series.empty:
        max_risk = max(1.0, float(risk_series.max()))
    curr_color_map = build_curr_color_map(rows)
    label_fontsize = 7
    occupied_label_boxes = obstacle_boxes_from_points(ax, obstacle_points, box_px=7.0) if avoid_label_overlap else []

    table_rows = []
    for idx, (curr, hist, row) in enumerate(rows, start=1):
        cxy = pose_by_id[curr]
        hxy = pose_by_id[hist]
        risk = pd.to_numeric(pd.Series([row.get("risk_score", 0)]), errors="coerce").fillna(0).iloc[0]
        color = review_color(plt, pd, row, idx, curr, color_by, curr_color_map, max_risk)
        linestyle = PAIR_LINESTYLES[(idx - 1) % len(PAIR_LINESTYLES)]
        linewidth = 0.9 + 0.45 * min(float(risk), 5.0)
        ax.plot([cxy["x"], hxy["x"]], [cxy["y"], hxy["y"]], color=color, linewidth=linewidth, alpha=0.82,
                linestyle=linestyle, zorder=2)
        ax.scatter([cxy["x"]], [cxy["y"]], marker="^", s=58, color=color, edgecolors="#111111",
                   linewidths=0.4, zorder=5)
        ax.scatter([hxy["x"]], [hxy["y"]], marker="o", s=42, color=color, edgecolors="#111111",
                   linewidths=0.4, zorder=5)

        mx = (cxy["x"] + hxy["x"]) * 0.5
        my = (cxy["y"] + hxy["y"]) * 0.5
        if idx <= label_max and label_mode != "none":
            if label_mode == "full":
                label = (
                    f"#{idx} {curr}->{hist}\n"
                    f"s={format_value(row.get('ndt_score'), 2)} d={format_value(row.get('xy_dist_m'), 1)} "
                    f"yaw={format_value(row.get('correction_yaw_deg'), 1)} chi2={format_value(row.get('loop_chi2'), 3)}"
                )
            else:
                label = f"{idx}"
            label_offset = (5, 5)
            if avoid_label_overlap:
                label_offset, label_bbox = choose_label_offset(
                    ax, label, (mx, my), occupied_label_boxes, label_fontsize
                )
                occupied_label_boxes.append(label_bbox)
            ax.annotate(
                label,
                xy=(mx, my),
                xytext=label_offset,
                textcoords="offset points",
                fontsize=label_fontsize,
                color="#111111",
                ha="center",
                va="center",
                bbox={"boxstyle": "round,pad=0.16", "fc": "white", "ec": color, "alpha": 0.76},
            )

        table_rows.append({
            "cells": [
                str(idx),
                f"{curr}->{hist}",
                format_value(row.get("ndt_score"), 2),
                format_value(row.get("xy_dist_m"), 1),
                format_value(row.get("correction_yaw_deg"), 1),
                format_value(row.get("loop_chi2"), 3),
                format_flags(first_text(row.get("risk_flags"), row.get("reject_reason_primary"))),
            ],
            "color": color,
        })
    return table_rows


def add_review_table(ax_table, table_rows, table_max):
    ax_table.axis("off")
    columns = ["#", "pair", "score", "dist", "yaw", "chi2", "flags"]
    shown_entries = table_rows[:table_max]
    shown = [entry["cells"] for entry in shown_entries]
    if len(table_rows) > table_max:
        shown.append(["...", f"+{len(table_rows) - table_max} more", "", "", "", "", ""])
    table = ax_table.table(cellText=shown, colLabels=columns, loc="upper left", cellLoc="left", colLoc="left",
                           colWidths=[0.06, 0.18, 0.11, 0.1, 0.1, 0.1, 0.35])
    table.auto_set_font_size(False)
    table.set_fontsize(7)
    table.scale(1.0, 1.18)
    for (row, col), cell in table.get_celld().items():
        if row == 0:
            cell.set_facecolor("#e9eef5")
            cell.set_text_props(weight="bold")
        else:
            if row <= len(shown_entries) and col == 0:
                cell.set_facecolor(shown_entries[row - 1]["color"])
                cell.set_text_props(color="#111111", weight="bold")
            else:
                cell.set_facecolor("#ffffff" if row % 2 else "#f7f7f7")
        cell.set_edgecolor("#c9c9c9")


def plot_review_pairs(plt, pd, keyframes, review_rows, output_path, label_max, label_mode, table_max, color_by,
                      avoid_label_overlap, fig_width, fig_height):
    rows, keyframes, skip_reason = make_review_plot_rows(pd, keyframes, review_rows)
    if skip_reason:
        return skip_reason
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(fig_width, fig_height))
    gs = fig.add_gridspec(1, 2, width_ratios=[3.2, 1.45], wspace=0.05)
    ax = fig.add_subplot(gs[0, 0])
    ax_table = fig.add_subplot(gs[0, 1])
    xs = keyframes["x"].to_numpy()
    ys = keyframes["y"].to_numpy()
    ax.plot(xs, ys, color="#1f77b4", linewidth=1.0, label=f"keyframes ({len(keyframes)})")
    ax.scatter(xs[0], ys[0], color="#1f77b4", marker="o", s=42,
               label="start", zorder=4)
    ax.scatter(xs[-1], ys[-1], color="#111111", marker="x", s=56,
               label="end", zorder=4)
    ax.set_aspect("equal", adjustable="box")
    fig.canvas.draw()

    pose_by_id = keyframes.set_index("kf_id")[["x", "y"]].to_dict("index")
    table_rows = draw_review_lines(ax, plt, pd, rows, pose_by_id, label_mode, label_max, color_by,
                                   avoid_label_overlap, list(zip(xs, ys)))
    add_review_table(ax_table, table_rows, table_max)

    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.45)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Loop Review Pairs")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=170)
    plt.close(fig)
    return f"written: {output_path}"


def plot_review_zoom(plt, pd, keyframes, review_rows, output_path, label_max, label_mode, color_by,
                     avoid_label_overlap, fig_width, fig_height, pad_ratio=0.12):
    rows, keyframes, skip_reason = make_review_plot_rows(pd, keyframes, review_rows)
    if skip_reason:
        return skip_reason
    pose_by_id = keyframes.set_index("kf_id")[["x", "y"]].to_dict("index")
    xs = []
    ys = []
    for curr, hist, _ in rows:
        xs.extend([pose_by_id[curr]["x"], pose_by_id[hist]["x"]])
        ys.extend([pose_by_id[curr]["y"], pose_by_id[hist]["y"]])
    if not xs or not ys:
        return "skipped: no zoom bounds"
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    span = max(xmax - xmin, ymax - ymin, 1.0)
    pad = span * pad_ratio

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(fig_width, fig_height))
    kxs = keyframes["x"].to_numpy()
    kys = keyframes["y"].to_numpy()
    ax.plot(kxs, kys, color="#1f77b4", linewidth=1.0, label=f"keyframes ({len(keyframes)})")
    ax.set_xlim(xmin - pad, xmax + pad)
    ax.set_ylim(ymin - pad, ymax + pad)
    ax.set_aspect("equal", adjustable="box")
    fig.canvas.draw()
    draw_review_lines(ax, plt, pd, rows, pose_by_id, label_mode, label_max, color_by,
                      avoid_label_overlap, list(zip(kxs, kys)))
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.45)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Loop Review Pairs Zoom")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
    return f"written: {output_path}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--loop_debug_dir", required=True)
    parser.add_argument("--review_pairs", default="", help="Comma separated curr:hist pairs, e.g. 1451:760,1451:802")
    parser.add_argument("--review_top_n", type=int, default=50,
                        help="Number of top suspect pairs to draw when review_pairs is empty")
    parser.add_argument("--review_label_max", type=int, default=50, help="Maximum number of pair labels to draw")
    parser.add_argument("--review_output", default="figures/loop_review_pairs.png",
                        help="Output PNG path, relative to loop_debug_dir unless absolute")
    parser.add_argument("--review_label_mode", choices=["id", "full", "none"], default="id",
                        help="Label style on trajectory: id, full, or none")
    parser.add_argument("--review_table_max", type=int, default=50, help="Maximum rows shown in the side table")
    parser.add_argument("--review_zoom", default="true", help="Generate a zoomed review plot when review_pairs is set")
    parser.add_argument("--review_color_by", choices=["pair", "risk", "curr"], default="pair",
                        help="Color loop review lines by pair, risk, or current keyframe")
    parser.add_argument("--review_split_by_curr", default="true",
                        help="Generate per-current-keyframe review plots when review_pairs is set")
    parser.add_argument("--review_avoid_label_overlap", default="true",
                        help="Try to place trajectory labels in nearby free space")
    parser.add_argument("--review_fig_width", type=float, default=26.0,
                        help="Width of the main review figure in inches")
    parser.add_argument("--review_fig_height", type=float, default=16.0,
                        help="Height of the main review figure in inches")
    parser.add_argument("--review_zoom_fig_width", type=float, default=18.0,
                        help="Width of zoom review figures in inches")
    parser.add_argument("--review_zoom_fig_height", type=float, default=14.0,
                        help="Height of zoom review figures in inches")
    args = parser.parse_args()

    require_deps()
    import pandas as pd
    import matplotlib.pyplot as plt

    root = Path(args.loop_debug_dir)
    if not root.is_dir():
        raise SystemExit(f"loop_debug_dir does not exist: {root}")
    figures = root / "figures"
    figures.mkdir(parents=True, exist_ok=True)

    metadata_path = root / "run_metadata.json"
    metadata = {}
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))

    keyframes = read_csv(pd, root / "keyframes.csv")
    candidates = read_csv(pd, root / "loop_candidates.csv")
    matches = read_csv(pd, root / "loop_matches.csv")
    gates = read_csv(pd, root / "loop_gate_decisions.csv")
    edges = read_csv(pd, root / "loop_edges.csv")
    pgo = read_csv(pd, root / "loop_pgo_impact.csv")
    candidates = ensure_pair_cols(pd, candidates)
    matches = ensure_pair_cols(pd, matches)
    edges = ensure_pair_cols(pd, edges)
    pgo = ensure_pair_cols(pd, pgo)
    gates_final = latest_gate_per_pair(gates)

    review_pairs = parse_review_pairs(args.review_pairs)
    suspects = build_suspects(pd, candidates, matches, gates_final, edges, pgo, metadata)
    suspects.to_csv(root / "loop_suspects.csv", index=False)
    review_rows = build_review_rows(pd, review_pairs, args.review_top_n, suspects, gates_final, matches, candidates,
                                    edges, pgo)
    review_output = Path(args.review_output)
    if not review_output.is_absolute():
        review_output = root / review_output
    review_status = plot_review_pairs(plt, pd, keyframes, review_rows, review_output, args.review_label_max,
                                      args.review_label_mode, args.review_table_max, args.review_color_by,
                                      parse_bool(args.review_avoid_label_overlap),
                                      args.review_fig_width, args.review_fig_height)
    review_zoom_output = None
    review_zoom_status = "disabled"
    if review_pairs and parse_bool(args.review_zoom):
        review_zoom_output = review_output.with_name(review_output.stem + "_zoom" + review_output.suffix)
        review_zoom_status = plot_review_zoom(plt, pd, keyframes, review_rows, review_zoom_output, args.review_label_max,
                                              args.review_label_mode, args.review_color_by,
                                              parse_bool(args.review_avoid_label_overlap),
                                              args.review_zoom_fig_width, args.review_zoom_fig_height)
    split_outputs = []
    if review_pairs and parse_bool(args.review_split_by_curr) and not review_rows.empty and "curr_kf_id" in review_rows:
        curr_ids = sorted({safe_int(v) for v in review_rows["curr_kf_id"].dropna().unique() if safe_int(v) is not None})
        for curr in curr_ids:
            curr_series = review_rows["curr_kf_id"].apply(safe_int)
            subset = review_rows[curr_series == curr].copy()
            split_output = review_output.with_name(f"{review_output.stem}_curr_{curr}{review_output.suffix}")
            status = plot_review_zoom(plt, pd, keyframes, subset, split_output, args.review_label_max,
                                      args.review_label_mode, args.review_color_by,
                                      parse_bool(args.review_avoid_label_overlap),
                                      args.review_zoom_fig_width, args.review_zoom_fig_height, pad_ratio=0.2)
            split_outputs.append((split_output, status))

    status_counts = gates_final["final_status"].value_counts() if not gates_final.empty else pd.Series(dtype=int)
    save_bar(plt, status_counts, figures / "loop_edges_by_status.png", "Loop edges by status", "status")
    if not suspects.empty:
        save_bar(plt, suspects["risk_score"].value_counts().sort_index(), figures / "loop_edges_by_risk.png", "Accepted loop risk score", "risk_score")
    else:
        save_bar(plt, pd.Series(dtype=int), figures / "loop_edges_by_risk.png", "Accepted loop risk score", "risk_score")
    if "ndt_score" in matches:
        save_hist(plt, matches["ndt_score"], figures / "loop_score_hist.png", "Loop NDT score", "ndt_score")
    if "xy_dist_m" in candidates:
        save_hist(plt, candidates["xy_dist_m"], figures / "loop_xy_dist_hist.png", "Loop candidate XY distance", "xy_dist_m")
    if "loop_chi2" in edges:
        save_hist(plt, edges["loop_chi2"], figures / "loop_chi2_hist.png", "Loop chi2", "loop_chi2")
    accepted = gates_final[gates_final["final_status"].isin(["accepted", "accepted_high_risk"])] if not gates_final.empty else pd.DataFrame()
    per_kf = accepted.groupby("curr_kf_id").size() if not accepted.empty else pd.Series(dtype=int)
    save_bar(plt, per_kf, figures / "loop_per_kf_count.png", "Accepted loops per current keyframe", "curr_kf_id")

    plt.figure(figsize=(7, 5))
    if not pgo.empty and {"curr_kf_id", "hist_kf_id", "max_pose_delta_near_loop_m"}.issubset(pgo.columns):
        sc = plt.scatter(pgo["curr_kf_id"], pgo["hist_kf_id"], c=pgo["max_pose_delta_near_loop_m"], cmap="viridis")
        plt.colorbar(sc, label="max_pose_delta_near_loop_m")
    plt.xlabel("curr_kf_id")
    plt.ylabel("hist_kf_id")
    plt.title("PGO pose delta heatmap")
    plt.tight_layout()
    plt.savefig(figures / "pgo_pose_delta_heatmap.png")
    plt.close()

    total_candidates = len(candidates)
    accepted_count = int(status_counts.get("accepted", 0) + status_counts.get("accepted_high_risk", 0))
    rejected_count = int(sum(v for k, v in status_counts.items() if str(k).startswith("rejected_")))
    reject_counts = status_counts[[str(k).startswith("rejected_") for k in status_counts.index]] if len(status_counts) else pd.Series(dtype=int)

    def recommend():
        lines = []
        if not suspects.empty and suspects["risk_flags"].str.contains("near_max_range", na=False).sum() > 0:
            lines.append("- 建议复核并可能降低 `max_range`：存在 accepted loop 接近最大搜索半径。")
        else:
            lines.append("- 暂无直接证据要求降低 `max_range`。")
        if not suspects.empty and suspects["risk_flags"].str.contains("low_score_margin", na=False).sum() > 0:
            lines.append("- 建议复核并可能提高 `ndt_score_th`：存在 accepted loop 的 NDT score margin 低于 0.3。")
        else:
            lines.append("- 暂无直接证据要求提高 `ndt_score_th`。")
        max_accepted_per_kf = int(accepted.groupby("curr_kf_id").size().max()) if not accepted.empty else 0
        if max_accepted_per_kf > 2:
            lines.append("- 建议增加 `closest_id_th` 或引入 NMS：同一 curr_kf_id 接收了超过 2 条回环。")
        else:
            lines.append("- 暂无直接证据要求增加 `closest_id_th`。")
        if not suspects.empty and suspects["risk_flags"].str.contains("high_chi2_margin", na=False).sum() > 0:
            lines.append("- 建议复核 `rk_loop_th`：存在 accepted loop 的 chi2 接近 robust/outlier 门限。")
        else:
            lines.append("- 暂无直接证据要求调整 `rk_loop_th`。")
        lines.append("- 建议评估 NMS：牛舍/长廊重复结构中，同一当前关键帧多候选时应优先保留空间和 score 最可信的候选。")
        return "\n".join(lines)

    todo = []
    for name, df, cols in [
        ("loop_matches.csv", matches, ["overlap_ratio", "inlier_ratio"]),
        ("loop_pgo_impact.csv", pgo, ["local_straightness_before", "local_straightness_after"]),
    ]:
        for col in cols:
            if col in df.columns and df[col].isna().all():
                todo.append(f"- `{name}` 的 `{col}` 当前全部为 NaN，需要后续补充可计算来源。")

    report = root / "LOOP_DEBUG_REPORT.md"
    with report.open("w", encoding="utf-8") as f:
        f.write("# Loop Debug Report\n\n")
        f.write(f"- loop_debug_dir: `{root}`\n")
        f.write(f"- run_id: `{metadata.get('run_id', 'unknown')}`\n\n")
        f.write("## Overview\n\n")
        f.write(f"- total_candidates: {total_candidates}\n")
        f.write(f"- accepted: {accepted_count}\n")
        f.write(f"- rejected: {rejected_count}\n\n")
        f.write("## Reject Reasons\n\n")
        if len(reject_counts):
            for status, count in reject_counts.items():
                f.write(f"- {status}: {int(count)}\n")
        else:
            f.write("- none\n")
        f.write("\n## Accepted Distributions\n\n")
        accepted_pairs = accepted[["curr_kf_id", "hist_kf_id"]] if not accepted.empty else pd.DataFrame()
        acc_matches = accepted_pairs.merge(matches, on=["curr_kf_id", "hist_kf_id"], how="left") if not accepted_pairs.empty else pd.DataFrame()
        acc_edges = accepted_pairs.merge(edges, on=["curr_kf_id", "hist_kf_id"], how="left") if not accepted_pairs.empty else pd.DataFrame()
        acc_candidates = accepted_pairs.merge(candidates, on=["curr_kf_id", "hist_kf_id"], how="left") if not accepted_pairs.empty else pd.DataFrame()
        for label, df, col in [("score", acc_matches, "ndt_score"), ("xy_dist", acc_candidates, "xy_dist_m"), ("chi2", acc_edges, "loop_chi2")]:
            if not df.empty and col in df:
                f.write(f"- accepted {label}: mean={df[col].mean():.4f}, p95={df[col].quantile(0.95):.4f}, max={df[col].max():.4f}\n")
            else:
                f.write(f"- accepted {label}: NaN\n")
        f.write("\n## High Risk Accepted Loop Top 20\n\n")
        if suspects.empty:
            f.write("No high-risk accepted loops found by current rules.\n")
        else:
            top = suspects.head(20)
            cols = list(top.columns)
            f.write("| " + " | ".join(cols) + " |\n")
            f.write("| " + " | ".join(["---"] * len(cols)) + " |\n")
            for _, row in top.iterrows():
                f.write("| " + " | ".join(str(row.get(c, "")) for c in cols) + " |\n")
            f.write("\n")
        f.write("\n## Parameter Suggestions\n\n")
        f.write(recommend())
        f.write("\n\n## Manual Review Keyframe Pairs\n\n")
        if suspects.empty:
            f.write("- none\n")
        else:
            for _, row in suspects.head(50).iterrows():
                f.write(f"- curr={int(row.curr_kf_id)}, hist={int(row.hist_kf_id)}, risk={row.risk_score}, flags={row.risk_flags}\n")
        f.write("\n## Review Figure\n\n")
        f.write(f"- loop_review_pairs: `{review_output}`\n")
        f.write(f"- status: {review_status}\n")
        if review_zoom_output is not None:
            f.write(f"- loop_review_pairs_zoom: `{review_zoom_output}`\n")
            f.write(f"- zoom_status: {review_zoom_status}\n")
        if split_outputs:
            f.write("- split_by_curr:\n")
            for path, status in split_outputs:
                f.write(f"  - `{path}`: {status}\n")
        if review_pairs:
            f.write("- requested_pairs:\n")
            found = set()
            if not review_rows.empty and set(PAIR_COLS).issubset(review_rows.columns):
                found = set((int(r.curr_kf_id), int(r.hist_kf_id)) for _, r in review_rows.iterrows())
            for curr, hist in review_pairs:
                f.write(f"  - {curr}->{hist}: {'found' if (curr, hist) in found else 'missing'}\n")
        f.write("\n## TODO / Missing Fields\n\n")
        if todo:
            f.write("\n".join(todo))
            f.write("\n")
        else:
            f.write("- none\n")

    print(f"report={report}")
    print(f"suspects={root / 'loop_suspects.csv'}")
    print(f"figures={figures}")
    print(f"loop_review_pairs={review_output}")
    if review_zoom_output is not None:
        print(f"loop_review_pairs_zoom={review_zoom_output}")
    for path, _ in split_outputs:
        print(f"loop_review_pair_split={path}")


if __name__ == "__main__":
    main()
