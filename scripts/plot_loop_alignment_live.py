#!/usr/bin/env python3
import argparse
import csv
import math
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


CLOUD_STYLE = {
    "target": {"color": "#60646c", "label": "history target", "alpha": 0.55, "size": 0.7},
    "source_before": {"color": "#d97706", "label": "source before NDT", "alpha": 0.65, "size": 0.8},
    "source_after": {"color": "#2563eb", "label": "source after NDT", "alpha": 0.70, "size": 0.8},
}


def parse_bool(value):
    return str(value).strip().lower() in {"1", "true", "yes", "on", "y"}


def read_points(path):
    clouds = {name: [] for name in CLOUD_STYLE}
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row.get("cloud", "")
            if name not in clouds:
                continue
            try:
                clouds[name].append((float(row["x"]), float(row["y"]), float(row["z"])))
            except (KeyError, TypeError, ValueError):
                continue
    return {name: np.asarray(points, dtype=np.float64) for name, points in clouds.items()}


def finite_points(clouds):
    pts = []
    for arr in clouds.values():
        if arr.size == 0:
            continue
        valid = np.isfinite(arr).all(axis=1)
        if valid.any():
            pts.append(arr[valid])
    if not pts:
        return np.empty((0, 3), dtype=np.float64)
    return np.vstack(pts)


def set_limits(ax, pts, axes):
    if pts.size == 0:
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        return
    x = pts[:, axes[0]]
    y = pts[:, axes[1]]
    xmin, xmax = np.percentile(x, [0.5, 99.5])
    ymin, ymax = np.percentile(y, [0.5, 99.5])
    if not all(math.isfinite(v) for v in (xmin, xmax, ymin, ymax)):
        xmin, xmax = float(np.min(x)), float(np.max(x))
        ymin, ymax = float(np.min(y)), float(np.max(y))
    span = max(xmax - xmin, ymax - ymin, 1.0)
    xmid = 0.5 * (xmin + xmax)
    ymid = 0.5 * (ymin + ymax)
    margin = 0.08 * span
    ax.set_xlim(xmid - 0.5 * span - margin, xmid + 0.5 * span + margin)
    ax.set_ylim(ymid - 0.5 * span - margin, ymid + 0.5 * span + margin)


def plot_view(row, clouds, out_path, view):
    if view == "bev":
        axes = (0, 1)
        xlabel = "x in history frame [m]"
        ylabel = "y in history frame [m]"
        title_view = "BEV X-Y"
    else:
        axes = (0, 2)
        xlabel = "x in history frame [m]"
        ylabel = "z in history frame [m]"
        title_view = "Y-side X-Z"

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(10.5, 7.0))
    for name in ("target", "source_before", "source_after"):
        arr = clouds.get(name)
        if arr is None or arr.size == 0:
            continue
        valid = np.isfinite(arr).all(axis=1)
        arr = arr[valid]
        if arr.size == 0:
            continue
        style = CLOUD_STYLE[name]
        ax.scatter(
            arr[:, axes[0]],
            arr[:, axes[1]],
            s=style["size"],
            c=style["color"],
            alpha=style["alpha"],
            linewidths=0,
            label=f"{style['label']} ({len(arr)})",
        )

    pts = finite_points(clouds)
    set_limits(ax, pts, axes)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", linewidth=0.35, alpha=0.35)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    score = row.get("ndt_score", "")
    source_type = row.get("source_type", "")
    converged = row.get("converged", "")
    ax.set_title(
        f"Loop {row.get('event_id', '')}: {row.get('curr_kf_id', '')}->{row.get('hist_kf_id', '')} "
        f"{title_view}\nsource={source_type} ndt_score={score} converged={converged}",
        fontsize=11,
    )
    ax.legend(loc="best", markerscale=5, fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def read_events(path):
    if not path.exists():
        return []
    rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("event_id") and row.get("points_file"):
                rows.append(row)
    return rows


def process_events(events_path, quiet):
    made = 0
    for row in read_events(events_path):
        points_path = Path(row["points_file"])
        bev_path = Path(row.get("bev_png") or "")
        yside_path = Path(row.get("yside_png") or "")
        if not points_path.exists():
            continue
        if bev_path.exists() and yside_path.exists():
            continue
        try:
            clouds = read_points(points_path)
            if not bev_path.exists():
                plot_view(row, clouds, bev_path, "bev")
                made += 1
            if not yside_path.exists():
                plot_view(row, clouds, yside_path, "yside")
                made += 1
        except Exception as exc:  # noqa: BLE001 - this is a best-effort live debug tool.
            if not quiet:
                print(f"[loop_alignment_live] failed event={row.get('event_id', '')}: {exc}", flush=True)
    return made


def main():
    parser = argparse.ArgumentParser(description="Live plot LoopClosing NDT source/target alignment debug exports.")
    parser.add_argument("--loop-debug-dir", default="", help="Path to evaluation/loop_debug")
    parser.add_argument("--alignment-dir", default="", help="Direct path to loop_alignment_live")
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    if args.alignment_dir:
        alignment_dir = Path(args.alignment_dir)
    elif args.loop_debug_dir:
        alignment_dir = Path(args.loop_debug_dir) / "loop_alignment_live"
    else:
        raise RuntimeError("provide --loop-debug-dir or --alignment-dir")
    events_path = alignment_dir / "loop_alignment_events.csv"

    if not args.quiet:
        print(f"loop_alignment_live_dir={alignment_dir}", flush=True)
    while True:
        process_events(events_path, args.quiet)
        if args.once:
            break
        time.sleep(max(0.2, args.interval))


if __name__ == "__main__":
    main()
