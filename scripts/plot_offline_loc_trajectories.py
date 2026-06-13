#!/usr/bin/env python3
import argparse
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


def main():
    parser = argparse.ArgumentParser(description="Plot offline localization TUM trajectories.")
    parser.add_argument("result_dir", help="Directory containing global_loc.tum, lio_local.tum, bag_odom.tum")
    parser.add_argument("--output", default="", help="Output PNG path. Default: result_dir/trajectory_xy.png")
    args = parser.parse_args()

    result_dir = Path(args.result_dir)
    output = Path(args.output) if args.output else result_dir / "trajectory_xy.png"

    series = [
        ("global_loc", result_dir / "global_loc.tum", "#1f77b4"),
        ("lio_local", result_dir / "lio_local.tum", "#ff7f0e"),
        ("bag_odom", result_dir / "bag_odom.tum", "#2ca02c"),
    ]

    plotted = 0
    plt.figure(figsize=(10, 8))
    for label, path, color in series:
        pts = read_tum(path)
        if not pts:
            print(f"skip empty trajectory: {path}")
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        plt.plot(xs, ys, label=f"{label} ({len(pts)})", linewidth=1.2, color=color)
        plt.scatter(xs[0], ys[0], marker="o", s=18, color=color)
        plt.scatter(xs[-1], ys[-1], marker="x", s=22, color=color)
        plotted += 1

    if plotted == 0:
        print(f"no non-empty trajectories under {result_dir}")
        return 1

    plt.axis("equal")
    plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(result_dir.name)
    plt.legend()
    plt.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output, dpi=150)
    print(f"trajectory_plot={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
