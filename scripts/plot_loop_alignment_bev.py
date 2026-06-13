#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import rospy
import yaml


def read_csv_row(path: Path, **keys):
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if all(row.get(k) == str(v) for k, v in keys.items()):
                return row
    raise RuntimeError(f"row not found in {path}: {keys}")


def read_csv_row_or_none(path: Path, **keys):
    if not path.exists():
        return None
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if all(row.get(k) == str(v) for k, v in keys.items()):
                return row
    return None


def read_loop_events(path: Path):
    events = {}
    loop_no = 0
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row.get("loop_candidate_id"):
                continue
            loop_no += 1
            curr = row.get("keyframe_id", "")
            hist = row.get("loop_candidate_id", "")
            events[(curr, hist)] = {
                "loop_no": loop_no,
                "status": row.get("loop_status", ""),
                "accepted": row.get("loop_accepted", ""),
                "reason": row.get("loop_reject_reason", ""),
            }
    return events


def parse_pairs(spec):
    pairs = []
    if not spec:
        return pairs
    for item in spec.split(","):
        item = item.strip()
        if not item:
            continue
        loop_no = ""
        if ":" in item:
            loop_no, item = item.split(":", 1)
            loop_no = loop_no.strip()
        if "->" not in item:
            raise RuntimeError(f"invalid pair item {item!r}; expected curr->hist or no:curr->hist")
        curr, hist = item.split("->", 1)
        pairs.append({"loop_no": loop_no, "curr": curr.strip(), "hist": hist.strip()})
    return pairs


def read_keyframes(path: Path, ids):
    ids = {str(i) for i in ids}
    rows = {}
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("kf_id") in ids:
                rows[row["kf_id"]] = row
    missing = ids - set(rows.keys())
    if missing:
        raise RuntimeError(f"missing keyframes in {path}: {sorted(missing)}")
    return rows


def to_float(row, key):
    return float(row[key])


def yaw_to_rot(yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def normalize_deg(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def relative_pose_2d(hist_row, curr_row):
    hist_xy = np.array([to_float(hist_row, "x"), to_float(hist_row, "y")], dtype=np.float64)
    curr_xy = np.array([to_float(curr_row, "x"), to_float(curr_row, "y")], dtype=np.float64)
    hist_yaw = math.radians(to_float(hist_row, "yaw_deg"))
    curr_yaw = math.radians(to_float(curr_row, "yaw_deg"))
    rel_xy = yaw_to_rot(-hist_yaw) @ (curr_xy - hist_xy)
    rel_yaw = normalize_deg(math.degrees(curr_yaw - hist_yaw))
    return rel_xy[0], rel_xy[1], rel_yaw


def transform_xy(points, dx, dy, yaw_deg):
    xy = points[:, :2]
    rot = yaw_to_rot(math.radians(yaw_deg))
    out = xy @ rot.T
    out[:, 0] += dx
    out[:, 1] += dy
    return out


def nearest_livox_msg(bag_or_path, topic, stamp, window_sec):
    start = rospy.Time.from_sec(stamp - window_sec)
    end = rospy.Time.from_sec(stamp + window_sec)
    best = None
    best_dt = None
    close_after = False
    if hasattr(bag_or_path, "read_messages"):
        bag = bag_or_path
    else:
        bag = rosbag.Bag(str(bag_or_path), "r")
        close_after = True
    try:
        for _, msg, t in bag.read_messages(topics=[topic], start_time=start, end_time=end):
            msg_stamp = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            dt = abs(msg_stamp - stamp)
            if best is None or dt < best_dt:
                best = msg
                best_dt = dt
    finally:
        if close_after:
            bag.close()
    if best is None:
        raise RuntimeError(f"no {topic} message near {stamp:.9f} within +/-{window_sec}s")
    return best, best_dt


def livox_points(msg, point_filter_num, blind, height_min, height_max):
    pts = []
    blind2 = blind * blind
    points = msg.points
    for i in range(1, max(1, msg.point_num - 1)):
        if i % point_filter_num != 0:
            continue
        p = points[i]
        x, y, z = float(p.x), float(p.y), float(p.z)
        if z < height_min or z > height_max:
            continue
        prev = points[i - 1]
        if abs(x - float(prev.x)) <= 1e-7 and abs(y - float(prev.y)) <= 1e-7 and abs(z - float(prev.z)) <= 1e-7:
            continue
        if x * x + y * y + z * z <= blind2:
            continue
        pts.append((x, y, z))
    if not pts:
        raise RuntimeError(f"empty filtered cloud at stamp {msg.header.stamp.to_sec():.9f}")
    return np.asarray(pts, dtype=np.float64)


def sample_points(xy, max_points):
    if len(xy) <= max_points:
        return xy
    idx = np.linspace(0, len(xy) - 1, max_points, dtype=np.int64)
    return xy[idx]


def plot_panel(ax, hist_xy, curr_xy, title, hist_label, curr_label):
    hist_xy = sample_points(hist_xy, 60000)
    curr_xy = sample_points(curr_xy, 60000)
    ax.scatter(hist_xy[:, 0], hist_xy[:, 1], s=0.25, c="#555555", alpha=0.45, label=hist_label)
    ax.scatter(curr_xy[:, 0], curr_xy[:, 1], s=0.35, c="#d62728", alpha=0.55, label=curr_label)
    ax.scatter([0.0], [0.0], s=34, c="#111111", marker="x", label="hist origin")
    ax.set_title(title)
    ax.set_xlabel("x in hist frame [m]")
    ax.set_ylabel("y in hist frame [m]")
    ax.axis("equal")
    ax.grid(True, linewidth=0.35, alpha=0.35)
    ax.legend(markerscale=8, frameon=True, fontsize=8, loc="best")


def format_float(row, key, fmt=".3f", default="N/A"):
    if not row or key not in row or row[key] == "":
        return default
    try:
        return format(float(row[key]), fmt)
    except (TypeError, ValueError):
        return default


def load_common_inputs(args):
    result_dir = Path(args.result_dir)
    loop_dir = result_dir / "evaluation" / "loop_debug"
    config_path = result_dir / "config" / "effective_config.yaml"
    with config_path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}

    if args.bag:
        bag_path = Path(args.bag)
    else:
        import json

        with (loop_dir / "run_metadata.json").open("r", encoding="utf-8") as f:
            bag_path = Path(json.load(f)["bag_path"])
    topic = config.get("common", {}).get("livox_lidar_topic", "/livox/lidar")
    fasterlio = config.get("fasterlio", {})
    roi = config.get("roi", {})
    point_filter_num = int(fasterlio.get("point_filter_num", 1))
    blind = float(fasterlio.get("blind", 0.0))
    height_min = float(roi.get("height_min", -float("inf")))
    height_max = float(roi.get("height_max", float("inf")))
    loop_events = read_loop_events(result_dir / "evaluation" / "slam_evaluation.csv")
    return {
        "result_dir": result_dir,
        "loop_dir": loop_dir,
        "bag_path": bag_path,
        "topic": topic,
        "point_filter_num": point_filter_num,
        "blind": blind,
        "height_min": height_min,
        "height_max": height_max,
        "loop_events": loop_events,
    }


def plot_one(common, curr_id, hist_id, out_path, time_window_sec, loop_no_hint=""):
    loop_dir = common["loop_dir"]
    hist_id = str(hist_id)
    curr_id = str(curr_id)
    keyframes = read_keyframes(loop_dir / "keyframes.csv", [hist_id, curr_id])
    hist_kf = keyframes[hist_id]
    curr_kf = keyframes[curr_id]
    match = read_csv_row(loop_dir / "loop_matches.csv", curr_kf_id=curr_id, hist_kf_id=hist_id)
    edge = read_csv_row_or_none(loop_dir / "loop_edges.csv", curr_kf_id=curr_id, hist_kf_id=hist_id)
    impact = read_csv_row_or_none(loop_dir / "loop_pgo_impact.csv", curr_kf_id=curr_id, hist_kf_id=hist_id)
    init_debug = read_csv_row_or_none(loop_dir / "loop_init_to_ndt_debug.csv", curr_kf_id=curr_id, hist_kf_id=hist_id)
    event = common["loop_events"].get((curr_id, hist_id), {})

    hist_stamp = to_float(hist_kf, "stamp")
    curr_stamp = to_float(curr_kf, "stamp")
    bag_source = common.get("bag_handle") or common["bag_path"]
    hist_msg, hist_dt = nearest_livox_msg(bag_source, common["topic"], hist_stamp, time_window_sec)
    curr_msg, curr_dt = nearest_livox_msg(bag_source, common["topic"], curr_stamp, time_window_sec)
    hist_cloud = livox_points(
        hist_msg, common["point_filter_num"], common["blind"], common["height_min"], common["height_max"]
    )
    curr_cloud = livox_points(
        curr_msg, common["point_filter_num"], common["blind"], common["height_min"], common["height_max"]
    )

    init_dx, init_dy, init_yaw = relative_pose_2d(hist_kf, curr_kf)
    if edge:
        ndt_dx = float(edge["dx"])
        ndt_dy = float(edge["dy"])
        ndt_yaw = float(edge["dyaw_deg"])
        transform_source = "loop_edges"
    else:
        ndt_dx = float(match["result_dx"])
        ndt_dy = float(match["result_dy"])
        ndt_yaw = float(match["result_dyaw_deg"])
        transform_source = "loop_matches"

    hist_xy = hist_cloud[:, :2]
    curr_before = transform_xy(curr_cloud, init_dx, init_dy, init_yaw)
    curr_after = transform_xy(curr_cloud, ndt_dx, ndt_dy, ndt_yaw)

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(16, 8), constrained_layout=True)
    plot_panel(
        axes[0],
        hist_xy,
        curr_before,
        f"Before NDT: T_init dx={init_dx:.3f}, dy={init_dy:.3f}, yaw={init_yaw:.2f} deg",
        f"hist {hist_id}",
        f"curr {curr_id}",
    )
    plot_panel(
        axes[1],
        hist_xy,
        curr_after,
        f"After NDT ({transform_source}): dx={ndt_dx:.3f}, dy={ndt_dy:.3f}, yaw={ndt_yaw:.2f} deg",
        f"hist {hist_id}",
        f"curr {curr_id}",
    )
    title_loop_no = loop_no_hint or str(event.get("loop_no", ""))
    title_prefix = f"Loop {title_loop_no}: " if title_loop_no else "Loop "
    status = event.get("status", "unknown")
    reason = event.get("reason", "")
    chi2_text = format_float(edge, "loop_chi2")
    max_delta_text = format_float(impact, "max_pose_delta_near_loop_m")
    straight_text = format_float(impact, "local_straightness_delta")
    init_xy_text = format_float(init_debug, "init_to_ndt_xy")
    init_yaw_text = format_float(init_debug, "init_to_ndt_yaw_deg")
    init_z_text = format_float(init_debug, "init_to_ndt_z")
    fig.suptitle(
        f"{title_prefix}{curr_id} -> {hist_id} BEV alignment, status={status}, reason={reason}\n"
        f"bag dt hist={hist_dt:.4f}s curr={curr_dt:.4f}s, "
        f"points hist={len(hist_cloud)} curr={len(curr_cloud)}, "
        f"ndt_score={float(match['ndt_score']):.3f}, "
        f"correction={float(match['correction_trans_m']):.3f}m/"
        f"{float(match['correction_yaw_deg']):.3f}deg, "
        f"init_to_ndt={init_xy_text}m/{init_yaw_text}deg/{init_z_text}m, "
        f"chi2={chi2_text}, max_delta={max_delta_text}m, straight_delta={straight_text}m",
        fontsize=12,
    )
    fig.savefig(out_path, dpi=220)
    plt.close(fig)
    print(f"output={out_path}")
    print(f"bag={common['bag_path']}")
    print(f"topic={common['topic']}")
    print(f"hist_stamp={hist_stamp:.9f} nearest_dt={hist_dt:.6f} points={len(hist_cloud)}")
    print(f"curr_stamp={curr_stamp:.9f} nearest_dt={curr_dt:.6f} points={len(curr_cloud)}")
    print(f"T_init dx={init_dx:.9f} dy={init_dy:.9f} yaw={init_yaw:.9f}")
    print(f"T_ndt dx={ndt_dx:.9f} dy={ndt_dy:.9f} yaw={ndt_yaw:.9f}")
    return {
        "loop_no": title_loop_no,
        "curr_kf_id": curr_id,
        "hist_kf_id": hist_id,
        "status": status,
        "reason": reason,
        "ndt_score": match.get("ndt_score", ""),
        "init_to_ndt_xy": init_debug.get("init_to_ndt_xy", "") if init_debug else "",
        "init_to_ndt_yaw_deg": init_debug.get("init_to_ndt_yaw_deg", "") if init_debug else "",
        "init_to_ndt_z": init_debug.get("init_to_ndt_z", "") if init_debug else "",
        "chi2": edge.get("loop_chi2", "") if edge else "",
        "max_delta": impact.get("max_pose_delta_near_loop_m", "") if impact else "",
        "local_straightness_delta": impact.get("local_straightness_delta", "") if impact else "",
        "transform_source": transform_source,
        "output_png": str(out_path),
    }


def main():
    parser = argparse.ArgumentParser(description="Plot before/after NDT BEV alignment for loop pairs.")
    parser.add_argument("--result_dir", required=True)
    parser.add_argument("--bag", default="")
    parser.add_argument("--hist", type=int)
    parser.add_argument("--curr", type=int)
    parser.add_argument("--pairs", default="", help="Comma list: curr->hist or loop_no:curr->hist")
    parser.add_argument("--output", default="")
    parser.add_argument("--output_dir", default="")
    parser.add_argument("--time_window_sec", type=float, default=0.25)
    args = parser.parse_args()

    common = load_common_inputs(args)
    pairs = parse_pairs(args.pairs)
    if args.curr is not None or args.hist is not None:
        if args.curr is None or args.hist is None:
            raise RuntimeError("--curr and --hist must be provided together")
        pairs.insert(0, {"loop_no": "", "curr": str(args.curr), "hist": str(args.hist)})
    if not pairs:
        raise RuntimeError("provide --curr/--hist or --pairs")

    rows = []
    output_dir = Path(args.output_dir) if args.output_dir else common["loop_dir"] / "alignment_bev_single_frame"

    def run_batch():
        for item in pairs:
            curr = item["curr"]
            hist = item["hist"]
            loop_no = item.get("loop_no", "")
            if args.output and len(pairs) == 1:
                out_path = Path(args.output)
            else:
                prefix = f"loop_{loop_no}_" if loop_no else "loop_"
                out_path = output_dir / f"{prefix}{curr}_{hist}_alignment_bev.png"
            rows.append(plot_one(common, curr, hist, out_path, args.time_window_sec, loop_no))

    if len(pairs) > 1:
        with rosbag.Bag(str(common["bag_path"]), "r") as bag:
            common["bag_handle"] = bag
            run_batch()
            common.pop("bag_handle", None)
    else:
        run_batch()

    if len(rows) > 1:
        output_dir.mkdir(parents=True, exist_ok=True)
        index_path = output_dir / "alignment_bev_index.csv"
        with index_path.open("w", newline="") as f:
            fieldnames = [
                "loop_no",
                "curr_kf_id",
                "hist_kf_id",
                "status",
                "reason",
                "ndt_score",
                "init_to_ndt_xy",
                "init_to_ndt_yaw_deg",
                "init_to_ndt_z",
                "chi2",
                "max_delta",
                "local_straightness_delta",
                "transform_source",
                "output_png",
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"index={index_path}")


if __name__ == "__main__":
    main()
