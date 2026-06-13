#!/usr/bin/env python3

import argparse
import os
import shutil
from pathlib import Path

import rosbag


def merge_bags(input_bags, output_bag):
    output_path = Path(output_bag)
    if output_path.exists():
        if output_path.is_dir():
            shutil.rmtree(output_path)
        else:
            output_path.unlink()

    with rosbag.Bag(str(output_path), "w") as writer:
        for input_bag in input_bags:
            print("merging", input_bag)
            with rosbag.Bag(input_bag, "r") as reader:
                for topic, msg, stamp in reader.read_messages():
                    writer.write(topic, msg, stamp)

    print("merge finished:", output_bag)


def find_ros1_bags(root_folder):
    root = Path(root_folder)
    return sorted(str(path) for path in root.rglob("*.bag"))


def main():
    parser = argparse.ArgumentParser(description="Merge ROS1 .bag files.")
    parser.add_argument("--output", "-o", required=True, help="Output ROS1 bag path.")
    parser.add_argument("inputs", nargs="*", help="Input ROS1 bag files or folders.")
    args = parser.parse_args()

    input_bags = []
    for item in args.inputs:
        if os.path.isdir(item):
            input_bags.extend(find_ros1_bags(item))
        else:
            input_bags.append(item)

    if not input_bags:
        raise RuntimeError("no input .bag files")

    merge_bags(input_bags, args.output)


if __name__ == "__main__":
    main()
