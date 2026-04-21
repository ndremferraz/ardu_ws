#!/usr/bin/env python3

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot position and orientation history from vins_history.csv."
    )
    parser.add_argument(
        "csv_file",
        nargs="?",
        default="vins_history.csv",
        help="Path to the CSV file exported by vins_mav_node.",
    )
    parser.add_argument(
        "--source",
        choices=["vins", "vicon"],
        help="Plot only one source instead of all sources in the CSV.",
    )
    return parser.parse_args()


def load_series(csv_path, requested_source=None):
    data = defaultdict(lambda: defaultdict(list))

    with csv_path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        required_columns = {
            "source",
            "stamp_sec",
            "stamp_nanosec",
            "x",
            "y",
            "z",
            "qx",
            "qy",
            "qz",
        }
        missing_columns = required_columns - set(reader.fieldnames or [])
        if missing_columns:
            missing_list = ", ".join(sorted(missing_columns))
            raise ValueError(f"Missing required CSV columns: {missing_list}")

        for row in reader:
            source = row["source"].strip()
            if requested_source and source != requested_source:
                continue

            timestamp = float(row["stamp_sec"]) + float(row["stamp_nanosec"]) * 1e-9
            data[source]["time"].append(timestamp)
            data[source]["x"].append(float(row["x"]))
            data[source]["y"].append(float(row["y"]))
            data[source]["z"].append(float(row["z"]))
            data[source]["qx"].append(float(row["qx"]))
            data[source]["qy"].append(float(row["qy"]))
            data[source]["qz"].append(float(row["qz"]))

    if not data:
        source_msg = f" for source '{requested_source}'" if requested_source else ""
        raise ValueError(f"No rows found in {csv_path}{source_msg}.")

    first_timestamp = min(series["time"][0] for series in data.values() if series["time"])
    for series in data.values():
        series["time"] = [timestamp - first_timestamp for timestamp in series["time"]]

    return data


def plot_series(data, csv_path):
    fig, axes = plt.subplots(6, 1, figsize=(12, 14), sharex=True)
    components = [
        ("x", "Position X"),
        ("y", "Position Y"),
        ("z", "Position Z"),
        ("qx", "Quaternion X"),
        ("qy", "Quaternion Y"),
        ("qz", "Quaternion Z"),
    ]

    for source, series in sorted(data.items()):
        time_values = series["time"]
        for axis, (component, title) in zip(axes, components):
            axis.plot(time_values, series[component], label=source)
            axis.set_title(title)
            axis.set_ylabel(component)
            axis.grid(True)
            axis.legend()

    axes[0].set_title(f"{axes[0].get_title()}: {csv_path.name}")
    axes[-1].set_xlabel("Time (s)")
    fig.tight_layout()
    plt.show()


def main():
    args = parse_args()
    csv_path = Path(args.csv_file).expanduser().resolve()

    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    data = load_series(csv_path, args.source)
    plot_series(data, csv_path)


if __name__ == "__main__":
    main()
