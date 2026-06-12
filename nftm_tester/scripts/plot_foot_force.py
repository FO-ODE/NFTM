#!/usr/bin/env python3

import argparse
import math
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib.pyplot as plt


FOOT_LABELS = ["FL", "FR", "RL", "RR"]


def read_list_values(lines, start_index, count, value_type=float):
    values = []
    cursor = start_index + 1
    while cursor < len(lines) and len(values) < count:
        line = lines[cursor].strip()
        if line.startswith("-"):
            values.append(value_type(line[1:].strip()))
        elif values:
            break
        cursor += 1
    return values, cursor


def parse_lowstate_log(log_path: Path):
    foot_force_samples = []
    accel_magnitudes = []
    lines = log_path.read_text(errors="ignore").splitlines()

    index = 0
    current_accel_magnitude = None
    while index < len(lines):
        line = lines[index].strip()

        if line == "accelerometer:":
            values, cursor = read_list_values(lines, index, 3, float)
            if len(values) == 3:
                current_accel_magnitude = math.sqrt(sum(value * value for value in values))
                index = cursor
                continue

        if line == "foot_force:":
            values, cursor = read_list_values(lines, index, 4, int)
            if len(values) == 4:
                foot_force_samples.append(values)
                accel_magnitudes.append(current_accel_magnitude)
                current_accel_magnitude = None
                index = cursor
                continue

        index += 1

    return foot_force_samples, accel_magnitudes


def select_samples(samples, interval_s: float, duration_s=None, tail_s=None):
    if duration_s is not None and tail_s is not None:
        raise RuntimeError("Use either --duration or --tail, not both.")

    if duration_s is not None:
        sample_count = max(1, int(duration_s / interval_s))
        return samples[:sample_count], 0

    if tail_s is not None:
        sample_count = max(1, int(tail_s / interval_s))
        start_index = max(0, len(samples) - sample_count)
        return samples[start_index:], start_index

    return samples, 0


def scale_accel_to_force(accel_magnitudes, force_samples):
    valid_accel = [value for value in accel_magnitudes if value is not None]
    if not valid_accel:
        return None

    max_accel = max(valid_accel)
    max_force = max(max(sample) for sample in force_samples)
    if max_accel <= 0.0 or max_force <= 0.0:
        return 1.0

    return 0.8 * max_force / max_accel


def plot_foot_force(
    foot_force_samples,
    accel_magnitudes,
    interval_s: float,
    output_path: Path,
    duration_s=None,
    tail_s=None,
    accel_scale=None,
):
    if not foot_force_samples:
        raise RuntimeError("No foot_force samples found.")

    foot_force_samples, start_index = select_samples(foot_force_samples, interval_s, duration_s, tail_s)
    accel_magnitudes = accel_magnitudes[start_index:start_index + len(foot_force_samples)]

    times = [(start_index + i) * interval_s for i in range(len(foot_force_samples))]
    series = list(zip(*foot_force_samples))

    plt.figure(figsize=(12, 6))
    for label, values in zip(FOOT_LABELS, series):
        plt.plot(times, values, label=label, linewidth=1.2)

    if any(value is not None for value in accel_magnitudes):
        if accel_scale is None:
            accel_scale = scale_accel_to_force(accel_magnitudes, foot_force_samples)
        scaled_accel = [
            value * accel_scale if value is not None else float("nan")
            for value in accel_magnitudes
        ]
        plt.plot(
            times,
            scaled_accel,
            label=f"|accel| x {accel_scale:.2f}",
            color="black",
            linestyle="--",
            linewidth=1.4,
        )

    plt.xlabel("Time (s)")
    plt.ylabel("Foot force / scaled accel")
    plt.title("GO2 Foot Force and Accelerometer Magnitude")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=160)


def main():
    parser = argparse.ArgumentParser(description="Plot GO2 foot_force values from a ros2 topic echo log.")
    parser.add_argument(
        "log",
        nargs="?",
        default="/home/zby/Rosbag/lowstate2.log",
        help="Path to lowstate log file.",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="/home/zby/ROS2/foot_force.png",
        help="Output plot image path.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.002,
        help="Sample interval in seconds. Default: 0.002",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Only plot the first N seconds.",
    )
    parser.add_argument(
        "--tail",
        type=float,
        default=None,
        help="Only plot the last N seconds.",
    )
    parser.add_argument(
        "--accel-scale",
        type=float,
        default=None,
        help="Scale factor for sqrt(ax^2 + ay^2 + az^2). Default: auto-scale.",
    )
    args = parser.parse_args()

    log_path = Path(args.log)
    output_path = Path(args.output)

    foot_force_samples, accel_magnitudes = parse_lowstate_log(log_path)
    plot_foot_force(
        foot_force_samples,
        accel_magnitudes,
        args.interval,
        output_path,
        args.duration,
        args.tail,
        args.accel_scale,
    )

    plotted_samples, start_index = select_samples(foot_force_samples, args.interval, args.duration, args.tail)
    duration = (len(plotted_samples) - 1) * args.interval if plotted_samples else 0.0
    start_time = start_index * args.interval
    end_time = start_time + duration
    print(f"Parsed {len(foot_force_samples)} samples from {log_path}")
    print(f"Parsed {sum(value is not None for value in accel_magnitudes)} accelerometer samples")
    print(f"Plotted {len(plotted_samples)} samples")
    print(f"Time range: {start_time:.3f} s to {end_time:.3f} s")
    print(f"Saved plot to {output_path}")


if __name__ == "__main__":
    main()
