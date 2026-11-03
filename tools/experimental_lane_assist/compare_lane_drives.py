#!/usr/bin/env python3
"""Print comparable lane-keeping metrics for one or more lane_drive.py output folders."""
import argparse
import csv
import math
import statistics
from pathlib import Path


def metrics(folder):
    path = folder / "drive.csv"
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"{path} contains no drive samples")
    moving = [row for row in rows if float(row["t"]) > 3 and float(row["speed"]) > 0.5]
    if not moving:
        raise ValueError(f"{path} contains no moving samples after the 3 s warm-up")
    lateral = [float(row["true_offset"]) for row in moving]
    valid = [row for row in moving if row["valid"] == "1"]
    stopped = [row for row in moving if row["state"].startswith("stopped")]
    controller = rows[0].get("controller", folder.name)
    return {
        "controller": controller,
        "samples": len(moving),
        "rms": math.sqrt(statistics.fmean(value * value for value in lateral)),
        "max": max(abs(value) for value in lateral),
        "departures": 100 * sum(abs(value) > 1.5 for value in lateral) / len(lateral),
        "valid": 100 * len(valid) / len(moving),
        "stopped": 100 * len(stopped) / len(moving),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path, help="lane_drive.py output folders")
    args = parser.parse_args()
    results = [metrics(folder) for folder in args.runs]
    print(f"{'controller':<16} {'samples':>8} {'RMS m':>8} {'max m':>8} "
          f"{'depart %':>10} {'valid %':>9} {'stopped %':>11}")
    for row in results:
        print(f"{row['controller']:<16} {row['samples']:>8d} {row['rms']:>8.3f} {row['max']:>8.3f} "
              f"{row['departures']:>10.1f} {row['valid']:>9.1f} {row['stopped']:>11.1f}")


if __name__ == "__main__":
    main()
