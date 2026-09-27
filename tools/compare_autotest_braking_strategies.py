"""Compare AutoTest strategy CSVs at the same one-inch path positions.

Usage: python tools/compare_autotest_braking_strategies.py BASELINE_DIR OTHER_DIR ...
The first run defines which path positions are accelerating or braking above
or below its ordinary kA command boundary. Each run's error is interpolated
onto the same distance grid before RMSE and bias are calculated.
"""

import argparse
import bisect
import csv
import math
from pathlib import Path


KS = .004891713679570547
KV = .006241447491199932
KA = .007276265254694199


def load(directory):
    files = list(directory.glob("auto-test-outbound-velocity_*.csv"))
    if len(files) != 1:
        raise ValueError(f"Expected one velocity CSV in {directory}, found {len(files)}")
    fields = ("elapsed_s", "path_distance_in", "controller_target_in_s",
              "controller_measured_in_s", "target_accel_in_s2",
              "left_front_command", "right_front_command")
    with files[0].open(newline="") as stream:
        rows = [{key: float(row[key]) for key in fields} for row in csv.DictReader(stream)]
    if len(rows) < 2:
        raise ValueError(f"Too few velocity rows in {files[0]}")
    rows.sort(key=lambda row: row["path_distance_in"])
    return rows


def at_distance(rows, distances, distance):
    index = bisect.bisect_left(distances, distance)
    if index == 0:
        return rows[0]
    if index == len(rows):
        return rows[-1]
    lower, upper = rows[index - 1], rows[index]
    span = upper["path_distance_in"] - lower["path_distance_in"]
    fraction = (distance - lower["path_distance_in"]) / span if span > 0 else 0.0
    return {key: lower[key] + fraction * (upper[key] - lower[key]) for key in lower}


def classify(row):
    target = row["controller_target_in_s"]
    measured = row["controller_measured_in_s"]
    acceleration = row["target_accel_in_s2"]
    if target <= 0:
        return None
    if acceleration > 5:
        return "accel"
    if acceleration >= -5:
        return None
    scale = min(1.0, max(0.0, measured / target))
    ordinary_power = KS + KV * target + KA * acceleration * scale * scale
    return "brake_above" if ordinary_power > 0 else "brake_below"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="+", type=Path)
    args = parser.parse_args()
    runs = [(directory, load(directory)) for directory in args.directory]
    max_distance = min(rows[-1]["path_distance_in"] for _, rows in runs)
    grid = range(1, math.floor(max_distance) + 1)
    reference = runs[0][1]
    reference_distances = [row["path_distance_in"] for row in reference]
    labels = {distance: classify(at_distance(reference, reference_distances, distance))
              for distance in grid}
    print("strategy outbound_s accel_rmse brake_rmse brake_peak_over brake_peak_under "
          "above_n above_rmse above_bias below_n below_rmse below_bias "
          "opposing_pct sign_switches")
    for directory, rows in runs:
        distances = [row["path_distance_in"] for row in rows]
        groups = {"accel": [], "brake_above": [], "brake_below": []}
        for distance in grid:
            label = labels[distance]
            if label is None:
                continue
            row = at_distance(rows, distances, distance)
            groups[label].append(row["controller_measured_in_s"]
                                 - row["controller_target_in_s"])

        def rmse(errors):
            return math.sqrt(sum(error * error for error in errors) / len(errors))

        above, below = groups["brake_above"], groups["brake_below"]
        braking = above + below
        braking_rows = [row for row in rows if row["target_accel_in_s2"] < -5
                        and row["controller_target_in_s"] > 0]
        signs = []
        opposing = 0
        for row in braking_rows:
            command = (row["left_front_command"] + row["right_front_command"]) / 2
            sign = 1 if command > .001 else -1 if command < -.001 else 0
            signs.append(sign)
            if sign < 0:
                opposing += 1
        switches = sum(a != b for a, b in zip(signs, signs[1:]))
        print(f"{directory.name} {max(row['elapsed_s'] for row in rows):.2f} "
              f"{rmse(groups['accel']):.2f} {rmse(braking):.2f} "
              f"{max(braking):+.2f} {min(braking):+.2f} "
              f"{len(above)} {rmse(above):.2f} {sum(above)/len(above):+.2f} "
              f"{len(below)} {rmse(below):.2f} {sum(below)/len(below):+.2f} "
              f"{100*opposing/len(braking_rows):.1f} {switches}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, ZeroDivisionError) as error:
        raise SystemExit(str(error)) from error
