"""Measure abrupt braking error changes in saved AutoTest outbound CSVs.

For consecutive samples in sustained braking, report the largest change in
(measured speed - target speed), normalized to a 20 ms control interval. This
separates abrupt braking dips from the intended fall in target velocity.
"""

import argparse
import csv
from pathlib import Path


def measure(directory):
    files = list(directory.glob("auto-test-outbound-velocity_*.csv"))
    if len(files) != 1:
        raise ValueError(f"Expected one outbound CSV in {directory}; found {len(files)}")
    with files[0].open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    changes = []
    for before, after in zip(rows, rows[1:]):
        if max(float(before["target_accel_in_s2"]),
               float(after["target_accel_in_s2"])) > -20.0:
            continue
        if min(float(before["controller_target_in_s"]),
               float(after["controller_target_in_s"])) <= 6.0:
            continue
        dt = float(after["elapsed_s"]) - float(before["elapsed_s"])
        if not 0.012 <= dt <= 0.055:
            continue
        error_before = (float(before["controller_measured_in_s"])
                        - float(before["controller_target_in_s"]))
        error_after = (float(after["controller_measured_in_s"])
                       - float(after["controller_target_in_s"]))
        changes.append(((error_after - error_before) * 0.020 / dt,
                        float(after["elapsed_s"])))
    if not changes:
        raise ValueError(f"No sustained braking pairs in {files[0]}")
    worst_drop = min(changes)
    print(f"{directory.name}: pairs={len(changes)} "
          f"worst_drop={worst_drop[0]:+.2f} in/s per 20 ms "
          f"at={worst_drop[1]:.2f}s "
          f"worst_rebound={max(change for change, _ in changes):+.2f} "
          f"drops_over_3={sum(change < -3 for change, _ in changes)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="+", type=Path)
    args = parser.parse_args()
    for directory in args.directory:
        measure(directory)


if __name__ == "__main__":
    main()
