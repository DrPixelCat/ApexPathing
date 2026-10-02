"""Locate opposing-motor commands and speed drops in an AutoTest velocity CSV.

Usage: python tools/analyze_autotest_braking.py path/to/auto-test-outbound-velocity.csv
Reports the first opposing command in each profiled braking interval and the
target/actual speed change over the following 0.08 s. This is a timing
diagnostic, not proof that the command alone caused the speed change.
"""

import argparse
import csv
from pathlib import Path


def load(path):
    fields = ("elapsed_s", "target_accel_in_s2", "controller_target_in_s",
              "controller_measured_in_s", "raw_velocity_in_s",
              "left_front_command", "right_front_command")
    with path.open(newline="") as stream:
        rows = [{key: float(row[key]) for key in fields} for row in csv.DictReader(stream)]
    if not rows:
        raise ValueError("Empty velocity CSV")
    return rows


def braking_intervals(rows):
    intervals = []
    start = None
    for index, row in enumerate(rows):
        braking = row["target_accel_in_s2"] < -5 and row["controller_target_in_s"] > 0
        if braking and start is None:
            start = index
        elif not braking and start is not None:
            intervals.append((start, index - 1))
            start = None
    if start is not None:
        intervals.append((start, len(rows) - 1))
    return intervals


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    rows = load(args.csv)
    intervals = [(start, end) for start, end in braking_intervals(rows)
                 if rows[end]["elapsed_s"] - rows[start]["elapsed_s"] >= .30]
    if not intervals:
        raise ValueError("No braking intervals at least 0.30 s long")
    print("interval start_s first_opposition_s duty_left duty_right "
          "target_drop_80ms filtered_drop_80ms raw_drop_80ms "
          "negative_samples total_samples")
    for number, (start, end) in enumerate(intervals, 1):
        negative = [index for index in range(start, end + 1)
                    if rows[index]["left_front_command"] < -.001
                    and rows[index]["right_front_command"] < -.001]
        if not negative:
            print(f"{number} {rows[start]['elapsed_s']:.3f} none -- -- -- -- -- 0 "
                  f"{end - start + 1}")
            continue
        first = negative[0]
        event = rows[first]
        horizon = event["elapsed_s"] + .08
        later = next((row for row in rows[first + 1:end + 1]
                      if row["elapsed_s"] >= horizon), None)
        if later is None:
            print(f"{number} {rows[start]['elapsed_s']:.3f} "
                  f"{event['elapsed_s']:.3f} {event['left_front_command']:+.3f} "
                  f"{event['right_front_command']:+.3f} -- -- -- "
                  f"{len(negative)} {end - start + 1}")
            continue
        print(f"{number} {rows[start]['elapsed_s']:.3f} "
              f"{event['elapsed_s']:.3f} {event['left_front_command']:+.3f} "
              f"{event['right_front_command']:+.3f} "
              f"{later['controller_target_in_s'] - event['controller_target_in_s']:+.1f} "
              f"{later['controller_measured_in_s'] - event['controller_measured_in_s']:+.1f} "
              f"{later['raw_velocity_in_s'] - event['raw_velocity_in_s']:+.1f} "
              f"{len(negative)} {end - start + 1}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError) as error:
        raise SystemExit(str(error)) from error
