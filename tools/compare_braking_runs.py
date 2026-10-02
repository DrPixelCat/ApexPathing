"""Compare robot and FTCodeSim Brake Characterization stops.

Usage: python tools/compare_braking_runs.py --robot robot.csv --sim sim/*.csv
Matches each robot stop to the nearest simulation start speed at the same
direction and brake power. Reports stopping-distance and braking-time error.
"""

import argparse
import csv
import math
from pathlib import Path


def load_stops(paths):
    stops = []
    for path in paths:
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                if row.get("phase") != "STOP":
                    continue
                try:
                    stop = dict(file=path.name, trial=row["trial"],
                                direction=int(row["direction"]),
                                power=float(row["brake_power"]),
                                speed=float(row["brake_start_speed_in_s"]),
                                distance=float(row["brake_distance_in"]),
                                time=float(row["time_s"]),
                                lateral=float(row["lateral_in"]),
                                heading=float(row["heading_change_deg"]),
                                final_speed=float(row["raw_speed_in_s"]))
                except (KeyError, ValueError) as error:
                    raise ValueError(f"Bad STOP row in {path}: {error}") from error
                values = (stop[k] for k in ("power", "speed", "distance", "time",
                                             "lateral", "heading", "final_speed"))
                if not all(math.isfinite(v) for v in values):
                    raise ValueError(f"Nonfinite STOP row in {path}, trial {stop['trial']}")
                if stop["speed"] <= 0 or stop["distance"] < 0 or stop["time"] <= 0:
                    raise ValueError(f"Invalid STOP row in {path}, trial {stop['trial']}")
                stops.append(stop)
    if not stops:
        raise ValueError("No completed stops found")
    return stops


def compare(robot, sim, speed_tolerance):
    matches = []
    unmatched = []
    for measured in robot:
        candidates = [row for row in sim
                      if row["direction"] == measured["direction"]
                      and abs(row["power"] - measured["power"]) < 1e-6]
        if not candidates:
            unmatched.append((measured, "no matching direction/brake power"))
            continue
        nearest = min(candidates, key=lambda row: abs(row["speed"] - measured["speed"]))
        speed_gap = abs(nearest["speed"] - measured["speed"])
        if speed_gap > speed_tolerance:
            unmatched.append((measured, f"start-speed gap {speed_gap:.2f} in/s"))
            continue
        matches.append((measured, nearest))
    return matches, unmatched


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", nargs="+", type=Path, required=True)
    parser.add_argument("--sim", nargs="+", type=Path, required=True)
    parser.add_argument("--speed-tolerance", type=float, default=3.0,
                        help="maximum difference in brake start speed (default: 3 in/s)")
    args = parser.parse_args()
    if args.speed_tolerance <= 0:
        parser.error("--speed-tolerance must be positive")
    robot, sim = load_stops(args.robot), load_stops(args.sim)
    matches, unmatched = compare(robot, sim, args.speed_tolerance)
    print("power dir robot_speed sim_speed robot_dist sim_dist dist_error robot_time sim_time time_error")
    for measured, modeled in matches:
        print(f"{measured['power']:.3f} {measured['direction']:+d} "
              f"{measured['speed']:.2f} {modeled['speed']:.2f} "
              f"{measured['distance']:.2f} {modeled['distance']:.2f} "
              f"{modeled['distance'] - measured['distance']:+.2f} "
              f"{measured['time']:.3f} {modeled['time']:.3f} "
              f"{modeled['time'] - measured['time']:+.3f}")
    for measured, reason in unmatched:
        print(f"Unmatched {measured['file']}:{measured['trial']}: {reason}")
    if not matches:
        raise ValueError("No matched trials; check conditions and start speeds")
    for power in sorted({measured["power"] for measured, _ in matches}):
        subset = [(r, s) for r, s in matches if r["power"] == power]
        distance_rmse = math.sqrt(sum((s["distance"] - r["distance"]) ** 2
                                      for r, s in subset) / len(subset))
        time_rmse = math.sqrt(sum((s["time"] - r["time"]) ** 2
                                  for r, s in subset) / len(subset))
        print(f"power={power:.3f}: n={len(subset)}, stop-distance RMSE="
              f"{distance_rmse:.2f} in, time RMSE={time_rmse:.3f} s")
    print("Errors include sensor and start-speed differences; inspect unmatched and drifted runs.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as error:
        raise SystemExit(str(error)) from error
