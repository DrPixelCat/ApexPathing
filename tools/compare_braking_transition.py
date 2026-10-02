"""Compare early raw-speed response around zero motor command.

Usage: python tools/compare_braking_transition.py --robot robot.csv --sim sim/*.csv
Positive brake_power opposes travel; negative continues driving. Each slope is
a least-squares fit of raw speed against time over the first 0.12 s after
BRAKE_START. Units are in/s^2; more negative means stronger braking.
"""

import argparse
import csv
import math
from pathlib import Path


def load_responses(paths):
    responses = []
    for path in paths:
        groups = {}
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                if row.get("phase") not in ("BRAKE_START", "BRAKE", "STOP", "PROBE_END"):
                    continue
                try:
                    key = row["trial"]
                    sample = (float(row["time_s"]), float(row["raw_speed_in_s"]))
                    speed = float(row["brake_start_speed_in_s"])
                    direction = int(row["direction"])
                    power = float(row["brake_power"])
                except (KeyError, ValueError) as error:
                    raise ValueError(f"Bad braking row in {path}: {error}") from error
                if not all(math.isfinite(v) for v in (*sample, speed, power)):
                    raise ValueError(f"Nonfinite braking row in {path}, trial {key}")
                group = groups.setdefault(key, dict(file=path.name, trial=key,
                                                     speed=speed, direction=direction,
                                                     power=power, samples=[]))
                if row["phase"] in ("BRAKE_START", "BRAKE") and 0 <= sample[0] <= .12:
                    group["samples"].append(sample)
        for group in groups.values():
            samples = group.pop("samples")
            if len(samples) < 3 or max(t for t, _ in samples) - min(t for t, _ in samples) < .06:
                continue
            mean_t = sum(t for t, _ in samples) / len(samples)
            mean_v = sum(v for _, v in samples) / len(samples)
            variance = sum((t - mean_t) ** 2 for t, _ in samples)
            group["acceleration"] = sum((t - mean_t) * (v - mean_v)
                                        for t, v in samples) / variance
            group["n"] = len(samples)
            responses.append(group)
    if not responses:
        raise ValueError("No braking trials with at least 0.06 s of early samples")
    return responses


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", nargs="+", type=Path, required=True)
    parser.add_argument("--sim", nargs="+", type=Path, required=True)
    parser.add_argument("--speed-tolerance", type=float, default=3.0)
    args = parser.parse_args()
    if args.speed_tolerance <= 0:
        parser.error("--speed-tolerance must be positive")
    robot, sim = load_responses(args.robot), load_responses(args.sim)
    print("power dir robot_speed sim_speed robot_accel sim_accel accel_error")
    matches = 0
    for measured in robot:
        candidates = [row for row in sim
                      if row["direction"] == measured["direction"]
                      and abs(row["power"] - measured["power"]) < 1e-6]
        if not candidates:
            print(f"Unmatched {measured['file']}:{measured['trial']}: direction/power")
            continue
        modeled = min(candidates, key=lambda row: abs(row["speed"] - measured["speed"]))
        if abs(modeled["speed"] - measured["speed"]) > args.speed_tolerance:
            print(f"Unmatched {measured['file']}:{measured['trial']}: start speed")
            continue
        matches += 1
        print(f"{measured['power']:+.3f} {measured['direction']:+d} "
              f"{measured['speed']:.2f} {modeled['speed']:.2f} "
              f"{measured['acceleration']:+.1f} {modeled['acceleration']:+.1f} "
              f"{modeled['acceleration'] - measured['acceleration']:+.1f}")
    if not matches:
        raise ValueError("No matched early braking trials")
    print("Early slopes are diagnostic estimates, sensitive to velocity noise and sample timing.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as error:
        raise SystemExit(str(error)) from error
