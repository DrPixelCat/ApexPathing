"""Summarize Apex Brake Characterization CSVs and fit FTCodeSim stopping distance.

Usage: python tools/fit_braking_model.py path/to/brake_characterization_*.csv
Only completed zero-power BRAKE trials enter the fit. Opposing-power trials are
reported separately because FTCodeSim uses the fitted drag for both regimes.
"""

import argparse
import csv
import math
from pathlib import Path


def fit_distance(samples):
    """Fit d(v) = linear*v + quadratic*v^2 with nonnegative coefficients."""
    if len(samples) < 3:
        raise ValueError("Need at least three completed zero-power stops")
    speeds = [v for v, _ in samples]
    if min(speeds) <= 0 or max(speeds) < 1.5 * min(speeds):
        raise ValueError("Need a wider range of starting speeds")
    s2 = sum(v * v for v, _ in samples)
    s3 = sum(v**3 for v, _ in samples)
    s4 = sum(v**4 for v, _ in samples)
    t1 = sum(v * d for v, d in samples)
    t2 = sum(v * v * d for v, d in samples)
    determinant = s2 * s4 - s3 * s3
    if determinant <= 1e-12 * s2 * s4:
        raise ValueError("Starting speeds are too similar for a two-term fit")
    candidates = [(max(0.0, t1 / s2), 0.0), (0.0, max(0.0, t2 / s4))]
    linear = (t1 * s4 - t2 * s3) / determinant
    quadratic = (s2 * t2 - s3 * t1) / determinant
    if linear >= 0 and quadratic >= 0:
        candidates.append((linear, quadratic))
    best = min(candidates, key=lambda c: sum(
        (d - c[0] * v - c[1] * v * v) ** 2 for v, d in samples))
    rmse = math.sqrt(sum((d - best[0] * v - best[1] * v * v) ** 2
                         for v, d in samples) / len(samples))
    return best[0], best[1], rmse


def load_trials(paths):
    trials = []
    for path in paths:
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                if row["phase"] != "STOP":
                    continue
                try:
                    trials.append({
                        "file": path.name,
                        "trial": row["trial"],
                        "direction": int(row["direction"]),
                        "power": float(row["brake_power"]),
                        "speed": float(row["brake_start_speed_in_s"]),
                        "distance": float(row["brake_distance_in"]),
                        "time": float(row["time_s"]),
                        "final_raw_speed": float(row["raw_speed_in_s"]),
                        "lateral": float(row["lateral_in"]),
                        "heading": float(row["heading_change_deg"]),
                    })
                except (ValueError, KeyError) as error:
                    raise ValueError(f"Bad STOP row in {path}: {error}") from error
    if not trials:
        raise ValueError("No completed trials found")
    for trial in trials:
        if not all(math.isfinite(trial[key]) for key in
                   ("power", "speed", "distance", "time", "final_raw_speed",
                    "lateral", "heading")):
            raise ValueError(f"Nonfinite measurement in {trial['file']} trial {trial['trial']}")
        if trial["speed"] <= 0 or trial["distance"] < 0:
            raise ValueError(f"Invalid stop in {trial['file']} trial {trial['trial']}")
    return trials


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", type=Path)
    args = parser.parse_args()
    trials = load_trials(args.csv)
    print("trial direction brake_power start_in_s stop_in time_s lateral_in heading_deg")
    for row in trials:
        print(f"{row['file']}:{row['trial']} {row['direction']:+d} "
              f"{row['power']:.3f} {row['speed']:.2f} {row['distance']:.2f} "
              f"{row['time']:.2f} {row['lateral']:.2f} {row['heading']:.2f}")
    coast = [row for row in trials if row["power"] == 0]
    drifted = [row for row in coast if abs(row["lateral"]) > 3.0
               or abs(row["heading"]) > 10.0 or row["final_raw_speed"] > 2.0]
    for row in drifted:
        print(f"Exclude {row['file']}:{row['trial']} from fit: "
              "lateral drift, heading change, or residual speed is too large")
    coast = [row for row in coast if row not in drifted]
    if len(coast) < 3:
        print("Collect at least three straight zero-power stops at different speeds to fit.")
        return
    if {row["direction"] for row in coast} != {-1, 1}:
        print("Only one direction has valid zero-power stops; repeat in the other direction.")
    samples = [(row["speed"], row["distance"]) for row in coast]
    linear, quadratic, rmse = fit_distance(samples)
    print(f"\nFTCodeSim candidate: linearBraking={linear:.9f}, "
          f"quadraticBraking={quadratic:.9f}")
    print(f"Zero-power stopping-distance RMSE: {rmse:.2f} in on {len(coast)} runs")
    print("Compare predicted and measured distances, then validate on held-out runs.")
    for row in coast:
        predicted = linear * row["speed"] + quadratic * row["speed"] ** 2
        print(f"  {row['speed']:.1f} in/s: measured {row['distance']:.2f} in, "
              f"predicted {predicted:.2f} in")
    for direction in (-1, 1):
        subset = [row for row in coast if row["direction"] == direction]
        if subset:
            bias = sum(row["distance"] - linear * row["speed"]
                       - quadratic * row["speed"] ** 2 for row in subset) / len(subset)
            print(f"  {'reverse' if direction < 0 else 'forward'} residual bias: {bias:+.2f} in")
    if any(row["power"] > 0 for row in trials):
        print("Opposing-power runs are excluded from this fit; compare them against simulation separately.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as error:
        raise SystemExit(str(error)) from error
