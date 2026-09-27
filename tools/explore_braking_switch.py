"""Explore the alpha.1 straight-line brake gap with a two-command controller.

This deliberately models one straight-line velocity degree of freedom. It is
not an AutoTest replay or a replacement for a full FTCodeSim run. The two
commands are zero-power BRAKE and the positive duty that approximately holds
speed against chassis drag. Pulse density modulation (PDM) schedules braking
from the requested acceleration alone. A greedy one-step selector uses actual
speed and is shown separately because it is feedback, even with the usual
velocity feedback gain set to zero.
"""

import argparse
import math


MAX_ACCEL = 150.0
MAX_VELOCITY = 75.0
NATURAL_DECEL = MAX_ACCEL * .06 / (1.0 - .06)
REGEN_RATE = 9.99  # installed alpha.1 fit is near its 10 / s search limit
REQUESTED_BRAKING = 127.122205


def drive_acceleration(speed, power):
    return (MAX_ACCEL + NATURAL_DECEL
            - MAX_ACCEL / MAX_VELOCITY * speed) * power - NATURAL_DECEL


def braking_acceleration(speed):
    return -REGEN_RATE * speed - NATURAL_DECEL


def simulate(dt, start_speed, stop_speed, strategy):
    duration = (start_speed - stop_speed) / REQUESTED_BRAKING
    count = math.ceil(duration / dt)
    speed = start_speed
    errors = []
    brakes = 0
    pulse_credit = .5
    for step in range(count):
        target = max(stop_speed, start_speed - REQUESTED_BRAKING * (step + 1) * dt)
        hold_power = NATURAL_DECEL / (MAX_ACCEL + NATURAL_DECEL
                                      - MAX_ACCEL / MAX_VELOCITY * speed)
        held = max(0.0, speed + drive_acceleration(speed, hold_power) * dt)
        braked = max(0.0, speed + braking_acceleration(speed) * dt)
        if strategy == "pdm":
            pulse_credit += min(1.0, REQUESTED_BRAKING / -braking_acceleration(speed))
            brake = pulse_credit >= 1.0
            if brake:
                pulse_credit -= 1.0
        else:
            brake = abs(braked - target) < abs(held - target)
        if brake:
            speed = braked
            brakes += 1
        else:
            speed = held
        errors.append(speed - target)
    return dict(dt=dt, cycles=count, brake_cycles=brakes,
                rmse=math.sqrt(sum(error * error for error in errors) / count),
                peak=max(abs(error) for error in errors),
                final_error=errors[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", type=float, default=60.0)
    parser.add_argument("--stop", type=float, default=6.0)
    args = parser.parse_args()
    if not 0 < args.stop < args.start <= MAX_VELOCITY:
        parser.error("Need 0 < stop < start <= 75 in/s")
    print(f"Model at 40 in/s: same-direction hold ~0 in/s^2, "
          f"zero-power BRAKE ~{braking_acceleration(40):.1f} in/s^2; "
          f"requested {(-REQUESTED_BRAKING):.1f} in/s^2")
    for strategy in ("pdm", "greedy"):
        for dt in (.020, .010, .005):
            result = simulate(dt, args.start, args.stop, strategy)
            print(f"{strategy} {dt * 1000:.0f} ms: "
                  f"{result['brake_cycles']}/{result['cycles']} "
                  f"brake cycles, speed RMSE {result['rmse']:.2f} in/s, "
                  f"peak error {result['peak']:.2f} in/s, "
                  f"final error {result['final_error']:+.2f} in/s")


if __name__ == "__main__":
    main()
