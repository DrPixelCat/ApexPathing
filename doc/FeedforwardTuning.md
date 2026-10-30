# Feedforward kS/kV tuning

The automatic tuner uses the slow power-ramp method from commit `12ebdeb` (`Add ramp regression for feedforward tuning`). Power rises by **0.1 per second**, up to the lower of **0.9** and the configured drivetrain power limit. The tuner fits:

```text
power = feedforward kS + kV * speed
```

kS is the extrapolated moving-power intercept. kV is the slope. Automatic ramp tuning **preserves the existing kA**; it does not estimate acceleration feedforward. Manual tuning still exposes all three gains.

## Acquisition

Drive uses one forward and one reverse ramp, each beginning from rest. Heading uses one CCW ramp. Each ramp ends at its power/time limit or its travel guard: **48 inches** for drive or **1.5 turns** for heading. Terminal velocity and a near-zero acceleration threshold are no longer required. The operator confirms each ramp from rest; the robot brakes normally between ramps.

Simulation stages the robot once before acquisition: drive starts at **(-12, -12) inches from field center, heading 45 degrees**; heading starts at field center. Hardware positioning is performed by the operator. No trial teleports the robot, resets its physical velocity, or requests repositioning between ramps. The field check includes powered travel and braking.

The fit uses native sensor velocity, projected into the raw pose's robot frame for drive. It pairs each observation with the preceding delivered command, averages moving velocity/power over approximately 0.1-second windows, and normalizes reverse samples to magnitudes. Stationary samples and intervals with duplicate timestamps or gaps greater than 0.1 seconds are excluded from the regression. Motion thresholds are 0.5 in/s for drive and 0.05 rad/s for heading.

Each required direction needs at least ten moving windows and a velocity span of 2 in/s for drive or 0.2 rad/s for heading. Acceptance requires finite nonnegative kS below 0.9, positive kV, **R² >= 0.98**, power residual **RMS <= 0.02**, and per-direction mean residual magnitude no greater than 0.02. A rejected fit leaves existing gains intact and reports its quality. There is no automatic outlier deletion or simulator motor override.

The last command observed below the movement threshold is retained separately as position-controller breakaway compensation. Drive uses the smaller forward/reverse value; heading uses the single CCW value. This measured boundary is distinct from the line's moving-power intercept.

## Raw-velocity repair

The localizer previously used a clamped controller interval for sensor derivatives. Its timing history could remain at the minimum interval after a long sampling gap, leaving raw velocity frozen while measurement timestamps continued advancing. Native velocity and pose-derived velocity now use the actual interval between accepted sensor timestamps. Invalid or repeated timestamps still fail validation. Regression tests cover a long first gap, normal subsequent samples, and another later gap.

## Validation and limitations

`QuasistaticFitTest` checks signed line recovery, a separate first-order linear feedforward plant, unchanged kA, missing/frozen data, nonlinear/asymmetric responses, and held-out velocities. The independent linear-plant experiment checks predicted steady powers within 0.01 duty. A slow ramp still carries inertial power, so its fitted kS is an approximation when acceleration is appreciable. High inertia can require a slower ramp; a high R² alone cannot detect this bias.

`FeedforwardAcquisitionSimulationTest` exercises the actual UI, native sensors, drivetrain output and field boundaries on the unchanged stock simulator. It verifies one initial placement, refreshed raw velocity, unchanged kA on both axes, and consistency between the reported fit and UI acceptance. If the stock plant's sampled response fails the line-quality gate, the expected behavior is explicit rejection without changed gains. This is not a requirement to accept every stock-plant acquisition.

The stock plant remains nonlinear even at constant velocity:

```text
forward power = 49 / (183 - (150/65) * velocity)
```

A line can pass over a limited measured interval while predicting poorly at other speeds. A dedicated held-out test demonstrates this mismatch; stock simulator fit values are not presented as exact physical kS/kV. See `doc/SimulatorPhysics.md` for the native equations.

`QuasistaticFieldSequenceTest` checks both drive ramps and final braking from one diagonal start with all walls active and no model changes. Its deterministic stock-plant run kept center coordinates between approximately -12.04 and +25.28 inches on each field axis, a forward/backward span of about 52.8 inches along the diagonal, including braking. Other robots require their own clear travel and stopping room.

Current evidence is in `TeamCode/build/ks-kv-acquisition/{DRIVE,HEADING}/`, including result text, traces and timestamped storage folders containing raw/window/fit CSVs. Earlier fixed-power graphs and logs are historical. The focused verification log is `tmp/quasistatic-ramp-verification.log`; JUnit results are in `TeamCode/build/test-results/simulatorPlantAudit/`.

## Latest focused run

**47 tests passed, 0 failed.** Coverage includes native and pose-derived gap recovery, Pinpoint packets, encoder localizers, ramp regression, independent linear-plant validation, stock-plant model mismatch, full acquisition/UI behavior, field clearance, motor-model preservation, manual profiles and existing regression checks. This was a focused run, not the full repository suite.

The latest actual stock-simulator acquisition completed drive in **17.0 seconds** and heading in **6.0 seconds**, including operator-confirmation automation and result rendering. Drive reported kS=0.30425, kV=0.00785, R²=0.99598; heading reported kS=0.09479, kV=0.05164, R²=0.99450. These are local ramp-fit approximations for this nonlinear plant. Test sentinels kA=0.013 drive and 0.027 heading remained unchanged on both axes. All recorded center coordinates stayed within ±48 inches of field center, and each axis was staged exactly once.
