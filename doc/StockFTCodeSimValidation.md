# Stock FTCodeSim validation

The custom motor model and custom chassis adapters have been removed. `ApexSimulation` now
instantiates the library's concrete `SimulatedMecanum` and `SimulatedTank`; registered motors
retain the upstream model and coefficients. Headless stepping runs the same upstream mechanism
as the interactive simulator. Apex's hardware names, centered origin, localization and telemetry
adapters remain integration conveniences, not physics replacements.

## Version and configuration

The release repository was checked on October 2, 2026: the latest published version is
`v0.1.0-alpha.1`. The current upstream master is `d5e7f965a5774ee019a5c55b0c3881547127746b`.
Mecanum configuration matches the effective values in the pinned
[DECODE example](https://github.com/jjophoven/FTCodeSim/blob/d5e7f965a5774ee019a5c55b0c3881547127746b/Examples/src/test/java/SimulateCodeBloodedDecode.java).

| Parameter | Value |
|---|---:|
| Wheelbase | 9.37008 in |
| Track width | 9.13386 in |
| Wheel radius | 1.889765 in |
| Maximum velocity / acceleration | 65 in/s / 150 in/s² |
| Natural deceleration input to motor registration | 33 in/s² |
| Forward / strafe chassis deceleration | 49 / 85 in/s² |
| Turn natural deceleration | 1 rad/s² |
| Static velocity region / friction | 0.001 / 55 |
| Quadratic / linear braking input | 0.0014846306 / 0.09533276 |
| Nominal battery voltage | 13 V |

These are an upstream example's parameters. A bare Java configuration has largely zero fields
and is not a usable robot configuration. Tank uses stock library physics with the project's
explicit tank geometry/configuration; there is no claim that these are an upstream tank example.

`StockSimulatorPlantTest` checks concrete classes, the example values, registered motor
coefficients, zero-power brake coefficients, and signed motor responses against fresh upstream
registration. No fitted or linearized motor model is installed.

## Retuning and test evidence

Fresh tuning and Auto Test outputs are isolated under
`TeamCode/build/stock-ftcodesim-retuned-20261002`, including tuning progress, phase CSVs,
fit graphs, saved constants and the final velocity graph.

The automatic production tuner runs drive/heading SysId, heading and position feedback, movement
limits, centripetal tuning, and measured profiled tracking. Its existing acceptance gates remain
in force. Acceleration must pass before a braking conclusion is drawn. Auto Test then uses the
accepted saved calibration. The follower, profile generator, constraints and braking algorithm
are not changed during this retuning exercise.

The earlier `braking-slope-sweep`, `profiled-braking-validation`,
`apex-generated-braking-validation`, and `auto-test-new-braking-20261002` graphs used the custom
plant. They are historical evidence only and do **not** validate stock FTCodeSim.

The complete automatic tuner and subsequent Auto Test passed on October 2, 2026, together
with both stock-plant guard tests. All five movements completed in 15.129 seconds. The outbound
trace reported:

| Measurement | Result |
|---|---:|
| Acceleration RMS velocity error | 3.480 in/s (139 samples) |
| Braking RMS velocity error | 2.097 in/s (124 samples) |
| Braking mean velocity error | -1.078 in/s |
| Peak braking overspeed | 2.420 in/s |
| Peak outbound cross-track error | 0.348 in |
| Average control loop period | 20.984 ms |

Acceleration was accepted before interpreting braking. The graph still shows acceleration
underspeed and some corner underspeed; these results demonstrate acceptable tracking on this
stock simulator, not exact tracking or a validation of every braking slope. The older sweep
must be rerun on the stock plant before its conclusions can be reused.

Subsequent [acceleration-envelope diagnosis](StockAccelerationDiagnosis.md) found that the
affine feedforward fit substantially overestimates high-speed acceleration authority and
underestimates high-speed holding power. Passing the existing workflow gates does not
establish that every generated acceleration state is physically attainable.

The accepted desktop calibration is now saved in
`TeamCode/src/test/resources/simulator/mecanum-alpha1.json`. It contains measured tuner fits:
translation kS = 0.238180, kV = 0.00822834, kA = 0.00528767, and velocity feedback gain =
0.0446484. Forward limits are 52.884 in/s and 94.734 in/s²; strafe limits are 39.656 in/s
and 63.931 in/s². These values were produced by the existing tuner, not supplied by an
inverse plant model. Physical robot calibration is not overwritten.

A separate Auto Test using that checked-in desktop fixture also passed, together with two
profiled feedforward checks, the slow-target/signed-acceleration follower check, and both
stock-plant guards (six tests total). Its artifacts are under `default-fixture-confirmation`.

The velocity graph is `TeamCode/build/stock-ftcodesim-retuned-20261002/AutoTest_Velocity_Tracking.png`.
The raw trace is `data/auto-test-outbound-velocity_20261002_102427_714.csv` under that same
directory; `data/constants.json` preserves the full accepted calibration. Only the mecanum
automatic workflow was retuned and validated in this run. Tank now also uses stock physics,
but this result does not validate tank tuning.
