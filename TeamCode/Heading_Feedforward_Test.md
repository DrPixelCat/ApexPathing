# Heading feedforward acceptance test

Run from the repository root:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.HeadingFeedforwardSimulationTest --rerun
```

The current calibration fails braking acceptance and clips in the fastest case. All twelve
cases are written before the test asserts, preserving diagnostics. Including this acceptance
test in the full unit-test suite currently makes that suite fail.

## Measurements

The test commands pure clockwise and counterclockwise rotation at 1, 2, and 3 rad/s, with
4 and 8 rad/s² ramps. Each case accelerates, cruises for 0.6 seconds, brakes, and remains
stopped for 0.6 seconds. It uses production feedforward and motor mixing, with no heading
feedback, translation or command allocation. Each 20 ms control update advances four 5 ms
complete drivetrain steps, including chassis friction. Angular rate comes from drivetrain
velocity; end-of-step actual and target rates are compared.

Acceptance requires each phase RMSE <= 0.50 rad/s, residual rate <= 0.25 rad/s at stop+200 ms,
and no clipping. These are engineering test limits, not hardware certification requirements.

Outputs in `build/heading-feedforward` include `summary.txt`, twelve CSV files and twelve plots.
They are replaced on each run. CSVs include target/actual angular velocity and acceleration,
requested/applied command, and target/actual heading.

## Gains and result

Defaults come from `src/test/resources/simulator/mecanum-alpha1.json`, shared with AutoTest:
kS=0.002890627190233272, kV=0.09654198489223237, kA=0.0928502553785572.
Override them with `APEX_HEADING_FF_KS`, `APEX_HEADING_FF_KV`, `APEX_HEADING_FF_KA` and use
`--rerun`; remove the overrides afterward. This test does not modify stored calibration.

At 3 rad/s with 4 rad/s² ramps: acceleration RMSE 0.1371, cruise 0.1602, braking 1.4862 rad/s;
stop+200 ms residual 0.0725 rad/s. Both directions fail braking at 2 and 3 rad/s. At 3 rad/s
with 8 rad/s² ramps, two control ticks clip in each direction. Acceleration, cruise and stopping
limits pass throughout. Braking is too strong and can reverse rotation prematurely.

This isolates a feedforward/plant mismatch without allocation contention. The simulator's
nonlinear drive/braking response is not captured accurately by these constant gains.
See `Simulator_Retuning_2026-09-23.md` for the full-physics calibration and passing route result.
Earlier wheel-only measurements are superseded by this complete-physics benchmark.
