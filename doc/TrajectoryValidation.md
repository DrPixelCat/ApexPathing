# Trajectory implementation validation

Validated on Windows with the cached Gradle 8.9 runtime on October 2, 2026.
The original sandboxed attempt could not load Gradle's Windows native library;
running the installed cached distribution with access to its native library and
user cache restored compilation and test execution. Dependencies were used offline;
no Gradle or dependency versions were changed.

The earlier broad run executed **242 tests: 236 passed, 6 failed, none skipped**.
All 202 selected unit tests passed. The six failures are in the tank simulations
below. Acceptance thresholds were preserved.

After simplifying stored trajectories to permanent precomputation, the focused
follow-up run passed **211 tests with no failures**: all 202 unit tests, both
Auto Test modes, three traversal lifecycle tests, both dual-actuated following
modes, the dual-actuated profile test, and the strict stored-turn response test.
Signature checks, invalidation, rebuild APIs, and JSON-based planning copies were
removed. Tests now verify that settings changes leave stored data unchanged and
that reuse performs no trajectory solves. The previously failing tank fixtures
were not rerun in this focused selection.

The subsequent turn update passed **229 tests with no failures**: all 201 selected
unit tests, all 19 Apex simulation tests, both Auto Test modes, four traversal
lifecycle tests, both dual-actuated following modes, and the drive-profile test.
Quick turns now use direct heading PID feedback without profiling or trajectory
solves. The new lifecycle check exercises both turn directions with profile limits
set to zero and verifies final-heading diagnostics and pause/resume. Stored turns
retain their precomputed angular LUT and passed the strict braking-response test.
The previously failing tank fixtures were not included in this focused run.

## Passing coverage

| Tests | Result |
| --- | --- |
| Geometry, profiles, path builders/reversal, controllers, core, trajectory, tuning math | 202 passed |
| Apex simulations: mecanum paths, turns, reversals, callbacks, centripetal following | 19 passed |
| Auto Test, ONLINE and MEMORY | 2 passed |
| Saved corner tracking | 1 passed |
| Dual-actuated following, ONLINE and MEMORY; drive profiles | 3 passed |
| Tracking phase manual editing and automatic course selection | 2 passed |
| Tank tracking prerequisite rejection | 1 passed |
| Traversal restart, pause/resume, paused turn unwrapping, permanent precomputation and solver failure handling | 3 passed |
| Other tank simulation checks | 3 passed |

The current 28 trajectory unit tests cover measured starts, overspeed and angular recovery,
future constraints, power/angular/curvature limits, stopping-horizon headroom,
bounded window size, short paths, terminal braking, bilinear weights, negative and
translated grids, edges/corners/outside-grid correction, spline extrema, heading
wrap and tangent alignment, permanent precomputation, and immutable stored inputs.
Solve-count assertions verify zero trajectory solves during stored sampling and
one bounded solve per ordinary online update.

Auto Test completed all stages and callback checks in 15.432 seconds ONLINE and
15.622 seconds MEMORY, within the original 18-second fixture budget. The saved
corner course completed in 18.678 seconds within its original 20-second budget;
outbound cross-track peak was 0.048 inches, heading peak 4.932 degrees, and braking
RMS 2.494 inches/second. These are desktop simulation measurements.

The old corner-entry speed-bias check now compares measured speed with the nominal
scalar course reference. Its original negative-bias threshold is unchanged.
An online braking preview is below its measured initial speed by construction,
so its instantaneous speed error has a different interpretation. Other tracking
checks and CSV target columns still use the actual commanded trajectory sample.

## Remaining tank failures and baseline comparison

An untouched archive of HEAD `6a8ea83` was tested separately with the same runner.
Five existing tank tests fail there too. The added online variant also misses
the original fixture's endpoint threshold. Some current and baseline runs fail
different assertions; tank acceptance remains unmet.

| Test | Untouched baseline | Current implementation |
| --- | --- | --- |
| Low-power steady-speed plant | 0.279 in/s at power 0.12; expected 4.787 | Same 0.279 in/s failure |
| Stored curve/turn/reverse | Endpoint error 5.835 in; maximum 3 in | Endpoint error 5.821 in |
| Online curve/turn/reverse (added) | No original online variant | Endpoint error 5.428 in |
| Tank Auto Test | Outbound endpoint error 30.118 in | Cross-track peak 3.298 in; maximum 2.2 in |
| Repeated Tank Auto Test | Outbound endpoint error 29.975 in | Cross-track peak 2.929 in |
| Tuned Tank Auto Test | Cross-track peak 2.217 in; maximum 2.2 in | Return endpoint error 3.622 in |

Calibration JSON keys, simulator plant behavior, feedback gains, and these
numeric acceptance thresholds were not changed to make the failures pass.

## Desktop benchmark

`TrajectoryTest.benchmarkInitializationMemoryAndQueries` uses a 100-inch straight
route, default 1-inch grid spacing and 12-inch padding, and an explicit 20-inch
online horizon. It warms both samplers, then measures 200 queries.

| Measurement | Result |
| --- | --- |
| ONLINE geometry build | 0.079 ms |
| MEMORY trajectory and field build | 3.448 ms |
| Packed field arrays | 275,000 bytes (125 by 25 nodes, 11 doubles per node) |
| ONLINE planning query, p95 | 0.0699 ms |
| Stored lookup query, p95 | 0.0024 ms |
| Nodes in the final online solve | 17 |

The CSV is generated at `TeamCode/build/trajectory-benchmark.csv`. Timings measure
sampler work on this desktop, not a full robot control loop or Android hardware.

## Reproduction

Run `:TeamCode:testDebugUnitTest` with the following Gradle `--tests` filters:

```text
geometry.*
feedforward.*
paths.*
core.*
controllers.*
trajectory.*
tuning.*
org.firstinspires.ftc.teamcode.sim.ApexSimulationTest
org.firstinspires.ftc.teamcode.sim.AutoTestSimulationTest
org.firstinspires.ftc.teamcode.sim.CornerTrackingSimulationTest
org.firstinspires.ftc.teamcode.sim.DualActuatedPathFollowingSimulationTest
org.firstinspires.ftc.teamcode.sim.DualActuatedProfilesTest
org.firstinspires.ftc.teamcode.sim.TrajectoryLifecycleSimulationTest
org.firstinspires.ftc.teamcode.sim.TankPathFollowingSimulationTest
org.firstinspires.ftc.teamcode.sim.TankAutoAccuracySimulationTest
org.firstinspires.ftc.teamcode.sim.ProfiledTrackingPhaseSimulationTest
org.firstinspires.ftc.teamcode.sim.TankProfiledTrackingPhaseSimulationTest
```

JUnit XML and the HTML report are under `TeamCode/build/test-results/` and
`TeamCode/build/reports/tests/`. This selection excludes the long complete tuning
workflows and parameter sweeps. The external `SquareVectorField.java` reference
was read without editing it. See [the API and architecture](TrajectoryPlanning.md).
