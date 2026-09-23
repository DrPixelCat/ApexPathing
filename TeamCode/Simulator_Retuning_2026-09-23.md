# FTCodeSim alpha.1 retuning — 2026-09-23

## Current result

The saved mecanum calibration passes all five AutoTest movements using the complete alpha.1
physics model: 16.68 s, average loop 22.39 ms. Correction retains priority. The isolated heading
feedforward acceptance test still fails braking and the fastest case clips; this is a working
route calibration, not a complete solution to velocity tracking.

This report supersedes the earlier motor-only retune. Those earlier results bypassed chassis
friction and must not be treated as validation of the interactive simulator.

## Simulator and tuning fixes

- Upgraded the desktop test dependency to `org.codeblooded.ftcodesim:ftcodesim:v0.1.0-alpha.1`.
- Corrected the mecanum wheel mapping for the release's forward/left/CCW convention.
- Configured new directional natural-deceleration fields. Leaving them zero made full drivetrain
  physics produce nonfinite positions through directional interpolation.
- The tuning workflow, AutoTest, and heading benchmark now advance full drivetrain physics.
  Headless tests suppress only the known post-physics missing-viewer exception. Test JVMs retain
  exception stack traces so repeated exceptions remain identifiable.
- Negative-friction regression fits are refitted on the kS=0 boundary, with existing validity
  checks retained; this is not a clamp of the original fit.

## Calibration

`src/test/resources/simulator/mecanum-alpha1.json` is the durable snapshot shared by AutoTest
and the heading test. It is also installed in `build/ftcodesim-data/constants.json` for desktop
use. The original runtime file remains in `build/retuning/constants-before-retune.json`.
No physical robot calibration was changed.

| Parameter | Translation | Heading |
|---|---:|---:|
| Feedforward kS | 0.004891714 | 0.002890627 |
| Feedforward kV | 0.006241447 | 0.096541985 |
| Feedforward kA | 0.007276265 | 0.092850255 |
| Position P | 0.096088613 | 1.485604086 |
| Position D | 0.020247255 | 0.646260058 |
| Correction static compensation | 0.08 | 0.08 |
| Velocity feedback | 0.066521205 | 0.25 |

Forward limits: 64.79 in/s, 130.96 in/s². Strafe: 64.89 in/s, 133.17 in/s².
Angular: 4.865 rad/s, 9.210 rad/s². Centripetal gain: 0.004848838.

The complete automatic workflow produced the dynamic/PD gains and limits, but preserved zero
static correction. Its subsequent AutoTest stalled on the point turn about 2.3 degrees short.
The final static correction values were therefore selected separately. A full-physics regression
confirms that 0.05 command remains stuck, while 0.08 starts forward, strafe, and turn motion.
With 0.08 static correction, the route passes. The automatic workflow still does not identify
static correction itself; rerunning it from cleared settings can reproduce the stall.
The unadjusted result and failure are retained in `build/retuning/full-physics-automatic.json`
and `full-physics-automatic.xml`.

## Validation and remaining limitations

Selected validation: AutoTest, regression fitting, command allocator and complete-physics axis
checks passed (10 tests); the heading feedforward test failed. A subsequent expanded physics
run passed both axis and breakaway tests. No acceptance limits were relaxed.

AutoTest acceleration speed RMSE: 4.62 in/s; braking RMSE: 7.18 in/s. Peak acceleration overspeed:
4.31 in/s. The trace `AutoTest_Velocity_Tracking.png` shows less pronounced acceleration shelves,
but substantial braking dips and target-speed lag remain. These are single-run observations,
not a controlled comparison with the older simulator snapshot.

Heading feedforward at 3 rad/s and 4 rad/s²: acceleration RMSE 0.137, cruise 0.160, braking
1.486 rad/s; residual rate at stop+200 ms 0.073 rad/s. Both directions fail braking at 2 and
3 rad/s. The 3 rad/s, 8 rad/s² case also clips twice in each direction. Acceleration, cruise,
and stop-residual limits pass in all cases.

The release model uses duty-dependent drag and a different regenerative-braking response when
command opposes motion. A constant kS/kV/kA fit to acceleration does not represent braking
accurately. Further work should address that model/feedforward mismatch and tune static
correction automatically; reducing correction priority would hide rather than resolve it.

## Reproduce

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.AutoTestSimulationTest
.\gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.HeadingFeedforwardSimulationTest
.\gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.ReleaseDrivetrainPhysicsTest
```

The heading command currently fails the documented acceptance limits. `TunedAutoTestSimulationTest`
performs a fresh automatic tune in `build/tuner-e2e-data`; it does not install the frozen resource
and currently reproduces the zero-static-correction limitation described above.
