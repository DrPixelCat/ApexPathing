# Angular characterization investigation

October 6, 2026. Follow-up to `FollowerTunerValidation.md`.

## Findings

Motion characterization identified motor response from the follower's adaptive
Kalman velocity estimate. That estimate has a transient response and retains its
learned noise settings across staging pose resets. The regression consequently
identified some estimator dynamics as motor acceleration authority and drag.
Validation inverted the fitted acceleration authority to predict power; near the
angular speed ceiling, that authority is small and amplifies measurement errors.

Controlled tests held the stock simulator, power sequence, sensing, and regression
constant while comparing native and filtered velocities. The six trials used
0.65/1.0 acquisition power and a separate 0.55/0.95 validation sequence. Both
rotation directions were tested, with a shared deterministic clock.

| Measurement/window | Fresh filter power RMS | Filter after 40 s stationary power RMS |
| --- | ---: | ---: |
| Native velocity, endpoint acceleration | 0.000161 | 0.000161 |
| Native velocity, OLS acceleration | 0.000065 | 0.000065 |
| Filtered velocity, endpoint acceleration | 0.026967 | 0.033919 |
| Filtered velocity, OLS acceleration | 0.027397 | 0.033646 |

RMS is normalized motor power on held-out windows. Stationary adaptation reduced
angular process variance from 25 to its lower bound of 2.5. All of these controlled
fits passed the existing acceptance limits, but an independent physical test
revealed bias: at an imposed 3 rad/s, the filtered OLS fit's predicted holding
power produced +0.4793 rad/s² acceleration with the fresh filter and +0.5316 after
stationary adaptation. The native endpoint fit produced +0.0373 rad/s². At
7.5 rad/s, native endpoint holding acceleration was +0.0020 rad/s², versus
+0.1313/+0.1492 for the filtered OLS fits. Native holding probes are asserted
within 0.05 rad/s² by the new test.

Replaying the rejected workflow's approximately 31 ms sample cadence, with newly
simulated motion and aligned acquisition timestamps, also passed. After stationary
adaptation, native endpoint power RMS was 0.000148; filtered OLS RMS was 0.034354.
Thus cadence alone does not reproduce the old rejection. It is incorrect to
attribute the failure solely to OLS arithmetic or a particular sampling period.

A wall-clock run with paired native/filtered measurements passed characterization
without changing its original calculation. Offline evaluation of identical windows
gave angular power RMS 0.02840/0.03083 for the original filtered method and
0.00569/0.00285 for native endpoint acceleration using acquisition time. Switching
only the filtered method to acquisition time gave 0.02837/0.03075. This identifies
filtering as the dominant measured bias in that run; timestamp alignment remains
necessary for consistent derivatives. The original rejected runs did not log native
velocity, so their exact native alternatives cannot be reconstructed.

The old rejection is intermittent: two earlier runs rejected positive angular
validation, while the instrumented original calculation passed and progressed
through feedback, limits, and centripetal tuning. That run later threw
`No feasible braking command for measured overspeed 30.07464906349059` during
trajectory tracking. Full workflow success requires separate verification of that
later failure.

## Corrections and workflow result

`MotionCharacterizationPhase` now uses native localizer velocity and its matching
raw pose for body-frame projection. The navigation estimator remains active.
Window durations use accepted sensor acquisition timestamps. Acceleration is
`(endVelocity - startVelocity) / windowTime`, so velocity, acceleration, power, and
power-times-velocity all describe the same time integral of the plant equation.
The previous OLS slope had a different temporal weighting from the other terms.
CSV rows retain native velocity and acquisition timestamps and also record
filtered velocity for comparison.

A separate retry defect appeared during validation: a >0.5 s scheduling gap
reduced strafe validation's high-speed coverage. `validationCovered()` checked
sample counts in the shared range but omitted `fit()`'s requirement to reach
85% of acquired maximum speed. It could therefore skip the available repeat and
immediately reject the fit. The retry check now enforces that same requirement;
a focused regression test covers sufficient sample counts with insufficient
speed range. Existing fit thresholds and the maximum repeat count are unchanged.

The corrected wall-clock run accepted all six axis/direction models. Their
held-out power RMS values were 0.001419/0.001771 for forward,
0.000644/0.000979 for strafe, and 0.006351/0.002498 for angular motion. It then
completed heading feedback, drive feedback, movement limits, and centripetal
tuning. Trajectory tracking did not complete: trial 42 threw
`No feasible braking command for measured overspeed 33.53931949027496`
from `PathProfileGenerator.recoveryMotion()`. The saved profile correctly retains
`profiledTrackingTuned=false`.

The final logged tracking sample had measured speed 43.377 in/s versus a
27.774 in/s target, followed by the recovery exception. This is a later control/
planning failure, not a characterization rejection. The powered characterization
map deliberately falls back to legacy SysId coefficients during braking;
`HolonomicDriveModel` (formerly the mecanum profile generator) combines that braking demand with lateral and heading
power, and recovery rejects commands exceeding its power budget. The earlier
stock/SysId holding-power mismatch therefore remains relevant to investigate,
but the trace does not yet establish which demand made this recovery infeasible.
No planner behavior or simulator physics was changed in this investigation.

The broad follow-up run executed 62 checks: 61 passed, including all three new
measurement tests, all five motor-authority regressions, and automatic tracking
with the existing simulator calibration. The manual tracking test failed during
simulator shutdown (`Simulated LinearOpMode did not stop within one second`);
an isolated rerun passed without changes. The fresh full-workflow test remains failing
at the braking exception above. These results do not claim complete tuning.

## Reproduction

`AngularCharacterizationMeasurementTest` runs fresh/shared adaptive filters,
stationary preconditioning, the recorded cadence, matched regression variants,
and independent physical holding probes. The cadence-only fixture is
`TeamCode/src/test/resources/simulator/angular-characterization-cadence.csv`,
extracted from `motion_characterization_20261006_160928_768.csv`.

```powershell
.\gradlew.bat -I work/follower-tuner-check.init.gradle :TeamCode:testDebugUnitTest `
  --tests '*AngularCharacterizationMeasurementTest' `
  --tests '*MotorAuthorityRegressionTest' --offline
```

Controlled samples/windows are generated under
`TeamCode/build/angular-characterization-measurements/`. The paired original
wall-clock CSV is `work/follower-tuner-check/tuning-data/
motion_characterization_20261006_172657_285.csv`; the corrected accepted models
are in `motion_characterization_20261006_173558_723.csv` beside it. These are
local generated evidence. No hardware measurements have been collected.
