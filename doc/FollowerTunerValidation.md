# Follower tuner validation

October 6, 2026. The requested pre-validation snapshot is commit `6af3de3`.
Validation used the desktop FTCodeSim mecanum setup and actual production tuner
state machines, gamepad inputs, separate tuning storage, and an isolated Gradle
build directory. No robot hardware was tested.

## Result

**The fresh automatic workflow does not complete.** The final focused run finished
with 70 tests: 69 passed and one failed, `FollowerTunerTelemetryTest.
automaticWorkflowCompletesWithoutVelocityFeedbackTuner`.

| Phase | Fresh automatic workflow result |
| --- | --- |
| Drive feedforward SysId | Accepted and saved; R-squared 0.999981 |
| Heading feedforward SysId | Accepted and saved; R-squared 0.999918 |
| Motion characterization | Forward/strafe directions passed internally; positive angular direction failed held-out high-speed validation |
| Heading feedback | Not reached |
| Drive feedback | Not reached |
| Movement limits | Not reached |
| Centripetal | Not reached |
| Trajectory tracking | Not reached in the fresh workflow; separate tests with existing calibration passed |

The final rejection was `Characterization validation failed: Validation failed in
speed band 3`. Independent replay of the angular windows gave normalized-power
prediction RMS 0.074294 and peak error 0.301322. Overall acceptance requires RMS
at most 0.04, peak at most 0.10, and RMS at most 0.05 in each speed band.

An earlier run also rejected positive angular characterization, with RMS 0.036088
and peak 0.102153. That harness advanced physics on a separate thread. The final
run advanced all elapsed physics immediately before each tuner frame on the
OpMode thread; the failure persisted. This rules out that particular concurrent
stepping arrangement as the sole cause, but does not establish the remaining root
cause. Acceptance thresholds and production algorithms were not changed.

Storage contains both accepted SysId axes, no accepted motion characterization,
zero feedback P gains/limits/centripetal gain, and `profiledTrackingTuned=false`.
The workflow correctly stops without claiming complete tuning or saving the
rejected characterization.

## Independent checks

Manual SysId controls, alternating direction, stick availability, Stop/discard,
phase order, and exact restoration of the temporary motor-model term passed.
Fresh gains completed both manual axes. In the final manual run, drive cruise
errors were +0.03628/-0.21292 in/s and heading errors +0.01595/-0.01970 rad/s.

These trials use the existing temporary SysId motor workaround. After the stock
model was restored, independent displacement profiles with the same gains had
drive cruise errors +9.36951/-9.19193 in/s and heading errors
+1.22463/-1.21938 rad/s. At 24 in/s, the fresh drive gains requested 0.575786
power; an independent stock-state probe measured +24.4792 in/s² acceleration.
Stock holding power was 0.383966 and produced zero acceleration. The SysId gains
alone therefore do not reproduce stock simulation behavior outside the phase.
The later motion-characterization map still needs to pass before the complete
workflow can be validated.

Two tracking tests initially checked the shared constants template instead of
the running follower's owned copy. Their assertions now inspect the active
instance or saved JSON; both passed, including manual review invalidation and
automatic completion with existing calibration. This does not validate tracking
with gains from the incomplete fresh workflow.

## Evidence and reproduction

The final JUnit XML/HTML report and accepted constants are under
`work/follower-tuner-check/TeamCode/{test-results,reports}/` and
`work/follower-tuner-check/tuning-data/`. The final characterization CSV is
`motion_characterization_20261006_160928_*.csv` in that tuning-data folder.
Fresh manual/profile traces and probes are under
`TeamCode/build/fresh-sysid-manual-validation/`. These are local generated files.

The local `work/follower-tuner-check.init.gradle` directs build products and
storage away from the interactive simulator. Reproduce with:

```powershell
.\gradlew.bat -I work/follower-tuner-check.init.gradle :TeamCode:testDebugUnitTest `
  --tests 'tuning.follower.*' --tests '*FollowerTunerSequenceTest' `
  --tests '*FollowerTunerTelemetryTest' --tests '*FreshSysIdManualValidationTest' `
  --tests '*ProfiledTrackingPhaseSimulationTest' `
  --tests '*SimulatorSysIdMotorOverrideTest' --offline
```
