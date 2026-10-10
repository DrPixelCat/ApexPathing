# Tuning validation in JUnit

Tuners acquire measurements, fit or search constants, and save the measured result.
They keep motion limits, finite-value guards, and excitation/sample requirements.
Independent quality checks run only under `TeamCode/src/test/java`.

- `tuning.follower.sysid.SysIdValidationTest` checks fit R squared, per-run bias,
  and prediction on independent bidirectional acceleration/cruise/braking samples.
  Recorded biased fits are checked by `SysIdRegressionTest`.
- `sim.ProfiledTrackingValidationTest` checks acceleration and braking RMS,
  cross-track and heading errors from the motion CSV. The mecanum and tank
  simulation tests exercise these assertions; endpoint assertions remain in the
  mecanum simulation test.
- `tuning.follower.acceleration.AccelerationValidationTest` owns the legacy
  repeatability allowance. `AccelerationFeedforwardPhaseSimulationTest` verifies
  the selected inertia against the known simulated plant.
- `sim.LocalizationTunerWorkflowTest` compares calibrated pose with simulator
  motion directly, after calibration. The localization tuner has no TEST phase.
- Motion-characterization held-out checks remain in the existing test-only
  `ValidatedMotorAuthorityRegression` fixture.

The profiled optimizer and legacy acceleration search complete after candidate
selection, without running the winner again. The PD tuner keeps its optional
user-triggered X verification, as requested. Manual tuning motions remain available
so the operator can measure responses while adjusting values.

Run with the normal `:TeamCode:testDebugUnitTest` Gradle task. No new main OpMode
is used for validation.
