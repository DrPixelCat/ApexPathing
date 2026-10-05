# Unified sensor-only Kalman localization

The default estimator in `BaseLocalizer` uses one Kalman state of **position, velocity and
acceleration** for each field axis (X, Y and heading). `getPose()`, `getVel()` and `getAccel()`
read that same state. There is no second velocity Kalman update in the default path.

`KalmanFilter` contains the reusable prediction, scalar linear-observation correction and
Joseph covariance update. It accepts a configurable state size and caller-supplied F, Q, H and R.
`PoseEstimator` owns the three-axis motion model, angle unwrapping, timestamps, sensor weighting
and bounded noise adaptation, and delegates all Kalman mathematics to that core. The existing
`AdaptiveKalmanFilter` API remains a two-state model and tuning wrapper around the same core;
it no longer implements another prediction/correction engine.

TwoWheel, ThreeWheel, TankDriveEncoders and MecanumDriveEncoders provide native position samples.
The estimator infers their instantaneous velocity and acceleration through its motion model and
position/velocity/acceleration cross-covariance. Finite differences of raw pose are retained for
`getRawVel()` and `getRawAccel()` diagnostics, but are not fed back as another measurement.

Pinpoint, OTOS and Octoquad also provide native velocity samples. These update the velocity
component of the same estimator, which can also adjust its position and acceleration. Position
and velocity from the same device are not assumed independent: equal-weight covariance
intersection combines their observation information conservatively. This is implemented by
halving each observation's information (doubling its measurement variance) within one update.
The three field axes remain independent of one another.

Native odometry continues to integrate as before. Geometry calibration uses `getRawPose()` so
filter delay does not distort scale or pod-offset measurements. No camera, vision measurement
API or external field-position correction is included.

## Configuration

The unified estimator and bounded automatic noise adaptation are enabled by default:

```java
localizer.setPoseFilteringEnabled(true);
localizer.setVelocityFilterMode(BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN);
localizer.setPoseAutoTuning(true);
Pose position = localizer.getPose();
Pose velocity = localizer.getVel();
Pose acceleration = localizer.getAccel();
Pose nativePosition = localizer.getRawPose();
```

`setPoseAutoTuning(false)` freezes all learned parameters. The existing `setIsTuning(...)` and
`setKalmanAutoTuning(tuneX, tuneY, tuneHeading)` controls also configure learning on the shared
estimator; position and velocity no longer have separate automatic-tuning switches in this mode.
Pose resets and pauses preserve learned parameters and per-axis learning switches.

`getPoseKalmanTuning()` / `setPoseKalmanTuning(...)` expose position measurement variances, native
velocity measurement variances, and shared process noise. `BaseLocalizerConstants.getPoseKalmanTuning()`
supplies initial values. Existing `getXKalmanTuning()` / `setXKalmanTuning(...)` methods (and Y/heading
variants) address native velocity measurement variance and the shared process noise in default mode.
For position-only localizers, no velocity observation is manufactured: tuning that velocity
measurement variance affects initial uncertainty, not a separate velocity correction.

The transition is a constant-acceleration model, with continuous white-jerk process noise:
`p_next = p + v*dt + a*dt*dt/2`, `v_next = v + a*dt`. Position measurement variances use square
inches / square radians; velocity variances use squared inches-per-second / radians-per-second.
Process spectral densities use square inches or square radians per second to the fifth power.
Measurement variance adaptation stays within 0.25-4 times its configured baseline, and process
noise within 0.1-10 times its baseline, additionally bounded to finite supported values.
Prediction errors guide tuning; these bounds do not guarantee accuracy.

`getPoseCovariance()` retains the position-only 3x3 diagnostic. `getKinematicCovariance()` exposes
a defensive 9x9 covariance ordered `[x,vx,ax,y,vy,ay,heading,angularVelocity,angularAcceleration]`.
Within each axis, cross-covariances couple the estimates.

## Explicit fallback modes

Selecting `MOVING_AVERAGE` preserves the existing moving-average velocity/acceleration behavior.
Position smoothing still runs, with position observations only. Bypassing pose smoothing via
`setPoseFilteringEnabled(false)` returns native pose and uses the existing velocity-only estimator.
These are explicit compatibility modes; the default unified path never runs those derivative
filters. Toggling the bypass clears motion history, and changing velocity modes preserves the
pose estimator's history while clearing the fallback derivative history.

## Timing, reset and persistence

One timestamp covers the entire observation. Invalid position/native-velocity samples or repeated
and out-of-order timestamps are rejected before any component changes. Heading is unwrapped
before filtering. A gap longer than 200 ms reacquires position and native velocity if available,
clearing stale acceleration. Without native velocity it starts with zero velocity and relearns
motion. `setPose()` clears pose and motion state without inventing motion from a teleport.

`LocalizationConstants.capture(...)` saves the model identifier `CONSTANT_ACCELERATION`, learned
noise, per-axis adaptation switches and bypass/mode settings. Schema-1 files without a pose record
continue to load. Old pose records without a model identifier use the existing velocity filter's
white-jerk calibration. If none exists, a new starting jerk baseline is chosen as 100 times the
old acceleration-noise value; this is a migration heuristic, not equivalent covariance. Unsupported
models and invalid variances are rejected before settings change.

The filter collection phase freezes learning and flushes/closes its CSV on completion or
interruption. Interrupted collection does not save accepted calibration. The review screen can
still show any CSV write error after resources are closed.

## Tests and evidence

The combined verification run passed **78 tests with zero failures and zero skipped tests**:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests 'localizers.*' --tests 'tuning.localizer.*' --tests 'org.firstinspires.ftc.teamcode.sim.LocalizationTunerWorkflowTest' --tests 'org.firstinspires.ftc.teamcode.sim.DualActuatedProfilesTest' --tests 'org.firstinspires.ftc.teamcode.sim.TrajectoryLifecycleSimulationTest' --offline
```

This is the targeted localization/tuning and related simulation suite, not the entire repository.
All 11 localization-tuner workflow tests passed, including the full geometry/filter/live-test flow.

Tests cover known accelerating motion at 10/20/40 ms loop periods, reversal and stopping, heading
wrap, invalid and repeated samples, pause recovery, covariance symmetry/positive definiteness,
calibration freezing/persistence, legacy migration and native encoder integration. A native-velocity
test uses perfectly correlated position/velocity noise and checks improvement over position-only
velocity inference. Integration tests check that native velocity adjusts the shared position state.

`KalmanFilterTest` checks hand-computed scalar and mixed-state observations, dimension handling,
independent instances, defensive snapshots and rejection of malformed inputs. Existing two-state
and three-state estimator tests retain their numerical behavior after the refactor.

`LocalizationNoise` and optional noise in `SimApexPinpoint` are test fixtures only. They add seeded
Gaussian position/heading and velocity noise, with configurable correlation, to reported readings.
Production code and simulator plant ground truth receive no artificial noise.

`PoseEstimatorTuningTest` trains with one seed, freezes the learned parameters, resets state, then
compares tuned and default estimators using exactly the same held-out noisy trajectory with another
seed. It covers position noise standard deviations of 0.025/0.05/0.10 inches, 10/20/40 ms loop
periods, correlated native velocity noise, and position-only learning. Across 7,191 held-out samples,
position RMS was **0.04290 vs 0.04463 inches**, and velocity RMS was **0.32557 vs 0.45022 inches/second**.
It also checks heading and angular-velocity error, plausible learned noise, and unchanged parameters
after freezing and reset. These results cover the tested signal models and configured learning bounds.

The simulator tuning workflow test injects 0.10-inch position, 0.02-radian heading, 0.8-inch/second
velocity and 0.16-radian/second angular-velocity noise with correlation 0.6. It runs STILL/MOVE/STOP
collection, review, acceptance, disk persistence and reload; checks that learned values changed;
then continues updates and verifies they remain frozen. A separate workflow interrupts collection,
checking stopped motors, frozen learning, flushed CSV data and no accepted calibration file.

The workflow fixtures acknowledge button consumption before sending another press, give the
required five rotations enough simulated travel time, expect the actual 45-degree/second spin
label, and allow the plant to settle before spin geometry calibration. The live pose test remains
open until the OpMode stops; its test does not expect an automatic return to the menu.

A seeded sinusoidal-motion test with 0.05-inch position noise compared the unified estimate with
the earlier finite-difference plus separate Kalman velocity estimator, using its default R=0.25
and Q=625. Velocity RMS error was **0.33541 vs 0.97945 inches/second**, and acceleration RMS error
was **2.07722 vs 12.06384 inches/second squared**. These figures describe that synthetic trajectory
and those settings, not all robots or optimally tuned alternatives.

The separate position-noise test measured **0.07198 inches raw vs 0.04013 inches filtered** RMS,
and **0.00993 radians raw vs 0.00568 radians filtered** heading RMS. The preceding two-state pose
filter measured 0.03802 inches and 0.00452 radians on the same signal; the unified acceleration
model trades some smoothing for coupled motion estimation. Both tests include tracking delay in
their errors rather than measuring only visual smoothness.

No physical robot or recorded robot dataset has been used. Integrated odometry has correlated
errors, so covariance is a model diagnostic, not an absolute accuracy guarantee. A test explicitly
confirms that accumulated input bias remains: the filter cannot discover encoder scale error,
slippage or a common heading bias from those readings alone. Physical calibration and endpoint
accuracy checks are still required.
