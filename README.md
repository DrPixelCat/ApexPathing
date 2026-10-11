# Apex Pathing
Check out our site at https://www.apexpathing.com/ for info about the project!

Class JavaDoc lists credited authors.
See the [main-source authorship audit](reviews/authorship/AUTHORSHIP.md) for Git evidence,
recorded AI involvement, and unresolved historical credits.

## Follower API

`Follower` is the abstract base for the follower hierarchy. `MecanumFollower`,
`SwerveFollower` and `OmniFollower` extend the shared `HolonomicFollower` implementation; `TankFollower` and
`DualActuatedFollower` extend `Follower` directly. The base owns localization, movement lifecycle, turns, callbacks,
diagnostics, and failure handling. Each subclass selects its path-tracking and pose-holding
behavior. Fixed-drive followers own only their matching tracking algorithm; the dual follower
reuses both algorithms with one drivetrain and one movement state. Mecanum retains directional
power allocation and wheel-model support; swerve uses isotropic allocation and its module steering.
Omni supports any number of uniformly spaced tangent-driving wheels, with at least three.
Its follower budgets signed translation and rotation using the configured wheel projections.
Three-wheel layouts use the same omni implementation as larger layouts. The generic
`HolonomicFollower` remains available for custom fixed holonomic drives.

Replace `new Follower(...)` with `Follower.create(...)`. The factory builds the drivetrain once
and selects a subclass from the resulting hardware capabilities, including dual drives that
start in tank mode:

```java
import follower.Follower;

Follower follower = Follower.create(constants, hardwareMap);
follower.setPose(startPose);
follower.follow(path);
// Call repeatedly in the active OpMode loop.
follower.update();
```

Explicit construction is also available as `new TankFollower(constants, hardwareMap)`,
`new MecanumFollower(constants, hardwareMap)`, `new SwerveFollower(constants, hardwareMap)`,
`new OmniFollower(constants, hardwareMap)`, or
`new DualActuatedFollower(constants, hardwareMap)`. Explicit constructors reject incompatible
drivetrains; fixed-drive followers cannot be used for dual-actuated hardware.

`Follower.create(...)` automatically selects `MecanumFollower` for `Mecanum` hardware,
`SwerveFollower` for `CoaxialSwerve` hardware, and `OmniFollower` for `Omni` hardware.
Existing generic `HolonomicFollower` construction
remains compatible; use the specialized constructors when declaring a specific drivetrain type.

Concrete followers now live in `follower.drivetrains`; update their imports accordingly.
The base, settings, and diagnostics retain their `follower` imports.

The factory and each subclass support `(constants, hardwareMap, tuningMode)` and
`(constants, hardwareMap, tuningMode, monotonicClock)` overloads. Existing follow, update,
pause/resume, stop, manual driving, tuning, and diagnostic calls remain available through
the base type. A holonomic braking model has no effect on tank-only followers, as before.

The base lifecycle methods are final. The protected extension methods are
`updatePath(Path)`, `holdPose(Pose)`, and `resetPathControllers()`, with optional
`prepareMovement(FollowerMovement)`, `synchronizeDriveProfile()`, and
`isDriveTransitioning()` hooks for mode-switching hardware. Constructors initialize shared
state without calling overridable follower methods.

## Circular omni-wheel drive

Use `Omni.Constants` for three or more omni wheels equally spaced around a circle centered
on the robot. Add motors in counterclockwise order as viewed from above. The first wheel's
radial position defaults to straight ahead (0 degrees); 90 degrees is the robot's left side.
Each wheel's powered direction must be tangent to the circle. Positive motor power should
drive along the counterclockwise tangent; use `Motor.reverse()` where needed. Wheels are
assumed to have matching size and gearing.

For example, the drivetrain portion of a robot configuration can declare a six-wheel layout:

```java
import drivetrains.Motor;
import drivetrains.Omni;
import geometry.Angle;

@Override
public Omni.Constants drivetrainConstants() {
    return new Omni.Constants()
            .setFirstWheelAngle(Angle.fromDeg(0))
            .addWheel(new Motor("wheel0")) // Front: 0 degrees
            .addWheel(new Motor("wheel1")) // 60 degrees
            .addWheel(new Motor("wheel2")) // 120 degrees
            .addWheel(new Motor("wheel3")) // Back: 180 degrees
            .addWheel(new Motor("wheel4")) // 240 degrees
            .addWheel(new Motor("wheel5")) // 300 degrees
            .setMaxPower(0.8);
}
```

The number of registered wheels determines `n`; no separate wheel-count setting is needed.
For wheel `i`, `theta = firstWheelAngle + i * 2*pi/n` and its command is
`-sin(theta)*forward + cos(theta)*left + turn`. Positive turn is counterclockwise.
All wheel powers scale together when any wheel reaches the configured power limit.
Turn is a normalized power contribution; tune angular gains for the actual mounting radius.

`Follower.create(constants, hardwareMap)` selects `OmniFollower`, or construct one explicitly
with `new OmniFollower(constants, hardwareMap)`. Its normal path, turn, hold, pause/resume,
and failure APIs match the other holonomic followers. Online and stored trajectory planning
use the exact largest absolute wheel command, including signed translation/rotation cancellation,
for every wheel count and mounting angle.

Use `getMotorCount()`, `getLastMotorPower(index)`, `getLastMotorPowers()`, and
`getMaxMotorPower()` for indexed output diagnostics. `Omni.setWheelPowers(...)` accepts one
power per registered wheel for direct output. Named front/back power getters describe only
the first four slots; shared tuning measurements include every motor.
Single-axis characterization converts wheel projections back to body-axis command units
before fitting gains, including when the configured output limit scales the motors.

The dedicated `Kiwi`, `KiwiFollower`, and `KIWI` drivetrain type have been removed.
Configure a three-wheel `Omni` instead, registering its motors in counterclockwise order
and setting `firstWheelAngle` and motor directions to match the physical layout.
Saved `KIWI` tuning settings load as `OMNI`; update Java hardware configuration explicitly.

Drivetrains can override `createDriveModel(settings, mode)` and
`createDirectionalKinematics(settings)`. Mecanum preserves its directional LUT by default;
omni derives directional limits from its actual wheel projections. Both online and stored
planning use these hooks, and execution uses the supplied drive model's power calculation.
See [custom drivetrains and directional kinematics](doc/CustomDrivetrains.md) for examples.

All localizers use a shared position, velocity and acceleration Kalman estimate by default,
with bounded automatic noise tuning from their existing sensor readings.
See [Kalman localization](doc/KalmanLocalization.md) for configuration and validation.

## Java package organization

The library sources are in `TeamCode/src/main/java`, with matching unit-test packages in
`TeamCode/src/test/java`:

| Package | Responsibility |
| --- | --- |
| `follower` | Public base lifecycle API, follower settings, and diagnostics |
| `follower.drivetrains` | Mecanum, swerve, omni, tank, dual-actuated, and shared holonomic followers; construction factory |
| `follower.internal` | Tracking engines, allocation, callbacks, completion, runtime state, and math; implementation details |
| `config` | Robot configuration contract (`ApexConstants`) |
| `storage` | Persisted tuning-data storage |
| `localizers` | Localization implementations, mode switching, and saved calibration |
| `feedforward` | Measured motor-response models and reference-power calculations |
| `trajectory` | Online planning, stored trajectory fields, and physical target samples |
| `trajectory.profile` | Motion-profile solvers, samples, and lookup tables |
| `trajectory.generators` | Shared path and turn profile generation and constraint scheduling |
| `tuning.follower` | Shared tuner context, phase contract, and measurement helpers |
| `tuning.follower.{sysid,characterization,feedback,limits,centripetal,tracking,acceleration}` | Each follower-tuning workflow and its supporting algorithms |
| `tuning.localizer.calibration` | Calibration data shared by localization adapters and phases |

Geometry, path construction, controllers, and drivetrain implementations retain their existing
packages. Robot OpModes remain under `org.firstinspires.ftc.teamcode.apexpathing`; desktop
simulation support remains on the test classpath.

Imports using the former `core`, `feedforward.generators`, or `tuning.follower.phases` packages
must be updated to the packages above. Motion-profile types formerly in `feedforward` now
live in `trajectory.profile`. Persisted tuning JSON and OpMode registrations are unchanged.

Use `GeometryFactory.tankPath(...)` or `GeometryFactory.holonomicPath(...)` to declare a path's
drive mode explicitly. The factory no longer chooses a builder from the drivetrain's current
state. On a dual-actuated drive, `Follower.follow(...)` switches hardware and tuning profiles to
match the built path; building the path itself does not actuate hardware. Turns can select their
profile with `setDriveProfile(...)`.

`AutoTest.java` defines separate tank and holonomic routes. `TankAutoTest` selects the tank route
and hardware, then runs the same `AutoTest` execution loop. There is no conditional factory path API.
Both tests now stop after their initial curve. The holonomic test uses heading nodes at 30%
(83 degrees) and 65% (-71 degrees), ending at 146 degrees, to exercise twisting in both directions.

Holonomic velocity control uses proportional feedback from the current velocity error.
Online execution uses a current-time reference with zero planner preview; stored
execution uses its constrained spatial reference. See [velocity feedback](doc/VelocityFeedback.md).

## Run the OpModes in FTCodeSim

The shared simulator setup is in `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim`.
It registers the four drivetrain motors plus the custom Apex Pinpoint and telemetry adapters used
by all three current OpModes. Install and open
[AdvantageScope 26.0.2](https://github.com/Mechanical-Advantage/AdvantageScope/releases/tag/v26.0.2)
at least once before launching FTCodeSim. In PowerShell, run:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest `
  --tests org.firstinspires.ftc.teamcode.sim.SimulateApexPathing
```

Before running it, set `RUN_INTERACTIVE_SIMULATOR` to `true` in
`SimulateApexPathing.java`. Set it back to `false` afterward so ordinary unit-test runs do not wait
indefinitely for the interactive simulator windows to close.

The simulated Driver Station presents `Apex Auto Test`, `Apex TeleOp Test`, `Follower Tuner`, and
`Localization Tuner`.
Select one, then press Init and Start; FTCodeSim initializes and runs only that selected OpMode.
Stop it before selecting another. This is an interactive JUnit test and remains active until the
simulator windows are closed. Tuner output from a simulation is stored under
`build/ftcodesim-data` instead of the Android device's external storage.

### Tank-drive simulation

Run `SimulateTankApexPathing.tuneAndTestTankPathFollowing` as a JUnit test to launch FTCodeSim
with differential-drive physics. The simulated Driver Station adds `Tank Localization Tuner`,
`Tank Follower Tuner`, and `Tank Apex Auto Test` under `Apex Tank Simulation`; run them in that
order. Tank tuning is stored separately under `build/ftcodesim-tank-data`, so it does not replace
the mecanum simulator's saved constants.

Do not select these tank-only OpModes from `SimulateApexPathing`; that launcher uses mecanum
physics. The tank OpModes detect the wrong launcher, and `Tank Apex Auto Test` also checks that
positive motion limits were saved by the follower tuner before following online movements or precomputing stored trajectories.

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest `
  --tests org.firstinspires.ftc.teamcode.sim.SimulateTankApexPathing
```

The tank simulator uses the drivetrain-backed Pinpoint adapter for localization because the
current FTCodeSim `SimMotor` reports no encoder position. Drivetrain motion is produced by
`SimulatedTank`, including its four tank motor channels and differential-drive kinematics.
The tank model uses 2-inch-radius wheels, a 14-inch track, and duty-independent back EMF
with moving-friction power 0.06, maximum speed 75 in/s, and maximum acceleration 150 in/s².
These are explicit simulation assumptions, not measurements of a physical robot. Rerun follower
tuning after this model change; constants from the previous high-friction model are incompatible.

FTCodeSim's original keyboard controls are preserved: arrows are the D-pad, WASD is the left
stick, `;` is gamepad A, `[` is B, `P` is X, and `-` is Y. Follower Tuner always opens its phase
options menu and requires `;` to accept the highlighted phase. Use D-pad Left/Right to choose
automatic or manual mode, A to select or advance, and X to run tests. Pressing Start first keeps
the menu open until a phase is explicitly selected. After Start, the sticks drive the robot
field-centrically on menus, prompts, results, and idle manual-tuning screens; tuner-controlled
motion takes exclusive control until its test finishes. All phases remain selectable for retuning.
Use Stop to cancel tuning or discard unsaved changes. The last accepted phase remains saved.
Automatic telemetry shows the constants being tuned and positioning instructions. The bottom
line says `Robot can be driven with sticks` only while stick driving is available.
See [tuner telemetry conventions](doc/TunerTelemetryConventions.md) for layout,
formatting, state-specific controls, and manual SysId error definitions.
After a phase's results are accepted, Follower Tuner saves them and advances through every
remaining phase in order: drive feedforward SysId, heading feedforward SysId,
[motion characterization](doc/MotionCharacterization.md), heading feedback,
drive feedback, movement limits, centripetal tuning (holonomic only), then trajectory tracking.
  Motion characterization measures holding power and acceleration authority versus speed,
  then validates a separate power sequence. Accepted maps constrain profile power and
  speed; the follower uses the same maps for powered acceleration and cruise.
  The final phase tests braking and tracking on a repeated corner course.
  Reposition the robot and press A to start each trial; it saves only
  measured improvements that pass repeat validation for the active drivetrain profile.
See [profiled tracking tuning](doc/ProfiledTrackingTuning.md) for the controls and metric definitions.

Each independent SysId phase identifies kS, kV and kA together with the model
`power = kS * sign(velocity) + kV * velocity + kA * acceleration`. It runs four open-loop
tests: quasistatic forward/reverse (0.1 power/s, at most 0.9 for 9 seconds) and dynamic
forward/reverse (0.5 power for at most 2 seconds). Press A from rest for each test; reposition
with the sticks between tests. Use Stop to cancel. Clear 96 inches in both
directions for drive, or room for two full turns for heading, with additional braking room.
Travel cutoffs stop collection automatically; the operator must stop before obstacles.

The fit uses signed 100 ms moving windows, integrating the preceding delivered motor power
and measured velocity, with acceleration calculated from the window's velocity change.
Acceptance requires at least five windows from each test, adequate speed range, identifiable
coefficients, nonnegative kS/kA, positive kV and R-squared >= 0.90. A saves the current
axis from the results screen. Rejected measurements require Stop and a fresh run of the phase.
No individual outliers are pruned.
Each axis saves independently. The separate acceleration-search phase is no longer in the workflow.
Both SysId phases also offer manual tuning: choose Manual with D-pad Left/Right, then press A.
LB/RB selects kS, kV or kA; D-pad Up/Down adjusts the value and Left/Right changes the increment.
X runs a fixed time-based trapezoidal velocity profile, alternating forward/backward for drive
and CCW/CW for heading. Each trial accelerates for 1 second, cruises for 1 second, then brakes
for 1 second. Drive targets 24 in/s (48 inches nominal travel); heading targets 1.5 rad/s
(3 radians nominal rotation). Follow the next-direction positioning prompt, release the sticks,
and start from rest. Stick positioning remains available between trials; Stop cancels.
Telemetry reports time-weighted mean signed velocity error (measured minus target) during
acceleration and during cruise from the first measured target-velocity crossing to cruise end.
If cruise velocity is never reached, cruise error is unavailable. Braking is excluded from both
scores. A power-clipping notice identifies trials whose requested feedforward exceeds the command
limit. A accepts the edited gains; press A again on the results screen to save and continue.
Position-controller breakaway settings remain separate. Velocity feedback is derived from the
identified plant after acceptance. Units remain normalized motor power, inches (drive), radians
(heading) and seconds; this is not WPILib voltage data and does not compensate battery voltage.

Raw and fitted CSVs are saved beside `constants.json` (`FIRST/ApexPathing` on the Robot Controller,
`build/ftcodesim-data` in simulation). Fit CSVs include all retained windows, predicted power,
residuals, coefficients, R-squared and acceptance eligibility. Stop closes the current log
without accepting the unfinished measurements.

Desktop simulation generates XChart PNGs automatically, using the same 1000x460 rendering,
legend, grid and line styling as `PathProfileConvergenceTest`. Each test gets a measured-versus-fitted
power graph, a velocity/acceleration graph and (when the fit is finite) a residual graph next to its
fit CSV. Hardware CSVs can be copied to the desktop and rendered by running the test-classpath
`tuning.follower.sysid.SysIdGraphs` main class with arguments `<fit.csv> <DRIVE|HEADING>`.
XChart stays off the Android runtime classpath. The `SysIdRegressionTest` graph test also writes
example PNGs under `TeamCode/build/sysid-graphs`; those examples use synthetic plant data.
The desktop simulator uses FTCodeSim's stock motor and chassis implementations. Mecanum physics
parameters match the upstream DECODE example. [Stock simulator validation](doc/StockFTCodeSimValidation.md)
records the source and fresh tuning. Earlier custom-plant braking graphs are historical and do
not validate stock FTCodeSim or a physical robot.
Profiled paths now use [continuous displacement-based braking](doc/ProfiledBrakingImplementation.md)
while retaining Apex's constrained profile generator and LUT interpolation.
Use [the profile power ceiling](doc/ProfilePowerLimit.md) to configure the generated
feedforward power budget per path or directly on a generator.

Automatic PDS tuning derives a critically damped starting point from the accepted kV/kA plant
model, then uses bounded Twiddle point-to-point tests to refine kP and kD. Every candidate is
tested in both directions, must beat the incumbent by a noise margin, and the best measured gains
are restored before operator validation. X runs a verification step after Twiddle; A accepts
the gains. The model result is only a seed, not an assumed final
answer. A response that has not settled after 2.5 seconds is scored as a poor candidate; successful
responses need only remain inside the position and velocity tolerances for 0.15 seconds. Between
responses, low-level velocity-estimator noise is tolerated after a short re-arm delay. PDS CSVs
record gains, cost, target, position, error, velocity and power. Automatic tuning saves
its measured winner without an extra validation run. Fit residual, tracking-quality,
and localization checks live under `TeamCode/src/test/java`; the PD tuner retains
the optional user-triggered X verification step.

## Localization tuner

Run `Localization Tuner` before follower tuning. It follows the same phase-picker pattern as the
follower tuner with compact prompts for the Driver Station. The available procedures
are a direction check, forward and strafe distance scale, rotation geometry, velocity/acceleration
filtering. Localization validation runs in JUnit. Phases that do not apply to the configured drivetrain
or localizer are shown as `N/A`.

The direction check saves the required encoder/pod reversals into localization calibration.
Distance procedures drive at fixed low power in both directions and ask for the physical distance
measured on the floor. Rotation ramps power until it first reaches 90 degrees per second, holds
that discovered power, then cuts power after five counterclockwise rotations. It uses a
hardware-map IMU named `imu` by default. Its relative
quaternion measurement does not require the hub mounting orientation; Dpad Left/Right selects a
manual reference if the IMU is unavailable or broken. Filter tuning lets the operator choose
adaptive Kalman or moving average. Kalman collection
includes still, free-drive, and stopped intervals, then displays the learned R/Q values and records
a CSV beside the constants files. The tuner reports evidence and leaves the choice to the operator.

Accepted results are saved immediately to `FIRST/ApexPathing/localization.json`, with a backup at
`localization.json.bak`. Follower constants remain in `constants.json`. A DualActuated robot shares
one localizer across both modes by default. Override `usesSharedLocalizer()` and
`localizerConstants(Profile)` in the robot's `ApexConstants` implementation when tank and
holonomic modes need separate localizers.

### Shared drive models

Followers and profile generation share one `MotionPowerCalculator` for reference power.
Hardware-independent drive models supply physical limits, wheel geometry, and power bounds.
Every drivetrain uses `PathProfileGenerator`; stationary turns retain `TurnProfileGenerator`. `new GeometryFactory(follower)` binds builders to the physical
drive and snapshots the appropriate tank or holonomic settings without actuating the robot.
For direct profile generation, use `path.createGenerator().generateRestToRest()`.
See [trajectory planning](doc/TrajectoryPlanning.md) for the model API and generator migration.

### Trajectory build modes

Use `.quickBuild()` for online planning from measured speed through a fixed arc-length
lookahead, or `.buildToMemory()` to precompute an interpolated vector field in RAM.
For stationary turns, `.quickBuild()` uses PID heading feedback without profiling;
`.buildToMemory()` precomputes the angular profile.
`.profiledBuild()` has been removed. Both modes use the same feedback and feedforward
control. See [online planning and stored fields](doc/TrajectoryPlanning.md) for lookahead,
grid configuration, permanent precomputation, diagnostics, and migration examples.
