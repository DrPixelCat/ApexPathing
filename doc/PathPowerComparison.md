# Expected versus actual path power

The validated combined-motion correction and current feedforward-only graphs are
described in [MecanumWheelFeedforward.md](MecanumWheelFeedforward.md). With its
optional wheel characterization enabled, the follower publishes the selected
four-wheel pulse peak before feedback and allocation. The independent planner
envelope remains in the trajectory LUT. Feedback-off runs use an independent
time reference, so their distance axis is scheduled path progress; feedback-on
runs retain measured spatial progress. These details supersede the axis-model
and spatial feedforward test descriptions below.

`AutoTest` records `expected_motor_power` alongside `command_power` in its outbound
CSV. The prediction comes from the trajectory generator's signed power model for
the published target, including translation, curvature and heading. It is retained
before follower heading compensation, endpoint blending, feedback and allocation.
Online trajectories evaluate their solved target; stored trajectories interpolate
the saved power channel alongside their motion channels.

Actual power is the maximum absolute command across all four wheels after drivetrain
allocation and clipping. Both signals are normalized motor power, not electrical watts.
Feedback corrections and saturation can make them differ; agreement alone does not
establish that the physical robot's feedforward model is accurate.

Running `org.firstinspires.ftc.teamcode.sim.AutoTestSimulationTest` writes XChart
graphs to `APEX_AUTOTEST_GRAPH_DIR` (or the test working directory):

- `AutoTest_Power_Tracking.png`: expected and actual power versus distance along the outbound path in inches.
- `AutoTest_Power_Tracking_Error.png`: actual minus expected power at the same distances.
- The stored-path run uses the prefix `AutoTest_Stored_Power_Tracking`.

Each render reports sample bias, RMSE and peak absolute error in normalized power
units. These statistics include endpoint capture and the final stopped command.
The distance axis uses each recorded sample's actual path progress, so repeated
distances at launch or settling are retained rather than averaged.

To graph a CSV copied from the robot, run the desktop test-classpath main class
`org.firstinspires.ftc.teamcode.sim.AutoTestPowerGraphs` with arguments
`<trace.csv> [output-directory]`. Record a fresh CSV after this update: older logs
contain `profile_power`, which is controller feedforward demand rather than the
independent planner prediction. XChart remains a desktop test dependency.

## Straight line with feedback disabled

Run `org.firstinspires.ftc.teamcode.sim.LinePowerSimulationTest` to follow a
80-inch forward line with constant heading using the simulator calibration.
Both online and stored trajectories are exercised. `disableControllers()` removes
holonomic position and velocity corrections, endpoint breakaway and endpoint
blending; feedforward remains active. The test keeps configured feedback gains
nonzero and asserts that every recorded correction and endpoint blend is zero.
It records the trace until completion or a 15-second timeout, then stops the motors.
Endpoint accuracy and settling are reported without assuming feedback-free
motion will satisfy the usual endpoint completion criteria.

The output directory is `TeamCode/build/path-power-graphs` by default, or
`APEX_AUTOTEST_GRAPH_DIR`. Each mode writes a CSV and three XChart PNGs with
prefix `Line_Online_Feedforward_Only` or `Line_Stored_Feedforward_Only`:
power comparison, `_Error`, and `_Velocity`. The velocity graph includes the
localizer measurement and independent simulator plant velocity. No artificial
terminal zero-power row is compared against the previous moving target.

The plant advances by actual elapsed wall time in at most 5 ms substeps,
matching the localizer's differentiation interval. A fixed 20 ms physics step
with a slower host loop underreports position-derived velocity and can obscure
the distinction between estimator lag and physical braking error.

For example, run:

```text
gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.LinePowerSimulationTest
```

Matching expected and commanded power checks planner/executor agreement, not
plant identification. The speed trace checks the resulting motion. Online
planning still adapts its target to measured motion, while stored planning
uses a fixed spatial envelope; neither is a fixed time-based open-loop profile.

### Straight line with a twist, feedforward only

`StraightTwistFeedforwardSimulationTest` runs the stored 80-inch straight line
from (-40, 0, 0 degrees) to (40, 0, 90 degrees). `SMOOTH_START_TO_END` supplies
the full-path cubic heading transition while the geometric path remains straight.
The follower uses the simulator calibration and has all translation and heading
controllers disabled. Angular demand is taken directly from the stored reference;
the usual measured-progress-speed compensation is bypassed when drive feedback
is disabled. Nonzero configured feedback gains are retained to check the disable
switch itself.

Every sample checks zero heading position and rate feedback, and matches the
angular velocity and acceleration against an independent stored-field sample.
The trace also verifies zero translation correction and endpoint blend. The
centered braking model is installed with measured simulator coefficients, but
only applies inside its calibrated forward/reverse body-axis domain; a rotated
strafe braking command uses the existing feedforward model.

Run:

```powershell
$env:APEX_AUTOTEST_GRAPH_DIR = Join-Path (Get-Location) 'TeamCode/build/path-power-graphs/straight-twist-feedforward'
./gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.StraightTwistFeedforwardSimulationTest
```

Files use the prefix `Straight_Twist_Stored_Feedforward_Only`: the CSV, peak power
comparison PNG, `_Error`, `_Velocity`, `_Heading`, `_Angular_Velocity`, and `_Path`.
Motion charts include independently read plant values alongside localizer values.
The diagnostic reports translation, angular and heading RMSE, sideways drift,
power allocation limiting and centered model activation. It does not require
feedback-free endpoint accuracy; the run stops on follower termination or after
15 seconds. Stored targets still follow measured spatial progress.

The first recorded run reached 80 inches of path progress but finished with an
actual heading of 9.14 degrees against the 90-degree target. Angular-rate RMSE
was 0.255 rad/s (peak actual 0.108 versus target 0.470 rad/s), translation-speed
RMSE 7.22 in/s, and maximum sideways error 3.32 inches. Every feedback term and
endpoint blend stayed zero. Peak-command prediction RMSE was 0.131, with ten
allocation-limited samples. These are diagnostic results, not a motion-accuracy
acceptance pass. They expose a substantial error in combined drive/rotation
feedforward. The tests check that the intended motion and feedback isolation
were exercised, rather than requiring the current model to track accurately.

Further controlled probes and wheel-level logging are documented in
[StraightTwistDiagnosis.md](StraightTwistDiagnosis.md). The twist run also produces
`_Turn_Power.png`, comparing requested turn feedforward to the signed demand
reconstructed from the four delivered wheel commands. `followerStopped` in the
summary means execution ended; it does not assert endpoint accuracy.

## Braking feasibility experiment

`org.firstinspires.ftc.teamcode.sim.BrakingFeedforwardDiagnosticTest` isolates
braking on the stock simulated plant. It fits eight response coefficients from
30 short acceleration probes: separate speed-dependent power authority and drag
for commands with and against travel. It checks different speeds and powers in
both directions using 32 held-out probes. It never reads or changes plant
coefficients. Exact zero power is excluded because it selects a separate motor
brake mode.

The experiment follows the same fixed 80-inch spatial profile with either the
current braking feedforward or the measured candidate, keeping powered
acceleration and cruise commands the same. No velocity or position correction
is applied. When a requested deceleration lies between the two measured power
regimes, a 50 Hz feedforward duty cycle alternates small forward and reverse
commands; the duty derives from the reference state alone.

The initial October 4, 2026 run measured braking-only speed RMSE of 9.241 to 4.039 in/s
forward and 9.369 to 4.196 in/s reverse, reductions of approximately 56% and
55%. Peak braking speed error fell from 14.559 to 7.218 in/s forward and
14.673 to 7.660 in/s reverse. Both comparisons passed the required 50% RMSE
reduction; the two ordinary line tests also passed with all feedback at zero.

CSV probes, raw path traces and before/after XChart graphs are saved under
`TeamCode/build/path-power-graphs/braking-diagnostic`. Run:

```text
gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.BrakingFeedforwardDiagnosticTest
```

This deterministic diagnostic supplies simulator calibration to the optional
execution model described below. The coefficients are not persisted as robot
tuning. Robot use requires independently measured braking regimes, including
the neutral-power mode and low-speed behavior.

### Centered braking and updated power prediction

The revised simulation starts reverse-pulse duty accumulation at 0.6 rather
than zero, removing the previous tendency to delay the first braking pulse.
It adds a 2.5 ms reference lead for the 20 ms command interval, tapering that
lead below 12 in/s. Both changes operate on the profile reference alone; no
velocity or position error term is added and the plotted distance coordinates
are unchanged.

Against the preceding measured model, forward braking RMSE fell from 4.039 to
0.802 in/s and signed bias from +3.676 to +0.367 in/s. Reverse RMSE fell from
4.196 to 0.920 in/s and bias from +3.816 to +0.543 in/s. The regression requires
RMSE below 30% of the preceding model, absolute bias below 0.6 in/s, and full
line traversal without stopping short, in both directions.

The `forward-centered.csv` and `reverse-centered.csv` logs include predicted
mean signed power, predicted pulse power, independently read applied motor
commands, and expected/applied absolute peak command. The new prediction uses
the revised signed braking model and pulse policy, rather than the old stored
profile's interpolated power column. Expected pulse peak and applied peak agree
to numerical precision in this simulation; that checks actuator command
delivery, not physical plant prediction or electrical power.

Each direction also produces `_Braking_Power.png` for expected versus applied
peak power and `_Braking_Signed_Power.png` for predicted mean power, actual
pulses and matching trailing 100 ms averages. `_Detail` versions zoom into
braking on the velocity and signed-power graphs. Both averages use the same
five samples, with shorter windows only at the start; averaging does not shift
the velocity curves or alter the data used for braking error statistics.

### Apply the model to Apex AutoTest

`CenteredBrakingFeedforward` implements the centered braking policy for the
holonomic follower. Install it with `Follower.setBrakingFeedforward(model)`;
calibration coefficients and the calibrated speed domain are supplied explicitly.
The follower resets pulse state when starting a movement, pausing or resuming.
Forward and reverse body-axis braking use the measured model. Acceleration,
strafe and turn control retain their existing models.

`CenteredBrakingAutoTestSimulationTest` runs the actual five-stage AutoTest
OpMode using measured simulator coefficients and normal feedback. AutoTest's
`configureFollower` hook installs the model before execution. Its outbound path
is stored in memory, and the existing completion and timing checks remain active.
The October 4 verification run completed all five stages and logged 100 outbound
samples using centered braking. Outbound braking velocity RMSE was 2.24 in/s,
with signed bias -1.32 in/s. Power prediction RMSE was 0.147 normalized command
units. The host-timed simulation produces some variation between runs.

During centered braking, `expected_motor_power` is recalculated from the new
signed pulse demand, plus curvature and heading feedforward, before feedback,
endpoint blending or motor-command allocation. It is an execution feedforward
prediction; the trajectory's feasibility envelope still uses its existing
planning model. Delivered wheel commands can differ because normal feedback
and allocation are active. `centered_braking_active` and
`centered_braking_mean_power` expose the model's activation and signed mean
tangent power in the outbound CSV.

Run:

```powershell
$env:APEX_AUTOTEST_GRAPH_DIR = Join-Path (Get-Location) 'TeamCode/build/path-power-graphs/apex-autotest-centered'
./gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.CenteredBrakingAutoTestSimulationTest
```

The directory contains `AutoTest_Centered_Braking_Velocity_Distance.png` and
`AutoTest_Centered_Braking_Velocity_Tracking.png`, plus
`AutoTest_Centered_Braking_Power_Tracking.png` and its `_Error.png` graph.
These charts cover the outbound path; the test verifies all five movements.
