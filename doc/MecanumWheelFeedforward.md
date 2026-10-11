# Coupled mecanum feedforward

Holonomic online velocity feedback now uses a continuous current-time reference
and zero planner preview. See [VelocityFeedback.md](VelocityFeedback.md) for
the derivation and new single-path Apex tests. Its current graphs are in
`TeamCode/build/path-power-graphs/velocity-feedback-current`.

The ApexAutoTest route was subsequently simplified to its initial curve, with
heading nodes at 30% (83 degrees) and 65% (-71 degrees) and a 146-degree end
heading. The five-movement results below describe the earlier route. The new
twisting-route graphs are in `TeamCode/build/path-power-graphs/apex-autotest-twist-nodes`.

The straight 80-inch path with a smooth 90-degree twist now follows the target in
the stock simulator with position, heading, velocity feedback and endpoint blending
disabled. The repeatable 20 ms run finishes at **89.960 degrees**, with velocity
RMSE **0.214 in/s**, angular velocity RMSE **0.00223 rad/s**, lateral drift
**0.030 inches**, and longitudinal endpoint error **+0.369 inches** in the follower pose. It executes
the complete braking reference through zero speed. No samples require power
allocation to reduce feedforward. These are simulator results, not hardware measurements.

## Root cause and evidence

The previous implementation inverted separately characterized forward, strafe and
angular responses, then added their powers. This duplicates isolated-axis friction
offsets and misses the motor interactions caused by simultaneous wheel motion.
It also switches angular braking to incompatible legacy coefficients. Controlled
probes, described in [StraightTwistDiagnosis.md](StraightTwistDiagnosis.md), show
these errors even without a follower, allocator, clipping or localization.

`MeasuredMecanumExperimentTest` established the replacement before integration:
90 imposed-state input/output probes identify common wheel authority, speed loss,
regenerative/neutral braking, forward/strafe/angular chassis drag and effective
turn radius. It does not read motor or chassis coefficients to fit the model, and
does not modify the stock motor/chassis configuration. Separate combined-motion
probes validate 649 feasible states, including negative directions, tiny strafe
components, angular braking and wheel power-sign transitions. Across 1,947
acceleration components, RMSE is about 4e-12 and peak error about 2e-11 in the
respective acceleration units. The near-exact fit reflects this particular
simulator's mechanics; it is not an expected hardware accuracy.

A fixed-reference dynamic prototype then completed the whole twist without error
feedback, with velocity RMSE 0.147 in/s, heading 89.683 degrees and lateral drift
0.021 inches. Production integration was started after these checks passed.

## Calculation order

`feedforward.MecanumWheelModel` first combines the desired field acceleration
(rotated into the body frame) with chassis drag along the combined linear motion
direction and angular drag. It maps those chassis forces to four wheel forces,
then inverts each motor response using that wheel's velocity and power regime.

For wheel velocity `w` and normalized power `u`, the measured response has a
powered branch `gain*u - speedLoss*w*abs(u)`, a regenerative branch
`gain*u - regenerativeBraking*w`, and a zero-command brake branch. A response gap
between powered and regenerative commands is covered by duty-weighted positive
and negative pulses. Pulse debt belongs to each follower and resets with its
execution lifecycle. It is not an error controller.

The four powers are retained through allocation and sent to the drivetrain
directly. Projecting them back into only forward/strafe/turn powers would discard
the fourth wheel-power mode needed by the nonlinear motor inverse. With feedback
enabled, allocated corrective authority is converted to chassis acceleration
before wheel inversion. Saturation scales chassis force, then solves the motor
regimes again. Adding corrective power after inversion can otherwise switch a
wheel's braking regime and invalidate the physical prediction.

`PathProfileGenerator` delegates to `HolonomicDriveModel`, which uses the same wheel model for power utilization, wheel-speed
domains and exact affine acceleration bounds. On straight geometry it checks held-acceleration
intervals at both endpoints and both speed bounds, preventing the
launch demand from overrunning power between solver rows. A separate test samples
2,000 time states between the rows, including the held-command preview.
Curved paths retain the existing geometry sampling: a crossed geometry/speed box
can falsely reject reachable states where curvature changes quickly. Curved
feedforward-only interval feasibility has not been established by the straight
test; the normal-feedback five-movement AutoTest covers execution on the curved route.

## Feedback-off reference and timing

For a mecanum path with a validated wheel model, `disableControllers()` selects
an independent rest-to-rest time reference. Online-built paths generate that
reference once; stored paths reuse their nominal LUT. There is no measured-speed
replanning or angular time compensation. Measured heading supplies the coordinate
transform, without adding heading error correction. Callbacks use scheduled
reference progress in this mode.

Time interpolation preserves `ds/dt=v` and constant acceleration within each
interval. The schedule continues through its planned zero-speed endpoint instead
of freezing when the robot stops slightly short. The usual endpoint completion
policy can stop motors early at up to 8 in/s; this mode waits for the reference
to finish and velocity to settle below 0.5 in/s and 0.1 rad/s.
The feedforward preview samples that future reference, including interval and
stop boundaries. Extrapolating current braking acceleration past zero velocity
would incorrectly select a reverse-motion friction regime just before stopping.

Repeatable acceptance uses a synchronized simulation clock for trajectory and
sensor acquisition, with 20 ms controls and 5 ms plant integration. It exercises
the production follower, allocator, drivetrain and stock simulator. Forward,
reverse, opposite-twist and online-built cases produce symmetric results. A
separate wall-timed diagnostic advances physics by actual elapsed time. Host
pauses have reached 849 ms while a previous power was held; such missed deadlines
cannot establish feedforward model accuracy. That diagnostic records its timing
and is marked inconclusive when an interval exceeds 50 ms, without weakening
the mandatory repeatable accuracy checks.

## Calibration and adoption

The optional `motionCharacterization.mecanumWheelModel` JSON object stores the
measured response, effective radius, maximum wheel-speed domain and validation
error. The simulator fixture adds this object to its existing characterization.
Existing axis-only profiles continue to use their existing implementation.
The follower falls back to bounded legacy allocation if the wheel inverse leaves
its validated domain, and exposes `isWheelFeedforwardActive()` for diagnostics.
This update covers holonomic path execution; point turns retain their turn controller.

`commandPreviewSeconds` defaults to zero and must match the intended execution
scheme. The simulator fixture uses 0.0075 s, the mean integration evaluation time
within a 20 ms command held in 5 ms steps. This value is not a universal hardware
latency. Physical robots need their own wheel-response and combined-motion
validation before enabling this model. Only the simulator test fixture contains
these measured coefficients; robot configuration is not overwritten.

## Power and graphs

The stored planner retains its power envelope. The follower's
`expected_motor_power` publishes the selected wheel feedforward pulse peak before
feedback, endpoint fading or allocation. With feedback enabled, inversion uses
measured native wheel motion; feedback-off inversion uses the independent
reference motion. `command_power` is the peak delivered normalized motor command,
not electrical watts. Agreement checks delivery; velocity and heading checks
establish the resulting physical response.

The repeatable twist's peak-power prediction RMSE is 0.000138 normalized power,
with maximum discrepancy 0.000993 from the drivetrain's command cache. Graphs and
CSV files are in `TeamCode/build/path-power-graphs/straight-twist-fixed`:

- `Straight_Twist_Stored_Feedforward_Only_Velocity.png`
- `_Velocity_Distance.png` additionally places the measured/plant curves at actual
  travelled distance, exposing residual braking displacement instead of aligning
  every series to the scheduled reference distance
- `Straight_Twist_Stored_Feedforward_Only_Heading.png`
- `Straight_Twist_Stored_Feedforward_Only_Angular_Velocity.png`
- `Straight_Twist_Stored_Feedforward_Only.png` and `_Error.png` for power
- `_Turn_Power.png` and `_Path.png` for command delivery and lateral drift

Reverse, negative-twist, online and wall-timed variants have distinct prefixes.
The five-movement AutoTest runs with normal feedback write their updated graphs
to `TeamCode/build/path-power-graphs/apex-autotest-wheel-model` when configured
as below. The earlier diagnosis graphs remain in their original directory.

```powershell
$env:APEX_AUTOTEST_GRAPH_DIR = Join-Path (Get-Location) 'TeamCode/build/path-power-graphs/straight-twist-fixed'
./gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.MeasuredMecanumExperimentTest --tests org.firstinspires.ftc.teamcode.sim.StraightTwistFeedforwardSimulationTest
$env:APEX_AUTOTEST_GRAPH_DIR = Join-Path (Get-Location) 'TeamCode/build/path-power-graphs/apex-autotest-wheel-model'
./gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.AutoTestSimulationTest
```

The offline desktop run uses the already cached Gradle 8.9 distribution. Core
checks include model serialization/domain rejection, tiny-direction and braking
continuity, measured held-out response, force-space saturation, preservation of
four wheel powers, reference independence/pause behavior, existing feedforward,
follower and trajectory regressions, and the stock-plant guard.

Final core verification passed 129 checks, with one wall-timed diagnostic skipped
for a 370 ms host interval. ApexAutoTest completed all five movements in both
online and stored runs. Its wall-timed benchmark passed earlier (20.7/22.2 ms
average loops, 15.7/16.2 s routes); the last repetition exceeded its 23 ms loop
guard (24.3/35.9 ms). The timing guard remains in place. Repeatable feedforward
accuracy is established independently of those host-timing variations.
