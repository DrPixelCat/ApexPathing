# Current-time velocity feedback

Holonomic velocity control applies proportional feedback to the current velocity
error: `u_velocity = effective_gain * (reference_velocity_now - measured_velocity_now)`.
Online execution integrates held planned acceleration and compares the reference
with velocity at the same sensor timestamp. Its gain is bounded by observed timing;
there is no acceleration-dependent error scaling or future velocity target.
The [spatial velocity pursuit](VelocityPursuit.md) design remains an offline experiment;
normal following does not activate it, including when loading an old saved pursuit setting.

Holonomic online execution uses a continuous velocity reference at the
measurement time. The planner still solves a stopping horizon from the measured
state; its published velocity has zero temporal preview. Stored execution keeps
its constrained spatial reference. Feedforward-only execution keeps the independent
time reference used to isolate the motor model. Tank execution retains its existing
controller and preview policy.

## Why the previous target biased acceleration

The rolling planner previously published

```text
v_target = v_measured + tau * a_planned,   tau = 0.050 s
u_feedback = k_v * (v_target - v_measured).
```

This manufactures an error `tau * a_planned` even when motion follows the plan
perfectly. It also resets that error to the latest measurement on each solve,
so it does not preserve a speed disturbance for the controller to reject.

For an accurate acceleration inverse, corrective normalized authority `u` adds
`b*u` to chassis acceleration. Consequently the old policy approximately gives
`a_actual = (1 + b*k_v*tau) * a_planned` before allocation. In the measured
simulator fixture, `b = 183 in/s^2` per unit authority and `k_v = 0.0446484`,
so `b*k_v = 8.17065 /s` and the multiplier is **1.4085**. This explains a
systematic acceleration/braking correction without invoking motor-model error.
Setting the target directly to the latest measurement would remove this bias
but would also remove tangential velocity disturbance rejection.

## Reference and closed-loop reasoning

`follower.VelocityReference` remembers the reference and the acceleration planned
for the interval that actually elapsed:

```text
r[k+1] = max(0, r[k] + h[k] * a_planned[k])
e[k]   = r(t_measurement[k]) - v_measured[k]
u[k]   = k_v * e[k].
```

On acquisition it seeds the reference from measured speed. After that, replanning
updates acceleration without replacing reference velocity with measured velocity.
The planner remains anchored in the measured physical state, so speed limits,
stopping feasibility and recovery are still evaluated from actual motion.

For a scalar tangential plant with exact feedforward, synchronous measurements,
no clipping, a fixed tangent and no reference stop clamp, let `d[k]` be extra
acceleration unaccounted for by the model. Over a held command:

```text
v[k+1] = v[k] + h[k] * (a_planned[k] + b*k_v*e[k] + d[k])
e[k+1] = (1 - lambda*h[k]) * e[k] - h[k]*d[k]
lambda = b*k_v.
```

Thus perfect planned motion produces zero correction, and a speed disturbance
decays rather than disappearing from the reference on the next solve. The ideal
discrete loop is stable for `0 < lambda*h < 2`; `lambda*h <= 1` avoids alternating
error in that ideal case. At 20 ms the fixture's pole is approximately **0.837**
and its continuous approximation has a **122 ms** time constant. A constant
unmodelled load leaves the proportional steady error `e = -d/lambda`.

This derivation is conditional: sensor filtering, changing path tangents,
motor-domain errors, saturation and latency change the dynamics. The implementation
does not claim this scalar proof establishes stability for arbitrary hardware.

## Timing, allocation and lifecycle

The reference stores 32 timestamped held-acceleration intervals without allocating
per update. It queries the interval covering the localizer's accepted acquisition
timestamp, so delayed measurements are compared with past reference motion,
never a guessed future speed. Uncovered, invalid or older-than-250-ms measurements
reset the reference and suppress its velocity correction. Backward control time
or a control gap above 250 ms reseeds the reference. New paths, pause/reset and
disabled online drive feedback also reset it.

The configured gain is capped by `1 / (b * (control_interval + known_sensor_age))`
when `b` is available, using a 1 ms denominator floor. This bounds correction
bandwidth conservatively for observed timing; it does not compensate unknown
sensor-filter delay or predict the next loop's duration.

After allocation, let `f` be the delivered fraction of velocity correction,
including endpoint fading, the relevant allocation stage and final wheel-force
scaling. The latest reference is conditioned by `r <- max(0, r - (1-f)*e)`.
This removes the undelivered portion of reference error, preventing an unattainable
speed demand from accumulating during saturation. It preserves older timestamped
reference history. Braking correction uses the corrective allocation stage;
other tangential correction uses the velocity stage.

The existing wheel model receives feedback as chassis acceleration before its
nonlinear motor inverse. Existing feedforward prediction continues to report raw
wheel demand before feedback, fading and allocation. The small held-command
force preview in the simulator calibration remains separate from velocity error:
its 7.5 ms value models the mean evaluation time of a 20 ms held command stepped
in 5 ms physics intervals. It is not a 50 ms velocity target.

Heading and drive PD controllers now use the follower's monotonic clock. Normal
execution still uses `System.nanoTime`; deterministic simulation uses the same
virtual timeline for controllers, sensors and physics.

## Validation design

The root-cause hypothesis was tested before integrating the new follower logic.
`VelocityFeedbackExperimentTest` compares no velocity correction, measured-speed
20/50-ms preview policies, and the continuous current-time reference on a fixed
80-inch line with a smooth 90-degree twist. It uses the unchanged stock plant,
the same calibrated motor inverse and the same angular controller for every
candidate. Scenarios add sustained 4 in/s^2 drag or a sudden 5 in/s speed loss
with 10/30/20-ms command intervals. The assertion compares tracking against the
independent fixed-time physical target. This experiment tests the old feedback
policy in isolation; it is not a full old-Apex-controller benchmark.

`VelocityFeedbackSimulationTest` then exercises the production follower and
native sensor on the actual single-path Apex geometry with heading nodes
83 degrees at 30%, -71 degrees at 65%, and the 146-degree end heading. It checks
completion, callback delivery, velocity and heading against the independent
plant, cross-track and endpoint errors. It repeats the three disturbance cases,
checks stored execution, and compares 0/20/50-ms planner previews with the new
controller held fixed. Preview changes only the test planner through reflection;
there is no new production preview tuning switch.

Velocity/heading RMS statistics in this deterministic test exclude endpoint
blending above 5%; the graphs and power statistics include the entire trace.
Normal endpoint acceptance remains unchanged and can finish before full rest.
The stricter feedforward-only tests execute the complete braking schedule to rest.
Power plots show peak normalized motor commands, not electrical watts; their
residual includes feedback, endpoint fading and allocation, not just model error.

Unit regressions cover perfect acceleration/braking with delayed and repeated
sensor timestamps, disturbances, allocation conditioning, invalid/stale data,
gain limits and reference stopping. The curved-planner regression verifies
that planning and feedback use exactly the same tangential projection.

## Repeatable results

The isolated fixed-target experiment produced these velocity RMS errors in in/s:

| Feedback policy | Clean | Extra drag | Speed loss and uneven timing |
| --- | ---: | ---: | ---: |
| No tangential correction | 0.240 | 8.761 | 3.785 |
| Measured speed + 20-ms acceleration | 2.567 | 7.594 | 2.739 |
| Measured speed + 50-ms acceleration | 11.419 | 6.366 | 4.211 |
| Continuous current-time reference | 0.052 | 0.357 | 0.435 |

These are measurements of the isolated feedback policies, not old-controller
Apex-route error estimates. The current-time policy beats the other policies in
each scenario, with velocity RMS below 0.6 in/s; both conditions are asserted.

Production-follower results on the Apex twisting path are:

| Case | Velocity RMS (in/s) | Heading RMS (deg) | Endpoint error (in) | Power residual RMS |
| --- | ---: | ---: | ---: | ---: |
| Online, clean | 0.230 | 0.342 | 0.043 | 0.04365 |
| Online, extra drag | 0.401 | 0.346 | 0.073 | 0.04779 |
| Online, speed loss and uneven timing | 0.485 | 0.344 | 0.083 | 0.05037 |
| Stored, clean | 0.373 | 0.368 | 0.294 | 0.03830 |

All four complete the path and callback, and satisfy the tracking and endpoint
assertions. The clean online traversal takes 10.00 s; the other cases take
9.96-10.02 s. Stored power residual remains slightly smaller: its raw prediction
and delivery still differ through feedback and allocation, and correlation alone
does not measure speed accuracy.

With the new feedback controller fixed, the explicit planner-preview comparison is:

| Planner preview | Clean velocity RMS | Drag velocity RMS | Speed-loss velocity RMS | Clean heading RMS (deg) | Clean power residual RMS |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 ms | 0.230 | 0.401 | 0.485 | 0.342 | 0.04365 |
| 20 ms | 0.215 | 0.413 | 0.470 | 0.361 | 0.04536 |
| 50 ms | 0.227 | 0.411 | 0.493 | 0.405 | 0.05100 |

Velocity columns are in in/s. Twenty milliseconds improves two velocity results
by about 0.015 in/s, but increases heading and power residual RMS in every
scenario. Fifty milliseconds shows no consistent velocity benefit and also
increases both heading and power residual RMS in every scenario. These tests
do not justify adopting a future-speed target, so the default remains zero.

The original controller, follower, feedforward and trajectory verification passed
**151 tests**, with **one wall-timed diagnostic skipped** after a 138-ms host
interval exceeded its existing 50-ms eligibility threshold. That diagnostic
now checks timing eligibility before trace-length and motion assertions; the
threshold and deterministic assertions are unchanged. The four deterministic
feedforward-only twist variants retain velocity RMS 0.214 in/s, terminal heading error
0.040 degrees, lateral drift 0.030 in and endpoint displacement 0.369 in.

## Wall-timed acceptance remains open

The normal OpMode tests were also run on the desktop. Two earlier repetitions
completed both online and stored paths but exceeded the existing 23-ms average
loop guard. An additional online trace assertion exposed two different tangent
projections; the planner now uses the exact distance-indexed tangent published
to feedback, without an additional reconstructed `PathPoint`. The strict trace
tolerance and existing geometry-allocation bound are retained.

The last wall-timed repetition failed online with `No feasible braking command
for measured overspeed 21.7491`. Its partial trace contains a control gap of
approximately one second, well beyond the designed command schedule and reference
reacquisition threshold. A long-held command can carry the robot past a braking
boundary; reseeding velocity feedback cannot make an already unreachable
trajectory feasible. This planner recovery limitation is not resolved by the
velocity-reference change. Stored execution completed but exceeded the timing
guard at 42.38 ms per loop. Neither wall-timed test passed final acceptance.

The earlier completed online trace and its graphs remain in `wall-timed`; the
last online partial trace is `AutoTest_Online_Incomplete.csv`. `completed-results.xml`
preserves the completed repetition and `test-results.xml` records the final
failures. These host-dependent runs are reported separately from the deterministic
plant comparisons. Hardware tracking and scheduling have not been validated.

Graphs, CSV traces and preserved test reports are in
`TeamCode/build/path-power-graphs/velocity-feedback-current`. The isolated
experiment writes `TeamCode/build/path-power-graphs/velocity-reference-experiment`.
Wall-timed OpMode artifacts are in the `wall-timed` subdirectory.

The later slope-scaling experiment is [archived](VelocityErrorProjection.md).
Its production helper and saved setting have been removed; old projection and
pursuit settings are ignored. Current-time proportional feedback is restored.
Fresh restoration graphs and reports are in
`TeamCode/build/path-power-graphs/velocity-current-time-restored`.
The restored controller reproduces the velocity and power RMS values above.
Fresh verification ran 157 tests: 156 passed, none failed, and one wall-timed
feedforward-only diagnostic was skipped after a 61.4 ms host interval exceeded
its existing 50 ms scheduling guard. The four deterministic feedforward-only
twist variants still pass. Wall-timed Apex OpMode acceptance remains open.

```powershell
$env:APEX_AUTOTEST_GRAPH_DIR = Join-Path (Get-Location) 'TeamCode/build/path-power-graphs/velocity-feedback-current'
./gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.VelocityFeedbackExperimentTest --tests org.firstinspires.ftc.teamcode.sim.VelocityFeedbackSimulationTest
$env:APEX_AUTOTEST_GRAPH_DIR = Join-Path (Get-Location) 'TeamCode/build/path-power-graphs/velocity-feedback-current/wall-timed'
./gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.AutoTestSimulationTest
```
