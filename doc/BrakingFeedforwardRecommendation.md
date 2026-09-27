# Braking feedforward recommendation

## Speed-dependent profile and corner entry

The mecanum generator already performs forward and backward velocity-envelope
passes with a motor-power check. The simulator's original `translationKV`
(`0.00624`) understated the *effective* power cost of high-speed travel in the
full drivetrain. At the large acceleration peaks, planned demand reached
roughly 1.5–2.0 power while the wheel command saturated near 1.0. The robot
then arrived at the velocity crest 9–10 in/s below the planned value.

`profileVelocityPowerCoefficient` lets every drivetrain generator use a separate
speed-dependent power slope in its local velocity ceiling and reachability
checks. The selected simulator value is `0.014`; the follower's existing
velocity feedforward coefficient is unchanged. This is an effective full-path
calibration, including the drivetrain's back-EMF behavior, rather than a
direct measurement of one motor's electrical constant. At speed `v`, the
generator still charges heading, lateral, static, and acceleration power in
addition to `profileVelocityPowerCoefficient * v`.

`profileCornerSpeedScale = 0.80` smoothly lowers the local ceiling as
curvature grows, with the full scale applied by curvature `0.08 in^-1`.
The backward sweep propagates this lower ceiling into the approach. With
`TOLERANCE_BRAKE`, a tighter `brakingCornerToleranceIn = -1.5` starts event
braking once a decelerating path reaches curvature `0.005 in^-1`. Negative
tolerance deliberately allows a pulse while measured speed is still up to
1.5 in/s below target, so the corner entry favors underspeed.

The selected FTCodeSim calibration is saved in
`TeamCode/src/test/resources/simulator/mecanum-alpha1.json`. The profile
slope and corner margin apply to mecanum, swerve, and tank while retaining
each drivetrain's directional and traction limits. `TOLERANCE_BRAKE` is now
the default profiled response. Tank uses a 0.10 pulse and a 3 in/s corner
tolerance; mecanum and swerve use a 0.30 pulse. These values persist in
follower JSON. The 0.014 speed slope is specific to the simulator; other
robots use their tuned `translationKV` unless a separate profile coefficient
is supplied.

| AutoTest measure | Earlier event brake | Selected profile and corner brake |
| --- | ---: | ---: |
| Acceleration RMSE | 6.56 in/s | 1.47 in/s |
| Braking RMSE, matched positions | 4.78 in/s | 2.59 in/s |
| Three large peak target/actual pairs | 64.8/56.2, 64.8/55.2, 63.1/52.7 in/s | about 58/56, 58/56, 56/53 in/s |
| Curved braking bias | +3.38 in/s | -1.56 in/s |

In the selected AutoTest run, all 37 curved braking samples were below their
target. The frozen saved-tuning corner fixture also passed: peak cross-track
error was 0.17 in, curve heading peak 8.7 degrees, braking RMS 3.17 in/s,
and curved braking bias -0.80 in/s. The new profile takes about 10.7 s for
the outbound path versus 9.6 s before; the slower crest is intentional
because the old target could not be reached in the available distance.

## Preventing a speed rebound during braking

The event brake's original between-pulse command could rise well above the
power needed to hold the current speed. After a brake pulse, velocity feedback
then accelerated the robot before the next pulse, producing the
decelerate-accelerate-decelerate shape. `TOLERANCE_BRAKE` now caps that command
at `kS + kV*measured_speed - 0.02` while the requested tangential acceleration
is below -5 in/s². The cap relaxes braking as the robot slows but does not
request a strong recovery drive. The one-cycle overspeed brake pulse remains.
Near a nearly flat velocity floor, the cap releases so the follower can keep
moving into the next segment.

On completed AutoTest runs at the 100 in/s² profile limit, the previous
event response produced 13 raw speed increases above 1 in/s between adjacent
samples with sustained requested deceleration below -20 in/s²; the capped
response produced none. Its largest increase was under 0.5 in/s, within the
noise and curve projection seen in this simulation. Matched-position braking
RMSE was 4.75 and 5.07 in/s in two capped runs versus 5.01 in/s for the
previous event run using the same comparison reference. The corner fixture passed with
0.36 in peak cross-track error and 5.00 in/s braking RMS. A regression check
now asserts that raw speed does not rebound by 1.5 in/s during sustained
intervals in which the target speed is actually falling.

This is a simulation-based limit rather than a mathematical guarantee that
physical speed is monotonic. Path corrections, heading control, measurement
noise, and near-flat profile transitions still affect the wheel commands.
The selected response is the follower default; `STANDARD` remains available
for explicit comparison.

## Earlier experiments

## Event-triggered brake experiment

`TOLERANCE_BRAKE` leaves the tangent drive command positive while profiled
braking is within 3 in/s of target. When measured speed exceeds target by
more than that tolerance, it sends one -0.30 tangent feedforward cycle, then
allows one control cycle for the estimator to respond before another pulse.
Position and heading corrections retain allocation priority. During
acceleration, ordinary feedforward and velocity feedback remain in place
unless overspeed triggers a pulse. During braking between pulses, the command
uses `kS + kV*v_target` plus velocity feedback and is clamped to a small
positive drive duty.

With the 100 in/s² profile limit, two full AutoTest runs completed. Their
matched-position braking RMSE was 3.50 and 4.08 in/s, versus 4.76 in/s for
the feedback-density candidate. The event brake had five abrupt error drops
over 3 in/s per 20 ms in each run, versus zero in that density run. The
event-brake drops are isolated single-cycle steps rather than the repeated
mid-ramp collapses of regular `kA`. The frozen corner fixture also passed:
peak cross-track error was 0.32 in (limit 1.25 in), curve heading peak was
19.0 degrees (limit 25 degrees), and braking RMS was 4.89 in/s (limit 8).

The zero-command boundary is not a low-speed threshold. At 100 in/s², the
simulator's translation `kA` term has magnitude about 0.728 power while the
`kS + kV*v_target` term at 65 in/s is about 0.411. The unmodified command
would therefore be negative as soon as full braking begins. In the saved
event-brake repeat, 95 of 102 sustained-braking samples were below that
boundary, and all 20 strong-ramp pulses occurred there. Several additional
pulses occurred around low-speed transitions, including one after acceleration
resumed. Restricting the mode to the computed negative-command region made
the boundary switch visible in the chart and raised abrupt-drop count from
five to eight, so that restriction was not retained.

With the current motor model and 20 ms control interval, one zero/opposing
power cycle still produces a finite speed step. A predictive trigger or
different tolerance may shift when it happens, but cannot create an
intermediate continuous braking force. The event brake was opt-in during
these experiments and is now the default profiled braking response.

## Smoothing the braking trace

The later AutoTest experiments isolated the sharp dips from the broader
tracking error. A zero-power BRAKE control cycle causes a large speed step in
FTCodeSim. Regular `kA` plus velocity feedback can remain in that regime for
several cycles, then drive hard to recover. Reducing the profile acceleration
limit alone helps tracking but does not remove those steps.

`PULSE_DENSITY_FEEDBACK` uses the simulator braking-rate estimate to distribute
zero-power BRAKE cycles across the entire deceleration ramp. The requested
fraction increases when measured speed is above target and decreases when it
is below target. Normal velocity feedback remains active during acceleration;
the brake selector replaces translation `kA` and velocity feedback during
deceleration. The best tested setup uses a braking-rate slope of 7.0 per
second, density feedback rate of 15 per second, and a 100 in/s² profile
acceleration limit.

| Response | Profile acceleration limit | Braking RMSE at matched positions | Abrupt braking-error drops over 3 in/s per 20 ms |
| --- | ---: | ---: | ---: |
| Regular `kA` + velocity feedback | 131 in/s² | 9.21 in/s | 9 |
| Regular `kA` + velocity feedback | 100 in/s² | 6.59 in/s | 7 |
| Density feedback | 131 in/s² | 6.12, 6.33 in/s | 1, 1 |
| Density feedback | 100 in/s² | 4.90, 4.87, 4.76 in/s | 0, 1, 0 |

The last density run used AutoTest's original control-loop scheduler. The
short-cycle metric is the change in measured-minus-target velocity between
two sustained-braking samples, normalized to a 20 ms interval. The 100 in/s²
density runs had worst single drops of 1.94, 3.72, and 2.34 in/s per 20 ms,
versus 7.10 for regular `kA` and feedback at that same profile limit. All
three density runs completed AutoTest. Their outbound graphs are in
`TeamCode/build/braking-experiments/density_15_regen7_accel100*` and
`TeamCode/build/braking-experiments/density_normal_timing`.

One BRAKE cycle still has a finite speed step because the simulator motor
response jumps at zero power. The new schedule removes the repeated large
mid-ramp collapses, but it cannot make the motor response continuous. It also
trades some acceleration tracking for smoother braking: acceleration RMSE is
about 7.8 in/s, versus 6.0 with regular `kA` and feedback at the 100 in/s²
limit. The frozen corner-tracking fixture exceeds its 1.25 in cross-track
threshold with density feedback (1.31 in at the 100 in/s² setting). The
unchanged controller passes that cross-track check but fails the fixture's
braking-RMS limit under the same precise control-loop timing. Keep the new
response opt-in until corner behavior is resolved. That candidate was not selected.

To reproduce the smooth AutoTest run, set `APEX_BRAKING_RESPONSE` to
`PULSE_DENSITY_FEEDBACK`, `APEX_BRAKING_REGEN_RATE` to `7`,
`APEX_BRAKING_DENSITY_FEEDBACK_RATE` to `15`,
`APEX_FORWARD_ACCEL_LIMIT_IN` to `100`, and
`APEX_BRAKING_EXPERIMENT` to `true` before running `AutoTestSimulationTest`.
Set `APEX_AUTOTEST_DATA_DIR` and `APEX_AUTOTEST_GRAPH_DIR` to an isolated
output directory for its CSV and graph. The normal simulator regression now
uses `TOLERANCE_BRAKE`.

## Current evidence

The current profiled translation command is approximately
`u = kS*sign(v_target) + kV*v_target + kA*a_target`, with the same `kA`
for acceleration and braking. During braking, `MotionFeedforward` scales the
negative acceleration by `(measured_speed/target_speed)^2`, clipped to `[0, 1]`,
when measured speed is below target. Velocity feedback is added separately.

The saved zero-feedback-gain AutoTest CSV shows negative feedforward commands
at the start of five sustained braking intervals. Over the next ~0.08 s,
target speed drops 5–9 in/s; raw speed drops 8–21 in/s and filtered speed
drops 13–28 in/s. The fast raw-speed fall confirms that filtering alone
does not cause the disparity.

The current FTCodeSim motor has a command-regime gap around zero. At 40 in/s,
the direct motor test reports about +0.08 in/s² with 0.001 same-direction power,
about -400 in/s² at zero power in BRAKE mode, and about -400 in/s² with 0.001
opposing power. The full OpMode's first-0.12-s speed fits include chassis drag
and localization: approximately -6 to -10 in/s² for small same-direction
power versus -250 to -280 in/s² at zero or small opposing power. The profile
requests roughly -127 in/s² during its main braking ramps. No static command
near zero produces that acceleration in the current simulator. Changing `kA`
alone moves the time when feedforward crosses into the strong-brake branch. A
separate `kA` while command remains positive can still change tracking there.

## Full-path experiments

The experimental boundary is where the ordinary, speed-scaled translation
feedforward first becomes nonpositive during braking. All experiments retain
the ordinary `kA` response above it unless a braking `kA` scale is specified.
The original and alternative braking-response runs set the normal velocity
feedback gain to zero. The standard-with-feedback run instead applies the
simulator's tuned 0.066521 velocity gain throughout the path.

| Below-boundary response | Above-boundary braking `kA` scale | Braking RMSE at matched path positions (in/s) | Outbound time (s) |
| --- | ---: | ---: | ---: |
| Original | 1.0 | 18.53 | 14.75 |
| Original with velocity feedback throughout | 1.0 | 9.40 | 10.52 |
| Small same-direction coast duty | 1.0 | 15.84 | 12.85 |
| Pulse-density zero-power BRAKE | 1.0 | 8.57 | 8.96 |
| Velocity feedback, no `kA` | 1.0 | 6.68, 6.53 | 9.57, 9.75 |
| Velocity feedback, no `kA` | 0.00 | 5.98, 6.16 | 9.59, 9.57 |
| Velocity feedback, no `kA` | 0.25 | 6.09 | 9.42 |
| Velocity feedback, no `kA` | 0.50 | 6.07, 6.32 | 9.60, 9.62 |
| Velocity feedback, no `kA` | 1.50 | 7.20 | 9.71 |

The matched-position report uses one-inch path positions and classifies each
position from the original run. The feedback strategy has the lowest braking
error among the three below-boundary approaches. Pulse-density braking
improves average tracking but introduces more overspeed and visible ripple;
coasting alone leaves substantial underspeed. All routes completed, though
the original zero-gain run exceeded the ordinary 18-second runtime gate.

Regular `kA` plus tuned velocity feedback throughout the route is already a
large improvement over the original zero-gain run: 9.40 versus 18.53 in/s
braking RMSE. Its error is concentrated mainly on the original positive-power
side of the braking boundary (10.15 in/s RMSE there, versus 4.84 below it).
The below-boundary feedback candidate with no braking `kA` reaches 5.98-6.16
in/s overall, so regular `kA` throughout is useful but does not remove the
remaining braking disparity in this simulator.

Reducing `kA` only in the positive-power braking region gives a modest extra
improvement with the feedback strategy. The zero, 0.25, and 0.50 results are
too close to select a precise nonzero coefficient from this route. A 1.50
scale is worse. There is no evidence here that a separate *nonzero* braking
`kA` is needed; omitting it throughout braking is the simplest candidate.

These alternative strategies remain selectable through `APEX_BRAKING_RESPONSE`
in `AutoTestSimulationTest`; the follower now defaults to `TOLERANCE_BRAKE`.
`tools/compare_autotest_braking_strategies.py` reproduces the matched-position
summary from the saved CSV directories under `TeamCode/build/braking-experiments`.

## Earlier zero-gain recommendation

The first zero-gain comparison favored the velocity-feedback-only branch below
the zero-command boundary. In that branch, omit `kA`, retain the `kS + kV*v_target`
drive term, and use the tuned velocity feedback gain to switch into stronger
braking when speed exceeds target. This is deliberately a feedback response,
even when the normal velocity feedback gain is set to zero. Those AutoTest
runs slightly favored omitting `kA` throughout braking, including the
positive-power side of the original boundary. The later short-cycle analysis
above found that this branch still permits abrupt BRAKE steps, so the density
feedback experiment is the smoother AutoTest candidate.

The original pulse-density option remains useful as an open-loop comparison.
Its straight-line calculation is described below, but that early full-path
run favored the velocity-feedback branch. For a profiled braking command in
the acceleration gap,
pulse-density modulation represents the requested *average* deceleration
with a sequence of driven and zero-power BRAKE control cycles.

For a straight-line first pass, choose a small same-direction duty that holds
speed against chassis drag. Let `a_drive(v)` be its predicted acceleration and
`a_brake(v)` the predicted acceleration at zero power. The desired fraction
of braking cycles is

`f = clamp((a_drive(v) - a_target) / (a_drive(v) - a_brake(v)), 0, 1)`.

A pulse-density accumulator can distribute those cycles. It uses target
acceleration and the plant model, so the normal velocity feedback gain can
remain zero. The 20 ms loop means individual speed steps remain visible; the
goal is to match the profile over several cycles, not every instant. A
measured-speed selector could reduce accumulated error, but that is a
separate feedback controller and must be described as such.

`tools/explore_braking_switch.py` is a deliberately simplified straight-line
feasibility calculation. For a 60-to-6 in/s ramp at 127.1 in/s², its 20 ms
pulse-density schedule gives 2.75 in/s RMS error and 5.08 in/s peak error.
At 40-to-6 in/s, the corresponding values are 1.83 and 3.10 in/s. These
figures exclude path curvature, wheel-by-wheel commands, localization lag,
and endpoint capture. The full AutoTest prototype above is the more relevant
comparison.

The pulse-density prototype modifies tangent feedforward before translation,
heading, and correction are combined. Individual wheel commands can therefore
differ from the intended zero-power BRAKE selection on curves. This was an
earlier comparison, before selecting the event-brake response as the default.
