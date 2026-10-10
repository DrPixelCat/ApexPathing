# Tuner telemetry conventions

Keep Driver Station telemetry compact enough that values, results, and available
controls stay visible together. The shared tuner context and manual feedback,
centripetal, and tracking phases provide the conventions below. Older phases may
still use longer labels or control hints; use these conventions when updating them.

## Layout and formatting

- Put editable values first. Use `TuningPhase.addTunableValue` to put `->` beside
  the selected value rather than adding a separate selection row.
- Show the editing increment once, after the values. Group related button hints
  into short lines and show only controls available in the current state.
- Use `number(...)` / `TunerContext.formatNumber(...)`: normal telemetry uses up
  to five decimal places without trailing zeros; debug mode uses full precision.
- Give metrics short, distinct labels and include units with their values. Keep
  unavailable results explicit as `N/A` with a brief reason.
- Put profile descriptions and metric explanations on the pre-run screen and in
  documentation. Idle screens retain direction and required clearance; running
  screens retain trial state, results, cancellation, and actionable warnings.
- Show clipping, invalid measurements, and aborted-trial messages when relevant.
  Do not hide them in debug mode or turn them into an apparent successful result.
- Let `TunerContext.publishFrame(...)` append `Robot can be driven with sticks`
  as the final row only when stick driving is available. An active routine owns
  drivetrain output and omits that row.

The context clears each frame, adds shared headers, and publishes once per loop.
Keep those responsibilities in the shared loop. Debug adds its banner and exit
hint; dual-actuated drivetrains also display the active tuning profile.

Automatic screens show the constants being tuned and required positioning or
operator actions. Detailed fit diagnostics, sample traces, and CSV paths belong
in debug output or saved logs rather than every normal running frame.

## Manual SysId

Drive and heading SysId follow the same layout: kS/kV/kA with a selection arrow,
increment, running/next direction, idle clearance, trial result, available velocity
errors, then state-specific controls. Normal frames use at most 10 rows before
the first trial, 12 after a trial, and 10 during motion, excluding optional
clipping warnings and shared debug/profile headers.

LB/RB selects a gain, D-pad Up/Down edits it, and Left/Right changes the increment.
X starts a trial from rest with sticks released. A accepts edited gains while
idle; A on the results screen saves and continues. During motion these actions
are ignored; Stop cancels. Directions alternate forward/backward or CCW/CW.

The pre-run screen describes the fixed profile: one second each of acceleration,
cruise, and braking. Drive targets 24 in/s with 48 inches nominal travel; heading
targets 1.5 rad/s with 3 radians nominal rotation. The idle prompt requires a
clear lane of 60 inches in the next drive direction, or rotation room for 3 radians
(about 172 degrees), plus braking room.

`Accel mean error` and `Cruise mean error` are **time-weighted mean signed velocity
errors**, measured minus target, in in/s for drive and rad/s for heading.
Acceleration covers the acceleration interval. Cruise starts at the first measured
target-velocity crossing and ends with cruise; if the target is never reached or
there are no usable samples, its result is unavailable. Braking is excluded.
Negative error means too slow forward/CCW; positive error means too slow
backward/CW. Errors refer to the current or most recent trial, whose direction is
shown by `Running` or the completed `Trial` message; `Next direction` describes
the upcoming trial. `Power clipped` means the errors include command saturation.

For UI changes, run the existing manual drive and heading cases in
`FollowerTunerTelemetryTest`. They cover editing, alternating directions, motion
ownership, compact row counts, units, controls, and the stick-availability footer.
