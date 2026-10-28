# On-robot tuning and CSV capture

For first-time configuration and instruction-only localization calibration, start with **Robot Setup** and the [guided workflow](RobotSetup.md). It saves hardware mappings, pod calibration and filter settings for later OpModes. The manual localization workflow below remains available as **Localization Tuner (Advanced)**.

Run **Localization Tuner** for pod measurements and velocity-filter tuning. Run **Follower Tuner** for characterization, then choose **FEEDFORWARD VERIFICATION** to record independent feedforward-only profiles. The localizer in `Constants.java` remains your choice; configure it for the actual robot before deployment.

Each OpMode creates a unique folder under:

```text
/sdcard/FIRST/ApexPathing/logs/<session>-<timestamp>-<unique-id>/
```

The code resolves Android shared storage through `Environment.getExternalStorageDirectory()` instead of hardcoding a physical SD-card mount. The resolved folder is shown in telemetry. Existing constants remain in `FIRST/ApexPathing/constants.json`. Retrieve all sessions after stopping the OpMode:

```powershell
adb pull /sdcard/FIRST/ApexPathing/logs ./robot-tuning-logs
```

CSV output uses UTF-8, decimal points independent of device locale, and full floating-point precision. `NaN` means an unavailable measurement, not zero. Times ending in `_s` are seconds. `time_s` is monotonic time since the session opened; file-name timestamps are only labels. Data is buffered, flushed at least once per second when a table is being written, at phase/run boundaries, and on normal Stop/exception cleanup. A power loss can lose buffered rows. A storage error raises an error and the OpMode cleanup stops the motors; it never silently continues without logging.

## Localizer and measured pod offsets

Keep the robot stationary while its IMU calibrates. After Start, all motors remain stopped; move the robot by hand.

1. Verify pod scale and encoder direction first by moving straight forward and left. The CSV measurements use the configured encoder resolution; they cannot independently correct a wrong wheel scale or direction.
2. Mark the robot's intended tracking center on the floor and use an external heading mark or protractor. Set the planned physical rotation with Dpad Up/Down in 90-degree steps; Left selects clockwise (negative), Right counterclockwise (positive). Default: +360 degrees.
3. Press **X** while stationary to capture starting pod readings. Turn by hand through exactly the selected angle, keeping the marked center fixed. Press **A** only after stopping at the external heading mark. `kind=result` rows contain measured offsets. Stopping the OpMode mid-trial does not create a result.
4. Repeat clockwise and counterclockwise. Compare repeatability and configured versus measured offsets in `pod_offsets.csv`. Apply the reviewed values to the localizer configuration yourself.

| Localizer | Logged offset channel | Measurement convention |
| --- | --- | --- |
| TwoWheel | `forward_offset`, `strafe_offset` | Signed pod travel / external rotation, matching the existing rotation-compensation coefficients in `TwoWheel.Constants.offsets.x/y`. The forward coefficient is the negative of the pod's physical lateral position. |
| ThreeWheel | `parallel_separation`, `strafe_offset` | (Left travel - right travel) / external rotation; strafe travel / external rotation. These map to `ThreeWheel.Constants.offsets.x/y`. Individual left/right offsets cannot be identified separately from this point-turn test. |
| Pinpoint | `x_pod_offset`, `y_pod_offset` | -Forward travel / external rotation; strafe travel / external rotation. X offset is positive left of center; Y offset is positive forward of center. These map to `Pinpoint.Constants.setOffsets(x,y,IN)`. |

These measurements use raw pod travel and the operator's external angle, so an incorrect localizer heading scale does not become the calibration reference. The logged `reported_sweep_rad` is an unwrapped sum for comparison only. ThreeWheel still needs an initial nonzero parallel separation to produce a usable pose. Localizers without an implemented pod channel adapter (including drive encoders, OTOS, and Octoquad) show an unavailable message for pod offset capture; motion and filter logging remain available.

The Pinpoint driver's offset registers expect millimeters. This change corrects the previous meter conversion at those register writes. Existing nonzero Pinpoint offset configurations will now be applied at their intended physical scale. The linear-velocity corruption ceiling is raised from 1000 to 10000 mm/s; the previous 39.4 in/s ceiling discarded ordinary high-speed tuning data.

## Kalman tuning

**B** toggles noise adaptation. Both raw and filtered velocity/acceleration are always recorded, including before adaptation, throughout learning, and after noise parameters are frozen. Move through stationary intervals, smooth forward/backward motion, strafe where supported, and turns. Press B again when you want to freeze Q/R and continue collecting the resulting filtered response. There is no automatic freeze or convergence timeout. **Y**, while no pod trial is active, resets pose and filter state and records a reset event.

`kalman.csv` has one row per axis per loop:

- Raw and filtered velocity and acceleration; translation uses inches/second and inches/second², heading radians/second and radians/second².
- R (measurement variance), Q (process variance), innovation, innovation variance, instantaneous and smoothed NIS, and velocity/acceleration covariance diagonals.
- Initialization, adaptation, bootstrap, sample count, effective filter dt, gate inclusion, and timing-anomaly flags.

`filter_reports_tuned` is the existing filter's status, not a new convergence test. In particular, that method returns true when adaptation is disabled; interpret it with `initialized`, `adapting`, `bootstrap_complete`, and the actual curves. Q is the continuous process-noise intensity used to construct the dt-scaled covariance matrix, not the full per-update Q matrix. Kalman diagnostics never update the filter or select which signal is logged. Moving-average mode has raw/filtered motion rows but no Kalman rows. Learned Q/R are recorded in CSV; this workflow does not persist or auto-apply a new localizer configuration between OpModes.

## Feedforward characterization

Tune **HEADING**, **LIMITS**, and **DRIVE** first, or load suitable existing constants, then select **FEEDFORWARD** in Follower Tuner. kS comes from the heading/drive controller tuning phases; the feedforward regression holds that value fixed and fits kV and kA. These coefficients use normalized motor power, not volts. Battery voltage is logged without adding voltage compensation.

Automatic characterization runs quasistatic and dynamic excitations in each direction for angular and forward motion. Each run requires operator confirmation. The existing fit-quality checks still govern whether candidate kV/kA values are applied; they are separate from the new CSV-only verification phase.

| File | Contents |
| --- | --- |
| `feedforward_samples.csv` | Every running sample and settling rows, attempt/run/axis/excitation/direction, raw and filtered response, command aligned to the measured interval, next requested/applied effort, battery voltage, and inclusion/exclusion reason. |
| `feedforward_fits.csv` | kS, candidate kV/kA, power RMSE, leave-one-run-out RMSE, uncentered R², sample count, and whether the candidate pair was applied. Failed fits are retained. |
| `feedforward_regression.csv` | Exact fit inputs, predicted power, and residuals. Plot `power_minus_kS_kA_a` versus `abs_velocity`; the fitted line is `kV_times_velocity`. This is the kV component of the joint kV/kA regression with fixed kS. |
| `feedforward_manual.csv` | Manual refinement targets and measured velocities, with the next requested/clipped command. |
| `coefficients.csv` | Translation and angular kS/kV/kA at each phase's start and completion. |

The model is `abs(power) = kS + kV*abs(velocity) + kA*directed_acceleration`. Acceleration retains its deceleration sign. Forward response is projected from field coordinates onto the current robot heading, so characterization still works after a turn. Aligned effort is derived from post-limit motor commands, including their write tolerance: front-pair inversion for tank/mecanum/dual-actuated, kiwi mixer inversion, and signed wheel-power magnitude for pure swerve drive/turn. Swerve's scalar effort does not establish that steering modules have aligned; inspect its motor commands and response before trusting a fit. Moving-average mode additionally aligns effort with the filter window.

## Feedforward verification (operator review only)

Select **FEEDFORWARD VERIFICATION** directly in Follower Tuner, or continue into it after characterization. It is labeled **LOG ONLY** and has no saved pass/fail state. Positive, finite velocity and acceleration limits must already be configured.

- **LB** selects drive or turn. **RB** selects positive or negative direction. The selected profile's predicted travel is displayed; allow additional clearance for overshoot.
- **X** starts one run while stationary. Select and run forward, reverse, counterclockwise, and clockwise separately.
- **B** aborts an active run and commands zero. Driver Station Stop also stops motors and closes logs.
- **A**, while idle, finishes this phase; on the results screen B continues to the next tuning phase.

Each trapezoid uses half the configured velocity and acceleration limits, a one-second cruise, and a 0.75-second zero-command settling capture. The drivetrain receives feedforward alone; heading/drive feedback controllers are disabled. No coefficients are changed, no verification regression is run, and no result is accepted or rejected automatically.

`feedforward_verification.csv` includes signed targets, raw/filtered measurements, velocity error, coefficients, battery voltage, and both interval and next power commands. Compare targets and measurements throughout acceleration, cruise, deceleration, and settling. `interval_*` describes the previous command active before the current sensor read; `next_*` describes the new command just issued. The next requested/applied difference records clipping, drivetrain power limits, and motor write tolerance. A stopped or aborted run is distinct from a completed profile.

## Shared diagnostics

`motion.csv` records each sampled pose, raw/filtered field-frame velocity and acceleration, battery voltage, loop dt, per-motor power, power limit, and active coefficients. It is sampled after the follower/localizer update and before the phase issues its next command. During feedforward-only tests these powers describe the preceding interval. Other follower tuning phases can write commands inside `Follower.update()`, so use their rows as instantaneous diagnostics rather than interval-aligned system-identification data. `events.csv` records phase/run boundaries, retries, aborts, resets, and filter adaptation changes. Tables are created only when used.

The nominal loop sleeps 20 ms after work; use measured loop dt rather than assuming exactly 50 Hz. File flushes and hardware access can extend a loop. Preserve raw outliers and timing anomalies for offline inspection. Treat candidate coefficients and measured offsets as data from the actual robot session; desktop tests establish software behavior, not physical tuning quality.

References: [FIRST shared-storage/log retrieval documentation](https://ftc-docs.firstinspires.org/en/latest/control_system_troubleshooting/using_log_files/using-log-files.html), [goBILDA Pinpoint units and offset conventions](https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf).
