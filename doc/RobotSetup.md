# Guided robot setup

Select **Robot Setup** in the Apex Pathing OpMode group and press Start. Localization uses hand movements. Motor assignment has an optional hold-to-test control. Every screen shows the next action rather than numerical tuning results. Raw and filtered data, measurements, decisions, and configuration snapshots are written to CSV automatically.

1. **Your robot:** choose mecanum or two-/four-motor tank, then Pinpoint or two-/three-wheel odometry. Assign devices from the actual Robot Controller hardware configuration. Only relevant fields appear. Choose the pod model, hub orientation where needed, an approximate parallel-pod spacing for three-wheel odometry, and a marked forward distance.
2. **Sensor checks:** follow the forward, left, and counterclockwise instructions. Press A **before** moving and A again after stopping. Reversed pod readings produce an explicit correction choice followed by a repeat check. Forward travel also measures encoder scale. For tank, the sideways check rolls the sideways pod by hand with the chassis stationary.
3. **Localization:** perform one externally marked full turn in each direction about a fixed tracking center. Choose **Use this measurement**, **Repeat this step**, or **Keep previous setting**. Accepted turns are combined into pod offsets. Then record stationary and smooth-motion filter data. The operator decides whether to use the collected filter settings; there is no automatic convergence acceptance.

Up/Down selects choices. A confirms or advances. B goes back or cancels the current collection. Numeric fields support finer/coarser bumper adjustments. Sensor rebuilding includes a stationary settling screen. Driver Station Stop cancels the session without saving drafts.

On drive-motor assignment and direction screens, keep the drive wheels clear and **hold X** to run only the selected motor at **positive 0.5 power**, using the displayed Normal/Reversed direction. Release X to command zero. Navigation, editing a selection/direction, or stopping the OpMode also stops the test. After navigation or edits, release X before holding it again. Encoder-port pickers do not run motors. The test restores the motor's prior running mode and direction on release and does not reset encoder counts (a prior reset mode is replaced with RUN_WITHOUT_ENCODER). Testing does not save the draft.

At the end, explicitly choose **Save robot setup** or **Keep the previously saved setup**. The profile is stored at `/sdcard/FIRST/ApexPathing/robot-setup.json`; its preceding version is retained as `.bak`. A canceled session never replaces the saved profile. Save failures offer retry. Unreadable profiles can be repaired through the wizard, with a readable previous version retained for recovery.

`Constants` automatically loads the profile for future OpModes, including Follower Tuner. Hardware selection, motor/pod directions, encoder resolution, offsets, and accepted Kalman noise settings no longer require Java edits for the supported layouts. Without a saved profile, the existing Java configuration continues to work. Custom/swerve/dual-actuated setups can continue using their existing Java configuration and the advanced tuner; use `new Constants(false)` to bypass any saved guided profile intentionally. The guided picker currently offers the layouts listed above.

Existing follower control gains remain in `constants.json`. Changing hardware or localization can make those gains stale; the setup wizard does not characterize feedforward or run its verification phase. The older localizer screens remain available as **Localization Tuner (Advanced)**.

Logs live in `/sdcard/FIRST/ApexPathing/logs/robot-setup-<time>-<id>/`:

| CSV | Background data |
| --- | --- |
| `motion.csv` | Raw/filtered pose, velocity and acceleration, loop timing, sample validity and battery voltage |
| `pods.csv` | Raw encoder counts and directed pod travel |
| `kalman.csv` | Per-axis measurement/process noise throughout collection |
| `diagnostics.csv` | Estimator internals and zero motor commands; main logs innovations/NIS/covariance and DeveloperTesting logs its joint estimator's covariance and pose-noise settings |
| `measurements.csv` | Physical reference rotations, measured travel and offsets |
| `profiles.csv` | Initial, edited, candidate, applied-draft, and saved profiles |
| `events.csv` | Button actions, screen transitions, retries, hardware/storage errors and cancellation |
| `motor_tests.csv` | Hold-test commands, selected motor/direction, reported power, encoder position/velocity and battery voltage; stop events include a zero command |

DeveloperTesting uses this same workflow controller and profile format, retains its joint pose estimator, and captures its complete native estimator settings in the profile. It also keeps the advanced localization tuner synchronized with that profile. Its desktop storage root can be redirected with the existing `apexpathing.storageDirectory` JVM property. The production Robot Setup OpMode is discoverable by Code Sim. Tests exercise all three pod-localizer workflows, persisted reload, direction corrections, repeats, cancellations, invalid samples and storage recovery.
