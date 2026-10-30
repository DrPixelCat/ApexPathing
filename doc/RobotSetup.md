# Guided robot setup

Select **Robot Setup** in the Apex Pathing OpMode group and press Start. This wizard configures robot hardware and encoders. Localization checks and calibration belong in **Localization Tuner**.

1. **Your robot:** choose mecanum, two-/four-motor tank, coaxial swerve, dual-actuated, or omni, then Pinpoint or two-/three-wheel odometry. Assign devices from the actual Robot Controller hardware configuration. Only relevant fields appear. Choose the odometry wheel model (or custom ticks per inch), encoder directions, and hub orientation where needed.
2. **Save:** after configuration, choose **Save robot setup**. No hand-movement tests, measured distance scaling, pod-offset calibration, or filter collection run in this wizard. Existing profiles can be reused or edited.
3. **Localization:** run **Localization Tuner** separately for direction checks, forward/sideways distance calibration, rotation geometry, and filtering. Accepted localization calibration stays in the saved profile.

Up/Down selects choices. A confirms or advances. There are no Back or Retry controls. Screens show **Press STOP to cancel or redo**; STOP ends the OpMode, and a new run starts a fresh session. Numeric fields support finer/coarser bumper adjustments. Driver Station Stop cancels the session without saving drafts. Hardware setup does not initialize or sample the localizer.

On drive-motor assignment and direction screens, keep the drive wheels clear and **hold X** to run only the selected motor at **positive 0.5 power**, using the displayed Normal/Reversed direction. Release X to command zero. Navigation, editing a selection/direction, or stopping the OpMode also stops the test. After navigation or edits, release X before holding it again. Encoder-port pickers do not run motors. The test restores the motor's prior running mode and direction on release and does not reset encoder counts (a prior reset mode is replaced with RUN_WITHOUT_ENCODER). Testing does not save the draft.

At the end, press A to **Save robot setup**, or STOP to keep the previously saved setup. The profile is stored at `/sdcard/FIRST/ApexPathing/robot-setup.json`; its preceding version is retained as `.bak`. A canceled session never replaces the saved profile. Save failures retain the previous setup and require STOP before a new run. Unreadable profiles can be repaired through the wizard, with a readable previous version retained for recovery.

`Constants` automatically loads the profile for future OpModes, including Follower Tuner. Hardware selection, motor/pod directions, and encoder resolution no longer require Java edits for the supported layouts. Without a saved profile, the existing Java configuration continues to work. Custom drivetrains stay outside the guided picker and use Java configuration and the advanced tuner; use `new Constants(false)` to bypass any saved guided profile intentionally. The guided picker currently offers the layouts listed above.


Swerve setup assigns each module's drive motor, steering CR servo, and analog absolute encoder. Drive motor, steering servo, and encoder directions are independent. On each forward-angle screen, point that wheel forward by hand and press **X** to capture its raw encoder zero; it stays in the draft until Save. You can also adjust the angle manually. Enter module spacing and steering P/D/static gains; P must be positive. Align modules by hand before the 0.5-power drive motor test. Setup does not run steering control.

Dual-actuated setup assigns the four drive motors and 1–8 positional actuation servos. Enter each servo's traction/holonomic positions (0–1), startup state, and settling time. Setup saves these positions without moving the servos. A normal driving OpMode applies the startup state and waits for the settling interval before enabling drive output. Localization Tuner follows the selected drive state's ability to move sideways.

Omni setup supports 3–8 uniformly spaced tangent-driving wheels. Register wheels in counterclockwise order and enter the first wheel's radial angle, measured counterclockwise from robot forward. Positive test power must propel each wheel along its counterclockwise tangent; adjust its motor direction accordingly. Custom wheel spacing or custom kinematics require Java configuration.

Existing version-1 profiles load with their original settings. New saves use version 2 and include the additional layout settings.

Existing follower control gains remain in `constants.json`. Changing hardware or localization can make those gains stale; the setup wizard does not characterize feedforward or run its verification phase. **Localization Tuner** owns all localization calibration, including geometry and filter settings; Robot Setup preserves existing calibration when settings are reused.

Logs live in `/sdcard/FIRST/ApexPathing/logs/robot-setup-<time>-<id>/`. `profiles.csv` records initial, edited, and saved profiles. `events.csv` records button actions, screen transitions, errors, and cancellation. `motor_tests.csv` records optional held-motor tests and their stop commands. Robot Setup does not create localization measurement logs.

Existing profile versions remain readable, including previously calibrated offsets and filter settings. The production Robot Setup OpMode is discoverable by Code Sim. Tests cover all three encoder layouts without localization initialization, saved calibration preservation, persisted reload, STOP cancellation, motor tests, and storage recovery.

Workflow ownership, input boundaries, and cleanup are documented in [StateWorkflows.md](StateWorkflows.md).
